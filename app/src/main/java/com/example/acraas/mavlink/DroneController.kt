package com.example.acraas.mavlink

import android.content.Context
import android.hardware.usb.UsbDeviceConnection
import android.hardware.usb.UsbManager
import android.os.SystemClock
import android.util.Log
import com.hoho.android.usbserial.driver.UsbSerialPort
import com.hoho.android.usbserial.driver.UsbSerialProber
import kotlinx.coroutines.*
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.util.concurrent.Executors

class DroneController(private val context: Context, private val listener: DroneListener) {
    // Scrierile si starea de control trec prin acelasi worker
    // Writes and control state go through the same worker
    private val worker = Executors.newSingleThreadExecutor().asCoroutineDispatcher()
    private val scope = CoroutineScope(SupervisorJob() + worker)
    private var serialPort: UsbSerialPort? = null
    private var connection: UsbDeviceConnection? = null
    private var telemetryJob: Job? = null
    private var controlJob: Job? = null
    private var codec = MavlinkCodec()
    @Volatile private var session = OffboardSession()
    private var targetSystemId: Int? = null
    private var targetComponentId: Int? = null
    private var lastHeartbeatAtMs = 0L
    private var confirmedOffboard: Boolean? = null
    private var connectionEpoch = 0
    @Volatile private var connected = false
    @Volatile private var disposed = false

    @Volatile private var attitudeSample: Pair<Long, AttitudeData>? = null
    @Volatile private var velocitySample: Pair<Long, FloatArray>? = null

    interface DroneListener {
        fun onDroneConnected()
        fun onDroneDisconnected()
        fun onDroneError(error: String)
        fun onOffboardModeChanged(isActive: Boolean)
        fun onTelemetryReceived(attitude: AttitudeData)
        fun onTrackingStopped(reason: String)
    }

    data class AttitudeData(val rollRad: Float, val pitchRad: Float, val yawRad: Float,
                            val rollRateRadS: Float, val pitchRateRadS: Float, val yawRateRadS: Float)

    private fun notifyListener(block: DroneListener.() -> Unit) {
        if (!disposed) scope.launch(Dispatchers.Main) { if (!disposed) listener.block() }
    }

    fun initialize() {
        if (disposed) return
        scope.launch {
            if (serialPort != null || disposed) return@launch
            try {
                val manager = context.getSystemService(Context.USB_SERVICE) as UsbManager
                val driver = UsbSerialProber.getDefaultProber().findAllDrivers(manager).firstOrNull()
                    ?: error("Nu s-au gasit dispozitive USB seriale conectate")
                check(manager.hasPermission(driver.device)) {
                    "Lipseste permisiunea USB. Reconecteaza Pixhawk si acorda acces aplicatiei"
                }
                connection = manager.openDevice(driver.device) ?: error("Nu s-a putut deschide conexiunea USB")
                val port = driver.ports.first()
                serialPort = port
                port.open(connection)
                port.setParameters(57600, 8, UsbSerialPort.STOPBITS_1, UsbSerialPort.PARITY_NONE)
                port.dtr = true
                port.rts = true
                codec = MavlinkCodec()
                session = OffboardSession()
                targetSystemId = null
                targetComponentId = null
                confirmedOffboard = null
                lastHeartbeatAtMs = SystemClock.elapsedRealtime()
                val epoch = ++connectionEpoch
                startTelemetry(port, epoch)
                startControlLoop()

            } catch (e: Exception) {
                fail("Conectare esuata: ${e.message}")
            }
        }
    }

    private fun startTelemetry(port: UsbSerialPort, epoch: Int) {
        telemetryJob = scope.launch(Dispatchers.IO) {
            val buffer = ByteArray(1024)
            try {
                while (isActive) {
                    val count = port.read(buffer, 100)
                    if (count > 0) {
                        val bytes = buffer.copyOf(count)
                        withContext(worker) {
                            // Ignor datele ramase de la conexiunea veche
                            // Ignore data left over from the old connection
                            if (epoch == connectionEpoch) codec.feed(bytes).forEach(::receive)
                        }
                    }
                }
            } catch (e: CancellationException) {
                throw e
            } catch (e: Exception) {
                withContext(worker) {
                    if (epoch == connectionEpoch) fail("Citire USB esuata: ${e.message}")
                }
            }
        }
    }

    private fun startControlLoop() {
        controlJob = scope.launch {
            var nextHeartbeatAtMs = 0L
            var nextControlTickAtMs = SystemClock.elapsedRealtime()
            try {
                while (isActive && serialPort != null) {
                    val nowMs = SystemClock.elapsedRealtime()
                    if (nowMs - lastHeartbeatAtMs >= OffboardSession.HEARTBEAT_TIMEOUT_MS) {
                        fail("Heartbeat PX4 pierdut sau neprimit")
                        return@launch
                    }
                    if (nowMs >= nextHeartbeatAtMs) {
                        send(MavlinkMessageIds.HEARTBEAT, MavlinkMessages.heartbeat())
                        nextHeartbeatAtMs = nowMs + 1000
                    }
                    for (action in session.tick(nowMs)) {
                        when (action) {
                            is ControlAction.Setpoint -> send(MavlinkMessageIds.SET_ATTITUDE_TARGET,
                                MavlinkMessages.bodyRateTarget(nowMs, requireNotNull(targetSystemId),
                                    requireNotNull(targetComponentId), action.command))
                            is ControlAction.Mode -> send(MavlinkMessageIds.SET_MODE,
                                MavlinkMessages.mode(requireNotNull(targetSystemId), action.mainMode))
                            ControlAction.Arm -> send(MavlinkMessageIds.COMMAND_LONG,
                                MavlinkMessages.arm(requireNotNull(targetSystemId), requireNotNull(targetComponentId)))
                            is ControlAction.Stopped -> notifyListener { onTrackingStopped(action.reason) }
                        }
                    }
                    nextControlTickAtMs += 50L
                    val afterWriteAtMs = SystemClock.elapsedRealtime()
                    if (nextControlTickAtMs <= afterWriteAtMs) nextControlTickAtMs = afterWriteAtMs + 50L
                    delay(nextControlTickAtMs - afterWriteAtMs)
                }
            } catch (e: CancellationException) {
                throw e
            } catch (e: Exception) {
                fail("Trimitere MAVLink esuata: ${e.message}")
            }
        }
    }

    private fun receive(frame: MavlinkFrame) {
        val body = ByteBuffer.wrap(frame.payload).order(ByteOrder.LITTLE_ENDIAN)
        if (frame.messageId == MavlinkMessageIds.HEARTBEAT) {
            // Conexiunea devine activa doar dupa un heartbeat valid de la PX4 principal
            // The connection becomes active only after a valid heartbeat from the primary PX4
            if (frame.systemId == 0 || frame.systemId == MavlinkCodec.SENDER_SYSTEM_ID || frame.componentId != 1 ||
                (frame.payload[5].toInt() and 255) != 12) return
            if (targetSystemId != null && frame.systemId != targetSystemId) return
            targetSystemId = frame.systemId
            targetComponentId = frame.componentId
            lastHeartbeatAtMs = SystemClock.elapsedRealtime()
            val customMode = body.int
            val offboard = ((customMode ushr 16) and 255) == Px4MainModes.OFFBOARD
            val armed = (frame.payload[6].toInt() and 128) != 0
            session.heartbeat((customMode ushr 16) and 255, armed, lastHeartbeatAtMs)
            if (!connected) {
                connected = true
                notifyListener { onDroneConnected() }
            }
            if (confirmedOffboard != offboard) {
                confirmedOffboard = offboard
                notifyListener { onOffboardModeChanged(offboard) }
            }
            return
        }
        if (frame.systemId != targetSystemId || frame.componentId != targetComponentId) return
        when (frame.messageId) {
            MavlinkMessageIds.ATTITUDE -> {
                body.int
                val attitude = AttitudeData(body.float, body.float, body.float,
                    body.float, body.float, body.float)
                attitudeSample = SystemClock.elapsedRealtime() to attitude
                notifyListener { onTelemetryReceived(attitude) }
            }
            MavlinkMessageIds.LOCAL_POSITION_NED -> {
                body.position(16)
                velocitySample = SystemClock.elapsedRealtime() to floatArrayOf(body.float, body.float, body.float)
            }
            MavlinkMessageIds.COMMAND_ACK -> {
                val command = body.short.toInt() and 65535

                val system = frame.payload.getOrElse(8) { 0 }.toInt() and 255
                val component = frame.payload.getOrElse(9) { 0 }.toInt() and 255
                if ((system != 0 && system != MavlinkCodec.SENDER_SYSTEM_ID) || (component != 0 && component != MavlinkCodec.SENDER_COMPONENT_ID)) return
                if (command == MavlinkCommandIds.COMPONENT_ARM_DISARM) session.armAcknowledged(body.get().toInt() and 255)
            }
        }
    }

    fun setTrackingEnabled(enabled: Boolean) {
        if (disposed) return
        scope.launch {
            if (enabled) {
                if (!connected || session.phase == OffboardSession.Phase.EXITING) {
                    notifyListener { onTrackingStopped("Asteapta conexiunea si confirmarea iesirii din OFFBOARD") }
                } else session.enable()
            } else session.disable()
        }
    }

    fun vehicleState(nowMs: Long = SystemClock.elapsedRealtime()): VehicleState? {
        val (attitudeAtMs, attitude) = attitudeSample ?: return null
        val (velocityAtMs, velocity) = velocitySample ?: return null
        if (nowMs - attitudeAtMs > ThrustAxisController.STATE_TIMEOUT_MS ||
            nowMs - velocityAtMs > ThrustAxisController.STATE_TIMEOUT_MS) return null
        return VehicleState(attitude.rollRad, attitude.pitchRad, attitude.yawRad, velocity[0], velocity[1], velocity[2])
    }

    // Aici doar actualizez setpoint-ul; timer-ul se ocupa de trimitere si timeout
    // Only update the setpoint here; the timer handles sending and timeout
    fun sendBodyRateCommand(rollRateRadS: Float, pitchRateRadS: Float, yawRateRadS: Float, thrust: Float,
                                observedAtMs: Long) {
        if (disposed) return
        val command = try {
            BodyRateCommand(rollRateRadS, pitchRateRadS, yawRateRadS, thrust).limited()
        } catch (_: IllegalArgumentException) {
            notifyListener { onTrackingStopped("Comanda invalida (NaN/Infinity)") }
            setTrackingEnabled(false)
            return
        }
        scope.launch { session.updateCommand(command, observedAtMs, SystemClock.elapsedRealtime()) }
    }

    private fun send(messageId: Int, payload: ByteArray) {
        checkNotNull(serialPort) { "Port USB inchis" }.write(codec.encode(messageId, payload), 200)
    }

    private fun fail(message: String) {
        Log.e(TAG, message)
        notifyListener { onDroneError(message) }
        disconnectInternal()
    }

    fun disconnect() {
        if (!disposed) scope.launch { disconnectInternal() }
    }

    private fun disconnectInternal() {
        // Incerc MANUAL inainte de inchidere, dar nu am confirmare aici
        // Try MANUAL before closing, but there is no confirmation here
        if (session.ownsOffboard && targetSystemId != null && serialPort != null) {
            try { send(MavlinkMessageIds.SET_MODE, MavlinkMessages.mode(targetSystemId!!, Px4MainModes.MANUAL)) }
            catch (e: Exception) { Log.w(TAG, "Cererea MANUAL la deconectare a esuat", e) }
        }
        ++connectionEpoch
        connected = false
        controlJob?.cancel()
        telemetryJob?.cancel()
        controlJob = null
        telemetryJob = null
        try { serialPort?.close() } catch (e: Exception) { Log.w(TAG, "USB close", e) }
        try { connection?.close() } catch (e: Exception) { Log.w(TAG, "Connection close", e) }
        serialPort = null
        connection = null
        targetSystemId = null
        targetComponentId = null
        confirmedOffboard = null
        session = OffboardSession()
        notifyListener { onDroneDisconnected() }
    }

    fun isConnected(): Boolean = connected

    fun cleanup() {
        if (disposed) return
        disposed = true
        scope.launch {
            try { disconnectInternal() } finally {
                scope.cancel()
                worker.close()
            }
        }
    }

    companion object { private const val TAG = "DroneController" }
}
