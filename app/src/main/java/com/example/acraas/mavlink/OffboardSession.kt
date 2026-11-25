package com.example.acraas.mavlink

sealed class ControlAction {
    data class Setpoint(val command: BodyRateCommand) : ControlAction()
    data class Mode(val mainMode: Int) : ControlAction()
    object Arm : ControlAction()
    data class Stopped(val reason: String) : ControlAction()
}

// tick vine din timer la 50 ms, independent de camera
// tick comes from a 50 ms timer, independent of the camera
class OffboardSession {
    enum class Phase { IDLE, PRIMING, ENTERING, ARMING, ACTIVE, EXITING, FAILED }
    @Volatile var phase = Phase.IDLE
        private set
    private var enabled = false
    private var latestCommand: BodyRateCommand? = null
    private var latestCommandObservedAtMs = 0L
    private var phaseStartedAtMs = 0L
    private var previousTickAtMs: Long? = null
    private var modeRequestedAtMs: Long? = null
    private var lastHeartbeatAtMs = Long.MIN_VALUE
    private var mainMode = 0
    @Volatile private var armed = false
    private var armRejection: Int? = null

    // Daca drona e deja armata, nu astept heartbeat-ul de 1 Hz fara steering
    // If the drone is already armed, do not wait for the 1 Hz heartbeat without steering
    val appliesSteering: Boolean get() = phase == Phase.ACTIVE || (phase == Phase.ENTERING && armed)

    val ownsOffboard: Boolean get() = phase in listOf(Phase.PRIMING, Phase.ENTERING,
        Phase.ARMING, Phase.ACTIVE, Phase.EXITING)

    fun enable() {
        if (enabled || phase == Phase.EXITING) return
        enabled = true
        latestCommand = null
        armRejection = null
        phase = Phase.IDLE
    }

    fun disable() {
        enabled = false
        latestCommand = null
        if (ownsOffboard && phase != Phase.EXITING) {
            phase = Phase.EXITING
            modeRequestedAtMs = null
        }
    }

    fun updateCommand(command: BodyRateCommand, observedAtMs: Long, nowMs: Long) {
        if (!enabled || observedAtMs > nowMs || nowMs - observedAtMs > TARGET_TIMEOUT_MS) return
        if (latestCommand != null && observedAtMs < latestCommandObservedAtMs) return
        latestCommand = command.limited()
        latestCommandObservedAtMs = observedAtMs
    }

    fun heartbeat(mainMode: Int, armed: Boolean, nowMs: Long) {
        this.mainMode = mainMode
        this.armed = armed
        lastHeartbeatAtMs = nowMs
    }

    fun armAcknowledged(result: Int) {
        if (phase == Phase.ARMING && result != 0 && result != 5) armRejection = result
    }

    fun tick(nowMs: Long): List<ControlAction> {
        val actions = mutableListOf<ControlAction>()
        fun stop(reason: String): List<ControlAction> {
            disable()
            return listOf(ControlAction.Stopped(reason)) + tick(nowMs)
        }
        if (phase == Phase.EXITING) {
            val requestedAtMs = modeRequestedAtMs
            if (requestedAtMs != null && lastHeartbeatAtMs >= requestedAtMs && mainMode != Px4MainModes.OFFBOARD) {
                phase = Phase.IDLE
            } else if (requestedAtMs == null) {
                modeRequestedAtMs = nowMs
                actions += ControlAction.Mode(Px4MainModes.MANUAL)
            } else if (nowMs - requestedAtMs >= MODE_TIMEOUT_MS) {
                phase = Phase.FAILED
                actions += ControlAction.Stopped("Revenirea din OFFBOARD nu a fost confirmata")
            }
            return actions
        }
        if (!enabled) return actions

        // Pilotul are prioritate. O cerere MANUAL aici i-ar putea schimba modul ales
        // The pilot has priority. A MANUAL request here could change their selected mode
        if ((phase == Phase.ACTIVE && (mainMode != Px4MainModes.OFFBOARD || !armed)) ||
            (phase == Phase.ARMING && mainMode != Px4MainModes.OFFBOARD)) {
            enabled = false
            latestCommand = null
            phase = Phase.IDLE
            return listOf(ControlAction.Stopped("Controlul a fost preluat de autopilot/pilot"))
        }
        val command = latestCommand ?: return actions
        if (nowMs - latestCommandObservedAtMs >= TARGET_TIMEOUT_MS) return stop("Tinta pierduta - tracking oprit")
        if (lastHeartbeatAtMs == Long.MIN_VALUE || nowMs - lastHeartbeatAtMs >= HEARTBEAT_TIMEOUT_MS) {
            return stop("Telemetrie pierduta - tracking oprit")
        }
        armRejection?.let { return stop("Armare refuzata de autopilot (MAV_RESULT=$it)") }
        if (phase == Phase.IDLE) {
            phase = Phase.PRIMING
            phaseStartedAtMs = nowMs
            previousTickAtMs = null
        }
        if (phase == Phase.PRIMING) {
            // O pauza mare in writer nu se pune ca prestream continuu
            // A long writer gap does not count as continuous prestream
            if (previousTickAtMs?.let { nowMs - it > MAX_STREAM_GAP_MS } == true) phaseStartedAtMs = nowMs
            if (nowMs - phaseStartedAtMs >= PRESTREAM_MS) {
                phase = Phase.ENTERING
                phaseStartedAtMs = nowMs
                actions += ControlAction.Mode(Px4MainModes.OFFBOARD)
            }
        } else if (phase == Phase.ENTERING) {
            if (lastHeartbeatAtMs >= phaseStartedAtMs && mainMode == Px4MainModes.OFFBOARD) {
                if (armed) phase = Phase.ACTIVE else {
                    phase = Phase.ARMING
                    phaseStartedAtMs = nowMs
                    actions += ControlAction.Arm
                }
            } else if (nowMs - phaseStartedAtMs >= MODE_TIMEOUT_MS) {
                return stop("Activarea OFFBOARD nu a fost confirmata")
            }
        } else if (phase == Phase.ARMING) {
            if (armed && lastHeartbeatAtMs >= phaseStartedAtMs) phase = Phase.ACTIVE
            else if (nowMs - phaseStartedAtMs >= MODE_TIMEOUT_MS) return stop("Armarea nu a fost confirmata")
        }
        previousTickAtMs = nowMs

        actions.add(0, ControlAction.Setpoint(if (appliesSteering) command else command.withoutBodyRates()))
        return actions
    }

    companion object {
        const val TARGET_TIMEOUT_MS = 1000L
        const val HEARTBEAT_TIMEOUT_MS = 3000L
        const val PRESTREAM_MS = 1100L
        const val MODE_TIMEOUT_MS = 3000L
        private const val MAX_STREAM_GAP_MS = 200L
    }
}
