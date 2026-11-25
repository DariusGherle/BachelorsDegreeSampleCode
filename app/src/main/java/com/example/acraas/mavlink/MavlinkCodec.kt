package com.example.acraas.mavlink

import java.nio.ByteBuffer
import java.nio.ByteOrder

object MavlinkMessageIds {
    const val HEARTBEAT = 0
    const val SET_MODE = 11
    const val ATTITUDE = 30
    const val LOCAL_POSITION_NED = 32
    const val COMMAND_LONG = 76
    const val COMMAND_ACK = 77
    const val SET_ATTITUDE_TARGET = 82
}

object MavlinkCommandIds {
    const val COMPONENT_ARM_DISARM = 400
}

object Px4MainModes {
    const val MANUAL = 1
    const val OFFBOARD = 6
}

object MavlinkMessages {
    private const val ATTITUDE_IGNORE_MASK = 128

    private fun payload(size: Int) = ByteBuffer.allocate(size).order(ByteOrder.LITTLE_ENDIAN)

    fun heartbeat(): ByteArray = payload(9).apply {
        putInt(0)
        put(6.toByte())
        put(8.toByte())
        put(0.toByte())
        put(3.toByte())
        put(3.toByte())
    }.array()

    fun mode(targetSystemId: Int, mainMode: Int): ByteArray = payload(6).apply {
        // PX4 pune main_mode in bitii 16..23 din custom_mode
        // PX4 puts main_mode in bits 16..23 of custom_mode
        putInt(mainMode shl 16)
        put(targetSystemId.toByte())
        put(1.toByte())
    }.array()

    fun arm(targetSystemId: Int, targetComponentId: Int): ByteArray = payload(33).apply {
        putFloat(1f)
        repeat(6) { putFloat(0f) }
        putShort(MavlinkCommandIds.COMPONENT_ARM_DISARM.toShort())
        put(targetSystemId.toByte())
        put(targetComponentId.toByte())
        put(0.toByte())
    }.array()

    fun bodyRateTarget(timeBootMs: Long, targetSystemId: Int, targetComponentId: Int,
                       command: BodyRateCommand): ByteArray = payload(39).apply {
        // Timestamp-ul e uint32 pe wire, deci face wrap la 2^32
        // The wire timestamp is uint32, so it wraps at 2^32
        putInt(timeBootMs.toInt())
        putFloat(1f)
        repeat(3) { putFloat(0f) }
        putFloat(command.rollRateRadS)
        putFloat(command.pitchRateRadS)
        putFloat(command.yawRateRadS)
        putFloat(command.thrust)
        put(targetSystemId.toByte())
        put(targetComponentId.toByte())
        // Ignor quaternionul de mai sus, trimit body rates si thrust
        // Ignore the quaternion above, send body rates and thrust
        put(ATTITUDE_IGNORE_MASK.toByte())
    }.array()
}

data class BodyRateCommand(val rollRateRadS: Float, val pitchRateRadS: Float,
                           val yawRateRadS: Float, val thrust: Float) {
    fun limited(): BodyRateCommand {
        require(listOf(rollRateRadS, pitchRateRadS, yawRateRadS, thrust).all { it.isFinite() })
        return BodyRateCommand(rollRateRadS.coerceIn(-MAX_ROLL_PITCH_RATE_RAD_S, MAX_ROLL_PITCH_RATE_RAD_S),
            pitchRateRadS.coerceIn(-MAX_ROLL_PITCH_RATE_RAD_S, MAX_ROLL_PITCH_RATE_RAD_S),
            yawRateRadS.coerceIn(-MAX_YAW_RATE_RAD_S, MAX_YAW_RATE_RAD_S),
            thrust.coerceIn(MIN_THRUST, MAX_THRUST))
    }

    fun withoutBodyRates() = copy(rollRateRadS = 0f, pitchRateRadS = 0f, yawRateRadS = 0f)

    companion object {
        // Limita e pe fiecare axa; controllerul limiteaza separat modulul ratei
        // This limit is per axis; the controller separately limits the rate magnitude
        const val MAX_ROLL_PITCH_RATE_RAD_S = 4f
        const val MAX_YAW_RATE_RAD_S = 1f
        const val MIN_THRUST = 0.3f
        const val MAX_THRUST = 0.8f
    }
}

data class MavlinkFrame(val systemId: Int, val componentId: Int,
                        val messageId: Int, val payload: ByteArray)

class MavlinkCodec {
    private var sequence = 0
    private var pending = byteArrayOf()

    fun encode(messageId: Int, payload: ByteArray): ByteArray {
        require(messageId in 0..255 && payload.size <= 255)
        val extra = requireNotNull(CRC_EXTRA[messageId]) { "Unknown MAVLink message $messageId" }
        val packet = ByteArray(payload.size + 8)
        packet[0] = MAVLINK_V1_START_BYTE.toByte()
        packet[1] = payload.size.toByte()
        packet[2] = (sequence++ and 255).toByte()
        packet[3] = SENDER_SYSTEM_ID.toByte()
        packet[4] = SENDER_COMPONENT_ID.toByte()
        packet[5] = messageId.toByte()
        payload.copyInto(packet, 6)
        val crc = checksum(packet, 1, payload.size + 5, extra)
        packet[packet.lastIndex - 1] = crc.toByte()
        packet[packet.lastIndex] = (crc ushr 8).toByte()
        return packet
    }

    fun feed(bytes: ByteArray): List<MavlinkFrame> {
        pending += bytes
        val frames = mutableListOf<MavlinkFrame>()
        var start = 0
        while (start < pending.size) {
            val magic = pending[start].toInt() and 255
            if (magic != MAVLINK_V1_START_BYTE && magic != MAVLINK_V2_START_BYTE) { start++; continue }
            val headerSize = if (magic == MAVLINK_V1_START_BYTE) 6 else 10
            if (pending.size - start < headerSize) break
            val length = pending[start + 1].toInt() and 255
            val flags = if (magic == MAVLINK_V2_START_BYTE) pending[start + 2].toInt() and 255 else 0
            val packetSize = headerSize + length + 2 + if (flags and 1 != 0) SIGNATURE_LENGTH_BYTES else 0
            if (pending.size - start < packetSize) break

            // Fara cheie si protectie replay nu accept pachete semnate
            // Without a key and replay protection, signed packets are rejected
            if (flags != 0) { start += packetSize; continue }
            val messageId = if (magic == MAVLINK_V1_START_BYTE) pending[start + 5].toInt() and 255 else
                (pending[start + 7].toInt() and 255) or
                    ((pending[start + 8].toInt() and 255) shl 8) or
                    ((pending[start + 9].toInt() and 255) shl 16)
            val extra = CRC_EXTRA[messageId]
            if (extra == null) { start += packetSize; continue }
            val crcOffset = start + headerSize + length
            val received = (pending[crcOffset].toInt() and 255) or
                ((pending[crcOffset + 1].toInt() and 255) shl 8)
            if (checksum(pending, start + 1, headerSize - 1 + length, extra) != received) {
                start++
                continue
            }
            val minimum = MIN_LENGTH.getValue(messageId)
            if ((magic == MAVLINK_V1_START_BYTE && length < minimum) || length == 0) {
                start += packetSize
                continue
            }

            // MAVLink 2 poate taia zerourile de la final; le pun la loc
            // MAVLink 2 can trim trailing zeros; put them back here
            val body = ByteArray(maxOf(minimum, length))
            pending.copyInto(body, 0, start + headerSize, crcOffset)
            val idOffset = start + if (magic == MAVLINK_V1_START_BYTE) 3 else 5
            frames += MavlinkFrame(pending[idOffset].toInt() and 255,
                pending[idOffset + 1].toInt() and 255, messageId, body)
            start += packetSize
        }
        pending = pending.copyOfRange(start, pending.size)
        return frames
    }

    companion object {
        const val SENDER_SYSTEM_ID = 255
        const val SENDER_COMPONENT_ID = 190
        private const val MAVLINK_V1_START_BYTE = 0xFE
        private const val MAVLINK_V2_START_BYTE = 0xFD
        private const val SIGNATURE_LENGTH_BYTES = 13
        private val CRC_EXTRA = mapOf(
            MavlinkMessageIds.HEARTBEAT to 50,
            MavlinkMessageIds.SET_MODE to 89,
            MavlinkMessageIds.ATTITUDE to 39,
            MavlinkMessageIds.LOCAL_POSITION_NED to 185,
            MavlinkMessageIds.COMMAND_LONG to 152,
            MavlinkMessageIds.COMMAND_ACK to 143,
            MavlinkMessageIds.SET_ATTITUDE_TARGET to 49)
        private val MIN_LENGTH = mapOf(
            MavlinkMessageIds.HEARTBEAT to 9,
            MavlinkMessageIds.SET_MODE to 6,
            MavlinkMessageIds.ATTITUDE to 28,
            MavlinkMessageIds.LOCAL_POSITION_NED to 28,
            MavlinkMessageIds.COMMAND_LONG to 33,
            MavlinkMessageIds.COMMAND_ACK to 3,
            MavlinkMessageIds.SET_ATTITUDE_TARGET to 39)

        private fun checksum(data: ByteArray, offset: Int, length: Int, extra: Int): Int {
            var crc = 0xFFFF
            fun accumulate(value: Int) {
                var tmp = value xor (crc and 255)
                tmp = tmp xor ((tmp shl 4) and 255)
                crc = ((crc ushr 8) xor (tmp shl 8) xor (tmp shl 3) xor (tmp ushr 4)) and 65535
            }
            for (i in offset until offset + length) accumulate(data[i].toInt() and 255)
            accumulate(extra)
            return crc
        }
    }
}
