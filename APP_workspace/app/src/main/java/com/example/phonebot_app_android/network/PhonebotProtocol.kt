package com.example.phonebot_app_android.network

import com.example.phonebot_app_android.sensors.ImuState
import com.example.phonebot_app_android.system.BatteryState
import java.nio.ByteBuffer
import java.nio.ByteOrder

/**
 * PhoneBot binary UDP protocol (little-endian).
 *
 * Goal: avoid JSON/text overhead and send fixed-layout float32 packets.
 */
object PhonebotProtocol {
    // 4-byte magic: ASCII "PBOT"
    private val MAGIC = byteArrayOf('P'.code.toByte(), 'B'.code.toByte(), 'O'.code.toByte(), 'T'.code.toByte())

    const val VERSION: Byte = 1

    const val MSG_TYPE_SENSOR: Byte = 1
    const val MSG_TYPE_MOTOR: Byte = 2

    // Header: magic[4] + version[u8] + msg_type[u8] + flags[u16] + seq[u32] + ts_ns[u64] = 20 bytes
    const val HEADER_SIZE_BYTES: Int = 20

    // SENSOR packet total size (see NOTE_protocal.md): 116 bytes
    const val SENSOR_PACKET_SIZE_BYTES: Int = 116

    // MOTOR packet total size (see NOTE_protocal.md): 176 bytes
    const val MOTOR_PACKET_SIZE_BYTES: Int = 176

    private const val MOTOR_COUNT: Int = 13

    data class MotorPacket(
        val seq: Long,
        val tsNs: Long,
        val pos: FloatArray, // size 13
        val vel: FloatArray, // size 13
        val tau: FloatArray, // size 13
    )

    fun packSensorPacket(seq: Long, imu: ImuState, battery: BatteryState): ByteArray {
        val buf = ByteBuffer.allocate(SENSOR_PACKET_SIZE_BYTES).order(ByteOrder.LITTLE_ENDIAN)
        // Header
        buf.put(MAGIC)
        buf.put(VERSION)
        buf.put(MSG_TYPE_SENSOR)
        buf.putShort(0) // flags u16 (reserved)
        buf.putInt(seq.toInt()) // seq u32 (wrap ok)
        buf.putLong(imu.timestampNs)

        fun put3(a: FloatArray?, d0: Float, d1: Float, d2: Float) {
            if (a == null || a.size < 3) {
                buf.putFloat(d0); buf.putFloat(d1); buf.putFloat(d2)
            } else {
                buf.putFloat(a[0]); buf.putFloat(a[1]); buf.putFloat(a[2])
            }
        }

        fun put4(a: FloatArray?, d0: Float, d1: Float, d2: Float, d3: Float) {
            if (a == null || a.size < 4) {
                buf.putFloat(d0); buf.putFloat(d1); buf.putFloat(d2); buf.putFloat(d3)
            } else {
                buf.putFloat(a[0]); buf.putFloat(a[1]); buf.putFloat(a[2]); buf.putFloat(a[3])
            }
        }

        // Accel / gyro
        put3(imu.accel, 0f, 0f, 0f)
        put3(imu.gyro, 0f, 0f, 0f)

        // rotvec block
        buf.putFloat(imu.rotVecHz ?: 0f)
        put4(imu.quat, 1f, 0f, 0f, 0f) // [w,x,y,z]
        put3(imu.yprDeg, 0f, 0f, 0f)

        // game_rotvec block
        buf.putFloat(imu.gameRotVecHz ?: 0f)
        put4(imu.gameQuat, 1f, 0f, 0f, 0f) // [w,x,y,z]
        put3(imu.gameYprDeg, 0f, 0f, 0f)

        // battery (mostly for debugging / monitoring)
        buf.putFloat((battery.percent ?: -1).toFloat())
        buf.put(if (battery.isCharging == true) 1 else 0)
        buf.put(mapPluggedToCode(battery.plugged))
        buf.put(mapStatusToCode(battery.status))
        buf.put(0) // reserved/padding

        return buf.array()
    }

    fun tryParseMotorPacket(payload: ByteArray): MotorPacket? {
        if (payload.size < MOTOR_PACKET_SIZE_BYTES) return null
        val buf = ByteBuffer.wrap(payload).order(ByteOrder.LITTLE_ENDIAN)

        val magic = ByteArray(4)
        buf.get(magic)
        if (!magic.contentEquals(MAGIC)) return null

        val version = buf.get()
        if (version != VERSION) return null

        val msgType = buf.get()
        if (msgType != MSG_TYPE_MOTOR) return null

        /* flags */ buf.short
        val seq = buf.int.toLong() and 0xFFFF_FFFFL
        val tsNs = buf.long

        val pos = FloatArray(MOTOR_COUNT)
        val vel = FloatArray(MOTOR_COUNT)
        val tau = FloatArray(MOTOR_COUNT)

        for (i in 0 until MOTOR_COUNT) pos[i] = buf.float
        for (i in 0 until MOTOR_COUNT) vel[i] = buf.float
        for (i in 0 until MOTOR_COUNT) tau[i] = buf.float

        return MotorPacket(seq = seq, tsNs = tsNs, pos = pos, vel = vel, tau = tau)
    }

    private fun mapPluggedToCode(plugged: String?): Byte {
        return when (plugged) {
            "AC" -> 1
            "USB" -> 2
            "WIRELESS" -> 3
            "UNPLUGGED" -> 0
            else -> 0
        }
    }

    private fun mapStatusToCode(status: String?): Byte {
        return when (status) {
            "CHARGING" -> 1
            "DISCHARGING" -> 2
            "FULL" -> 3
            "NOT_CHARGING" -> 4
            else -> 0
        }
    }
}


