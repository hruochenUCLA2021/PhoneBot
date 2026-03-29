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

    // Single protocol version for the current architecture.
    const val VERSION: Byte = 3

    const val MSG_TYPE_SENSOR: Byte = 1
    const val MSG_TYPE_MOTOR: Byte = 2
    const val MSG_TYPE_TORQUE: Byte = 3
    const val MSG_TYPE_MOTOR_STATUS: Byte = 4
    const val MSG_TYPE_POLICY_ENABLE: Byte = 5
    const val MSG_TYPE_CMD_VEL: Byte = 6

    private const val MOTOR_COUNT: Int = 13

    // Header: magic[4] + version[u8] + msg_type[u8] + flags[u16] + seq[u32] + ts_ns[u64] = 20 bytes
    const val HEADER_SIZE_BYTES: Int = 20

    // SENSOR packet total size (see NOTE_protocal.md): 116 bytes
    // (No motor arrays in SENSOR packets in the current architecture.)
    const val SENSOR_PACKET_SIZE_BYTES: Int = 116

    // MOTOR packet total size (see NOTE_protocal.md): 176 bytes
    const val MOTOR_PACKET_SIZE_BYTES: Int = 176
    // TORQUE packet total size: header(20) + enable(u8) = 21 bytes
    const val TORQUE_PACKET_SIZE_BYTES: Int = HEADER_SIZE_BYTES + 1
    // POLICY_ENABLE packet total size: header(20) + enable(u8) = 21 bytes
    const val POLICY_ENABLE_PACKET_SIZE_BYTES: Int = HEADER_SIZE_BYTES + 1
    // MOTOR_STATUS packet total size: header(20) + 6*13 float32 = 332 bytes
    const val MOTOR_STATUS_PACKET_SIZE_BYTES: Int = HEADER_SIZE_BYTES + (6 * MOTOR_COUNT * 4)
    // CMD_VEL packet total size: header(20) + 3 float32 = 32 bytes
    const val CMD_VEL_PACKET_SIZE_BYTES: Int = HEADER_SIZE_BYTES + (3 * 4)

    data class MotorPacket(
        val seq: Long,
        val tsNs: Long,
        val pos: FloatArray, // size 13
        val vel: FloatArray, // size 13
        val tau: FloatArray, // size 13
    )

    data class TorquePacket(
        val seq: Long,
        val tsNs: Long,
        val enable: Boolean,
    )

    data class PolicyEnablePacket(
        val seq: Long,
        val tsNs: Long,
        val enable: Boolean,
    )

    data class CmdVelPacket(
        val seq: Long,
        val tsNs: Long,
        val vx: Float,
        val vy: Float,
        val wz: Float,
    )

    data class MotorStatusPacket(
        val seq: Long,
        val tsNs: Long,
        val pwmPercent: FloatArray, // 13
        val loadPercent: FloatArray, // 13
        val posRad: FloatArray, // 13
        val velRadS: FloatArray, // 13
        val vinV: FloatArray, // 13
        val tempC: FloatArray, // 13
    )

    fun packSensorPacket(
        seq: Long,
        imu: ImuState,
        battery: BatteryState,
    ): ByteArray {
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

    fun tryParseMotorStatusPacket(payload: ByteArray): MotorStatusPacket? {
        if (payload.size < MOTOR_STATUS_PACKET_SIZE_BYTES) return null
        val buf = ByteBuffer.wrap(payload).order(ByteOrder.LITTLE_ENDIAN)

        val magic = ByteArray(4)
        buf.get(magic)
        if (!magic.contentEquals(MAGIC)) return null

        val version = buf.get()
        if (version != VERSION) return null

        val msgType = buf.get()
        if (msgType != MSG_TYPE_MOTOR_STATUS) return null

        /* flags */ buf.short
        val seq = buf.int.toLong() and 0xFFFF_FFFFL
        val tsNs = buf.long

        fun read13(): FloatArray {
            val a = FloatArray(MOTOR_COUNT)
            for (i in 0 until MOTOR_COUNT) a[i] = buf.float
            return a
        }

        val pwm = read13()
        val load = read13()
        val pos = read13()
        val vel = read13()
        val vin = read13()
        val temp = read13()

        return MotorStatusPacket(
            seq = seq,
            tsNs = tsNs,
            pwmPercent = pwm,
            loadPercent = load,
            posRad = pos,
            velRadS = vel,
            vinV = vin,
            tempC = temp,
        )
    }

    fun packMotorPacket(
        seq: Long,
        tsNs: Long,
        pos: FloatArray,
        vel: FloatArray,
        tau: FloatArray,
    ): ByteArray {
        val buf = ByteBuffer.allocate(MOTOR_PACKET_SIZE_BYTES).order(ByteOrder.LITTLE_ENDIAN)
        buf.put(MAGIC)
        buf.put(VERSION)
        buf.put(MSG_TYPE_MOTOR)
        buf.putShort(0) // flags
        buf.putInt(seq.toInt())
        buf.putLong(tsNs)

        for (i in 0 until MOTOR_COUNT) buf.putFloat(pos.getOrNull(i) ?: 0f)
        for (i in 0 until MOTOR_COUNT) buf.putFloat(vel.getOrNull(i) ?: 0f)
        for (i in 0 until MOTOR_COUNT) buf.putFloat(tau.getOrNull(i) ?: 0f)
        return buf.array()
    }

    fun packTorquePacket(
        seq: Long,
        tsNs: Long,
        enable: Boolean,
    ): ByteArray {
        val buf = ByteBuffer.allocate(TORQUE_PACKET_SIZE_BYTES).order(ByteOrder.LITTLE_ENDIAN)
        buf.put(MAGIC)
        buf.put(VERSION)
        buf.put(MSG_TYPE_TORQUE)
        buf.putShort(0) // flags
        buf.putInt(seq.toInt())
        buf.putLong(tsNs)
        buf.put(if (enable) 1 else 0)
        return buf.array()
    }

    fun packPolicyEnablePacket(
        seq: Long,
        tsNs: Long,
        enable: Boolean,
    ): ByteArray {
        val buf = ByteBuffer.allocate(POLICY_ENABLE_PACKET_SIZE_BYTES).order(ByteOrder.LITTLE_ENDIAN)
        buf.put(MAGIC)
        buf.put(VERSION)
        buf.put(MSG_TYPE_POLICY_ENABLE)
        buf.putShort(0) // flags
        buf.putInt(seq.toInt())
        buf.putLong(tsNs)
        buf.put(if (enable) 1 else 0)
        return buf.array()
    }

    fun tryParsePolicyEnablePacket(payload: ByteArray): PolicyEnablePacket? {
        if (payload.size < POLICY_ENABLE_PACKET_SIZE_BYTES) return null
        val buf = ByteBuffer.wrap(payload).order(ByteOrder.LITTLE_ENDIAN)
        val magic = ByteArray(4)
        buf.get(magic)
        if (!magic.contentEquals(MAGIC)) return null
        val version = buf.get()
        if (version != VERSION) return null
        val msgType = buf.get()
        if (msgType != MSG_TYPE_POLICY_ENABLE) return null
        /* flags */ buf.short
        val seq = buf.int.toLong() and 0xFFFF_FFFFL
        val tsNs = buf.long
        val enable = (buf.get().toInt() != 0)
        return PolicyEnablePacket(seq = seq, tsNs = tsNs, enable = enable)
    }

    fun packCmdVelPacket(
        seq: Long,
        tsNs: Long,
        vx: Float,
        vy: Float,
        wz: Float,
    ): ByteArray {
        val buf = ByteBuffer.allocate(CMD_VEL_PACKET_SIZE_BYTES).order(ByteOrder.LITTLE_ENDIAN)
        buf.put(MAGIC)
        buf.put(VERSION)
        buf.put(MSG_TYPE_CMD_VEL)
        buf.putShort(0)
        buf.putInt(seq.toInt())
        buf.putLong(tsNs)
        buf.putFloat(vx)
        buf.putFloat(vy)
        buf.putFloat(wz)
        return buf.array()
    }

    fun tryParseCmdVelPacket(payload: ByteArray): CmdVelPacket? {
        if (payload.size < CMD_VEL_PACKET_SIZE_BYTES) return null
        val buf = ByteBuffer.wrap(payload).order(ByteOrder.LITTLE_ENDIAN)
        val magic = ByteArray(4)
        buf.get(magic)
        if (!magic.contentEquals(MAGIC)) return null
        val version = buf.get()
        if (version != VERSION) return null
        val msgType = buf.get()
        if (msgType != MSG_TYPE_CMD_VEL) return null
        /* flags */ buf.short
        val seq = buf.int.toLong() and 0xFFFF_FFFFL
        val tsNs = buf.long
        val vx = buf.float
        val vy = buf.float
        val wz = buf.float
        return CmdVelPacket(seq = seq, tsNs = tsNs, vx = vx, vy = vy, wz = wz)
    }

    fun tryParseTorquePacket(payload: ByteArray): TorquePacket? {
        if (payload.size < TORQUE_PACKET_SIZE_BYTES) return null
        val buf = ByteBuffer.wrap(payload).order(ByteOrder.LITTLE_ENDIAN)

        val magic = ByteArray(4)
        buf.get(magic)
        if (!magic.contentEquals(MAGIC)) return null

        val version = buf.get()
        if (version != VERSION) return null

        val msgType = buf.get()
        if (msgType != MSG_TYPE_TORQUE) return null

        /* flags */ buf.short
        val seq = buf.int.toLong() and 0xFFFF_FFFFL
        val tsNs = buf.long
        val enable = (buf.get().toInt() and 0xFF) != 0
        return TorquePacket(seq = seq, tsNs = tsNs, enable = enable)
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


