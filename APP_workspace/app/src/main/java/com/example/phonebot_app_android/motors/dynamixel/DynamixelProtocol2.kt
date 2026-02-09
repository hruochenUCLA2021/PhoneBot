package com.example.phonebot_app_android.motors.dynamixel

import java.nio.ByteBuffer
import java.nio.ByteOrder

/**
 * Minimal Dynamixel Protocol 2.0 packet builder (enough for XL430 position control).
 *
 * Packet format:
 *  header: 0xFF 0xFF 0xFD 0x00
 *  id: u8
 *  length: u16  (instruction + params + CRC)
 *  instruction: u8
 *  params: ...
 *  crc: u16
 */
object DynamixelProtocol2 {
    private val HEADER = byteArrayOf(0xFF.toByte(), 0xFF.toByte(), 0xFD.toByte(), 0x00.toByte())

    // Instructions
    private const val INST_WRITE: Byte = 0x03
    private const val INST_READ: Byte = 0x02
    private const val INST_SYNC_WRITE: Byte = 0x83.toByte()
    private const val INST_SYNC_READ: Byte = 0x82.toByte()

    // CRC-16/IBM (poly 0x8005, init 0x0000) as used by Dynamixel Protocol 2.0.
    fun crc16(data: ByteArray, offset: Int = 0, length: Int = data.size - offset): Int {
        var crc = 0
        for (i in 0 until length) {
            var c = (data[offset + i].toInt() and 0xFF) shl 8
            for (_b in 0 until 8) {
                val mix = (crc xor c) and 0x8000
                crc = (crc shl 1) and 0xFFFF
                if (mix != 0) crc = crc xor 0x8005
                c = (c shl 1) and 0xFFFF
            }
        }
        return crc and 0xFFFF
    }

    fun buildWrite1(id: Int, addr: Int, value: Int): ByteArray =
        buildWrite(id, addr, byteArrayOf((value and 0xFF).toByte()))

    fun buildWrite2(id: Int, addr: Int, value: Int): ByteArray {
        val v = value and 0xFFFF
        return buildWrite(
            id,
            addr,
            byteArrayOf((v and 0xFF).toByte(), ((v shr 8) and 0xFF).toByte()),
        )
    }

    fun buildWrite4(id: Int, addr: Int, value: Long): ByteArray {
        val v = value.toInt()
        return buildWrite(
            id,
            addr,
            byteArrayOf(
                (v and 0xFF).toByte(),
                ((v shr 8) and 0xFF).toByte(),
                ((v shr 16) and 0xFF).toByte(),
                ((v shr 24) and 0xFF).toByte(),
            ),
        )
    }

    fun buildWrite(id: Int, addr: Int, data: ByteArray): ByteArray {
        val paramsLen = 2 + data.size // addr_l, addr_h, data...
        val lengthField = 1 + paramsLen + 2 // instruction + params + CRC

        val buf = ByteBuffer.allocate(HEADER.size + 1 + 2 + 1 + paramsLen + 2).order(ByteOrder.LITTLE_ENDIAN)
        buf.put(HEADER)
        buf.put((id and 0xFF).toByte())
        buf.putShort(lengthField.toShort())
        buf.put(INST_WRITE)
        buf.put((addr and 0xFF).toByte())
        buf.put(((addr shr 8) and 0xFF).toByte())
        buf.put(data)

        val raw = buf.array()
        val crc = crc16(raw, 0, raw.size - 2)
        raw[raw.size - 2] = (crc and 0xFF).toByte()
        raw[raw.size - 1] = ((crc shr 8) and 0xFF).toByte()
        return raw
    }

    /**
     * SYNC_WRITE packet:
     * params = addr(u16) + data_len(u16) + [id + data[data_len]] * N
     */
    fun buildSyncWrite(addr: Int, dataLen: Int, idToData: List<Pair<Int, ByteArray>>): ByteArray {
        require(dataLen in 1..255)
        for ((id, data) in idToData) {
            require(id in 0..252)
            require(data.size == dataLen)
        }

        val paramsLen = 2 + 2 + idToData.size * (1 + dataLen)
        val lengthField = 1 + paramsLen + 2

        val buf = ByteBuffer.allocate(HEADER.size + 1 + 2 + 1 + paramsLen + 2).order(ByteOrder.LITTLE_ENDIAN)
        buf.put(HEADER)
        buf.put(0xFE.toByte()) // broadcast ID
        buf.putShort(lengthField.toShort())
        buf.put(INST_SYNC_WRITE)
        buf.put((addr and 0xFF).toByte())
        buf.put(((addr shr 8) and 0xFF).toByte())
        buf.put((dataLen and 0xFF).toByte())
        buf.put(0x00) // dataLen high byte (<=255)
        for ((id, data) in idToData) {
            buf.put((id and 0xFF).toByte())
            buf.put(data)
        }

        val raw = buf.array()
        val crc = crc16(raw, 0, raw.size - 2)
        raw[raw.size - 2] = (crc and 0xFF).toByte()
        raw[raw.size - 1] = ((crc shr 8) and 0xFF).toByte()
        return raw
    }

    /**
     * SYNC_READ packet:
     * params = start_addr(u16) + data_len(u16) + [id] * N
     */
    fun buildSyncRead(startAddr: Int, dataLen: Int, ids: IntArray): ByteArray {
        require(dataLen in 1..65535)
        require(ids.isNotEmpty())
        val paramsLen = 2 + 2 + ids.size
        val lengthField = 1 + paramsLen + 2

        val buf = ByteBuffer.allocate(HEADER.size + 1 + 2 + 1 + paramsLen + 2).order(ByteOrder.LITTLE_ENDIAN)
        buf.put(HEADER)
        buf.put(0xFE.toByte()) // broadcast ID
        buf.putShort(lengthField.toShort())
        buf.put(INST_SYNC_READ)
        buf.put((startAddr and 0xFF).toByte())
        buf.put(((startAddr shr 8) and 0xFF).toByte())
        buf.put((dataLen and 0xFF).toByte())
        buf.put(((dataLen shr 8) and 0xFF).toByte())
        for (id in ids) buf.put((id and 0xFF).toByte())

        val raw = buf.array()
        val crc = crc16(raw, 0, raw.size - 2)
        raw[raw.size - 2] = (crc and 0xFF).toByte()
        raw[raw.size - 1] = ((crc shr 8) and 0xFF).toByte()
        return raw
    }

    /**
     * Remove protocol 2.0 "byte stuffing" (0xFD inserted after 0xFF 0xFF 0xFD sequence).
     * We scan from index 4 (after header) to avoid treating the header itself as stuffable data.
     */
    fun unstuff(packet: ByteArray): ByteArray {
        if (packet.size < 8) return packet
        val out = ByteArray(packet.size)
        var wi = 0
        // Copy header as-is
        for (i in 0 until 4) out[wi++] = packet[i]
        var i = 4
        while (i < packet.size) {
            // If we see FF FF FD FD, drop the extra FD (the 4th byte)
            if (i + 3 < packet.size &&
                (packet[i].toInt() and 0xFF) == 0xFF &&
                (packet[i + 1].toInt() and 0xFF) == 0xFF &&
                (packet[i + 2].toInt() and 0xFF) == 0xFD &&
                (packet[i + 3].toInt() and 0xFF) == 0xFD
            ) {
                out[wi++] = packet[i]
                out[wi++] = packet[i + 1]
                out[wi++] = packet[i + 2]
                i += 4
                continue
            }
            out[wi++] = packet[i]
            i += 1
        }
        return out.copyOf(wi)
    }
}


