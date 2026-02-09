package com.example.phonebot_app_android.motors.dynamixel

import android.app.PendingIntent
import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.hardware.usb.UsbDevice
import android.hardware.usb.UsbManager
import android.os.Build
import android.util.Log
import androidx.core.content.ContextCompat
import com.hoho.android.usbserial.driver.UsbSerialPort
import com.hoho.android.usbserial.driver.UsbSerialProber
import kotlinx.coroutines.CancellationException
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.withContext
import kotlinx.coroutines.sync.Mutex
import kotlinx.coroutines.sync.withLock
import java.util.concurrent.atomic.AtomicBoolean
import kotlin.math.PI
import kotlin.math.roundToInt

/**
 * Dynamixel controller over USB-Serial (Protocol 2.0), tuned for XL430-W250 position control.
 *
 * Notes / assumptions:
 * - Motor IDs default to 1..13 (change in code if needed).
 * - Incoming UDP motor positions are interpreted as radians in [-pi, +pi].
 * - Radians are mapped to raw position ticks [0..4095] with 0 rad ~= 2048.
 */
class DynamixelController(private val context: Context) {
    data class Config(
        val motorIds: IntArray = IntArray(13) { i -> i + 1 },
        val baudRate: Int = 1_000_000,
        val kp: Int = 5, // Position P Gain register units
        // User-friendly value. XL430 Position D Gain is an integer register; we map this float to a register value.
        val kd: Float = 0.1f,
        val initialGoalTicks: Int = 2048,
    )

    @Volatile var lastStatus: String = "Motor controller idle."
        private set

    private var port: UsbSerialPort? = null
    private var usbPermReceiver: BroadcastReceiver? = null

    private val running = AtomicBoolean(false)
    private val ioMutex = Mutex()

    data class MotorStatus(
        val rxHz: Float? = null,
        val posRad: FloatArray? = null, // size N
        val velRadS: FloatArray? = null, // size N
    )

    private class RateEstimator(private val alpha: Float = 0.1f) {
        private var lastNs: Long = 0L
        private var emaHz: Float? = null

        fun update(nowNs: Long): Float? {
            if (lastNs != 0L) {
                val dt = nowNs - lastNs
                if (dt > 0) {
                    val instHz = 1_000_000_000f / dt.toFloat()
                    emaHz = if (emaHz == null) instHz else (emaHz!! * (1 - alpha) + instHz * alpha)
                }
            }
            lastNs = nowNs
            return emaHz
        }
    }

    private val statusRate = RateEstimator()
    private val rxBuf = ByteArray(4096)
    private var rxLen = 0
    private val tmpRead = ByteArray(512)

    suspend fun enable(config: Config = Config()): Boolean = withContext(Dispatchers.IO) {
        if (running.get()) return@withContext true
        running.set(true)
        try {
            lastStatus = "Searching USB serial device..."
            val usbManager = context.getSystemService(Context.USB_SERVICE) as UsbManager
            val driver = UsbSerialProber.getDefaultProber().findAllDrivers(usbManager).firstOrNull()
            if (driver == null) {
                lastStatus = "No USB serial driver found. Plug in adapter (FTDI/CH340/CP210x)."
                return@withContext false
            }

            val device = driver.device
            if (!usbManager.hasPermission(device)) {
                lastStatus = "Requesting USB permission..."
                val ok = requestUsbPermission(usbManager, device)
                if (!ok) {
                    lastStatus = "USB permission denied."
                    return@withContext false
                }
            }

            lastStatus = "Opening USB serial port..."
            val connection = usbManager.openDevice(device)
            if (connection == null) {
                lastStatus = "openDevice() failed (permission? cable?)."
                return@withContext false
            }

            val p = driver.ports.firstOrNull()
            if (p == null) {
                lastStatus = "USB serial has no ports?"
                return@withContext false
            }
            p.open(connection)
            p.setParameters(config.baudRate, 8, UsbSerialPort.STOPBITS_1, UsbSerialPort.PARITY_NONE)
            // Low latency for control
            p.dtr = true
            p.rts = true
            port = p

            lastStatus = "Configuring ${config.motorIds.size} motors..."

            // 0) Reduce RX traffic: status return level = 1 (return for READ only).
            // We need this for SYNC_READ feedback.
            for (id in config.motorIds) {
                write(DynamixelProtocol2.buildWrite1(id, Xl430Registers.ADDR_STATUS_RETURN_LEVEL, 1))
            }

            // 1) Torque OFF before changing operating mode and gains.
            for (id in config.motorIds) {
                write(DynamixelProtocol2.buildWrite1(id, Xl430Registers.ADDR_TORQUE_ENABLE, 0))
            }

            // 2) Position mode
            for (id in config.motorIds) {
                write(
                    DynamixelProtocol2.buildWrite1(
                        id,
                        Xl430Registers.ADDR_OPERATING_MODE,
                        Xl430Registers.MODE_POSITION_CONTROL,
                    )
                )
            }

            // 3) PID gains (register units)
            val kdReg = kdToReg(config.kd)
            for (id in config.motorIds) {
                write(DynamixelProtocol2.buildWrite2(id, Xl430Registers.ADDR_POSITION_P_GAIN, config.kp))
                write(DynamixelProtocol2.buildWrite2(id, Xl430Registers.ADDR_POSITION_D_GAIN, kdReg))
            }

            // 4) Goal position = 0 rad (2048 ticks)
            sendGoalTicksSync(config.motorIds, IntArray(config.motorIds.size) { config.initialGoalTicks })

            // 5) Torque ON
            for (id in config.motorIds) {
                write(DynamixelProtocol2.buildWrite1(id, Xl430Registers.ADDR_TORQUE_ENABLE, 1))
            }

            lastStatus = "Motors ON (position mode)."
            true
        } catch (ce: CancellationException) {
            running.set(false)
            throw ce
        } catch (t: Throwable) {
            lastStatus = "Enable failed: ${t.message ?: t::class.java.simpleName}"
            Log.e("DynamixelController", "enable failed", t)
            safeClose()
            running.set(false)
            false
        }
    }

    suspend fun disable(config: Config = Config()): Unit = withContext(Dispatchers.IO) {
        try {
            lastStatus = "Motors OFF (torque disable)..."
            val ids = config.motorIds
            for (id in ids) {
                runCatching { write(DynamixelProtocol2.buildWrite1(id, Xl430Registers.ADDR_TORQUE_ENABLE, 0)) }
            }
        } finally {
            safeClose()
            running.set(false)
            lastStatus = "Motor controller idle."
        }
    }

    suspend fun sendGoalPositionsRad(config: Config, posRad: FloatArray) = withContext(Dispatchers.IO) {
        if (!running.get()) return@withContext
        ioMutex.withLock {
            val ids = config.motorIds
            val n = minOf(ids.size, posRad.size)
            val ticks = IntArray(n)
            for (i in 0 until n) {
                ticks[i] = radToTicks(posRad[i])
            }
            sendGoalTicksSync(ids.copyOfRange(0, n), ticks)
        }
    }

    suspend fun readPresentPosVel(config: Config): MotorStatus = withContext(Dispatchers.IO) {
        if (!running.get()) return@withContext MotorStatus()
        ioMutex.withLock {
            val ids = config.motorIds
            // Read 8 bytes starting at Present Velocity (4) + Present Position (4).
            val pkt = DynamixelProtocol2.buildSyncRead(Xl430Registers.ADDR_PRESENT_VELOCITY, 8, ids)
            write(pkt)

            val vel = FloatArray(ids.size)
            val pos = FloatArray(ids.size)

            var got = 0
            val deadline = System.currentTimeMillis() + 50L // ~50ms budget
            while (got < ids.size && System.currentTimeMillis() < deadline) {
                val st = readOneStatusPacket(timeoutMs = 10) ?: continue
                val id = st.id
                var idx = -1
                for (i in ids.indices) {
                    if (ids[i] == id) {
                        idx = i
                        break
                    }
                }
                if (idx < 0) continue
                if (st.params.size < 1 + 8) continue
                // params[0]=error, then 8 bytes: vel(int32), pos(int32)
                val vRaw = leInt32(st.params, 1)
                val pRaw = leInt32(st.params, 5)
                vel[idx] = vRaw.toFloat() * 0.0239691227f
                pos[idx] = ticksToRad(pRaw)
                got += 1
            }

            MotorStatus(
                rxHz = statusRate.update(System.nanoTime()),
                posRad = pos,
                velRadS = vel,
            )
        }
    }

    private fun radToTicks(rad: Float): Int {
        // XL430 model file states:
        //   min_rad=-pi -> 0
        //   0 rad -> 2048
        //   +pi -> 4095
        val clamped = rad.coerceIn((-PI).toFloat(), PI.toFloat())
        val ticks = 2048f + (clamped * (2048f / PI.toFloat()))
        return ticks.roundToInt().coerceIn(0, 4095)
    }

    private fun sendGoalTicksSync(ids: IntArray, ticks: IntArray) {
        val items =
            ids.mapIndexed { idx, id ->
                val v = ticks[idx]
                val data =
                    byteArrayOf(
                        (v and 0xFF).toByte(),
                        ((v shr 8) and 0xFF).toByte(),
                        ((v shr 16) and 0xFF).toByte(),
                        ((v shr 24) and 0xFF).toByte(),
                    )
                id to data
            }
        write(DynamixelProtocol2.buildSyncWrite(Xl430Registers.ADDR_GOAL_POSITION, 4, items))
    }

    private fun write(packet: ByteArray) {
        val p = port ?: error("USB serial not open")
        // Note: we don't read status packets; we try to reduce them via STATUS_RETURN_LEVEL=0.
        p.write(packet, 50)
    }

    private data class StatusPacket(val id: Int, val params: ByteArray)

    private fun readOneStatusPacket(timeoutMs: Int): StatusPacket? {
        val p = port ?: return null
        val n = try {
            p.read(tmpRead, timeoutMs)
        } catch (_: Throwable) {
            return null
        }
        if (n > 0) {
            if (rxLen + n > rxBuf.size) {
                // drop if overflow
                rxLen = 0
            } else {
                System.arraycopy(tmpRead, 0, rxBuf, rxLen, n)
                rxLen += n
            }
        }

        // Search for header
        var start = -1
        for (i in 0 until (rxLen - 3)) {
            if ((rxBuf[i].toInt() and 0xFF) == 0xFF &&
                (rxBuf[i + 1].toInt() and 0xFF) == 0xFF &&
                (rxBuf[i + 2].toInt() and 0xFF) == 0xFD &&
                (rxBuf[i + 3].toInt() and 0xFF) == 0x00
            ) {
                start = i
                break
            }
        }
        if (start < 0) {
            // keep last few bytes in case header straddles reads
            if (rxLen > 16) rxLen = 0
            return null
        }
        if (start > 0) {
            // drop junk before header
            System.arraycopy(rxBuf, start, rxBuf, 0, rxLen - start)
            rxLen -= start
        }
        if (rxLen < 7) return null // header(4)+id(1)+len(2)

        val id = rxBuf[4].toInt() and 0xFF
        val len = (rxBuf[5].toInt() and 0xFF) or ((rxBuf[6].toInt() and 0xFF) shl 8)
        val packetSize = 4 + 1 + 2 + len
        if (rxLen < packetSize) return null

        val raw = rxBuf.copyOfRange(0, packetSize)
        // Remove from buffer
        val remaining = rxLen - packetSize
        if (remaining > 0) {
            System.arraycopy(rxBuf, packetSize, rxBuf, 0, remaining)
        }
        rxLen = remaining

        // CRC check on raw bytes (stuffed)
        val crcExpected = (raw[raw.size - 2].toInt() and 0xFF) or ((raw[raw.size - 1].toInt() and 0xFF) shl 8)
        val crcCalc = DynamixelProtocol2.crc16(raw, 0, raw.size - 2)
        if (crcExpected != crcCalc) return null

        val unstuffed = DynamixelProtocol2.unstuff(raw)
        if (unstuffed.size < 10) return null
        val inst = unstuffed[7].toInt() and 0xFF
        if (inst != 0x55) return null // status packet

        // params in status packet: error(1) + data...
        val params = unstuffed.copyOfRange(8, unstuffed.size - 2)
        return StatusPacket(id = id, params = params)
    }

    private fun leInt32(b: ByteArray, off: Int): Int {
        return (b[off].toInt() and 0xFF) or
            ((b[off + 1].toInt() and 0xFF) shl 8) or
            ((b[off + 2].toInt() and 0xFF) shl 16) or
            ((b[off + 3].toInt() and 0xFF) shl 24)
    }

    private fun ticksToRad(ticks: Int): Float {
        val t = ticks.coerceIn(0, 4095)
        return (t - 2048).toFloat() * (PI.toFloat() / 2048f)
    }

    private fun kdToReg(kd: Float): Int {
        // XL430 D gain is a 2-byte integer; there is no fractional register value.
        // Policy: round-to-int with a minimum of 1 when kd > 0.
        // This makes default kd=0.1 map to reg=1.
        if (!kd.isFinite() || kd <= 0f) return 0
        val reg = kd.roundToInt()
        return if (reg <= 0) 1 else reg
    }

    private fun safeClose() {
        runCatching { port?.close() }
        port = null
        unregisterUsbPermissionReceiver()
    }

    private fun requestUsbPermission(usbManager: UsbManager, device: UsbDevice): Boolean {
        val action = "${context.packageName}.USB_PERMISSION"
        val granted = AtomicBoolean(false)
        val done = AtomicBoolean(false)

        val receiver =
            object : BroadcastReceiver() {
                override fun onReceive(ctx: Context?, intent: Intent?) {
                    if (intent?.action != action) return
                    val dev = intent.getParcelableExtra(UsbManager.EXTRA_DEVICE) as? UsbDevice
                    if (dev?.deviceId != device.deviceId) return
                    granted.set(intent.getBooleanExtra(UsbManager.EXTRA_PERMISSION_GRANTED, false))
                    done.set(true)
                    unregisterUsbPermissionReceiver()
                }
            }

        // Register receiver (targetSdk>=33 requires export flags; use ContextCompat helper)
        unregisterUsbPermissionReceiver()
        usbPermReceiver = receiver
        ContextCompat.registerReceiver(
            context,
            receiver,
            IntentFilter(action),
            ContextCompat.RECEIVER_NOT_EXPORTED,
        )

        val flags =
            if (Build.VERSION.SDK_INT >= 31) PendingIntent.FLAG_MUTABLE or PendingIntent.FLAG_UPDATE_CURRENT
            else PendingIntent.FLAG_UPDATE_CURRENT
        val pi = PendingIntent.getBroadcast(context, 0, Intent(action), flags)
        usbManager.requestPermission(device, pi)

        // Busy-wait up to ~2s. (Simple for now; can be upgraded to suspend/await.)
        val start = System.currentTimeMillis()
        while (!done.get() && System.currentTimeMillis() - start < 2000) {
            try {
                Thread.sleep(10)
            } catch (_: InterruptedException) {
                break
            }
        }
        return granted.get()
    }

    private fun unregisterUsbPermissionReceiver() {
        val r = usbPermReceiver ?: return
        runCatching { context.unregisterReceiver(r) }
        usbPermReceiver = null
    }
}


