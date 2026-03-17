package com.example.motor_control_app.motors.dynamixel

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
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.update
import kotlinx.coroutines.sync.Mutex
import kotlinx.coroutines.sync.withLock
import kotlinx.coroutines.withContext
import java.util.concurrent.atomic.AtomicBoolean
import kotlin.math.PI
import kotlin.math.roundToInt

/**
 * Torque-only bring-up controller for XL430 via U2D2 (Protocol 2.0).
 *
 * This intentionally does NOT write operating mode / PID / goal position.
 * It only tries to turn torque on and read back status/diagnostics.
 */
class DynamixelController(private val context: Context) {
    data class Config(
        val motorIds: IntArray = IntArray(13) { i -> i + 1 },
        val baudRate: Int = 1_000_000,
    )

    data class MotorDiag(
        val id: Int,
        val connected: Boolean,
        val torqueEnabled: Boolean? = null,
        val operatingMode: Int? = null,
        val hwError: Int? = null,
        val inputVoltage01V: Int? = null,
        val temperatureC: Int? = null,
        val presentPosTicks: Int? = null,
        val presentPosRad: Float? = null,
        val presentVelRaw: Int? = null,
        val presentVelRadS: Float? = null,
        val goalPosTicks: Int? = null,
        val goalPosRad: Float? = null,
        val lastSeenMs: Long? = null,
    )

    private val _logLines = MutableStateFlow<List<String>>(listOf("Motor controller idle."))
    val logLines: StateFlow<List<String>> = _logLines.asStateFlow()

    private val _txrxLogLines = MutableStateFlow<List<String>>(listOf("TxRx log idle."))
    val txrxLogLines: StateFlow<List<String>> = _txrxLogLines.asStateFlow()

    private val _motorDiag = MutableStateFlow<List<MotorDiag>>(emptyList())
    val motorDiag: StateFlow<List<MotorDiag>> = _motorDiag.asStateFlow()

    @Volatile var lastStatus: String = "Motor controller idle."
        private set

    @Volatile var lastTxRxStatus: String = "TxRx log idle."
        private set

    private fun pushStatus(line: String) {
        lastStatus = line
        val t = System.currentTimeMillis()
        _logLines.update { prev ->
            val next = prev + "${t}ms: $line"
            if (next.size > 200) next.takeLast(200) else next
        }
    }

    private fun pushTxRxStatus(line: String) {
        lastTxRxStatus = line
        val t = System.currentTimeMillis()
        _txrxLogLines.update { prev ->
            val next = prev + "${t}ms: $line"
            if (next.size > 200) next.takeLast(200) else next
        }
    }

    private fun devInfo(d: UsbDevice): String {
        val vid = "0x" + d.vendorId.toString(16)
        val pid = "0x" + d.productId.toString(16)
        return "UsbDevice(name=${d.deviceName}, vid=$vid, pid=$pid)"
    }

    private var port: UsbSerialPort? = null
    private var usbPermReceiver: BroadcastReceiver? = null

    private val running = AtomicBoolean(false)
    private val ioMutex = Mutex()
    private val rxBuf = ByteArray(4096)
    private var rxLen = 0
    private val tmpRead = ByteArray(512)
    private val lastSeenById = HashMap<Int, Long>()

    // DynamixelSDK-style comm result codes (subset; enough for logging).
    private companion object {
        private const val COMM_SUCCESS = 0
        private const val COMM_TX_FAIL = -1001
        private const val COMM_RX_TIMEOUT = -3001
        private const val COMM_RX_CORRUPT = -3002
    }

    private fun commResultToString(code: Int): String =
        when (code) {
            COMM_SUCCESS -> "COMM_SUCCESS"
            COMM_TX_FAIL -> "COMM_TX_FAIL"
            COMM_RX_TIMEOUT -> "COMM_RX_TIMEOUT"
            COMM_RX_CORRUPT -> "COMM_RX_CORRUPT"
            else -> "COMM_$code"
        }

    private fun dxlErrorToString(err: Int): String = "0x" + (err and 0xFF).toString(16).padStart(2, '0')

    private fun flushRxLocked(maxMs: Int = 30) {
        rxLen = 0
        val p = port ?: return
        val start = System.currentTimeMillis()
        while (System.currentTimeMillis() - start < maxMs) {
            val n = try {
                p.read(tmpRead, 1)
            } catch (_: Throwable) {
                return
            }
            if (n <= 0) return
        }
    }

    private fun openPortLocked(config: Config): Boolean {
        if (port != null) return true

        pushStatus("Searching USB serial device...")
        val usbManager = context.getSystemService(Context.USB_SERVICE) as UsbManager
        val drivers = UsbSerialProber.getDefaultProber().findAllDrivers(usbManager)
        pushStatus("USB drivers found: ${drivers.size}")
        if (drivers.isNotEmpty()) {
            pushStatus("First device: " + devInfo(drivers.first().device))
        }
        val driver = drivers.firstOrNull()
        if (driver == null) {
            pushStatus("No USB serial driver found.")
            return false
        }

        val device = driver.device
        if (!usbManager.hasPermission(device)) {
            pushStatus("Requesting USB permission for ${devInfo(device)} ...")
            val granted = requestUsbPermission(usbManager, device)
            if (!granted) {
                pushStatus("USB permission denied.")
                return false
            }
        }

        pushStatus("Opening USB serial port...")
        val connection = usbManager.openDevice(device)
        if (connection == null) {
            pushStatus("openDevice() failed.")
            return false
        }

        val p = driver.ports.firstOrNull()
        if (p == null) {
            pushStatus("USB serial has no ports?")
            return false
        }
        p.open(connection)
        p.setParameters(config.baudRate, 8, UsbSerialPort.STOPBITS_1, UsbSerialPort.PARITY_NONE)
        p.dtr = true
        p.rts = true
        port = p
        flushRxLocked(maxMs = 30)
        return true
    }

    suspend fun enable(config: Config = Config()): Boolean = withContext(Dispatchers.IO) {
        if (running.get()) return@withContext true
        running.set(true)
        var ok = false
        try {
            ioMutex.withLock {
                if (!openPortLocked(config)) return@withContext false

                pushStatus("Torque-only enable (no mode/PID/goal writes).")
                if (!setTorqueWithRetry(config, enable = true, tries = 3)) {
                    pushStatus("Torque ON failed after retries.")
                    return@withContext false
                }

                runCatching { updateMotorDiagLocked(config) }
                pushStatus("Motors torque ON (verified).")
            }
            ok = true
            true
        } catch (ce: CancellationException) {
            pushStatus("Enable cancelled.")
            running.set(false)
            throw ce
        } catch (t: Throwable) {
            pushStatus("Enable failed: ${t::class.java.simpleName}: ${t.message ?: "(no message)"}")
            Log.e("DynamixelController", "enable failed", t)
            false
        } finally {
            if (!ok) {
                safeClose()
                running.set(false)
            }
        }
    }

    /**
     * Torque enable/disable using "write1ByteTxRx"-style behavior:
     * send WRITE(ADDR_TORQUE_ENABLE), then wait for the Status Packet for that write.
     *
     * This is meant for debugging differences vs the "write + syncRead verify" path.
     */
    suspend fun setTorqueTxRxLikeSdk(
        config: Config,
        enable: Boolean,
        perIdTimeoutMs: Int = 200,
    ): Boolean = withContext(Dispatchers.IO) {
        ioMutex.withLock {
            if (!openPortLocked(config)) return@withContext false
            running.set(true)
            val ids = config.motorIds
            val target = if (enable) 1 else 0

            pushTxRxStatus("TxRx torque ${if (enable) "ON" else "OFF"} start (ids=${ids.joinToString(",")})")
            var okAll = true
            for (id in ids) {
                val (comm, err) = write1ByteTxRxLocked(
                    id = id,
                    addr = Xl430Registers.ADDR_TORQUE_ENABLE,
                    value = target,
                    timeoutMs = perIdTimeoutMs,
                )
                val ok = (comm == COMM_SUCCESS && err == 0)
                okAll = okAll && ok
                pushTxRxStatus(
                    "id=$id write1ByteTxRx: comm=${commResultToString(comm)} err=${dxlErrorToString(err)} (${if (ok) "OK" else "FAIL"})",
                )
            }

            // Update diag snapshot after the operation (read-only), so you can see "torque actually on"
            // even if the write's Status Packet was missing.
            runCatching { updateMotorDiagLocked(config) }

            okAll
        }
    }

    suspend fun disable(config: Config = Config()): Unit = withContext(Dispatchers.IO) {
        try {
            pushStatus("Motors OFF (torque disable)...")
            ioMutex.withLock {
                runCatching { setTorqueWithRetry(config, enable = false, tries = 3) }
                safeClose()
            }
        } finally {
            running.set(false)
            pushStatus("Motor controller idle.")
        }
    }

    suspend fun updateMotorDiag(config: Config): Unit = withContext(Dispatchers.IO) {
        if (!running.get()) return@withContext
        ioMutex.withLock { updateMotorDiagLocked(config) }
    }

    suspend fun setGoalPositionRad(config: Config, goalRad: Float): Unit = withContext(Dispatchers.IO) {
        if (!running.get()) return@withContext
        ioMutex.withLock {
            val id = config.motorIds.firstOrNull() ?: return@withLock
            val ticks = radToTicksExtended(goalRad)
            pushStatus("Set goal position: id=$id goalRad=$goalRad ticks=$ticks")
            write(DynamixelProtocol2.buildWrite4(id, Xl430Registers.ADDR_GOAL_POSITION, ticks))
        }
    }

    private fun updateMotorDiagLocked(config: Config) {
        val ids = config.motorIds
        val now = System.currentTimeMillis()

        val mode = syncRead(Xl430Registers.ADDR_OPERATING_MODE, 1, ids)
        val torque = syncRead(Xl430Registers.ADDR_TORQUE_ENABLE, 1, ids)
        val hwerr = syncRead(Xl430Registers.ADDR_HARDWARE_ERROR_STATUS, 1, ids)
        val vin = syncRead(Xl430Registers.ADDR_PRESENT_INPUT_VOLTAGE, 2, ids)
        val temp = syncRead(Xl430Registers.ADDR_PRESENT_TEMPERATURE, 1, ids)
        val ppos = syncRead(Xl430Registers.ADDR_PRESENT_POSITION, 4, ids)
        val pvel = syncRead(Xl430Registers.ADDR_PRESENT_VELOCITY, 4, ids)
        val goal = syncRead(Xl430Registers.ADDR_GOAL_POSITION, 4, ids)

        val diags =
            ids.map { id ->
                val seen = lastSeenById[id]
                val connected = seen != null && (now - seen) <= 1500L
                val torqueVal = torque[id]?.get(0)?.toInt()?.and(0xFF)
                val modeVal = mode[id]?.get(0)?.toInt()?.and(0xFF)
                val hwVal = hwerr[id]?.get(0)?.toInt()?.and(0xFF)
                val tempVal = temp[id]?.get(0)?.toInt()?.and(0xFF)
                val vinBytes = vin[id]
                val vinVal =
                    if (vinBytes != null && vinBytes.size >= 2)
                        (vinBytes[0].toInt() and 0xFF) or ((vinBytes[1].toInt() and 0xFF) shl 8)
                    else null
                val pposTicks = ppos[id]?.let { leInt32(it, 0) }
                val pvelRaw = pvel[id]?.let { leInt32(it, 0) }
                val goalTicks = goal[id]?.let { leInt32(it, 0) }

                MotorDiag(
                    id = id,
                    connected = connected,
                    torqueEnabled = torqueVal?.let { it != 0 },
                    operatingMode = modeVal,
                    hwError = hwVal,
                    inputVoltage01V = vinVal,
                    temperatureC = tempVal,
                    presentPosTicks = pposTicks,
                    presentPosRad = pposTicks?.let { ticksToRadExtended(it) },
                    presentVelRaw = pvelRaw,
                    presentVelRadS = pvelRaw?.let { velRawToRadS(it) },
                    goalPosTicks = goalTicks,
                    goalPosRad = goalTicks?.let { ticksToRadExtended(it) },
                    lastSeenMs = seen,
                )
            }
        _motorDiag.value = diags
    }

    private fun setTorqueWithRetry(config: Config, enable: Boolean, tries: Int): Boolean {
        val ids = config.motorIds
        val target = if (enable) 1 else 0

        for (attempt in 1..tries) {
            pushStatus("Torque ${if (enable) "ON" else "OFF"} attempt $attempt/$tries ...")
            for (id in ids) {
                runCatching { write(DynamixelProtocol2.buildWrite1(id, Xl430Registers.ADDR_TORQUE_ENABLE, target)) }
            }
            try {
                Thread.sleep(30)
            } catch (_: InterruptedException) {
            }

            val vals = syncRead(Xl430Registers.ADDR_TORQUE_ENABLE, 1, ids, perPacketTimeoutMs = 10, totalBudgetMs = 120)
            val okCount = ids.count { id ->
                val b = vals[id]?.get(0)?.toInt()?.and(0xFF)
                b != null && b == target
            }
            val seenCount = vals.size
            pushStatus("Torque verify: ok=$okCount/${ids.size}, responded=$seenCount/${ids.size}")
            runCatching { updateMotorDiagLocked(config) }
            if (okCount == ids.size) return true
        }
        return false
    }

    private fun syncRead(
        startAddr: Int,
        dataLen: Int,
        ids: IntArray,
        perPacketTimeoutMs: Int = 10,
        totalBudgetMs: Int = 80,
    ): Map<Int, ByteArray> {
        val pkt = DynamixelProtocol2.buildSyncRead(startAddr, dataLen, ids)
        write(pkt)
        val out = HashMap<Int, ByteArray>()
        val deadline = System.currentTimeMillis() + totalBudgetMs
        while (System.currentTimeMillis() < deadline && out.size < ids.size) {
            val st = readOneStatusPacket(timeoutMs = perPacketTimeoutMs) ?: continue
            if (st.params.isEmpty()) continue
            if (st.params.size < 1 + dataLen) continue
            val data = st.params.copyOfRange(1, 1 + dataLen)
            out[st.id] = data
            lastSeenById[st.id] = System.currentTimeMillis()
        }
        return out
    }

    private fun write1ByteTxRxLocked(
        id: Int,
        addr: Int,
        value: Int,
        timeoutMs: Int,
    ): Pair<Int, Int> {
        flushRxLocked(maxMs = 10)

        val pkt = DynamixelProtocol2.buildWrite1(id, addr, value)
        try {
            write(pkt)
        } catch (t: Throwable) {
            return COMM_TX_FAIL to 0
        }

        val deadline = System.currentTimeMillis() + timeoutMs
        while (System.currentTimeMillis() < deadline) {
            val remain = (deadline - System.currentTimeMillis()).toInt().coerceAtLeast(1)
            val st = readOneStatusPacket(timeoutMs = minOf(10, remain)) ?: continue
            if (st.id != id) continue
            val err = st.params.firstOrNull()?.toInt()?.and(0xFF) ?: 0
            return COMM_SUCCESS to err
        }
        return COMM_RX_TIMEOUT to 0
    }

    private fun write(packet: ByteArray) {
        val p = port ?: error("USB serial not open")
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
            if (rxLen > 16) rxLen = 0
            return null
        }
        if (start > 0) {
            System.arraycopy(rxBuf, start, rxBuf, 0, rxLen - start)
            rxLen -= start
        }
        if (rxLen < 7) return null

        val id = rxBuf[4].toInt() and 0xFF
        val len = (rxBuf[5].toInt() and 0xFF) or ((rxBuf[6].toInt() and 0xFF) shl 8)
        val packetSize = 4 + 1 + 2 + len
        if (rxLen < packetSize) return null

        val raw = rxBuf.copyOfRange(0, packetSize)
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

        val params = unstuffed.copyOfRange(8, unstuffed.size - 2) // error(1) + data...
        return StatusPacket(id = id, params = params)
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

        unregisterUsbPermissionReceiver()
        usbPermReceiver = receiver
        ContextCompat.registerReceiver(
            context,
            receiver,
            IntentFilter(action),
            ContextCompat.RECEIVER_NOT_EXPORTED,
        )

        // Android 14+ requires immutable PendingIntent for implicit broadcast intents.
        val flags =
            PendingIntent.FLAG_UPDATE_CURRENT or
                (if (Build.VERSION.SDK_INT >= 23) PendingIntent.FLAG_IMMUTABLE else 0)
        val permIntent = Intent(action).setPackage(context.packageName)
        val pi = PendingIntent.getBroadcast(context, 0, permIntent, flags)
        usbManager.requestPermission(device, pi)

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

    private fun leInt32(b: ByteArray, off: Int): Int {
        return (b[off].toInt() and 0xFF) or
            ((b[off + 1].toInt() and 0xFF) shl 8) or
            ((b[off + 2].toInt() and 0xFF) shl 16) or
            ((b[off + 3].toInt() and 0xFF) shl 24)
    }

    private fun ticksToRadExtended(ticks: Int): Float {
        // Extended position mode: signed ticks, 4096 ticks per revolution.
        return ticks.toFloat() * (PI.toFloat() / 2048f)
    }

    private fun radToTicksExtended(rad: Float): Int {
        val ticks = rad * (2048f / PI.toFloat())
        return ticks.roundToInt().coerceIn(-1_048_575, 1_048_575)
    }

    private fun velRawToRadS(vRaw: Int): Float {
        // XL430 Present Velocity unit: 0.229 rpm per unit. Convert to rad/s.
        // 0.229 * 2π / 60 = 0.0239691227 rad/s
        return vRaw.toFloat() * 0.0239691227f
    }
}

