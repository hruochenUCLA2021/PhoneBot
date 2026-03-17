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
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.ExecutorCoroutineDispatcher
import kotlinx.coroutines.Job
import kotlinx.coroutines.SupervisorJob
import kotlinx.coroutines.cancel
import kotlinx.coroutines.cancelAndJoin
import kotlinx.coroutines.channels.Channel
import kotlinx.coroutines.delay
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.update
import kotlinx.coroutines.sync.Mutex
import kotlinx.coroutines.sync.withLock
import kotlinx.coroutines.asCoroutineDispatcher
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.atomic.AtomicReference
import java.util.concurrent.Executors
import kotlin.math.PI
import kotlin.math.roundToInt

/**
 * Dynamixel controller over USB-Serial (Protocol 2.0), tuned for XL430-W250 position control.
 *
 * Notes / assumptions:
 * - Motor IDs default to 1..13 (change in code if needed).
 * - Incoming UDP motor positions are interpreted as radians in [-pi, +pi].
 * - In Extended Position Control Mode: ticks are signed, 4096 ticks/rev, and 0 rad ~= 0 ticks.
 */
class DynamixelController(private val context: Context) {
    data class Config(
        val motorIds: IntArray = IntArray(13) { i -> i + 1 },
        val baudRate: Int = 1_000_000,
        val kp: Int = 5, // Position P Gain register units
        // User-friendly value. XL430 Position D Gain is an integer register; we map this float to a register value.
        val kd: Float = 0.1f,
        // Kept for backwards compatibility; not used in the current enable() sequence (we set goal = present position).
        val initialGoalTicks: Int = 0,
    )

    private val _logLines = MutableStateFlow<List<String>>(listOf("Motor controller idle."))
    val logLines: StateFlow<List<String>> = _logLines.asStateFlow()

    data class MotorDiag(
        val id: Int,
        val connected: Boolean,
        val torqueEnabled: Boolean? = null,
        val operatingMode: Int? = null,
        val hwError: Int? = null,
        val inputVoltage01V: Int? = null,
        val temperatureC: Int? = null,
        val lastSeenMs: Long? = null,
    )

    private val _motorDiag = MutableStateFlow<List<MotorDiag>>(emptyList())
    val motorDiag: StateFlow<List<MotorDiag>> = _motorDiag.asStateFlow()

    private val _motorFeedback = MutableStateFlow<MotorStatus?>(null)
    val motorFeedback: StateFlow<MotorStatus?> = _motorFeedback.asStateFlow()

    private val lastSeenById = HashMap<Int, Long>()

    @Volatile var lastStatus: String = "Motor controller idle."
        private set

    private fun pushStatus(line: String) {
        lastStatus = line
        val t = System.currentTimeMillis()
        _logLines.update { prev ->
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

    private sealed class UsbCmd {
        data class Torque(val enable: Boolean) : UsbCmd()
    }

    enum class DiagMode { SYNC_READ, PER_ID_TXRX }

    private val _usbThreadRunning = MutableStateFlow(false)
    val usbThreadRunning: StateFlow<Boolean> = _usbThreadRunning.asStateFlow()

    @Volatile private var usbDiagMode: DiagMode = DiagMode.PER_ID_TXRX
    @Volatile private var usbTrackPcGoals: Boolean = false
    private val latestPcGoalRad = AtomicReference<FloatArray?>(null)

    private val usbCmdQ = Channel<UsbCmd>(capacity = Channel.BUFFERED)
    private var usbExecutor: java.util.concurrent.ExecutorService? = null
    private var usbDispatcher: ExecutorCoroutineDispatcher? = null
    private var usbScope: CoroutineScope? = null
    private var usbJob: Job? = null

    // Hold-last-value caches so missing replies don't snap to 0.
    // Size matches `config.motorIds.size` (typically 13).
    private var lastPosRadCache: FloatArray? = null
    private var lastVelRadSCache: FloatArray? = null

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

    // DynamixelSDK-style comm result codes (subset; used for logging/diagnostics).
    private companion object {
        private const val COMM_SUCCESS = 0
        private const val COMM_TX_FAIL = -1001
        private const val COMM_RX_TIMEOUT = -3001
    }

    private fun commResultToString(code: Int): String =
        when (code) {
            COMM_SUCCESS -> "COMM_SUCCESS"
            COMM_TX_FAIL -> "COMM_TX_FAIL"
            COMM_RX_TIMEOUT -> "COMM_RX_TIMEOUT"
            else -> "COMM_$code"
        }

    private fun dxlErrorToString(err: Int): String = "0x" + (err and 0xFF).toString(16).padStart(2, '0')

    fun setLatestPcGoals(posRad: FloatArray?) {
        latestPcGoalRad.set(posRad)
    }

    fun setUsbDiagMode(mode: DiagMode) {
        usbDiagMode = mode
    }

    fun setUsbTrackPcGoals(enabled: Boolean) {
        usbTrackPcGoals = enabled
    }

    fun enqueueTorque(enable: Boolean) {
        usbCmdQ.trySend(UsbCmd.Torque(enable))
    }

    suspend fun startUsbThread(config: Config = Config()) {
        if (_usbThreadRunning.value) return

        val exec = Executors.newSingleThreadExecutor { r ->
            Thread(r, "dxl-usb-worker").apply { isDaemon = true }
        }
        val disp = exec.asCoroutineDispatcher()
        val scope = CoroutineScope(SupervisorJob() + disp)

        usbExecutor = exec
        usbDispatcher = disp
        usbScope = scope

        _usbThreadRunning.value = true
        pushStatus("USB thread: START")

        usbJob =
            scope.launch {
                var nextDiagMs = 0L
                var nextGoalMs = 0L
                var nextFbMs = 0L
                while (isActive) {
                    // Execute queued commands (enable/disable)
                    while (true) {
                        val cmd = usbCmdQ.tryReceive().getOrNull() ?: break
                        when (cmd) {
                            is UsbCmd.Torque -> {
                                val ok = torqueTxRxLocked(config, enable = cmd.enable, perIdTimeoutMs = 200)
                                pushStatus("USB thread: torque ${if (cmd.enable) "ON" else "OFF"} done (ok=$ok)")
                            }
                        }
                    }

                    val now = System.currentTimeMillis()

                    // Periodic diagnostics (2 Hz)
                    if (now >= nextDiagMs) {
                        runCatching {
                            val usePerId = (usbDiagMode == DiagMode.PER_ID_TXRX)
                            updateMotorDiagLockedWorker(config, usePerIdTxRx = usePerId, perReadTimeoutMs = 200)
                        }.onFailure { t ->
                            pushStatus("USB thread: diag failed: ${t::class.java.simpleName}: ${t.message ?: "(no message)"}")
                        }
                        nextDiagMs = now + 500L
                    }

                    // Periodic motor feedback for PC (present pos/vel)
                    if (now >= nextFbMs) {
                        runCatching { _motorFeedback.value = readPresentPosVelLockedWorker(config) }
                        nextFbMs = now + 50L // 20 Hz
                    }

                    // Track PC goals (latest-only), if enabled
                    if (usbTrackPcGoals && now >= nextGoalMs) {
                        val goals = latestPcGoalRad.get()
                        if (goals != null) {
                            runCatching { sendGoalPositionsRadLocked(config, goals) }
                        }
                        nextGoalMs = now + 10L // 100 Hz
                    }

                    delay(2)
                }
            }
    }

    suspend fun stopUsbThread() {
        if (!_usbThreadRunning.value) return
        _usbThreadRunning.value = false
        pushStatus("USB thread: STOP requested")

        val job = usbJob
        val scope = usbScope
        usbJob = null
        usbScope = null

        if (job != null) {
            runCatching { job.cancelAndJoin() }
        }
        scope?.cancel()

        ioMutex.withLock {
            safeClose()
            running.set(false)
        }

        runCatching { usbDispatcher?.close() }
        usbDispatcher = null

        usbExecutor?.let { runCatching { it.shutdownNow() } }
        usbExecutor = null

        pushStatus("USB thread: STOPPED")
    }

    private fun ensurePortOpenLocked(config: Config): Boolean {
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
            pushStatus("No USB serial driver found. Plug in adapter (FTDI/CH340/CP210x).")
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
            pushStatus("openDevice() failed (permission? cable?).")
            return false
        }

        val p = driver.ports.firstOrNull()
        if (p == null) {
            pushStatus("USB serial has no ports?")
            return false
        }
        p.open(connection)
        p.setParameters(config.baudRate, 8, UsbSerialPort.STOPBITS_1, UsbSerialPort.PARITY_NONE)
        // Low latency for control
        p.dtr = true
        p.rts = true
        port = p
        running.set(true)
        return true
    }

    /**
     * Minimal torque command for debugging:
     * - Opens USB serial (if needed)
     * - Sends Torque Enable/Disable using TxRx-style (WRITE then wait Status Packet)
     * - Prints per-ID comm result + dxl_error to the on-screen log
     * - Keeps port open so diag polling can run
     */
    suspend fun torqueTxRxOnce(config: Config = Config(), enable: Boolean, perIdTimeoutMs: Int = 200): Boolean =
        withContext(Dispatchers.IO) {
            ioMutex.withLock {
                var ok = false
                try {
                    if (!ensurePortOpenLocked(config)) return@withLock false

                    val ids = config.motorIds
                    val target = if (enable) 1 else 0
                    pushStatus("Torque TxRx ONCE: ${if (enable) "ON" else "OFF"} ids=${ids.joinToString(",")}")

                    var okAll = true
                    for (id in ids) {
                        val (comm, dxlErr) =
                            write1ByteTxRx(
                                id = id,
                                addr = Xl430Registers.ADDR_TORQUE_ENABLE,
                                value = target,
                                timeoutMs = perIdTimeoutMs,
                            )
                        val okId = (comm == COMM_SUCCESS && dxlErr == 0)
                        okAll = okAll && okId
                        pushStatus("  id=$id comm=${commResultToString(comm)} err=${dxlErrorToString(dxlErr)}")
                    }

                    // Read-back snapshot so you can see actual state even if WRITE status didn't return.
                    runCatching { updateMotorDiagLockedSyncRead(config) }

                    ok = okAll
                    okAll
                } catch (t: Throwable) {
                    pushStatus("Torque TxRx failed: ${t::class.java.simpleName}: ${t.message ?: "(no message)"}")
                    false
                } finally {
                    // Keep the port open so diag polling can continue.
                    pushStatus("Torque TxRx once done.")
                    if (!ok) {
                        // keep status line already printed
                    }
                }
            }
        }

    private fun torqueTxRxLocked(config: Config, enable: Boolean, perIdTimeoutMs: Int): Boolean {
        if (!ensurePortOpenLocked(config)) return false
        val ids = config.motorIds
        val target = if (enable) 1 else 0
        pushStatus("Torque TxRx: ${if (enable) "ON" else "OFF"} ids=${ids.joinToString(",")}")
        var okAll = true
        for (id in ids) {
            val (comm, dxlErr) =
                write1ByteTxRx(
                    id = id,
                    addr = Xl430Registers.ADDR_TORQUE_ENABLE,
                    value = target,
                    timeoutMs = perIdTimeoutMs,
                )
            val okId = (comm == COMM_SUCCESS && dxlErr == 0)
            okAll = okAll && okId
            pushStatus("  id=$id comm=${commResultToString(comm)} err=${dxlErrorToString(dxlErr)}")
        }
        return okAll
    }

    suspend fun enable(config: Config = Config()): Boolean = withContext(Dispatchers.IO) {
        if (running.get()) return@withContext true
        running.set(true)
        var ok = false
        try {
            pushStatus("Searching USB serial device...")
            val usbManager = context.getSystemService(Context.USB_SERVICE) as UsbManager
            val drivers = UsbSerialProber.getDefaultProber().findAllDrivers(usbManager)
            pushStatus("USB drivers found: ${drivers.size}")
            if (drivers.isNotEmpty()) {
                pushStatus("First device: " + devInfo(drivers.first().device))
            }
            val driver = drivers.firstOrNull()
            if (driver == null) {
                pushStatus("No USB serial driver found. Plug in adapter (FTDI/CH340/CP210x).")
                return@withContext false
            }

            val device = driver.device
            if (!usbManager.hasPermission(device)) {
                pushStatus("Requesting USB permission for ${devInfo(device)} ...")
                val granted = requestUsbPermission(usbManager, device)
                if (!granted) {
                    pushStatus("USB permission denied.")
                    return@withContext false
                }
            }

            pushStatus("Opening USB serial port...")
            val connection = usbManager.openDevice(device)
            if (connection == null) {
                pushStatus("openDevice() failed (permission? cable?).")
                return@withContext false
            }

            val p = driver.ports.firstOrNull()
            if (p == null) {
                pushStatus("USB serial has no ports?")
                return@withContext false
            }
            p.open(connection)
            p.setParameters(config.baudRate, 8, UsbSerialPort.STOPBITS_1, UsbSerialPort.PARITY_NONE)
            // Low latency for control
            p.dtr = true
            p.rts = true
            port = p

            pushStatus("Torque-only enable (no mode/PID/goal writes).")

            // Torque ON with verification + retry.
            // NOTE: Read-back requires motors to return status for READ (Status Return Level = 1 or 2).
            if (!setTorqueWithRetry(config, enable = true, tries = 3)) {
                pushStatus("Torque ON failed after retries.")
                return@withContext false
            }

            // Update diag right after enable (read-only)
            runCatching { updateMotorDiagLockedSyncRead(config) }

            pushStatus("Motors torque ON (verified).")
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
                if (lastStatus.isBlank()) {
                    pushStatus("Enable failed (unknown).")
                }
            }
        }
    }

    suspend fun disable(config: Config = Config()): Unit = withContext(Dispatchers.IO) {
        try {
            pushStatus("Motors OFF (torque disable)...")
            runCatching { setTorqueWithRetry(config, enable = false, tries = 3) }
        } finally {
            safeClose()
            running.set(false)
            pushStatus("Motor controller idle.")
            // Keep last motor status snapshot for debugging even after disable/failure.
        }
    }

    suspend fun sendGoalPositionsRad(config: Config, posRad: FloatArray) = withContext(Dispatchers.IO) {
        if (!running.get()) return@withContext
        ioMutex.withLock {
            sendGoalPositionsRadLocked(config, posRad)
        }
    }

    private fun sendGoalPositionsRadLocked(config: Config, posRad: FloatArray) {
        if (!ensurePortOpenLocked(config)) return
        val ids = config.motorIds
        val n = minOf(ids.size, posRad.size)
        val ticks = IntArray(n)
        for (i in 0 until n) {
            ticks[i] = radToTicksExtended(posRad[i])
        }
        sendGoalTicksSync(ids.copyOfRange(0, n), ticks)
    }

    private fun readPresentPosVelLockedWorker(config: Config): MotorStatus {
        if (!ensurePortOpenLocked(config)) return MotorStatus()
        val ids = config.motorIds
        val pkt = DynamixelProtocol2.buildSyncRead(Xl430Registers.ADDR_PRESENT_VELOCITY, 8, ids)
        write(pkt)

        if (lastPosRadCache == null || lastPosRadCache?.size != ids.size) {
            lastPosRadCache = FloatArray(ids.size) { 0f }
            lastVelRadSCache = FloatArray(ids.size) { 0f }
        }
        val vel = lastVelRadSCache!!.copyOf()
        val pos = lastPosRadCache!!.copyOf()

        var got = 0
        val deadline = System.currentTimeMillis() + 50L
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
            val vRaw = leInt32(st.params, 1)
            val pRaw = leInt32(st.params, 5)
            vel[idx] = vRaw.toFloat() * 0.0239691227f
            pos[idx] = ticksToRadExtended(pRaw)
            got += 1
            lastSeenById[id] = System.currentTimeMillis()
        }

        lastPosRadCache = pos
        lastVelRadSCache = vel

        return MotorStatus(
            rxHz = statusRate.update(System.nanoTime()),
            posRad = pos,
            velRadS = vel,
        )
    }

    suspend fun readPresentPosVel(config: Config): MotorStatus = withContext(Dispatchers.IO) {
        if (!running.get()) return@withContext MotorStatus()
        ioMutex.withLock {
            val ids = config.motorIds
            // Read 8 bytes starting at Present Velocity (4) + Present Position (4).
            val pkt = DynamixelProtocol2.buildSyncRead(Xl430Registers.ADDR_PRESENT_VELOCITY, 8, ids)
            write(pkt)

            if (lastPosRadCache == null || lastPosRadCache?.size != ids.size) {
                lastPosRadCache = FloatArray(ids.size) { 0f }
                lastVelRadSCache = FloatArray(ids.size) { 0f }
            }
            val vel = lastVelRadSCache!!.copyOf()
            val pos = lastPosRadCache!!.copyOf()

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
                pos[idx] = ticksToRadExtended(pRaw)
                got += 1

                // connectivity timestamp
                lastSeenById[id] = System.currentTimeMillis()
            }

            // Opportunistically refresh the "connected" field in the diag without extra reads.
            // (Torque/mode/error require register reads; we update those elsewhere.)
            val now = System.currentTimeMillis()
            val existing = _motorDiag.value
            if (existing.isNotEmpty()) {
                _motorDiag.update { prev ->
                    prev.map { d ->
                        val seen = lastSeenById[d.id]
                        d.copy(
                            connected = seen != null && (now - seen) <= 1000L,
                            lastSeenMs = seen,
                        )
                    }
                }
            }

            // Update caches (hold-last-value behavior)
            lastPosRadCache = pos
            lastVelRadSCache = vel

            MotorStatus(
                rxHz = statusRate.update(System.nanoTime()),
                posRad = pos,
                velRadS = vel,
            )
        }
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
            // params[0] = error, then dataLen bytes
            if (st.params.size < 1 + dataLen) continue
            val data = st.params.copyOfRange(1, 1 + dataLen)
            out[st.id] = data
            lastSeenById[st.id] = System.currentTimeMillis()
        }
        return out
    }

    private fun updateMotorDiagLockedSyncRead(config: Config) {
            val ids = config.motorIds
            val now = System.currentTimeMillis()

            val mode = syncRead(Xl430Registers.ADDR_OPERATING_MODE, 1, ids)
            val torque = syncRead(Xl430Registers.ADDR_TORQUE_ENABLE, 1, ids)
            val hwerr = syncRead(Xl430Registers.ADDR_HARDWARE_ERROR_STATUS, 1, ids)
            val vin = syncRead(Xl430Registers.ADDR_PRESENT_INPUT_VOLTAGE, 2, ids)
            val temp = syncRead(Xl430Registers.ADDR_PRESENT_TEMPERATURE, 1, ids)

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

                    MotorDiag(
                        id = id,
                        connected = connected,
                        torqueEnabled = torqueVal?.let { it != 0 },
                        operatingMode = modeVal,
                        hwError = hwVal,
                        inputVoltage01V = vinVal,
                        temperatureC = tempVal,
                        lastSeenMs = seen,
                    )
                }
            _motorDiag.value = diags
    }

    private fun updateMotorDiagLockedPerIdTxRx(config: Config, perReadTimeoutMs: Int) {
        val ids = config.motorIds
        val now = System.currentTimeMillis()

        val diags =
            ids.map { id ->
                val (modeData, modeComm, modeErr) =
                    readTxRx(id, Xl430Registers.ADDR_OPERATING_MODE, 1, timeoutMs = perReadTimeoutMs)
                val (tqData, tqComm, tqErr) =
                    readTxRx(id, Xl430Registers.ADDR_TORQUE_ENABLE, 1, timeoutMs = perReadTimeoutMs)
                val (hwData, hwComm, hwErr) =
                    readTxRx(id, Xl430Registers.ADDR_HARDWARE_ERROR_STATUS, 1, timeoutMs = perReadTimeoutMs)
                val (vinData, vinComm, vinErr) =
                    readTxRx(id, Xl430Registers.ADDR_PRESENT_INPUT_VOLTAGE, 2, timeoutMs = perReadTimeoutMs)
                val (tempData, tempComm, tempErr) =
                    readTxRx(id, Xl430Registers.ADDR_PRESENT_TEMPERATURE, 1, timeoutMs = perReadTimeoutMs)

                // If any read had non-zero comm/error, print a compact line (debugging only).
                if (modeComm != COMM_SUCCESS || modeErr != 0 ||
                    tqComm != COMM_SUCCESS || tqErr != 0 ||
                    hwComm != COMM_SUCCESS || hwErr != 0 ||
                    vinComm != COMM_SUCCESS || vinErr != 0 ||
                    tempComm != COMM_SUCCESS || tempErr != 0
                ) {
                    pushStatus(
                        "Diag TxRx id=$id: " +
                            "mode(${commResultToString(modeComm)},${dxlErrorToString(modeErr)}) " +
                            "tq(${commResultToString(tqComm)},${dxlErrorToString(tqErr)}) " +
                            "hw(${commResultToString(hwComm)},${dxlErrorToString(hwErr)}) " +
                            "vin(${commResultToString(vinComm)},${dxlErrorToString(vinErr)}) " +
                            "temp(${commResultToString(tempComm)},${dxlErrorToString(tempErr)})",
                    )
                }

                val seen = lastSeenById[id]
                val connected = seen != null && (now - seen) <= 1500L

                val modeVal = modeData?.getOrNull(0)?.toInt()?.and(0xFF)
                val torqueVal = tqData?.getOrNull(0)?.toInt()?.and(0xFF)
                val hwVal = hwData?.getOrNull(0)?.toInt()?.and(0xFF)
                val tempVal = tempData?.getOrNull(0)?.toInt()?.and(0xFF)
                val vinVal =
                    if (vinData != null && vinData.size >= 2)
                        (vinData[0].toInt() and 0xFF) or ((vinData[1].toInt() and 0xFF) shl 8)
                    else null

                MotorDiag(
                    id = id,
                    connected = connected,
                    torqueEnabled = torqueVal?.let { it != 0 },
                    operatingMode = modeVal,
                    hwError = hwVal,
                    inputVoltage01V = vinVal,
                    temperatureC = tempVal,
                    lastSeenMs = seen,
                )
            }
        _motorDiag.value = diags
    }

    private fun updateMotorDiagLockedWorker(
        config: Config,
        usePerIdTxRx: Boolean,
        perReadTimeoutMs: Int,
    ) {
        if (!ensurePortOpenLocked(config)) return
        if (usePerIdTxRx) {
            updateMotorDiagLockedPerIdTxRx(config, perReadTimeoutMs = perReadTimeoutMs)
        } else {
            updateMotorDiagLockedSyncRead(config)
        }
    }

    suspend fun updateMotorDiag(
        config: Config,
        usePerIdTxRx: Boolean = false,
        perReadTimeoutMs: Int = 200,
    ): Unit = withContext(Dispatchers.IO) {
        ioMutex.withLock {
            if (!ensurePortOpenLocked(config)) return@withLock
            if (usePerIdTxRx) {
                updateMotorDiagLockedPerIdTxRx(config, perReadTimeoutMs = perReadTimeoutMs)
            } else {
                updateMotorDiagLockedSyncRead(config)
            }
        }
    }

    private fun setTorqueWithRetry(config: Config, enable: Boolean, tries: Int): Boolean {
        val ids = config.motorIds
        val target = if (enable) 1 else 0

        for (attempt in 1..tries) {
            pushStatus("Torque ${if (enable) "ON" else "OFF"} attempt $attempt/$tries ...")

            // TxRx-style torque write (like DynamixelSDK write1ByteTxRx): write then wait for Status Packet.
            // If a motor doesn't return Status Packets for WRITE (e.g., Status Return Level = 1),
            // this will timeout but the write may still have applied. We therefore still do a read-back verify below.
            var commOk = 0
            var commTimeout = 0
            var dxlErrNonzero = 0
            for (id in ids) {
                val (comm, dxlErr) = write1ByteTxRx(
                    id = id,
                    addr = Xl430Registers.ADDR_TORQUE_ENABLE,
                    value = target,
                    timeoutMs = 200,
                )
                if (comm == COMM_SUCCESS) commOk++ else if (comm == COMM_RX_TIMEOUT) commTimeout++
                if (dxlErr != 0) dxlErrNonzero++
                if (comm != COMM_SUCCESS || dxlErr != 0) {
                    pushStatus("Torque TxRx id=$id: comm=${commResultToString(comm)} err=${dxlErrorToString(dxlErr)}")
                }
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
            pushStatus(
                "Torque TxRx: ok=$commOk/${ids.size}, timeout=$commTimeout/${ids.size}, dxlErrNonzero=$dxlErrNonzero | readback ok=$okCount/${ids.size}, responded=$seenCount/${ids.size}",
            )

            // Update diag each attempt so UI shows which IDs are missing / wrong.
            runCatching { updateMotorDiagLockedSyncRead(config) }

            if (okCount == ids.size) return true
        }
        return false
    }

    private fun radToTicksExtended(rad: Float): Int {
        // Extended position mode: int32 ticks, 4096 ticks per revolution.
        // ticks = rad * (4096 / (2*pi)) = rad * (2048 / pi)
        val ticks = rad * (2048f / PI.toFloat())
        // XL430 supports +/-256 rev in extended position mode (from eManual): +/-1,048,575
        return ticks.roundToInt().coerceIn(-1_048_575, 1_048_575)
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
        // We may read Status Packets (TxRx torque + syncRead) and parse them via readOneStatusPacket().
        p.write(packet, 50)
    }

    private data class StatusPacket(val id: Int, val params: ByteArray)

    private fun flushRx(maxMs: Int = 20) {
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

    private fun readTxRx(
        id: Int,
        addr: Int,
        dataLen: Int,
        timeoutMs: Int,
    ): Triple<ByteArray?, Int, Int> {
        flushRx(maxMs = 10)
        val pkt = DynamixelProtocol2.buildRead(id, addr, dataLen)
        try {
            write(pkt)
        } catch (_: Throwable) {
            return Triple(null, COMM_TX_FAIL, 0)
        }

        val deadline = System.currentTimeMillis() + timeoutMs
        while (System.currentTimeMillis() < deadline) {
            val remain = (deadline - System.currentTimeMillis()).toInt().coerceAtLeast(1)
            val st = readOneStatusPacket(timeoutMs = minOf(10, remain)) ?: continue
            if (st.id != id) continue
            val dxlErr = st.params.firstOrNull()?.toInt()?.and(0xFF) ?: 0
            if (st.params.size < 1 + dataLen) return Triple(null, COMM_SUCCESS, dxlErr)
            val data = st.params.copyOfRange(1, 1 + dataLen)
            lastSeenById[id] = System.currentTimeMillis()
            return Triple(data, COMM_SUCCESS, dxlErr)
        }
        return Triple(null, COMM_RX_TIMEOUT, 0)
    }

    private fun write1ByteTxRx(
        id: Int,
        addr: Int,
        value: Int,
        timeoutMs: Int,
    ): Pair<Int, Int> {
        flushRx(maxMs = 10)
        val pkt = DynamixelProtocol2.buildWrite1(id, addr, value)
        try {
            write(pkt)
        } catch (_: Throwable) {
            return COMM_TX_FAIL to 0
        }

        val deadline = System.currentTimeMillis() + timeoutMs
        while (System.currentTimeMillis() < deadline) {
            val remain = (deadline - System.currentTimeMillis()).toInt().coerceAtLeast(1)
            val st = readOneStatusPacket(timeoutMs = minOf(10, remain)) ?: continue
            if (st.id != id) continue
            val dxlErr = st.params.firstOrNull()?.toInt()?.and(0xFF) ?: 0
            return COMM_SUCCESS to dxlErr
        }
        return COMM_RX_TIMEOUT to 0
    }

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

    private fun ticksToRadExtended(ticks: Int): Float {
        // Extended position mode: signed ticks, 4096 ticks per revolution.
        return ticks.toFloat() * (PI.toFloat() / 2048f)
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

        // Android 14+ (targetSdk 34+) forbids creating a *mutable* PendingIntent with an implicit Intent.
        // For USB permission requests we don't need mutability, so use an *immutable* PendingIntent and
        // make the intent package-explicit.
        val flags =
            PendingIntent.FLAG_UPDATE_CURRENT or
                (if (Build.VERSION.SDK_INT >= 23) PendingIntent.FLAG_IMMUTABLE else 0)
        val permIntent = Intent(action).setPackage(context.packageName)
        val pi = PendingIntent.getBroadcast(context, 0, permIntent, flags)
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


