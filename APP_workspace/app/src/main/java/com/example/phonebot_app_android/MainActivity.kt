package com.example.phonebot_app_android

import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.enableEdgeToEdge
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.Button
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedTextField
import androidx.compose.material3.Scaffold
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.DisposableEffect
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.unit.dp
import com.example.phonebot_app_android.sensors.ImuMonitor
import com.example.phonebot_app_android.sensors.ImuState
import com.example.phonebot_app_android.system.BatteryMonitor
import com.example.phonebot_app_android.ui.theme.PhoneBot_App_AndroidTheme
import android.view.WindowManager
import com.example.phonebot_app_android.network.UdpSender
import java.util.concurrent.atomic.AtomicBoolean
import com.example.phonebot_app_android.network.PhonebotProtocol
import com.example.phonebot_app_android.network.UdpReceiver
import java.net.Inet4Address
import java.net.NetworkInterface
import java.util.concurrent.atomic.AtomicLong
import java.util.concurrent.atomic.AtomicReference
import kotlinx.coroutines.delay
import kotlinx.coroutines.isActive
import com.example.phonebot_app_android.motors.dynamixel.DynamixelController

class MainActivity : ComponentActivity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        enableEdgeToEdge()
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        setContent {
            PhoneBot_App_AndroidTheme {
                Scaffold(modifier = Modifier.fillMaxSize()) { innerPadding ->
                    RobotDashboardScreen(modifier = Modifier.padding(innerPadding))
                }
            }
        }
    }
}

private enum class RunMode { LOCAL, REMOTE_UDP }

private data class MotorUiState(
    val seq: Long = 0L,
    val tsNs: Long = 0L,
    val hz: Float? = null,
    val pos: FloatArray? = null,
    val vel: FloatArray? = null,
    val tau: FloatArray? = null,
    val note: String? = null,
)

private data class MotorFeedbackUiState(
    val ctrlHz: Float? = null,
    val rxHz: Float? = null,
    val readHz: Float? = null,
    val posRad: FloatArray? = null,
    val velRadS: FloatArray? = null,
    val note: String? = null,
)

@Composable
fun RobotDashboardScreen(modifier: Modifier = Modifier) {
    val context = LocalContext.current

    // Battery
    val batteryMonitor = remember { BatteryMonitor(context) }
    val battery by batteryMonitor.state.collectAsState()

    // IMU
    val imuMonitor = remember { ImuMonitor(context) }
    var imu by remember { mutableStateOf(ImuState(note = "Starting IMU...")) }

    // UDP streaming
    var runMode by remember { mutableStateOf(RunMode.LOCAL) }
    var udpHost by remember { mutableStateOf("192.168.20.11") }
    var udpPortText by remember { mutableStateOf("5005") }
    val udpSender = remember { UdpSender() }
    val remoteEnabled = remember { AtomicBoolean(false) }
    val sensorSeq = remember { AtomicLong(0L) }

    // Motor RX (PC -> Android)
    var motorListenPortText by remember { mutableStateOf("6006") }
    val udpReceiver = remember { UdpReceiver() }
    var motorUi by remember { mutableStateOf(MotorUiState(note = "Waiting for motor UDP...")) }
    val latestMotorRef =
        remember { AtomicReference<Pair<PhonebotProtocol.MotorPacket, Float?>?>(null) }

    // Dynamixel USB motor control
    val dxlController = remember { DynamixelController(context) }
    var motorHwEnabled by remember { mutableStateOf(false) }
    var motorHwStatus by remember { mutableStateOf("Motor HW: OFF") }
    val dxlConfig = remember { DynamixelController.Config() } // default: ids 1..13, baud 1M, kp=5, kd=0.1
    var motorFbUi by remember { mutableStateOf(MotorFeedbackUiState(note = "Motor feedback: not started")) }
    val latestMotorFbRef =
        remember { AtomicReference<DynamixelController.MotorStatus?>(null) }
    val motorCtrlHzRef = remember { AtomicReference<Float?>(null) }
    val motorReadHzRef = remember { AtomicReference<Float?>(null) }
    val ctrlRate = remember { SimpleRateEstimator() }
    val readRate = remember { SimpleRateEstimator() }

    DisposableEffect(Unit) {
        batteryMonitor.start()

        imuMonitor.onUiUpdate = { imu = it }
        imuMonitor.onRawUpdate = { sample ->
            if (remoteEnabled.get()) {
                val seq = sensorSeq.incrementAndGet()
                val fb = latestMotorFbRef.get()
                val payload =
                    PhonebotProtocol.packSensorPacket(
                        seq = seq,
                        imu = sample,
                        battery = batteryMonitor.state.value,
                        motorPosRad = fb?.posRad,
                        motorVelRadS = fb?.velRadS,
                    )
                udpSender.offer(payload)
            }
        }
        imuMonitor.startFast()

        udpReceiver.onMotorPacket = { pkt, hz ->
            // Do NOT touch Compose state from this thread; just store latest.
            latestMotorRef.set(pkt to hz)
        }

        onDispose {
            batteryMonitor.stop()
            imuMonitor.stop()
            udpSender.stop()
            udpReceiver.stop()
            // Best-effort torque-off on app exit
            runCatching {
                // fire-and-forget; don't block dispose
                motorHwEnabled = false
            }
        }
    }

    // Throttle motor UI updates to ~20Hz (avoid formatting/recompose cost at motor packet rate).
    LaunchedEffect(Unit) {
        while (isActive) {
            val latest = latestMotorRef.get()
            if (latest != null) {
                val (pkt, hz) = latest
                // Copy arrays so UI holds a stable snapshot (safe even if parsing becomes reuse-based later).
                motorUi =
                    MotorUiState(
                        seq = pkt.seq,
                        tsNs = pkt.tsNs,
                        hz = hz,
                        pos = pkt.pos.copyOf(),
                        vel = pkt.vel.copyOf(),
                        tau = pkt.tau.copyOf(),
                        note = null,
                    )
            }
            delay(50) // ~20 Hz
        }
    }

    // Motor HW enable/disable lifecycle
    LaunchedEffect(motorHwEnabled) {
        if (motorHwEnabled) {
            motorHwStatus = "Motor HW: enabling..."
            val ok = dxlController.enable(dxlConfig)
            motorHwStatus = if (ok) "Motor HW: ON (USB)" else "Motor HW: enable failed (${dxlController.lastStatus})"
            if (!ok) {
                motorHwEnabled = false
                return@LaunchedEffect
            }

            motorFbUi = MotorFeedbackUiState(note = "Motor feedback: running (~20Hz UI)")

            // While enabled, stream goals AND read feedback at 100 Hz.
            // UI remains throttled elsewhere.
            while (isActive && motorHwEnabled) {
                val latest = latestMotorRef.get()
                val pkt = latest?.first
                if (pkt != null) {
                    dxlController.sendGoalPositionsRad(dxlConfig, pkt.pos)
                    motorCtrlHzRef.set(ctrlRate.update(System.nanoTime()))
                }
                val st = dxlController.readPresentPosVel(dxlConfig)
                motorReadHzRef.set(readRate.update(System.nanoTime()))
                latestMotorFbRef.set(st)
                delay(10) // 100 Hz loop
            }
        } else {
            motorHwStatus = "Motor HW: disabling..."
            runCatching { dxlController.disable(dxlConfig) }
            motorHwStatus = "Motor HW: OFF"
            motorFbUi = MotorFeedbackUiState(note = "Motor feedback: stopped (motor HW off)")
            latestMotorFbRef.set(null)
        }
    }

    // Throttle motor feedback UI to ~20 Hz.
    LaunchedEffect(Unit) {
        while (isActive) {
            val st = latestMotorFbRef.get()
            if (st != null) {
                motorFbUi =
                    MotorFeedbackUiState(
                        ctrlHz = motorCtrlHzRef.get(),
                        rxHz = st.rxHz,
                        readHz = motorReadHzRef.get(),
                        posRad = st.posRad?.copyOf(),
                        velRadS = st.velRadS?.copyOf(),
                        note = null,
                    )
            }
            delay(50)
        }
    }

    // Apply runMode + target changes
    DisposableEffect(runMode, udpHost, udpPortText) {
        val port = udpPortText.toIntOrNull() ?: 0
        if (runMode == RunMode.REMOTE_UDP && port in 1..65535) {
            remoteEnabled.set(true)
            udpSender.setTarget(udpHost, port)
            udpSender.start()
        } else {
            remoteEnabled.set(false)
            udpSender.stop()
        }
        onDispose { /* no-op */ }
    }

    // Apply motor listen port changes
    DisposableEffect(motorListenPortText) {
        val port = motorListenPortText.toIntOrNull() ?: 0
        if (port in 1..65535) {
            udpReceiver.start(port)
        } else {
            udpReceiver.stop()
        }
        onDispose { /* no-op */ }
    }

    Column(
        modifier =
            modifier
                .fillMaxSize()
                .background(MaterialTheme.colorScheme.background),
    ) {
        Column(
            modifier =
                Modifier
                    .fillMaxSize()
                    .verticalScroll(rememberScrollState())
                    .padding(12.dp),
            verticalArrangement = Arrangement.spacedBy(10.dp),
        ) {
            SectionTitle("Network")
            val ip = getLocalIpv4Address() ?: "?"
            MonoBlock(
                lines =
                    listOf(
                        "phone ip     : $ip",
                        "motor listen : ${motorListenPortText} (${udpReceiver.lastError?.let { "ERROR: $it" } ?: "OK"})",
                    ),
            )
            Row(modifier = Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                OutlinedTextField(
                    modifier = Modifier.weight(1f),
                    value = motorListenPortText,
                    onValueChange = { motorListenPortText = it.filter { c -> c.isDigit() }.take(5) },
                    label = { Text("Motor UDP listen port") },
                    singleLine = true,
                )
            }

            SectionTitle("Mode")
            Row(modifier = Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                Button(
                    onClick = { runMode = RunMode.LOCAL },
                    enabled = runMode != RunMode.LOCAL,
                ) { Text("Local") }
                Button(
                    onClick = { runMode = RunMode.REMOTE_UDP },
                    enabled = runMode != RunMode.REMOTE_UDP,
                ) { Text("Remote UDP") }
            }

            if (runMode == RunMode.REMOTE_UDP) {
                MonoBlock(
                    lines =
                        listOf(
                            "udp target   : ${udpHost}:${udpPortText}",
                            "udp status   : ${udpSender.lastError?.let { "ERROR: $it" } ?: "OK"}",
                            "udp send hz  : ${udpSender.lastSendHz?.let { "%.1f".format(it) } ?: "?"}",
                            "send format  : binary (float32, little-endian)",
                            "send policy  : latest-only (drops older packets if busy)",
                        )
                )

                Row(modifier = Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                    OutlinedTextField(
                        modifier = Modifier.weight(1f),
                        value = udpHost,
                        onValueChange = { udpHost = it },
                        label = { Text("PC IP / Host") },
                        singleLine = true,
                    )
                    OutlinedTextField(
                        modifier = Modifier.weight(1f),
                        value = udpPortText,
                        onValueChange = { udpPortText = it.filter { c -> c.isDigit() }.take(5) },
                        label = { Text("UDP Port") },
                        singleLine = true,
                    )
                }
            }

            SectionTitle("Battery")
            MonoBlock(
                lines =
                    listOf(
                        "percent      : ${battery.percent?.let { "$it%" } ?: "?"}",
                        "charging     : ${battery.isCharging ?: "?"}",
                        "plugged      : ${battery.plugged ?: "?"}",
                        "status       : ${battery.status ?: "?"}",
                    )
            )

            SectionTitle("Motor (PC → Android UDP)")
            MonoBlock(lines = listOf("note         : motor UI throttled ~20Hz"))
            motorUi.note?.let { MonoBlock(lines = listOf("last note    : $it")) }
            MonoBlock(
                lines =
                    listOf(
                        "seq          : ${motorUi.seq}",
                        "timestamp(ns): ${motorUi.tsNs}",
                        formatHz("motorRx(Hz)", motorUi.hz),
                        formatFloatList("pos[13]", motorUi.pos),
                        formatFloatList("vel[13]", motorUi.vel),
                        formatFloatList("tau[13]", motorUi.tau),
                    ),
            )

            SectionTitle("Motor HW (USB → Dynamixel XL430)")
            MonoBlock(
                lines =
                    listOf(
                        "status       : $motorHwStatus",
                        "dxl note     : ${dxlController.lastStatus}",
                        "ids          : 1..13 (default)",
                        "baud         : 1,000,000",
                        "mode         : position",
                        "kp / kd      : 5 / 0.1 (kd maps to integer register)",
                        "init goal    : 0 rad (~2048 ticks)",
                        "cmd source   : UDP pos[13] (rad) @ 100Hz loop",
                    ),
            )
            Row(modifier = Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                Button(
                    onClick = { motorHwEnabled = !motorHwEnabled },
                ) { Text(if (motorHwEnabled) "Motor HW: ON (tap to OFF)" else "Motor HW: OFF (tap to ON)") }
            }

            SectionTitle("Motor Feedback (XL430 present state)")
            motorFbUi.note?.let { MonoBlock(lines = listOf("note         : $it")) }
            MonoBlock(
                lines =
                    listOf(
                        formatHz("ctrl(Hz)", motorFbUi.ctrlHz),
                        formatHz("dxlRx(Hz)", motorFbUi.rxHz),
                        formatHz("read(Hz)", motorFbUi.readHz),
                        formatFloatList("pos(rad)", motorFbUi.posRad),
                        formatFloatList("vel(rad/s)", motorFbUi.velRadS),
                    ),
            )

            SectionTitle("IMU (fast sampling; UI throttled ~20Hz)")
            imu.note?.let { MonoBlock(lines = listOf("note         : $it")) }

            MonoBlock(
                lines =
                    listOf(
                        "timestamp(ns): ${imu.timestampNs}",
                        formatHz("rotVec(Hz)", imu.rotVecHz),
                        formatHz("gameRot(Hz)", imu.gameRotVecHz),
                        formatHz("accel(Hz)", imu.accelHz),
                        formatHz("gyro(Hz)", imu.gyroHz),
                        formatQuat(imu.quat),
                        formatYpr(imu.yprDeg),
                        formatQuat("gameQuat", imu.gameQuat),
                        formatYpr("gameYPR", imu.gameYprDeg),
                        formatVec3("accel(m/s^2)", imu.accel),
                        formatVec3("gyro(rad/s)", imu.gyro),
                    )
            )
        }
    }
}

@Composable
private fun SectionTitle(text: String) {
    Text(text = text, style = MaterialTheme.typography.titleMedium)
}

@Composable
private fun MonoBlock(lines: List<String>) {
    Column(
        modifier =
            Modifier
                .fillMaxWidth()
                .background(MaterialTheme.colorScheme.surfaceVariant)
                .padding(10.dp),
        verticalArrangement = Arrangement.spacedBy(2.dp),
    ) {
        for (line in lines) {
            Text(text = line, fontFamily = FontFamily.Monospace)
        }
    }
}

private fun formatQuat(q: FloatArray?): String = formatQuat("quat", q)

private fun formatQuat(label: String, q: FloatArray?): String {
    if (q == null || q.size < 4) return "quat[w,x,y,z]: ?"
    return "${label.padEnd(8)}[w,x,y,z]: ${q[0].fmt()}, ${q[1].fmt()}, ${q[2].fmt()}, ${q[3].fmt()}"
}

private fun formatYpr(yprDeg: FloatArray?): String = formatYpr("ypr(deg)", yprDeg)

private fun formatYpr(label: String, yprDeg: FloatArray?): String {
    if (yprDeg == null || yprDeg.size < 3) return "${label.padEnd(12)}: ?"
    return "${label.padEnd(12)}: yaw=${yprDeg[0].fmt()} pitch=${yprDeg[1].fmt()} roll=${yprDeg[2].fmt()}"
}

private fun formatVec3(name: String, v: FloatArray?): String {
    if (v == null || v.size < 3) return "${name.padEnd(12)}: ?"
    return "${name.padEnd(12)}: ${v[0].fmt()}, ${v[1].fmt()}, ${v[2].fmt()}"
}

private fun formatHz(name: String, hz: Float?): String {
    if (hz == null) return "${name.padEnd(12)}: ?"
    return "${name.padEnd(12)}: ${hz.fmt()} Hz"
}

private fun Float.fmt(): String = "% .4f".format(this)

private fun formatFloatList(label: String, a: FloatArray?): String {
    if (a == null || a.isEmpty()) return "${label.padEnd(12)}: ?"
    val s = a.joinToString(prefix = "[", postfix = "]") { it.fmt() }
    return "${label.padEnd(12)}: $s"
}

private fun getLocalIpv4Address(): String? {
    return try {
        val ifaces = NetworkInterface.getNetworkInterfaces() ?: return null
        for (iface in ifaces) {
            if (!iface.isUp || iface.isLoopback) continue
            val addrs = iface.inetAddresses
            for (addr in addrs) {
                if (addr is Inet4Address && !addr.isLoopbackAddress) {
                    val ip = addr.hostAddress
                    if (!ip.isNullOrBlank() && ip != "127.0.0.1") return ip
                }
            }
        }
        null
    } catch (_: Throwable) {
        null
    }
}

private class SimpleRateEstimator(private val alpha: Float = 0.1f) {
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