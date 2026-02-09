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
import java.nio.charset.StandardCharsets
import java.util.concurrent.atomic.AtomicBoolean

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
    var udpHost by remember { mutableStateOf("192.168.1.2") }
    var udpPortText by remember { mutableStateOf("5005") }
    val udpSender = remember { UdpSender() }
    val remoteEnabled = remember { AtomicBoolean(false) }

    DisposableEffect(Unit) {
        batteryMonitor.start()

        imuMonitor.onUiUpdate = { imu = it }
        imuMonitor.onRawUpdate = { sample ->
            if (remoteEnabled.get()) {
                val payload =
                    buildUdpJson(sample, batteryMonitor.state.value)
                        .toByteArray(StandardCharsets.UTF_8)
                udpSender.offer(payload)
            }
        }
        imuMonitor.startFast()

        onDispose {
            batteryMonitor.stop()
            imuMonitor.stop()
            udpSender.stop()
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

private fun buildUdpJson(imu: ImuState, battery: com.example.phonebot_app_android.system.BatteryState): String {
    fun f(x: Float?): String = x?.let { "%.6f".format(it) } ?: "null"
    fun arr3(a: FloatArray?): String =
        if (a == null || a.size < 3) "null"
        else "[${"%.6f".format(a[0])},${"%.6f".format(a[1])},${"%.6f".format(a[2])}]"
    fun arr4(a: FloatArray?): String =
        if (a == null || a.size < 4) "null"
        else "[${"%.6f".format(a[0])},${"%.6f".format(a[1])},${"%.6f".format(a[2])},${"%.6f".format(a[3])}]"

    return buildString {
        append("{")
        append("\"ts_ns\":").append(imu.timestampNs).append(',')
        append("\"accel\":").append(arr3(imu.accel)).append(',')
        append("\"gyro\":").append(arr3(imu.gyro)).append(',')
        append("\"rotvec\":{")
        append("\"hz\":").append(f(imu.rotVecHz)).append(',')
        append("\"quat\":").append(arr4(imu.quat)).append(',')
        append("\"ypr_deg\":").append(arr3(imu.yprDeg))
        append("},")
        append("\"game_rotvec\":{")
        append("\"hz\":").append(f(imu.gameRotVecHz)).append(',')
        append("\"quat\":").append(arr4(imu.gameQuat)).append(',')
        append("\"ypr_deg\":").append(arr3(imu.gameYprDeg))
        append("},")
        append("\"battery\":{")
        append("\"percent\":").append(battery.percent ?: "null").append(',')
        append("\"is_charging\":").append(battery.isCharging ?: "null").append(',')
        append("\"plugged\":").append(if (battery.plugged == null) "null" else "\"${battery.plugged}\"").append(',')
        append("\"status\":").append(if (battery.status == null) "null" else "\"${battery.status}\"")
        append("}")
        append("}")
    }
}