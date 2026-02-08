package com.example.phonebot_app_android

import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.enableEdgeToEdge
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.MaterialTheme
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

class MainActivity : ComponentActivity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        enableEdgeToEdge()
        setContent {
            PhoneBot_App_AndroidTheme {
                Scaffold(modifier = Modifier.fillMaxSize()) { innerPadding ->
                    RobotDashboardScreen(modifier = Modifier.padding(innerPadding))
                }
            }
        }
    }
}

@Composable
fun RobotDashboardScreen(modifier: Modifier = Modifier) {
    val context = LocalContext.current

    // Battery
    val batteryMonitor = remember { BatteryMonitor(context) }
    val battery by batteryMonitor.state.collectAsState()

    // IMU
    val imuMonitor = remember { ImuMonitor(context) }
    var imu by remember { mutableStateOf(ImuState(note = "Starting IMU...")) }

    DisposableEffect(Unit) {
        batteryMonitor.start()

        imuMonitor.onUiUpdate = { imu = it }
        imuMonitor.startFast()

        onDispose {
            batteryMonitor.stop()
            imuMonitor.stop()
        }
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
                        formatQuat(imu.quat),
                        formatYpr(imu.yprDeg),
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

private fun formatQuat(q: FloatArray?): String {
    if (q == null || q.size < 4) return "quat[w,x,y,z]: ?"
    return "quat[w,x,y,z]: ${q[0].fmt()}, ${q[1].fmt()}, ${q[2].fmt()}, ${q[3].fmt()}"
}

private fun formatYpr(yprDeg: FloatArray?): String {
    if (yprDeg == null || yprDeg.size < 3) return "ypr(deg)     : ?"
    return "ypr(deg)     : yaw=${yprDeg[0].fmt()} pitch=${yprDeg[1].fmt()} roll=${yprDeg[2].fmt()}"
}

private fun formatVec3(name: String, v: FloatArray?): String {
    if (v == null || v.size < 3) return "${name.padEnd(12)}: ?"
    return "${name.padEnd(12)}: ${v[0].fmt()}, ${v[1].fmt()}, ${v[2].fmt()}"
}

private fun Float.fmt(): String = "% .4f".format(this)