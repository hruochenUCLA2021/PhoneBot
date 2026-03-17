package com.example.motor_control_app

import android.os.Bundle
import android.view.WindowManager
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
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.unit.dp
import com.example.motor_control_app.motors.dynamixel.DynamixelController
import kotlinx.coroutines.delay
import kotlinx.coroutines.isActive


class MainActivity : ComponentActivity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        enableEdgeToEdge()
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        setContent {
            MaterialTheme {
                Scaffold(modifier = Modifier.fillMaxSize()) { innerPadding ->
                    MotorControlScreen(modifier = Modifier.padding(innerPadding))
                }
            }
        }
    }
}


@Composable
private fun MotorControlScreen(modifier: Modifier = Modifier) {
    val scroll = rememberScrollState()

    val context = LocalContext.current
    val dxlController = remember(context) { DynamixelController(context = context) }
    val config = remember { DynamixelController.Config() } // ids 1..13, baud 1_000_000

    var motorHwEnabled by remember { mutableStateOf(false) }
    var motorHwStatus by remember { mutableStateOf("Motor HW: OFF") }

    // UI-throttled snapshots (~20 Hz)
    var uiLogLines by remember { mutableStateOf(listOf("Motor controller idle.")) }
    var uiTxRxLogLines by remember { mutableStateOf(listOf("TxRx log idle.")) }
    var uiDiag by remember { mutableStateOf(emptyList<DynamixelController.MotorDiag>()) }

    // Simple settings fields (optional)
    var baudText by remember { mutableStateOf(config.baudRate.toString()) }
    var motorIdText by remember { mutableStateOf("1") }
    var goalRadText by remember { mutableStateOf("0.0") }
    var setGoalReq by remember { mutableStateOf<Float?>(null) }
    var actionReq by remember { mutableStateOf<String?>(null) }

    fun currentConfig(): DynamixelController.Config {
        val baud = baudText.toIntOrNull() ?: config.baudRate
        val id = motorIdText.toIntOrNull()?.coerceIn(1, 252) ?: 1
        return config.copy(baudRate = baud, motorIds = intArrayOf(id))
    }

    // UI throttle loop (~20 Hz)
    LaunchedEffect(Unit) {
        while (isActive) {
            uiLogLines = dxlController.logLines.value
            uiTxRxLogLines = dxlController.txrxLogLines.value
            uiDiag = dxlController.motorDiag.value
            delay(50)
        }
    }

    // One-shot actions (explicit buttons)
    LaunchedEffect(actionReq) {
        val a = actionReq ?: return@LaunchedEffect
        actionReq = null

        when (a) {
            "enable_verify" -> {
                motorHwStatus = "Motor HW: enabling (verify via SyncRead)..."
                val ok = dxlController.enable(currentConfig())
                motorHwEnabled = ok
                motorHwStatus = if (ok) "Motor HW: ON (SyncRead verified)" else "Motor HW: enable failed (${dxlController.lastStatus})"
            }
            "enable_txrx" -> {
                motorHwStatus = "Motor HW: enabling (TxRx like SDK)..."
                val ok = dxlController.setTorqueTxRxLikeSdk(currentConfig(), enable = true)
                motorHwEnabled = ok
                motorHwStatus = if (ok) "Motor HW: ON (TxRx OK)" else "Motor HW: TxRx enable failed (${dxlController.lastTxRxStatus})"
            }
            "disable" -> {
                motorHwStatus = "Motor HW: disabling..."
                runCatching { dxlController.disable(currentConfig()) }
                motorHwEnabled = false
                motorHwStatus = "Motor HW: OFF"
            }
            "disable_txrx" -> {
                motorHwStatus = "Motor HW: torque OFF (TxRx like SDK)..."
                val ok = dxlController.setTorqueTxRxLikeSdk(currentConfig(), enable = false)
                motorHwStatus = if (ok) "Motor HW: torque OFF (TxRx OK)" else "Motor HW: TxRx disable failed (${dxlController.lastTxRxStatus})"
            }
        }
    }

    // Poll detailed motor status at low rate (read-only)
    LaunchedEffect(motorHwEnabled) {
        if (!motorHwEnabled) return@LaunchedEffect
        while (isActive && motorHwEnabled) {
            runCatching { dxlController.updateMotorDiag(currentConfig()) }
            delay(200) // 5 Hz polling; UI remains 20 Hz
        }
    }

    Column(
        modifier = modifier.fillMaxSize().verticalScroll(scroll).padding(12.dp),
        verticalArrangement = Arrangement.spacedBy(10.dp),
    ) {
        SectionTitle("Motor Enable + Status (Dynamixel XL430 via U2D2)")

        MonoBlock(
            lines = listOf(
                "status       : $motorHwStatus",
                "dxl note     : ${dxlController.lastStatus}",
                "motor id     : ${motorIdText}",
                "baud         : ${baudText}",
                "protocol     : Dynamixel Protocol 2.0",
                "mode         : torque-only bring-up (no PID/mode/goal writes)",
                "ui throttle  : ~20 Hz",
            )
        )

        Row(modifier = Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            OutlinedTextField(
                modifier = Modifier.weight(1f),
                value = baudText,
                onValueChange = { baudText = it.filter { c -> c.isDigit() }.take(7) },
                label = { Text("Baud") },
                singleLine = true,
            )
            OutlinedTextField(
                modifier = Modifier.weight(1f),
                value = motorIdText,
                onValueChange = { motorIdText = it.filter { c -> c.isDigit() }.take(3) },
                label = { Text("Motor ID") },
                singleLine = true,
            )
        }

        Row(modifier = Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            Button(
                modifier = Modifier.weight(1f),
                onClick = { actionReq = "enable_verify" },
            ) {
                Text("Enable (SyncRead verify)")
            }
            Button(
                modifier = Modifier.weight(1f),
                onClick = { actionReq = "enable_txrx" },
            ) {
                Text("Enable (TxRx like SDK)")
            }
        }

        Row(modifier = Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            Button(
                modifier = Modifier.weight(1f),
                onClick = { actionReq = "disable" },
            ) {
                Text("Disable (close port)")
            }
            Button(
                modifier = Modifier.weight(1f),
                onClick = { actionReq = "disable_txrx" },
            ) {
                Text("Torque OFF (TxRx)")
            }
        }

        SectionTitle("Dynamixel log (SyncRead-verify path)")
        MonoBlock(lines = uiLogLines.takeLast(20))

        SectionTitle("Dynamixel log (TxRx path)")
        MonoBlock(lines = uiTxRxLogLines.takeLast(20))

        SectionTitle("Motor status (selected ID)")
        val lines =
            if (uiDiag.isEmpty()) {
                listOf("(no data yet)")
            } else {
                listOf("id | conn | torque | mode | hwErr | Vin | Temp | pos(rad) | vel(rad/s) | goal(rad)") +
                    uiDiag.map { d ->
                    val conn = if (d.connected) "Y" else "N"
                    val tq = d.torqueEnabled?.let { if (it) "ON" else "OFF" } ?: "?"
                    val mode = d.operatingMode?.toString() ?: "?"
                    val hw = d.hwError?.toString() ?: "?"
                    val vin = d.inputVoltage01V?.let { String.format("%.1fV", it / 10.0) } ?: "?"
                    val tmp = d.temperatureC?.let { "${it}C" } ?: "?"
                    val pos = d.presentPosRad?.let { String.format("%.3f", it) } ?: "?"
                    val vel = d.presentVelRadS?.let { String.format("%.3f", it) } ?: "?"
                    val goal = d.goalPosRad?.let { String.format("%.3f", it) } ?: "?"
                    "${d.id.toString().padStart(2)} | $conn | ${tq.padEnd(3)} | ${mode.padEnd(2)} | ${hw.padEnd(3)} | ${vin.padEnd(5)} | ${tmp.padEnd(3)} | ${pos.padStart(7)} | ${vel.padStart(9)} | ${goal.padStart(8)}"
                }
            }
        MonoBlock(lines = lines)

        SectionTitle("Goal position (extended position mode)")
        MonoBlock(
            lines = listOf(
                "input goal(rad): $goalRadText",
                "note           : 0 rad = 0 ticks in extended mode",
            )
        )
        Row(modifier = Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            OutlinedTextField(
                modifier = Modifier.weight(1f),
                value = goalRadText,
                onValueChange = { goalRadText = it.take(16) },
                label = { Text("Goal rad") },
                singleLine = true,
            )
            Button(
                onClick = {
                    val g = goalRadText.toFloatOrNull()
                    if (g != null) {
                        setGoalReq = g
                    }
                }
            ) { Text("Set goal") }
        }

        // Handle "Set goal" button via a one-shot state trigger to call suspend API safely.
        LaunchedEffect(setGoalReq) {
            val g = setGoalReq ?: return@LaunchedEffect
            setGoalReq = null
            runCatching { dxlController.setGoalPositionRad(currentConfig(), g) }
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

