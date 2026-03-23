package com.example.phonebot_app_android

import android.Manifest
import android.os.Bundle
import android.content.pm.PackageManager
import androidx.activity.ComponentActivity
import androidx.activity.compose.rememberLauncherForActivityResult
import androidx.activity.compose.setContent
import androidx.activity.enableEdgeToEdge
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.aspectRatio
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.Button
import androidx.compose.material3.DropdownMenuItem
import androidx.compose.material3.DropdownMenu
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
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
import androidx.compose.runtime.saveable.rememberSaveable
import androidx.compose.runtime.setValue
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.unit.dp
import androidx.compose.ui.viewinterop.AndroidView
import androidx.core.content.ContextCompat
import com.example.phonebot_app_android.sensors.ImuMonitor
import com.example.phonebot_app_android.sensors.ImuState
import com.example.phonebot_app_android.system.BatteryMonitor
import com.example.phonebot_app_android.ui.theme.PhoneBot_App_AndroidTheme
import android.view.WindowManager
import android.view.TextureView
import android.util.Range
import com.example.phonebot_app_android.network.UdpSender
import com.example.phonebot_app_android.network.PhonebotProtocol
import com.example.phonebot_app_android.network.UdpReceiver
import com.example.phonebot_app_android.camera.CameraOption
import com.example.phonebot_app_android.camera.CameraTestingController
import java.net.Inet4Address
import java.net.NetworkInterface
import java.util.concurrent.Executors
import java.util.concurrent.ScheduledExecutorService
import java.util.concurrent.ScheduledFuture
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicLong
import java.util.concurrent.atomic.AtomicReference
import kotlinx.coroutines.delay
import kotlinx.coroutines.withContext
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.isActive
import kotlin.math.PI
import kotlin.math.sin

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

private data class MotorCmdUiState(
    val seq: Long = 0L,
    val tsNs: Long = 0L,
    val hz: Float? = null,
    val pos: FloatArray? = null,
    val vel: FloatArray? = null,
    val tau: FloatArray? = null,
    val note: String? = null,
)

private data class MotorStatusUiState(
    val seq: Long = 0L,
    val tsNs: Long = 0L,
    val hz: Float? = null,
    val pwmPercent: FloatArray? = null,
    val loadPercent: FloatArray? = null,
    val posRad: FloatArray? = null,
    val velRadS: FloatArray? = null,
    val vinV: FloatArray? = null,
    val tempC: FloatArray? = null,
    val note: String? = null,
)

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

@Composable
private fun DropdownSelector(
    label: String,
    value: String,
    options: List<String>,
    enabled: Boolean = true,
    onSelect: (String) -> Unit,
) {
    var expanded by remember { mutableStateOf(false) }
    Box(modifier = Modifier.fillMaxWidth()) {
        OutlinedButton(
            modifier = Modifier.fillMaxWidth(),
            enabled = enabled,
            onClick = { expanded = true },
        ) {
            val shown = value.ifBlank { "(select)" }
            Text("${label}: ${shown}")
        }
        DropdownMenu(
            expanded = expanded && enabled,
            onDismissRequest = { expanded = false },
        ) {
            for (opt in options) {
                DropdownMenuItem(
                    text = { Text(opt) },
                    onClick = {
                        expanded = false
                        onSelect(opt)
                    },
                )
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

    // UDP streaming
    var udpHost by remember { mutableStateOf("192.168.20.15") }
    var udpPortText by remember { mutableStateOf("5005") }
    val udpSensorSender = remember { UdpSender() }
    val udpCtrlSender = remember { UdpSender() }
    var udpEnabled by remember { mutableStateOf(false) }
    val sensorSeq = remember { AtomicLong(0L) }
    val ctrlSeq = remember { AtomicLong(0L) }

    // Motor RX (PC -> Android)
    var motorListenPortText by remember { mutableStateOf("6006") }
    val udpReceiver = remember { UdpReceiver() }
    var motorCmdUi by remember { mutableStateOf(MotorCmdUiState(note = "Waiting for motor UDP...")) }
    var motorStatusUi by remember { mutableStateOf(MotorStatusUiState(note = "Waiting for motor status UDP...")) }
    val latestMotorCmdRef =
        remember { AtomicReference<Pair<PhonebotProtocol.MotorPacket, Float?>?>(null) }
    val latestMotorStatusRef =
        remember { AtomicReference<Pair<PhonebotProtocol.MotorStatusPacket, Float?>?>(null) }

    // Motor control (Android -> PC -> ROS2 -> Pi)
    var testSwingOn by remember { mutableStateOf(false) }
    var swingHzText by remember { mutableStateOf("50") }
    var swingOfferHz by remember { mutableStateOf<Float?>(null) }
    val swingOfferHzRef = remember { AtomicReference<Float?>(null) }
    val swingExecRef = remember { AtomicReference<ScheduledExecutorService?>(null) }
    val swingFutureRef = remember { AtomicReference<ScheduledFuture<*>?>(null) }

    // Page selection
    var page by rememberSaveable { mutableStateOf(0) } // 0=dashboard, 1=camera testing

    // Camera testing page
    val camCtrl = remember { CameraTestingController(context) }
    var camOn by rememberSaveable { mutableStateOf(false) }
    var camOptions by remember { mutableStateOf<List<CameraOption>>(emptyList()) }
    var camId by rememberSaveable { mutableStateOf("") }
    var camResText by rememberSaveable { mutableStateOf("") }
    var camFpsText by rememberSaveable { mutableStateOf("") }
    var camPreviewFps by remember { mutableStateOf<Float?>(null) }
    var camError by remember { mutableStateOf<String?>(null) }
    var camTextureView by remember { mutableStateOf<TextureView?>(null) }
    val cameraLauncher =
        rememberLauncherForActivityResult(ActivityResultContracts.RequestPermission()) { granted ->
            if (granted) {
                camOn = true
            }
        }

    DisposableEffect(Unit) {
        batteryMonitor.start()

        imuMonitor.onUiUpdate = { imu = it }
        imuMonitor.onRawUpdate = { sample ->
            if (udpEnabled) {
                val seq = sensorSeq.incrementAndGet()
                val payload =
                    PhonebotProtocol.packSensorPacket(
                        seq = seq,
                        imu = sample,
                        battery = batteryMonitor.state.value,
                    )
                udpSensorSender.offer(payload)
            }
        }
        imuMonitor.startFast()

        udpReceiver.onMotorPacket = { pkt, hz ->
            // Do NOT touch Compose state from this thread; just store latest.
            latestMotorCmdRef.set(pkt to hz)
        }
        udpReceiver.onMotorStatusPacket = { pkt, hz ->
            latestMotorStatusRef.set(pkt to hz)
        }

        onDispose {
            batteryMonitor.stop()
            imuMonitor.stop()
            udpSensorSender.stop()
            udpCtrlSender.stop()
            udpReceiver.stop()
            camCtrl.stop()
        }
    }

    // Refresh camera info at ~1Hz while on camera page.
    LaunchedEffect(page) {
        if (page != 1) return@LaunchedEffect
        while (isActive && page == 1) {
            val opts =
                withContext(Dispatchers.Default) {
                    camCtrl.listCameras()
                }
            camOptions = opts
            camError = camCtrl.lastError
            camPreviewFps = camCtrl.previewFpsHz
            if (camId.isBlank() && opts.isNotEmpty()) camId = opts.first().cameraId
            delay(1000)
        }
    }

    // Stop camera when leaving camera page.
    LaunchedEffect(page) {
        if (page != 1) {
            camOn = false
            camCtrl.stop()
        }
    }

    // Start/stop camera preview when toggled.
    DisposableEffect(camOn, camId, camResText, camFpsText, camTextureView) {
        val tv = camTextureView
        val granted = ContextCompat.checkSelfPermission(context, Manifest.permission.CAMERA) == PackageManager.PERMISSION_GRANTED
        if (!camOn || tv == null || !granted) {
            camCtrl.stop()
            onDispose { /* no-op */ }
        } else {
            val opt = camOptions.firstOrNull { it.cameraId == camId }
            val size =
                opt?.previewSizes?.firstOrNull { s -> "${s.width}x${s.height}" == camResText }
                    ?: opt?.previewSizes?.firstOrNull()
            val fpsRange =
                opt?.aeFpsRanges?.firstOrNull { r -> "${r.lower}-${r.upper}" == camFpsText }
                    ?: opt?.aeFpsRanges?.firstOrNull()
            if (size != null) {
                camCtrl.startPreview(
                    tv,
                    CameraTestingController.StartConfig(
                        cameraId = camId,
                        targetSize = size,
                        targetFpsRange = fpsRange,
                    )
                )
            }
            onDispose { camCtrl.stop() }
        }
    }

    // Throttle motor UI updates to ~20Hz (avoid formatting/recompose cost at motor packet rate).
    LaunchedEffect(Unit) {
        while (isActive) {
            val latestCmd = latestMotorCmdRef.get()
            if (latestCmd != null) {
                val (pkt, hz) = latestCmd
                motorCmdUi =
                    MotorCmdUiState(
                        seq = pkt.seq,
                        tsNs = pkt.tsNs,
                        hz = hz,
                        pos = pkt.pos.copyOf(),
                        vel = pkt.vel.copyOf(),
                        tau = pkt.tau.copyOf(),
                        note = null,
                    )
            }
            val latestSt = latestMotorStatusRef.get()
            if (latestSt != null) {
                val (pkt, hz) = latestSt
                motorStatusUi =
                    MotorStatusUiState(
                        seq = pkt.seq,
                        tsNs = pkt.tsNs,
                        hz = hz,
                        pwmPercent = pkt.pwmPercent.copyOf(),
                        loadPercent = pkt.loadPercent.copyOf(),
                        posRad = pkt.posRad.copyOf(),
                        velRadS = pkt.velRadS.copyOf(),
                        vinV = pkt.vinV.copyOf(),
                        tempC = pkt.tempC.copyOf(),
                        note = null,
                    )
            }
            swingOfferHz = swingOfferHzRef.get()
            delay(50) // ~20 Hz
        }
    }

    // Apply UDP target changes (always remote UDP)
    DisposableEffect(udpHost, udpPortText) {
        val port = udpPortText.toIntOrNull() ?: 0
        if (port in 1..65535) {
            udpEnabled = true
            udpSensorSender.setTarget(udpHost, port)
            udpCtrlSender.setTarget(udpHost, port)
            udpSensorSender.start()
            udpCtrlSender.start()
        } else {
            udpEnabled = false
            udpSensorSender.stop()
            udpCtrlSender.stop()
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
            Row(modifier = Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                Button(onClick = { page = 0 }) { Text("Dashboard") }
                Button(onClick = { page = 1 }) { Text("Camera testing") }
            }

            if (page == 1) {
                SectionTitle("Camera testing page")

                val camGranted =
                    ContextCompat.checkSelfPermission(context, Manifest.permission.CAMERA) ==
                        PackageManager.PERMISSION_GRANTED
                MonoBlock(
                    lines =
                        listOf(
                            "camera perm : ${if (camGranted) "GRANTED" else "MISSING"}",
                            "cam on      : $camOn",
                            formatHz("camFPS(Hz)", camPreviewFps),
                            "cam error   : ${camError ?: "OK"}",
                        ),
                )

                Row(modifier = Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                    Button(
                        onClick = {
                            if (!camGranted) {
                                cameraLauncher.launch(Manifest.permission.CAMERA)
                            } else {
                                camOn = !camOn
                            }
                        },
                    ) { Text(if (camOn) "Camera: ON" else "Camera: OFF") }
                }

                val selected = camOptions.firstOrNull { it.cameraId == camId } ?: camOptions.firstOrNull()
                val fpsOptions = selected?.aeFpsRanges?.map { r -> "${r.lower}-${r.upper}" } ?: emptyList()
                val resOptions = selected?.previewSizes?.map { s -> "${s.width}x${s.height}" } ?: emptyList()

                LaunchedEffect(selected?.cameraId, camOptions) {
                    val ids = camOptions.map { it.cameraId }
                    if (ids.isNotEmpty() && camId !in ids) camId = ids.first()
                    val s = camOptions.firstOrNull { it.cameraId == camId }
                    val fps = s?.aeFpsRanges?.map { r -> "${r.lower}-${r.upper}" } ?: emptyList()
                    val res = s?.previewSizes?.map { sz -> "${sz.width}x${sz.height}" } ?: emptyList()
                    if (camFpsText.isBlank() && fps.isNotEmpty()) camFpsText = fps.first()
                    if (camFpsText.isNotBlank() && fps.isNotEmpty() && camFpsText !in fps) camFpsText = fps.first()
                    if (camResText.isBlank() && res.isNotEmpty()) camResText = res.first()
                    if (camResText.isNotBlank() && res.isNotEmpty() && camResText !in res) camResText = res.first()
                }

                DropdownSelector(
                    label = "Camera ID",
                    value = camId,
                    options = camOptions.map { it.cameraId },
                    onSelect = { camId = it },
                )
                DropdownSelector(
                    label = "Target FPS range",
                    value = camFpsText,
                    options = fpsOptions,
                    enabled = fpsOptions.isNotEmpty(),
                    onSelect = { camFpsText = it },
                )
                DropdownSelector(
                    label = "Resolution",
                    value = camResText,
                    options = resOptions,
                    enabled = resOptions.isNotEmpty(),
                    onSelect = { camResText = it },
                )

                SectionTitle("Camera preview")
                val selectedSize =
                    selected?.previewSizes?.firstOrNull { s -> "${s.width}x${s.height}" == camResText }
                        ?: selected?.previewSizes?.firstOrNull()
                val aspect =
                    if (selectedSize != null && selectedSize.height > 0) {
                        selectedSize.width.toFloat() / selectedSize.height.toFloat()
                    } else {
                        16f / 9f
                    }
                Box(modifier = Modifier.fillMaxWidth().aspectRatio(aspect).padding(top = 8.dp)) {
                    AndroidView(
                        factory = { ctx -> TextureView(ctx).also { camTextureView = it } },
                        modifier = Modifier.fillMaxSize(),
                    )
                }

                MonoBlock(
                    lines =
                        listOf(
                            "available cameras:",
                        ) + camOptions.flatMap { opt ->
                            val isLogical = opt.capabilities.contains(android.hardware.camera2.CameraCharacteristics.REQUEST_AVAILABLE_CAPABILITIES_LOGICAL_MULTI_CAMERA)
                            listOf(
                                "- id=${opt.cameraId} lensFacing=${opt.lensFacing} orient=${opt.sensorOrientationDeg} logicalMultiCam=$isLogical",
                                "  physicalIds=${opt.physicalCameraIds.joinToString()} (empty means not a logical multi-cam id)",
                                "  fpsRanges=${opt.aeFpsRanges.joinToString { "${it.lower}-${it.upper}" }}",
                                "  previewSizes=${opt.previewSizes.joinToString { "${it.width}x${it.height}" }}",
                            )
                        },
                )
            } else {

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

            SectionTitle("PC UDP target (always remote UDP)")
            MonoBlock(
                lines =
                    listOf(
                        "udp target   : ${udpHost}:${udpPortText}",
                        "udp status   : ${(udpSensorSender.lastError ?: udpCtrlSender.lastError)?.let { "ERROR: $it" } ?: "OK"}",
                        "sensor send  : ${udpSensorSender.lastSendHz?.let { "%.1f".format(it) } ?: "?"} Hz (latest-only)",
                        "ctrl send    : ${udpCtrlSender.lastSendHz?.let { "%.1f".format(it) } ?: "?"} Hz (latest-only)",
                        "format       : binary (little-endian)",
                    ),
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

            SectionTitle("Motor control (Android → PC UDP)")
            MonoBlock(lines = listOf("note         : torque + goal commands are sent to PC over UDP, then bridged to ROS2"))
            MonoBlock(
                lines =
                    listOf(
                        formatHz("swingOffer(Hz)", swingOfferHz),
                        "ctrlSend(Hz) : ${udpCtrlSender.lastSendHz?.let { "%.1f".format(it) } ?: "?"} Hz (sender thread)",
                    ),
            )
            Row(modifier = Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                Button(
                    onClick = {
                        val seq = ctrlSeq.incrementAndGet()
                        val payload = PhonebotProtocol.packTorquePacket(seq = seq, tsNs = System.nanoTime(), enable = true)
                        udpCtrlSender.offer(payload)
                    },
                    enabled = udpEnabled,
                ) { Text("Torque ON") }
                Button(
                    onClick = {
                        val seq = ctrlSeq.incrementAndGet()
                        val payload = PhonebotProtocol.packTorquePacket(seq = seq, tsNs = System.nanoTime(), enable = false)
                        udpCtrlSender.offer(payload)
                    },
                    enabled = udpEnabled,
                ) { Text("Torque OFF") }
            }
            Row(modifier = Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                Button(
                    onClick = {
                        val seq = ctrlSeq.incrementAndGet()
                        val pos = FloatArray(13) { 0f }
                        val vel = FloatArray(13) { 0f }
                        val tau = FloatArray(13) { 0f }
                        udpCtrlSender.offer(
                            PhonebotProtocol.packMotorPacket(seq = seq, tsNs = System.nanoTime(), pos = pos, vel = vel, tau = tau),
                        )
                    },
                    enabled = udpEnabled,
                ) { Text("Zero position") }
                OutlinedTextField(
                    modifier = Modifier.weight(1f),
                    value = swingHzText,
                    onValueChange = { swingHzText = it.filter { c -> c.isDigit() || c == '.' }.take(6) },
                    label = { Text("Swing Hz") },
                    singleLine = true,
                )
                Button(
                    onClick = { testSwingOn = !testSwingOn },
                    enabled = udpEnabled,
                ) { Text(if (testSwingOn) "Test swing: ON" else "Test swing: OFF") }
            }

            val swingHz = swingHzText.toFloatOrNull()
            val swingPeriodNs =
                swingHz
                    ?.takeIf { it.isFinite() && it > 0.0f }
                    ?.let { (1_000_000_000.0 / it.toDouble()).toLong().coerceAtLeast(1L) }

            DisposableEffect(testSwingOn, udpEnabled, swingPeriodNs) {
                // Always stop any previous task first (defensive).
                swingFutureRef.getAndSet(null)?.cancel(false)
                swingExecRef.getAndSet(null)?.shutdownNow()
                swingOfferHzRef.set(null)

                if (testSwingOn && udpEnabled && swingPeriodNs != null) {
                    val exec =
                        Executors.newSingleThreadScheduledExecutor { r ->
                            Thread(r, "phonebot-swing-sender").apply { isDaemon = true }
                        }
                    val rate = SimpleRateEstimator(alpha = 0.1f)
                    val ampPos = 0.25f
                    val w = (2.0 * PI * 0.5).toFloat()
                    val t0 = System.nanoTime()
                    val pos = FloatArray(13)
                    val vel = FloatArray(13) { 0f }
                    val tau = FloatArray(13) { 0f }

                    val fut =
                        exec.scheduleAtFixedRate(
                            {
                                val nowNs = System.nanoTime()
                                val t = (nowNs - t0).toFloat() * 1e-9f
                                for (i in 0 until 13) {
                                    val phase = 0.5f * i.toFloat()
                                    pos[i] = ampPos * sin(w * t + phase)
                                }
                                val seq = ctrlSeq.incrementAndGet()
                                udpCtrlSender.offer(
                                    PhonebotProtocol.packMotorPacket(seq = seq, tsNs = nowNs, pos = pos, vel = vel, tau = tau),
                                )
                                swingOfferHzRef.set(rate.update(nowNs))
                            },
                            0L,
                            swingPeriodNs,
                            TimeUnit.NANOSECONDS,
                        )
                    swingExecRef.set(exec)
                    swingFutureRef.set(fut)
                }

                onDispose {
                    swingFutureRef.getAndSet(null)?.cancel(false)
                    swingExecRef.getAndSet(null)?.shutdownNow()
                    swingOfferHzRef.set(null)
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

            SectionTitle("Motor status (PC → Android UDP)")
            MonoBlock(lines = listOf("note         : UI throttled ~20Hz; shows receive Hz"))
            motorStatusUi.note?.let { MonoBlock(lines = listOf("last note    : $it")) }
            MonoBlock(
                lines =
                    listOf(
                        "seq          : ${motorStatusUi.seq}",
                        "timestamp(ns): ${motorStatusUi.tsNs}",
                        formatHz("statusRx(Hz)", motorStatusUi.hz),
                        formatFloatList("pwm(%)", motorStatusUi.pwmPercent),
                        formatFloatList("load(%)", motorStatusUi.loadPercent),
                        formatFloatList("pos(rad)", motorStatusUi.posRad),
                        formatFloatList("vel(rad/s)", motorStatusUi.velRadS),
                        formatFloatList("vin(V)", motorStatusUi.vinV),
                        formatFloatList("temp(C)", motorStatusUi.tempC),
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