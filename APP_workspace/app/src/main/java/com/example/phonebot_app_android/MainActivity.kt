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
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.graphics.nativeCanvas
import androidx.compose.foundation.Canvas
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.platform.LocalLifecycleOwner
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.unit.dp
import androidx.compose.ui.viewinterop.AndroidView
import androidx.core.content.ContextCompat
import com.example.phonebot_app_android.sensors.ImuMonitor
import com.example.phonebot_app_android.sensors.ImuState
import com.example.phonebot_app_android.system.BatteryMonitor
import com.example.phonebot_app_android.ui.theme.PhoneBot_App_AndroidTheme
import android.view.WindowManager
import android.util.Range
import com.example.phonebot_app_android.network.UdpSender
import com.example.phonebot_app_android.network.PhonebotProtocol
import com.example.phonebot_app_android.network.UdpReceiver
import com.example.phonebot_app_android.camera.CameraOption
import com.example.phonebot_app_android.camera.CameraTestingController
import android.graphics.Bitmap
import android.graphics.BitmapFactory
import android.graphics.ImageFormat
import android.graphics.Rect
import android.util.Size
import android.hardware.camera2.CameraCharacteristics
import android.hardware.camera2.CaptureRequest
import android.graphics.YuvImage
import java.util.concurrent.atomic.AtomicBoolean
import java.net.Inet4Address
import java.net.NetworkInterface
import java.io.ByteArrayOutputStream
import java.util.concurrent.Executors
import java.util.concurrent.ExecutorService
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
import kotlin.math.max
import kotlin.math.min

// CameraX
import androidx.camera.core.CameraSelector
import androidx.camera.core.ImageAnalysis
import androidx.camera.core.ImageProxy
import androidx.camera.core.Preview
import androidx.camera.lifecycle.ProcessCameraProvider
import androidx.camera.view.PreviewView
import androidx.camera.camera2.interop.Camera2CameraInfo
import androidx.camera.camera2.interop.Camera2Interop

// ML Kit
import com.google.mlkit.vision.common.InputImage
import com.google.mlkit.vision.face.Face
import com.google.mlkit.vision.face.FaceDetection
import com.google.mlkit.vision.face.FaceDetector
import com.google.mlkit.vision.face.FaceDetectorOptions
import com.google.mlkit.vision.pose.PoseDetection
import com.google.mlkit.vision.pose.PoseDetector
import com.google.mlkit.vision.pose.PoseLandmark
import com.google.mlkit.vision.pose.defaults.PoseDetectorOptions
import com.google.mlkit.vision.objects.ObjectDetection
import com.google.mlkit.vision.objects.ObjectDetector
import com.google.mlkit.vision.objects.defaults.ObjectDetectorOptions

// MediaPipe Tasks
import com.google.mediapipe.framework.image.BitmapImageBuilder
import com.google.mediapipe.tasks.core.BaseOptions
import com.google.mediapipe.tasks.vision.core.RunningMode
import com.google.mediapipe.tasks.vision.facelandmarker.FaceLandmarker
import com.google.mediapipe.tasks.vision.facelandmarker.FaceLandmarkerResult
import com.google.mediapipe.tasks.vision.gesturerecognizer.GestureRecognizer
import com.google.mediapipe.tasks.vision.gesturerecognizer.GestureRecognizerResult
import com.google.mediapipe.tasks.vision.handlandmarker.HandLandmarker
import com.google.mediapipe.tasks.vision.handlandmarker.HandLandmarkerResult
import com.google.mediapipe.tasks.vision.poselandmarker.PoseLandmarker
import com.google.mediapipe.tasks.vision.poselandmarker.PoseLandmarkerResult
import com.google.mediapipe.tasks.components.containers.NormalizedLandmark

enum class VisionLibrary { NONE, MLKIT, MEDIAPIPE }
enum class VisionTask { NONE, FACE, POSE, OBJECT, HAND, GESTURE }
enum class MediaPipePoseModel { LITE, FULL, HEAVY }

private data class NormRect(val left: Float, val top: Float, val right: Float, val bottom: Float)
private data class NormPoint(val x: Float, val y: Float)
private data class NormLine(val x0: Float, val y0: Float, val x1: Float, val y1: Float)
private data class NormBox(val rect: NormRect, val text: String? = null)
private data class VisionOverlayState(
    val imageWidth: Int,
    val imageHeight: Int,
    val boxes: List<NormBox> = emptyList(),
    val lines: List<NormLine> = emptyList(),
    val points: List<NormPoint> = emptyList(),
    val label: String? = null,
    val procHz: Float? = null,
)

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
private fun VisionOverlay(overlay: VisionOverlayState?) {
    if (overlay == null) return
    Canvas(modifier = Modifier.fillMaxSize()) {
        val w = size.width
        val h = size.height
        val stroke = Stroke(width = 3f)
        val paint =
            android.graphics.Paint().apply {
                color = android.graphics.Color.GREEN
                textSize = 32f
                isAntiAlias = true
                style = android.graphics.Paint.Style.FILL
            }
        for (ln in overlay.lines) {
            drawLine(
                color = Color(0xFF00FF00),
                start = androidx.compose.ui.geometry.Offset(ln.x0 * w, ln.y0 * h),
                end = androidx.compose.ui.geometry.Offset(ln.x1 * w, ln.y1 * h),
                strokeWidth = 3f,
            )
        }
        for (p in overlay.points) {
            drawCircle(
                color = Color(0xFF00FF00),
                radius = 4f,
                center = androidx.compose.ui.geometry.Offset(p.x * w, p.y * h),
            )
        }
        for (b in overlay.boxes) {
            val l = b.rect.left * w
            val t = b.rect.top * h
            val r = b.rect.right * w
            val bb = b.rect.bottom * h
            drawRect(
                color = Color(0xFF00FF00),
                topLeft = androidx.compose.ui.geometry.Offset(l, t),
                size = androidx.compose.ui.geometry.Size(max(0f, r - l), max(0f, bb - t)),
                style = stroke,
            )
            val txt = b.text
            if (!txt.isNullOrBlank()) {
                drawContext.canvas.nativeCanvas.drawText(
                    txt,
                    l.coerceAtLeast(0f),
                    (t - 6f).coerceAtLeast(14f),
                    paint,
                )
            }
        }
    }
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
    val camPreviewFpsRef = remember { AtomicReference<Float?>(null) }
    var camError by remember { mutableStateOf<String?>(null) }
    var camPreviewView by remember { mutableStateOf<PreviewView?>(null) }
    var visionLib by rememberSaveable { mutableStateOf(VisionLibrary.NONE) }
    var visionTask by rememberSaveable { mutableStateOf(VisionTask.NONE) }
    var visionNote by remember { mutableStateOf<String?>(null) }
    val visionNoteRef = remember { AtomicReference<String?>(null) }
    var mpPoseModel by rememberSaveable { mutableStateOf(MediaPipePoseModel.LITE) }
    val visionBusy = remember { AtomicBoolean(false) }
    val visionOverlayRef = remember { AtomicReference<VisionOverlayState?>(null) }
    var visionOverlayUi by remember { mutableStateOf<VisionOverlayState?>(null) }
    val visionRate = remember { SimpleRateEstimator(alpha = 0.1f) }
    val camPreviewRate = remember { SimpleRateEstimator(alpha = 0.1f) }

    val faceDetectorRef = remember { AtomicReference<FaceDetector?>(null) }
    val poseDetectorRef = remember { AtomicReference<PoseDetector?>(null) }
    val objectDetectorRef = remember { AtomicReference<ObjectDetector?>(null) }

    val mpFaceRef = remember { AtomicReference<FaceLandmarker?>(null) }
    val mpHandRef = remember { AtomicReference<HandLandmarker?>(null) }
    val mpPoseRef = remember { AtomicReference<PoseLandmarker?>(null) }
    val mpGestureRef = remember { AtomicReference<GestureRecognizer?>(null) }

    val camProviderRef = remember { AtomicReference<ProcessCameraProvider?>(null) }
    val camAnalysisExecRef = remember { AtomicReference<ExecutorService?>(null) }
    val previewOutputTransformRef =
        remember { AtomicReference<androidx.camera.view.transform.OutputTransform?>(null) }
    val previewViewSizeRef = remember { AtomicReference<Pair<Int, Int>?>(null) }
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
            camProviderRef.getAndSet(null)?.unbindAll()
            camAnalysisExecRef.getAndSet(null)?.shutdownNow()
            runCatching { faceDetectorRef.getAndSet(null)?.close() }
            runCatching { poseDetectorRef.getAndSet(null)?.close() }
            runCatching { objectDetectorRef.getAndSet(null)?.close() }
            runCatching { mpFaceRef.getAndSet(null)?.close() }
            runCatching { mpHandRef.getAndSet(null)?.close() }
            runCatching { mpPoseRef.getAndSet(null)?.close() }
            runCatching { mpGestureRef.getAndSet(null)?.close() }
            camCtrl.stop()
        }
    }

    // Refresh camera info at ~2Hz while on camera page.
    LaunchedEffect(page) {
        if (page != 1) return@LaunchedEffect
        while (isActive && page == 1) {
            val opts =
                withContext(Dispatchers.Default) {
                    camCtrl.listCameras()
                }
            camOptions = opts
            camError = camCtrl.lastError
            if (camId.isBlank() && opts.isNotEmpty()) camId = opts.first().cameraId
            delay(500)
        }
    }

    // Stop camera when leaving camera page.
    LaunchedEffect(page) {
        if (page != 1) {
            camOn = false
            camProviderRef.get()?.unbindAll()
            camCtrl.stop()
        }
    }

    // If MediaPipe pose model changes, rebuild the PoseLandmarker.
    LaunchedEffect(mpPoseModel) {
        runCatching { mpPoseRef.getAndSet(null)?.close() }
    }

    // Start/stop CameraX preview + analysis when toggled.
    val lifecycleOwner = LocalLifecycleOwner.current
    DisposableEffect(camOn, camId, camResText, camFpsText, camPreviewView, visionLib, visionTask, mpPoseModel) {
        val pv = camPreviewView
        val granted =
            ContextCompat.checkSelfPermission(context, Manifest.permission.CAMERA) == PackageManager.PERMISSION_GRANTED

        // Always ensure we stop CameraX when toggled off / no view / no permission.
        if (!camOn || pv == null || !granted) {
            camProviderRef.get()?.unbindAll()
            onDispose { /* no-op */ }
        } else {
            val opt = camOptions.firstOrNull { it.cameraId == camId }
            val size =
                opt?.previewSizes?.firstOrNull { s -> "${s.width}x${s.height}" == camResText }
                    ?: opt?.previewSizes?.firstOrNull()
            val fpsRange =
                opt?.aeFpsRanges?.firstOrNull { r -> "${r.lower}-${r.upper}" == camFpsText }
                    ?: opt?.aeFpsRanges?.firstOrNull()

            val analysisExec =
                camAnalysisExecRef.get()
                    ?: Executors.newSingleThreadExecutor().also { camAnalysisExecRef.set(it) }

            val providerFuture = ProcessCameraProvider.getInstance(context)
            val mainExec = ContextCompat.getMainExecutor(context)
            var disposed = false

            providerFuture.addListener(
                {
                    val provider = runCatching { providerFuture.get() }.getOrNull() ?: return@addListener
                    camProviderRef.set(provider)
                    if (disposed) return@addListener

                    fun bindWithSelector(selector: CameraSelector) {
                        provider.unbindAll()

                        val previewBuilder = Preview.Builder()
                        val analysisBuilder =
                            ImageAnalysis.Builder()
                                .setBackpressureStrategy(ImageAnalysis.STRATEGY_KEEP_ONLY_LATEST)

                        if (size != null) {
                            runCatching { previewBuilder.setTargetResolution(size) }
                            runCatching { analysisBuilder.setTargetResolution(size) }
                        }
                        if (fpsRange != null) {
                            val extPrev = Camera2Interop.Extender(previewBuilder)
                            val extAna = Camera2Interop.Extender(analysisBuilder)
                            extPrev.setCaptureRequestOption(CaptureRequest.CONTROL_AE_TARGET_FPS_RANGE, fpsRange)
                            extAna.setCaptureRequestOption(CaptureRequest.CONTROL_AE_TARGET_FPS_RANGE, fpsRange)
                        }

                        val isFrontCamera = opt?.lensFacing == CameraCharacteristics.LENS_FACING_FRONT

                        val preview =
                            previewBuilder.build().also { it.setSurfaceProvider(pv.surfaceProvider) }
                        val analysis = analysisBuilder.build()

                        analysis.setAnalyzer(
                            analysisExec,
                            analyzer@{ imageProxy: ImageProxy ->
                                // Measured preview/analysis callback FPS (throttled to UI elsewhere).
                                camPreviewFpsRef.set(camPreviewRate.update(System.nanoTime()))

                                val lib = visionLib
                                val task = visionTask
                                if (lib == VisionLibrary.NONE || task == VisionTask.NONE) {
                                    visionOverlayRef.set(null)
                                    visionNoteRef.set(null)
                                    imageProxy.close()
                                    return@analyzer
                                }

                                val img = imageProxy.image
                                if (img == null) {
                                    imageProxy.close()
                                    return@analyzer
                                }

                                val rotDeg = imageProxy.imageInfo.rotationDegrees
                                val crop = imageProxy.cropRect
                                val baseW = crop.width()
                                val baseH = crop.height()
                                val (iw, ih) =
                                    if (rotDeg == 90 || rotDeg == 270) {
                                        baseH to baseW
                                    } else {
                                        baseW to baseH
                                    }
                                val mapper =
                                    tryCreateOverlayMapper(
                                        imageProxy = imageProxy,
                                        previewTransform = previewOutputTransformRef.get(),
                                        previewSize = previewViewSizeRef.get(),
                                    )

                                if (!visionBusy.compareAndSet(false, true)) {
                                    imageProxy.close()
                                    return@analyzer
                                }

                                when (lib) {
                                    VisionLibrary.MLKIT -> {
                                        val input = InputImage.fromMediaImage(img, rotDeg)
                                        when (task) {
                                            VisionTask.FACE -> {
                                                val detector =
                                                    faceDetectorRef.get()
                                                        ?: FaceDetection.getClient(
                                                            FaceDetectorOptions.Builder()
                                                                .setPerformanceMode(FaceDetectorOptions.PERFORMANCE_MODE_FAST)
                                                                .build()
                                                        ).also { faceDetectorRef.set(it) }

                                                detector.process(input)
                                                    .addOnSuccessListener(analysisExec) { faces: List<Face> ->
                                                        val boxes =
                                                            faces.mapNotNull { f ->
                                                                val r: Rect = f.boundingBox
                                                                val rect =
                                                                    mapper?.mapRectNorm(
                                                                        r.left.toFloat(),
                                                                        r.top.toFloat(),
                                                                        r.right.toFloat(),
                                                                        r.bottom.toFloat(),
                                                                    )
                                                                        ?: if (iw > 0 && ih > 0) {
                                                                            NormRect(
                                                                                left =
                                                                                    (r.left.toFloat() / iw.toFloat())
                                                                                        .coerceIn(0f, 1f),
                                                                                top =
                                                                                    (r.top.toFloat() / ih.toFloat())
                                                                                        .coerceIn(0f, 1f),
                                                                                right =
                                                                                    (r.right.toFloat() / iw.toFloat())
                                                                                        .coerceIn(0f, 1f),
                                                                                bottom =
                                                                                    (r.bottom.toFloat() / ih.toFloat())
                                                                                        .coerceIn(0f, 1f),
                                                                            )
                                                                        } else {
                                                                            null
                                                                        }
                                                                rect?.let { NormBox(it) }
                                                            }
                                                        val hz = visionRate.update(System.nanoTime())
                                                        visionOverlayRef.set(
                                                            VisionOverlayState(
                                                                imageWidth = iw,
                                                                imageHeight = ih,
                                                                boxes = boxes,
                                                                lines = emptyList(),
                                                                points = emptyList(),
                                                                label = "mlkit face: ${faces.size}",
                                                                procHz = hz,
                                                            )
                                                        )
                                                        visionNoteRef.set(null)
                                                    }
                                                    .addOnFailureListener(analysisExec) { e ->
                                                        visionNoteRef.set(
                                                            "ML Kit face failed: ${e.message ?: e::class.java.simpleName}"
                                                        )
                                                    }
                                                    .addOnCompleteListener(analysisExec) {
                                                        visionBusy.set(false)
                                                        imageProxy.close()
                                                    }
                                            }

                                            VisionTask.POSE -> {
                                                val detector =
                                                    poseDetectorRef.get()
                                                        ?: PoseDetection.getClient(
                                                            PoseDetectorOptions.Builder()
                                                                .setDetectorMode(PoseDetectorOptions.STREAM_MODE)
                                                                .build()
                                                        ).also { poseDetectorRef.set(it) }

                                                detector.process(input)
                                                    .addOnSuccessListener(analysisExec) { pose ->
                                                        fun mapPt(xPx: Float, yPx: Float): NormPoint? {
                                                            return mapper?.mapPointNorm(xPx, yPx)
                                                                ?: if (iw > 0 && ih > 0) {
                                                                    NormPoint(
                                                                        x = (xPx / iw.toFloat()).coerceIn(0f, 1f),
                                                                        y = (yPx / ih.toFloat()).coerceIn(0f, 1f),
                                                                    )
                                                                } else {
                                                                    null
                                                                }
                                                        }

                                                        fun getLandmark(type: Int): NormPoint? {
                                                            val lm = pose.getPoseLandmark(type) ?: return null
                                                            val p = lm.position
                                                            return mapPt(p.x, p.y)
                                                        }

                                                        val points =
                                                            pose.allPoseLandmarks.mapNotNull { lm ->
                                                                val p = lm.position
                                                                mapPt(p.x, p.y)
                                                            }

                                                        val connections =
                                                            listOf(
                                                                PoseLandmark.NOSE to PoseLandmark.LEFT_SHOULDER,
                                                                PoseLandmark.NOSE to PoseLandmark.RIGHT_SHOULDER,
                                                                PoseLandmark.LEFT_SHOULDER to PoseLandmark.RIGHT_SHOULDER,
                                                                PoseLandmark.LEFT_HIP to PoseLandmark.RIGHT_HIP,
                                                                PoseLandmark.LEFT_SHOULDER to PoseLandmark.LEFT_ELBOW,
                                                                PoseLandmark.LEFT_ELBOW to PoseLandmark.LEFT_WRIST,
                                                                PoseLandmark.RIGHT_SHOULDER to PoseLandmark.RIGHT_ELBOW,
                                                                PoseLandmark.RIGHT_ELBOW to PoseLandmark.RIGHT_WRIST,
                                                                PoseLandmark.LEFT_SHOULDER to PoseLandmark.LEFT_HIP,
                                                                PoseLandmark.RIGHT_SHOULDER to PoseLandmark.RIGHT_HIP,
                                                                PoseLandmark.LEFT_HIP to PoseLandmark.LEFT_KNEE,
                                                                PoseLandmark.LEFT_KNEE to PoseLandmark.LEFT_ANKLE,
                                                                PoseLandmark.RIGHT_HIP to PoseLandmark.RIGHT_KNEE,
                                                                PoseLandmark.RIGHT_KNEE to PoseLandmark.RIGHT_ANKLE,
                                                            )

                                                        val lines =
                                                            connections.mapNotNull { (a, b) ->
                                                                val pa = getLandmark(a) ?: return@mapNotNull null
                                                                val pb = getLandmark(b) ?: return@mapNotNull null
                                                                NormLine(pa.x, pa.y, pb.x, pb.y)
                                                            }
                                                        val hz = visionRate.update(System.nanoTime())
                                                        visionOverlayRef.set(
                                                            VisionOverlayState(
                                                                imageWidth = iw,
                                                                imageHeight = ih,
                                                                boxes = emptyList(),
                                                                lines = lines,
                                                                points = points,
                                                                label = "mlkit pose: lms=${pose.allPoseLandmarks.size}",
                                                                procHz = hz,
                                                            )
                                                        )
                                                        visionNoteRef.set(null)
                                                    }
                                                    .addOnFailureListener(analysisExec) { e ->
                                                        visionNoteRef.set(
                                                            "ML Kit pose failed: ${e.message ?: e::class.java.simpleName}"
                                                        )
                                                    }
                                                    .addOnCompleteListener(analysisExec) {
                                                        visionBusy.set(false)
                                                        imageProxy.close()
                                                    }
                                            }

                                            VisionTask.OBJECT -> {
                                                val detector =
                                                    objectDetectorRef.get()
                                                        ?: ObjectDetection.getClient(
                                                            ObjectDetectorOptions.Builder()
                                                                .setDetectorMode(ObjectDetectorOptions.STREAM_MODE)
                                                                .enableMultipleObjects()
                                                                .enableClassification()
                                                                .build()
                                                        ).also { objectDetectorRef.set(it) }

                                                detector.process(input)
                                                    .addOnSuccessListener(analysisExec) { objs ->
                                                        val boxes =
                                                            objs.mapNotNull { o ->
                                                                val r = o.boundingBox
                                                                val rect =
                                                                    mapper?.mapRectNorm(
                                                                        r.left.toFloat(),
                                                                        r.top.toFloat(),
                                                                        r.right.toFloat(),
                                                                        r.bottom.toFloat(),
                                                                    )
                                                                        ?: if (iw > 0 && ih > 0) {
                                                                            NormRect(
                                                                                left =
                                                                                    (r.left.toFloat() / iw.toFloat())
                                                                                        .coerceIn(0f, 1f),
                                                                                top =
                                                                                    (r.top.toFloat() / ih.toFloat())
                                                                                        .coerceIn(0f, 1f),
                                                                                right =
                                                                                    (r.right.toFloat() / iw.toFloat())
                                                                                        .coerceIn(0f, 1f),
                                                                                bottom =
                                                                                    (r.bottom.toFloat() / ih.toFloat())
                                                                                        .coerceIn(0f, 1f),
                                                                            )
                                                                        } else {
                                                                            null
                                                                        }
                                                                val lab = o.labels.firstOrNull()
                                                                val text =
                                                                    if (lab == null) {
                                                                        "obj"
                                                                    } else {
                                                                        "${lab.text} ${(lab.confidence * 100f).toInt()}%"
                                                                    }
                                                                rect?.let { NormBox(it, text) }
                                                            }
                                                        val hz = visionRate.update(System.nanoTime())
                                                        visionOverlayRef.set(
                                                            VisionOverlayState(
                                                                imageWidth = iw,
                                                                imageHeight = ih,
                                                                boxes = boxes,
                                                                lines = emptyList(),
                                                                points = emptyList(),
                                                                label = "mlkit obj: ${objs.size}",
                                                                procHz = hz,
                                                            )
                                                        )
                                                        visionNoteRef.set(null)
                                                    }
                                                    .addOnFailureListener(analysisExec) { e ->
                                                        visionNoteRef.set(
                                                            "ML Kit object failed: ${e.message ?: e::class.java.simpleName}"
                                                        )
                                                    }
                                                    .addOnCompleteListener(analysisExec) {
                                                        visionBusy.set(false)
                                                        imageProxy.close()
                                                    }
                                            }

                                            VisionTask.HAND -> {
                                                // ML Kit hand is not part of this app's ML Kit set.
                                                visionNoteRef.set("ML Kit HAND not available (use MediaPipe HAND)")
                                                visionBusy.set(false)
                                                imageProxy.close()
                                            }

                                            else -> {
                                                visionBusy.set(false)
                                                imageProxy.close()
                                            }
                                        }
                                    }

                                    VisionLibrary.MEDIAPIPE -> {
                                        try {
                                            val bmp0 =
                                                imageProxyToBitmap(imageProxy)
                                                    ?: throw IllegalStateException("failed to convert YUV frame to Bitmap")
                                            val bmp =
                                                rotateAndMirrorBitmap(
                                                    src = bmp0,
                                                    rotationDegrees = rotDeg,
                                                    mirrorHorizontal = isFrontCamera,
                                                )
                                            val bmpArgb =
                                                if (bmp.config == Bitmap.Config.ARGB_8888) bmp else bmp.copy(Bitmap.Config.ARGB_8888, false)
                                            val mpImage = BitmapImageBuilder(bmpArgb).build()
                                            val mpW = bmpArgb.width
                                            val mpH = bmpArgb.height

                                            when (task) {
                                                VisionTask.FACE -> {
                                                    val landmarker =
                                                        mpFaceRef.get()
                                                            ?: run {
                                                                val base =
                                                                    BaseOptions.builder()
                                                                        .setModelAssetPath(
                                                                            "mediapipe_models/face_landmarker.task"
                                                                        )
                                                                        .build()
                                                                val opts =
                                                                    FaceLandmarker.FaceLandmarkerOptions.builder()
                                                                        .setBaseOptions(base)
                                                                        .setRunningMode(RunningMode.IMAGE)
                                                                        .build()
                                                                FaceLandmarker.createFromOptions(context, opts).also {
                                                                    mpFaceRef.set(it)
                                                                }
                                                            }
                                                    val res: FaceLandmarkerResult = landmarker.detect(mpImage)
                                                    fun boxFromPoints(ps: List<NormPoint>): NormRect? {
                                                        if (ps.isEmpty()) return null
                                                        val xs = ps.map { it.x }
                                                        val ys = ps.map { it.y }
                                                        return NormRect(
                                                            left = xs.min().coerceIn(0f, 1f),
                                                            top = ys.min().coerceIn(0f, 1f),
                                                            right = xs.max().coerceIn(0f, 1f),
                                                            bottom = ys.max().coerceIn(0f, 1f),
                                                        )
                                                    }

                                                    val points = mutableListOf<NormPoint>()
                                                    val boxes = mutableListOf<NormBox>()
                                                    for (lms in res.faceLandmarks()) {
                                                        val mapped =
                                                            lms.mapNotNull { lm ->
                                                                NormPoint(
                                                                    x = lm.x().coerceIn(0f, 1f),
                                                                    y = lm.y().coerceIn(0f, 1f),
                                                                )
                                                            }
                                                        // Face has many points; thin for UI.
                                                        mapped.forEachIndexed { idx, p -> if (idx % 3 == 0) points.add(p) }
                                                        boxFromPoints(mapped)?.let { boxes.add(NormBox(it, "face")) }
                                                    }
                                                    val hz = visionRate.update(System.nanoTime())
                                                    visionOverlayRef.set(
                                                        VisionOverlayState(
                                                            imageWidth = mpW,
                                                            imageHeight = mpH,
                                                            boxes = boxes,
                                                            lines = emptyList(),
                                                            points = points,
                                                            label = "mp face: ${boxes.size}",
                                                            procHz = hz,
                                                        )
                                                    )
                                                    visionNoteRef.set(null)
                                                }

                                                VisionTask.GESTURE -> {
                                                    val recognizer =
                                                        mpGestureRef.get()
                                                            ?: run {
                                                                val base =
                                                                    BaseOptions.builder()
                                                                        .setModelAssetPath(
                                                                            "mediapipe_models/gesture_recognizer.task"
                                                                        )
                                                                        .build()
                                                                val opts =
                                                                    GestureRecognizer.GestureRecognizerOptions.builder()
                                                                        .setBaseOptions(base)
                                                                        .setRunningMode(RunningMode.IMAGE)
                                                                        .build()
                                                                GestureRecognizer.createFromOptions(context, opts).also {
                                                                    mpGestureRef.set(it)
                                                                }
                                                            }

                                                    val res: GestureRecognizerResult = recognizer.recognize(mpImage)

                                                    fun boxFromPoints(ps: List<NormPoint>): NormRect? {
                                                        if (ps.isEmpty()) return null
                                                        val xs = ps.map { it.x }
                                                        val ys = ps.map { it.y }
                                                        return NormRect(
                                                            left = xs.min().coerceIn(0f, 1f),
                                                            top = ys.min().coerceIn(0f, 1f),
                                                            right = xs.max().coerceIn(0f, 1f),
                                                            bottom = ys.max().coerceIn(0f, 1f),
                                                        )
                                                    }

                                                    val points = mutableListOf<NormPoint>()
                                                    val lines = mutableListOf<NormLine>()
                                                    val boxes = mutableListOf<NormBox>()

                                                    // res.landmarks(): List<List<NormalizedLandmark>> (per detected hand)
                                                    // res.gestures():  List<List<Category>> (per detected hand; can be empty or "None")
                                                    for ((handIdx, lms) in res.landmarks().withIndex()) {
                                                        val handGesture =
                                                            res.gestures().getOrNull(handIdx)?.firstOrNull()
                                                        val handText =
                                                            if (handGesture == null) {
                                                                "None"
                                                            } else {
                                                                val name = handGesture.categoryName()
                                                                val scorePct = (handGesture.score() * 100f).toInt()
                                                                "${name} ${scorePct}%"
                                                            }

                                                        val idxToPt =
                                                            lms.mapIndexedNotNull { idx, lm ->
                                                                val p =
                                                                    NormPoint(
                                                                        x = lm.x().coerceIn(0f, 1f),
                                                                        y = lm.y().coerceIn(0f, 1f),
                                                                    )
                                                                idx to p
                                                            }.toMap()
                                                        points.addAll(idxToPt.values)
                                                        for (c in HandLandmarker.HAND_CONNECTIONS) {
                                                            val a = idxToPt[c.start()] ?: continue
                                                            val b = idxToPt[c.end()] ?: continue
                                                            lines.add(NormLine(a.x, a.y, b.x, b.y))
                                                        }
                                                        boxFromPoints(idxToPt.values.toList())?.let { boxes.add(NormBox(it, handText)) }
                                                    }

                                                    val topGesture =
                                                        res.gestures().firstOrNull()?.firstOrNull()
                                                    val gestureText =
                                                        if (topGesture == null) {
                                                            "mp gesture: (none)"
                                                        } else {
                                                            val name = topGesture.categoryName()
                                                            val scorePct = (topGesture.score() * 100f).toInt()
                                                            "mp gesture: $name ${scorePct}%"
                                                        }

                                                    val hz = visionRate.update(System.nanoTime())
                                                    visionOverlayRef.set(
                                                        VisionOverlayState(
                                                            imageWidth = mpW,
                                                            imageHeight = mpH,
                                                            boxes = boxes,
                                                            lines = lines,
                                                            points = points,
                                                            label = gestureText,
                                                            procHz = hz,
                                                        )
                                                    )
                                                    visionNoteRef.set(null)
                                                }

                                                VisionTask.HAND -> {
                                                    val landmarker =
                                                        mpHandRef.get()
                                                            ?: run {
                                                                val base =
                                                                    BaseOptions.builder()
                                                                        .setModelAssetPath(
                                                                            "mediapipe_models/hand_landmarker.task"
                                                                        )
                                                                        .build()
                                                                val opts =
                                                                    HandLandmarker.HandLandmarkerOptions.builder()
                                                                        .setBaseOptions(base)
                                                                        .setRunningMode(RunningMode.IMAGE)
                                                                        .build()
                                                                HandLandmarker.createFromOptions(context, opts).also {
                                                                    mpHandRef.set(it)
                                                                }
                                                            }
                                                    val res: HandLandmarkerResult = landmarker.detect(mpImage)
                                                    fun boxFromPoints(ps: List<NormPoint>): NormRect? {
                                                        if (ps.isEmpty()) return null
                                                        val xs = ps.map { it.x }
                                                        val ys = ps.map { it.y }
                                                        return NormRect(
                                                            left = xs.min().coerceIn(0f, 1f),
                                                            top = ys.min().coerceIn(0f, 1f),
                                                            right = xs.max().coerceIn(0f, 1f),
                                                            bottom = ys.max().coerceIn(0f, 1f),
                                                        )
                                                    }

                                                    val points = mutableListOf<NormPoint>()
                                                    val lines = mutableListOf<NormLine>()
                                                    val boxes = mutableListOf<NormBox>()
                                                    for (lms in res.landmarks()) {
                                                        val idxToPt =
                                                            lms.mapIndexedNotNull { idx, lm ->
                                                                val p =
                                                                    NormPoint(
                                                                        x = lm.x().coerceIn(0f, 1f),
                                                                        y = lm.y().coerceIn(0f, 1f),
                                                                    )
                                                                idx to p
                                                            }.toMap()
                                                        points.addAll(idxToPt.values)
                                                        for (c in HandLandmarker.HAND_CONNECTIONS) {
                                                            val a = idxToPt[c.start()] ?: continue
                                                            val b = idxToPt[c.end()] ?: continue
                                                            lines.add(NormLine(a.x, a.y, b.x, b.y))
                                                        }
                                                        boxFromPoints(idxToPt.values.toList())?.let { boxes.add(NormBox(it, "hand")) }
                                                    }
                                                    val hz = visionRate.update(System.nanoTime())
                                                    visionOverlayRef.set(
                                                        VisionOverlayState(
                                                            imageWidth = mpW,
                                                            imageHeight = mpH,
                                                            boxes = boxes,
                                                            lines = lines,
                                                            points = points,
                                                            label = "mp hand: ${boxes.size}",
                                                            procHz = hz,
                                                        )
                                                    )
                                                    visionNoteRef.set(null)
                                                }

                                                VisionTask.POSE -> {
                                                    val modelPath =
                                                        when (mpPoseModel) {
                                                            MediaPipePoseModel.LITE ->
                                                                "mediapipe_models/pose_landmarker_lite.task"
                                                            MediaPipePoseModel.FULL ->
                                                                "mediapipe_models/pose_landmarker_full.task"
                                                            MediaPipePoseModel.HEAVY ->
                                                                "mediapipe_models/pose_landmarker_heavy.task"
                                                        }
                                                    val landmarker =
                                                        mpPoseRef.get()
                                                            ?: run {
                                                                val base =
                                                                    BaseOptions.builder()
                                                                        .setModelAssetPath(modelPath)
                                                                        .build()
                                                                val opts =
                                                                    PoseLandmarker.PoseLandmarkerOptions.builder()
                                                                        .setBaseOptions(base)
                                                                        .setRunningMode(RunningMode.IMAGE)
                                                                        .build()
                                                                PoseLandmarker.createFromOptions(context, opts).also {
                                                                    mpPoseRef.set(it)
                                                                }
                                                            }

                                                    val res: PoseLandmarkerResult = landmarker.detect(mpImage)
                                                    fun boxFromPoints(ps: List<NormPoint>): NormRect? {
                                                        if (ps.isEmpty()) return null
                                                        val xs = ps.map { it.x }
                                                        val ys = ps.map { it.y }
                                                        return NormRect(
                                                            left = xs.min().coerceIn(0f, 1f),
                                                            top = ys.min().coerceIn(0f, 1f),
                                                            right = xs.max().coerceIn(0f, 1f),
                                                            bottom = ys.max().coerceIn(0f, 1f),
                                                        )
                                                    }

                                                    val points = mutableListOf<NormPoint>()
                                                    val lines = mutableListOf<NormLine>()
                                                    val boxes = mutableListOf<NormBox>()
                                                    for (lms in res.landmarks()) {
                                                        val idxToPt =
                                                            lms.mapIndexedNotNull { idx, lm ->
                                                                val p =
                                                                    NormPoint(
                                                                        x = lm.x().coerceIn(0f, 1f),
                                                                        y = lm.y().coerceIn(0f, 1f),
                                                                    )
                                                                idx to p
                                                            }.toMap()
                                                        points.addAll(idxToPt.values)
                                                        for (c in PoseLandmarker.POSE_LANDMARKS) {
                                                            val a = idxToPt[c.start()] ?: continue
                                                            val b = idxToPt[c.end()] ?: continue
                                                            lines.add(NormLine(a.x, a.y, b.x, b.y))
                                                        }
                                                        boxFromPoints(idxToPt.values.toList())?.let { boxes.add(NormBox(it, "pose")) }
                                                    }
                                                    val hz = visionRate.update(System.nanoTime())
                                                    visionOverlayRef.set(
                                                        VisionOverlayState(
                                                            imageWidth = mpW,
                                                            imageHeight = mpH,
                                                            boxes = boxes,
                                                            lines = lines,
                                                            points = points,
                                                            label = "mp pose: ${boxes.size}",
                                                            procHz = hz,
                                                        )
                                                    )
                                                    visionNoteRef.set(null)
                                                }

                                                VisionTask.OBJECT -> {
                                                    visionNoteRef.set(
                                                        "MediaPipe OBJECT requires an object detector .task (not added yet)"
                                                    )
                                                    visionOverlayRef.set(null)
                                                }

                                                else -> {
                                                    visionOverlayRef.set(null)
                                                }
                                            }
                                        } catch (e: Exception) {
                                            visionNoteRef.set(
                                                "MediaPipe failed: ${e.message ?: e::class.java.simpleName}"
                                            )
                                        } finally {
                                            visionBusy.set(false)
                                            imageProxy.close()
                                        }
                                    }

                                    else -> {
                                        visionBusy.set(false)
                                        imageProxy.close()
                                    }
                                }
                            },
                        )

                        provider.bindToLifecycle(lifecycleOwner, selector, preview, analysis)
                    }

                    try {
                        val selector =
                            if (camId.isNotBlank()) {
                                CameraSelector.Builder()
                                    .addCameraFilter { infos ->
                                        infos.filter { info ->
                                            runCatching {
                                                Camera2CameraInfo.from(info).cameraId == camId
                                            }.getOrDefault(false)
                                        }
                                    }
                                    .build()
                            } else {
                                CameraSelector.DEFAULT_BACK_CAMERA
                            }

                        runCatching { bindWithSelector(selector) }.recoverCatching {
                            // If camera-id selection fails, fall back to a default selector.
                            bindWithSelector(CameraSelector.DEFAULT_BACK_CAMERA)
                        }.getOrThrow()

                        camError = null
                    } catch (e: Exception) {
                        camError = "CameraX bind failed: ${e.message ?: e::class.java.simpleName}"
                    }
                },
                mainExec,
            )

            onDispose {
                disposed = true
                camProviderRef.get()?.unbindAll()
            }
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
            camPreviewFps = camPreviewFpsRef.get()
            visionOverlayUi = visionOverlayRef.get()
            visionNote = visionNoteRef.get()
            val pv = camPreviewView
            if (pv != null) {
                previewViewSizeRef.set(pv.width to pv.height)
                runCatching { previewOutputTransformRef.set(pv.outputTransform) }
            } else {
                previewViewSizeRef.set(null)
                previewOutputTransformRef.set(null)
            }
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

                val libOptions = listOf(VisionLibrary.NONE, VisionLibrary.MLKIT, VisionLibrary.MEDIAPIPE)
                val taskOptions =
                    when (visionLib) {
                        VisionLibrary.NONE -> listOf(VisionTask.NONE)
                        VisionLibrary.MLKIT -> listOf(VisionTask.NONE, VisionTask.FACE, VisionTask.POSE, VisionTask.OBJECT)
                        VisionLibrary.MEDIAPIPE -> listOf(VisionTask.NONE, VisionTask.FACE, VisionTask.POSE, VisionTask.OBJECT, VisionTask.HAND, VisionTask.GESTURE)
                    }
                DropdownSelector(
                    label = "Vision library",
                    value = visionLib.name,
                    options = libOptions.map { it.name },
                    onSelect = { visionLib = runCatching { VisionLibrary.valueOf(it) }.getOrDefault(VisionLibrary.NONE); visionTask = VisionTask.NONE },
                )
                DropdownSelector(
                    label = "Vision task",
                    value = visionTask.name,
                    options = taskOptions.map { it.name },
                    enabled = visionLib != VisionLibrary.NONE,
                    onSelect = { visionTask = runCatching { VisionTask.valueOf(it) }.getOrDefault(VisionTask.NONE) },
                )
                if (visionLib == VisionLibrary.MEDIAPIPE && visionTask == VisionTask.POSE) {
                    DropdownSelector(
                        label = "MediaPipe pose model",
                        value = mpPoseModel.name,
                        options = MediaPipePoseModel.entries.map { it.name },
                        onSelect = { mpPoseModel = runCatching { MediaPipePoseModel.valueOf(it) }.getOrDefault(MediaPipePoseModel.LITE) },
                    )
                }

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
                        factory = { ctx ->
                            PreviewView(ctx).also { view ->
                                view.scaleType = PreviewView.ScaleType.FIT_CENTER
                                camPreviewView = view
                            }
                        },
                        modifier = Modifier.fillMaxSize(),
                    )
                    VisionOverlay(overlay = visionOverlayUi)
                }

                MonoBlock(
                    lines =
                        listOf(
                            formatHz("visionHz(Hz)", visionOverlayUi?.procHz),
                            "vision label : ${visionOverlayUi?.label ?: "?"}",
                            "vision note  : ${visionNote ?: "OK"}",
                        ),
                )

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

private fun normRectFromLandmarks(lms: List<NormalizedLandmark>): NormRect? {
    if (lms.isEmpty()) return null
    var minX = Float.POSITIVE_INFINITY
    var minY = Float.POSITIVE_INFINITY
    var maxX = Float.NEGATIVE_INFINITY
    var maxY = Float.NEGATIVE_INFINITY
    for (lm in lms) {
        val x = lm.x()
        val y = lm.y()
        if (!x.isFinite() || !y.isFinite()) continue
        if (x < minX) minX = x
        if (y < minY) minY = y
        if (x > maxX) maxX = x
        if (y > maxY) maxY = y
    }
    if (!minX.isFinite() || !minY.isFinite() || !maxX.isFinite() || !maxY.isFinite()) return null
    return NormRect(
        left = minX.coerceIn(0f, 1f),
        top = minY.coerceIn(0f, 1f),
        right = maxX.coerceIn(0f, 1f),
        bottom = maxY.coerceIn(0f, 1f),
    )
}

private class OverlayMapper(
    private val ct: androidx.camera.view.transform.CoordinateTransform,
    private val viewW: Int,
    private val viewH: Int,
) {
    fun mapPointNorm(xPx: Float, yPx: Float): NormPoint? {
        if (viewW <= 0 || viewH <= 0) return null
        val pts = floatArrayOf(xPx, yPx)
        ct.mapPoints(pts)
        val x = (pts[0] / viewW.toFloat()).coerceIn(0f, 1f)
        val y = (pts[1] / viewH.toFloat()).coerceIn(0f, 1f)
        return NormPoint(x, y)
    }

    fun mapRectNorm(lPx: Float, tPx: Float, rPx: Float, bPx: Float): NormRect? {
        if (viewW <= 0 || viewH <= 0) return null
        val pts = floatArrayOf(lPx, tPx, rPx, tPx, rPx, bPx, lPx, bPx)
        ct.mapPoints(pts)
        var minX = Float.POSITIVE_INFINITY
        var minY = Float.POSITIVE_INFINITY
        var maxX = Float.NEGATIVE_INFINITY
        var maxY = Float.NEGATIVE_INFINITY
        var i = 0
        while (i < pts.size) {
            val x = pts[i]
            val y = pts[i + 1]
            if (x < minX) minX = x
            if (y < minY) minY = y
            if (x > maxX) maxX = x
            if (y > maxY) maxY = y
            i += 2
        }
        return NormRect(
            left = (minX / viewW.toFloat()).coerceIn(0f, 1f),
            top = (minY / viewH.toFloat()).coerceIn(0f, 1f),
            right = (maxX / viewW.toFloat()).coerceIn(0f, 1f),
            bottom = (maxY / viewH.toFloat()).coerceIn(0f, 1f),
        )
    }
}

private fun tryCreateOverlayMapper(
    imageProxy: ImageProxy,
    previewTransform: androidx.camera.view.transform.OutputTransform?,
    previewSize: Pair<Int, Int>?,
): OverlayMapper? {
    if (previewTransform == null || previewSize == null) return null
    val (vw, vh) = previewSize
    if (vw <= 0 || vh <= 0) return null
    val src =
        androidx.camera.view.transform.ImageProxyTransformFactory().apply {
            setUsingCropRect(true)
            setUsingRotationDegrees(true)
        }.getOutputTransform(imageProxy)
    val ct = androidx.camera.view.transform.CoordinateTransform(src, previewTransform)
    return OverlayMapper(ct = ct, viewW = vw, viewH = vh)
}

private fun imageProxyToBitmap(imageProxy: ImageProxy, jpegQuality: Int = 80): Bitmap? {
    val image = imageProxy.image ?: return null
    val crop = Rect(imageProxy.cropRect)
    // Align crop to even pixels for chroma planes.
    crop.left = crop.left and 1.inv()
    crop.top = crop.top and 1.inv()
    crop.right = crop.right and 1.inv()
    crop.bottom = crop.bottom and 1.inv()
    val width = crop.width()
    val height = crop.height()
    if (width <= 0 || height <= 0) return null

    val nv21 = yuv420888ToNv21Cropped(image, crop) ?: return null
    val yuvImage = YuvImage(nv21, ImageFormat.NV21, width, height, null)
    val out = ByteArrayOutputStream()
    if (!yuvImage.compressToJpeg(Rect(0, 0, width, height), jpegQuality.coerceIn(1, 100), out)) {
        return null
    }
    val bytes = out.toByteArray()
    return BitmapFactory.decodeByteArray(bytes, 0, bytes.size)
}

private fun rotateAndMirrorBitmap(src: Bitmap, rotationDegrees: Int, mirrorHorizontal: Boolean): Bitmap {
    val rot = ((rotationDegrees % 360) + 360) % 360
    if (rot == 0 && !mirrorHorizontal) return src

    val m =
        android.graphics.Matrix().apply {
            if (rot != 0) postRotate(rot.toFloat())
            if (mirrorHorizontal) {
                // Mirror around the center of the image.
                postScale(-1f, 1f, src.width / 2f, src.height / 2f)
            }
        }
    return Bitmap.createBitmap(src, 0, 0, src.width, src.height, m, true)
}

private fun yuv420888ToNv21Cropped(image: android.media.Image, crop: Rect): ByteArray? {
    val width = crop.width()
    val height = crop.height()
    if (width <= 0 || height <= 0) return null
    if (image.format != ImageFormat.YUV_420_888) return null

    val yPlane = image.planes.getOrNull(0) ?: return null
    val uPlane = image.planes.getOrNull(1) ?: return null
    val vPlane = image.planes.getOrNull(2) ?: return null

    val yBuf = yPlane.buffer.duplicate()
    val uBuf = uPlane.buffer.duplicate()
    val vBuf = vPlane.buffer.duplicate()

    val yRowStride = yPlane.rowStride
    val yPixelStride = yPlane.pixelStride
    val uRowStride = uPlane.rowStride
    val uPixelStride = uPlane.pixelStride
    val vRowStride = vPlane.rowStride
    val vPixelStride = vPlane.pixelStride

    val out = ByteArray(width * height * 3 / 2)
    var outPos = 0

    // Y plane
    for (row in 0 until height) {
        val srcRow = crop.top + row
        val rowOffset = srcRow * yRowStride
        for (col in 0 until width) {
            val srcCol = crop.left + col
            out[outPos++] = yBuf.get(rowOffset + srcCol * yPixelStride)
        }
    }

    // VU plane (NV21)
    val chromaHeight = height / 2
    val chromaWidth = width / 2
    for (row in 0 until chromaHeight) {
        val srcRow = (crop.top / 2) + row
        val uRowOffset = srcRow * uRowStride
        val vRowOffset = srcRow * vRowStride
        for (col in 0 until chromaWidth) {
            val srcCol = (crop.left / 2) + col
            val uIndex = uRowOffset + srcCol * uPixelStride
            val vIndex = vRowOffset + srcCol * vPixelStride
            out[outPos++] = vBuf.get(vIndex)
            out[outPos++] = uBuf.get(uIndex)
        }
    }

    return out
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