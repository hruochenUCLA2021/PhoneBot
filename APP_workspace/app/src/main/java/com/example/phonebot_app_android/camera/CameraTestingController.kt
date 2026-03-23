package com.example.phonebot_app_android.camera

import android.annotation.SuppressLint
import android.content.Context
import android.graphics.SurfaceTexture
import android.hardware.camera2.CameraAccessException
import android.hardware.camera2.CameraCaptureSession
import android.hardware.camera2.CameraCharacteristics
import android.hardware.camera2.CameraDevice
import android.hardware.camera2.CameraMetadata
import android.hardware.camera2.CameraManager
import android.hardware.camera2.CaptureRequest
import android.hardware.camera2.TotalCaptureResult
import android.hardware.camera2.params.StreamConfigurationMap
import android.media.Image
import android.media.ImageReader
import android.os.Handler
import android.os.HandlerThread
import android.util.Log
import android.util.Range
import android.util.Size
import android.view.Surface
import android.view.TextureView
import android.graphics.ImageFormat

data class CameraOption(
    val cameraId: String,
    val lensFacing: Int?,
    val sensorOrientationDeg: Int?,
    val physicalCameraIds: List<String>,
    val capabilities: List<Int>,
    val aeFpsRanges: List<Range<Int>>,
    val previewSizes: List<Size>,
)

class CameraTestingController(private val context: Context) {
    private val camManager = context.getSystemService(Context.CAMERA_SERVICE) as CameraManager

    // Background thread for camera callbacks / session work.
    private var camThread: HandlerThread? = null
    private var camHandler: Handler? = null

    private var camDevice: CameraDevice? = null
    private var session: CameraCaptureSession? = null
    private var imageReader: ImageReader? = null

    @Volatile var lastError: String? = null
        private set

    // Updated from camera callback thread.
    @Volatile var previewFpsHz: Float? = null
        private set

    /**
     * Optional analysis callback (called on the camera background thread).
     * The receiver MUST close the [Image] (or close it later after async processing).
     */
    @Volatile var onFrame: ((image: Image, rotationDegrees: Int) -> Unit)? = null

    private var fpsCount: Int = 0
    private var fpsLastReportNs: Long = 0L

    fun listCameras(): List<CameraOption> {
        val out = mutableListOf<CameraOption>()
        val ids =
            try {
                camManager.cameraIdList.toList()
            } catch (e: CameraAccessException) {
                lastError = "cameraIdList failed: ${e.reason}"
                return emptyList()
            } catch (t: Throwable) {
                lastError = "cameraIdList failed: ${t.message ?: t::class.java.simpleName}"
                return emptyList()
            }

        for (id in ids) {
            try {
                val ch = camManager.getCameraCharacteristics(id)
                val lensFacing = ch.get(CameraCharacteristics.LENS_FACING)
                val orientation = ch.get(CameraCharacteristics.SENSOR_ORIENTATION)
                val physicalIds = runCatching { ch.physicalCameraIds.toList() }.getOrNull() ?: emptyList()
                val caps = ch.get(CameraCharacteristics.REQUEST_AVAILABLE_CAPABILITIES)?.toList() ?: emptyList()
                val fpsRanges = ch.get(CameraCharacteristics.CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES)?.toList() ?: emptyList()
                val map: StreamConfigurationMap? = ch.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP)
                val sizes =
                    map
                        ?.getOutputSizes(SurfaceTexture::class.java)
                        ?.toList()
                        ?.sortedWith(compareBy({ it.width }, { it.height }))
                        ?: emptyList()
                out.add(
                    CameraOption(
                        cameraId = id,
                        lensFacing = lensFacing,
                        sensorOrientationDeg = orientation,
                        physicalCameraIds = physicalIds,
                        capabilities = caps,
                        aeFpsRanges = fpsRanges,
                        previewSizes = sizes,
                    )
                )
            } catch (e: CameraAccessException) {
                // Skip this camera but keep listing others.
                lastError = "getCameraCharacteristics($id) failed: ${e.reason}"
            } catch (t: Throwable) {
                lastError = "getCameraCharacteristics($id) failed: ${t.message ?: t::class.java.simpleName}"
            }
        }
        return out
    }

    private fun ensureThread() {
        if (camThread != null && camHandler != null) return
        camThread = HandlerThread("CameraTestingThread").also {
            it.start()
            camHandler = Handler(it.looper)
        }
    }

    fun stop() {
        runCatching { session?.close() }
        session = null
        runCatching { camDevice?.close() }
        camDevice = null
        runCatching { imageReader?.close() }
        imageReader = null
        previewFpsHz = null
        fpsCount = 0
        fpsLastReportNs = 0L

        camThread?.quitSafely()
        camThread = null
        camHandler = null
    }

    data class StartConfig(
        val cameraId: String,
        val targetSize: Size,
        val targetFpsRange: Range<Int>?,
        val rotationDegrees: Int = 0,
        val enableAnalysis: Boolean = true,
    )

    /**
     * Start a preview streaming session to the given TextureView.
     *
     * Call this only after CAMERA permission is granted.
     */
    @SuppressLint("MissingPermission")
    fun startPreview(textureView: TextureView, cfg: StartConfig) {
        stop()
        ensureThread()
        lastError = null

        val handler = camHandler ?: return

        val listener =
            object : TextureView.SurfaceTextureListener {
                override fun onSurfaceTextureAvailable(st: SurfaceTexture, width: Int, height: Int) {
                    openWithSurface(st, cfg, handler)
                }

                override fun onSurfaceTextureSizeChanged(st: SurfaceTexture, width: Int, height: Int) = Unit

                override fun onSurfaceTextureDestroyed(st: SurfaceTexture): Boolean {
                    // Stop camera when UI surface is destroyed.
                    stop()
                    return true
                }

                override fun onSurfaceTextureUpdated(st: SurfaceTexture) = Unit
            }

        if (textureView.isAvailable) {
            openWithSurface(textureView.surfaceTexture ?: return, cfg, handler)
        } else {
            textureView.surfaceTextureListener = listener
        }
    }

    @SuppressLint("MissingPermission")
    private fun openWithSurface(st: SurfaceTexture, cfg: StartConfig, handler: Handler) {
        try {
            st.setDefaultBufferSize(cfg.targetSize.width, cfg.targetSize.height)
        } catch (t: Throwable) {
            lastError = "setDefaultBufferSize failed: ${t.message ?: t::class.java.simpleName}"
        }

        val previewSurface = Surface(st)

        val readerSurface: Surface? =
            if (cfg.enableAnalysis) {
                runCatching {
                    ImageReader.newInstance(cfg.targetSize.width, cfg.targetSize.height, ImageFormat.YUV_420_888, 2).also { r ->
                        imageReader = r
                        r.setOnImageAvailableListener(
                            { rr ->
                                val img = runCatching { rr.acquireLatestImage() }.getOrNull()
                                if (img == null) return@setOnImageAvailableListener
                                val cb = onFrame
                                if (cb != null) {
                                    try {
                                        cb(img, cfg.rotationDegrees)
                                    } catch (t: Throwable) {
                                        lastError = "onFrame failed: ${t.message ?: t::class.java.simpleName}"
                                        runCatching { img.close() }
                                    }
                                } else {
                                    runCatching { img.close() }
                                }
                            },
                            handler,
                        )
                    }
                }.onFailure { t ->
                    lastError = "ImageReader failed: ${t.message ?: t::class.java.simpleName}"
                }.getOrNull()?.surface
            } else {
                null
            }

        camManager.openCamera(
            cfg.cameraId,
            object : CameraDevice.StateCallback() {
                override fun onOpened(camera: CameraDevice) {
                    camDevice = camera
                    createSession(camera, previewSurface, readerSurface, cfg, handler)
                }

                override fun onDisconnected(camera: CameraDevice) {
                    lastError = "camera disconnected"
                    runCatching { camera.close() }
                    camDevice = null
                }

                override fun onError(camera: CameraDevice, error: Int) {
                    lastError = "camera error=$error"
                    runCatching { camera.close() }
                    camDevice = null
                }
            },
            handler,
        )
    }

    private fun createSession(
        camera: CameraDevice,
        previewSurface: Surface,
        readerSurface: Surface?,
        cfg: StartConfig,
        handler: Handler
    ) {
        try {
            val outputs = mutableListOf<Surface>()
            outputs.add(previewSurface)
            if (readerSurface != null) outputs.add(readerSurface)
            camera.createCaptureSession(
                outputs,
                object : CameraCaptureSession.StateCallback() {
                    override fun onConfigured(sess: CameraCaptureSession) {
                        session = sess
                        val req =
                            camera.createCaptureRequest(CameraDevice.TEMPLATE_PREVIEW).apply {
                                addTarget(previewSurface)
                                if (readerSurface != null) addTarget(readerSurface)
                                set(CaptureRequest.CONTROL_MODE, CameraMetadata.CONTROL_MODE_AUTO)
                                cfg.targetFpsRange?.also { set(CaptureRequest.CONTROL_AE_TARGET_FPS_RANGE, it) }
                            }

                        fpsCount = 0
                        fpsLastReportNs = 0L
                        previewFpsHz = null

                        sess.setRepeatingRequest(
                            req.build(),
                            object : CameraCaptureSession.CaptureCallback() {
                                override fun onCaptureCompleted(
                                    session: CameraCaptureSession,
                                    request: CaptureRequest,
                                    result: TotalCaptureResult
                                ) {
                                    val nowNs = System.nanoTime()
                                    if (fpsLastReportNs == 0L) {
                                        fpsLastReportNs = nowNs
                                        fpsCount = 0
                                    }
                                    fpsCount += 1
                                    val dt = nowNs - fpsLastReportNs
                                    if (dt >= 1_000_000_000L) {
                                        previewFpsHz = fpsCount.toFloat() * 1_000_000_000f / dt.toFloat()
                                        fpsCount = 0
                                        fpsLastReportNs = nowNs
                                    }
                                }
                            },
                            handler,
                        )
                    }

                    override fun onConfigureFailed(sess: CameraCaptureSession) {
                        lastError = "capture session configure failed"
                        runCatching { sess.close() }
                        session = null
                    }
                },
                handler,
            )
        } catch (t: Throwable) {
            lastError = "createSession failed: ${t.message ?: t::class.java.simpleName}"
        }
    }
}

