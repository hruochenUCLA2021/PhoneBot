# NOTE: ML Kit vs MediaPipe (CameraX) – common issues & fixes

## Summary (what happened in our app)

We moved the camera pipeline to **CameraX** (`PreviewView` + `ImageAnalysis`) and ran vision tasks in the analyzer:

- **ML Kit** worked immediately because it accepts **YUV_420_888** frames directly via `InputImage.fromMediaImage(image, rotationDegrees)`.
- **MediaPipe Tasks** initially failed and/or drew overlays incorrectly because:
  - **Input pixel format** expectations differ (MediaPipe Tasks often expects **RGBA/ARGB Bitmap** for the easiest path).
  - **Rotation / front-camera mirror** expectations differ depending on how you feed frames (sample code typically **rotates + mirrors the Bitmap BEFORE inference**).
  - **Preview overlay mapping** must account for **crop / rotation / letterboxing** done by `PreviewView`.

This note explains:

- The exact errors we saw
- Why they happened
- How we fixed them in our codebase (with line references)
- A “mental model” of the CameraX stream + ML Kit + MediaPipe pipelines


## The two key differences

### 1) Frame format: YUV vs RGBA/ARGB

- **CameraX `ImageAnalysis`** gives an `ImageProxy` whose `image` is usually **YUV_420_888**.
- **ML Kit**: happily consumes that via `InputImage.fromMediaImage(...)`.
- **MediaPipe Tasks**: easiest/most common Android samples convert to **ARGB_8888 Bitmap** and then build an `MPImage` from the Bitmap.

#### The error we hit

When we tried to feed a YUV `android.media.Image` directly to MediaPipe, we got:

- **`android media image must use rgba_8888 config`**

Meaning: the path we used expected an RGBA/ARGB-backed image, not YUV.

#### Fix

Convert the `ImageProxy` to a **Bitmap (ARGB_8888)**, then:

- `val mpImage = BitmapImageBuilder(bitmapArgb8888).build()`

In our code this conversion is done by:

```1664:1752:/media/hrc/T7_UBUNTU_ONLY/android_humanoid_all_files/PhoneBot/APP_workspace/app/src/main/java/com/example/phonebot_app_android/MainActivity.kt
private fun imageProxyToBitmap(imageProxy: ImageProxy, jpegQuality: Int = 80): Bitmap? {
    // Uses cropRect + YUV_420_888 -> NV21 -> JPEG -> Bitmap
}
```

Notes:

- We respect `imageProxy.cropRect` and align it to even pixels (needed for chroma planes).
- The conversion uses `NV21 -> YuvImage -> JPEG -> Bitmap` for simplicity.
  - This is not the fastest method, but it’s stable and easy to debug.


### 2) Rotation + mirroring (front camera)

**ML Kit path**:

- You pass `rotationDegrees` into `InputImage.fromMediaImage(image, rotDeg)`.
- ML Kit returns results in a consistent coordinate system for the rotated input.

**MediaPipe sample path (important)**:

Most MediaPipe Android samples do this for LIVE_STREAM:

- Convert frame to `Bitmap`
- **Rotate bitmap** by `imageProxy.imageInfo.rotationDegrees`
- **Mirror bitmap** if the front camera is used
- Run inference on this “display-corrected” bitmap
- Draw landmarks directly using `normalizedLandmark.x() * imageWidth`, `normalizedLandmark.y() * imageHeight`

If you don’t mirror/rotate the MediaPipe input the same way the sample does, you can get overlays that look “flipped” or “rotated wrong” even if the model itself is fine.

#### Fix in our code

We now rotate/mirror the bitmap **before** calling MediaPipe:

```793:809:/media/hrc/T7_UBUNTU_ONLY/android_humanoid_all_files/PhoneBot/APP_workspace/app/src/main/java/com/example/phonebot_app_android/MainActivity.kt
val bmp0 = imageProxyToBitmap(imageProxy) ?: ...
val bmp = rotateAndMirrorBitmap(src = bmp0, rotationDegrees = rotDeg, mirrorHorizontal = isFrontCamera)
val bmpArgb = if (bmp.config == Bitmap.Config.ARGB_8888) bmp else bmp.copy(Bitmap.Config.ARGB_8888, false)
val mpImage = BitmapImageBuilder(bmpArgb).build()
```

The helper that implements that behavior is:

```1686:1699:/media/hrc/T7_UBUNTU_ONLY/android_humanoid_all_files/PhoneBot/APP_workspace/app/src/main/java/com/example/phonebot_app_android/MainActivity.kt
private fun rotateAndMirrorBitmap(src: Bitmap, rotationDegrees: Int, mirrorHorizontal: Boolean): Bitmap {
    // Matrix postRotate + optional postScale(-1,1) about center
}
```


## Overlay mapping: why ML Kit looked correct but MediaPipe looked wrong

This is the subtle part.

### The preview is transformed

`PreviewView` is not always a 1:1 pixel mapping of the camera buffer:

- CameraX can apply **cropRect**
- `PreviewView` can **scale** and **letterbox** the preview to fit your UI aspect ratio
- Rotation is applied so the preview looks upright

So if you draw a box using “raw image coordinates” without mapping into the preview’s coordinate space, boxes will appear shifted/scaled.

### Our solution

We use the official CameraX transform utilities to map **image coordinates → PreviewView coordinates**:

- `PreviewView.outputTransform`
- `ImageProxyTransformFactory.getOutputTransform(imageProxy)`
- `CoordinateTransform(src, dst)` to map points/rects

Code:

```1606:1662:/media/hrc/T7_UBUNTU_ONLY/android_humanoid_all_files/PhoneBot/APP_workspace/app/src/main/java/com/example/phonebot_app_android/MainActivity.kt
private fun tryCreateOverlayMapper(imageProxy: ImageProxy, previewTransform: OutputTransform?, previewSize: Pair<Int, Int>?): OverlayMapper? { ... }
```

That mapper is used in the **ML Kit** path to draw bounding boxes/skeleton on the preview correctly.

For **MediaPipe**, after we switched to the “rotate/mirror bitmap before inference” approach (like the sample), the normalized landmarks are already in the same orientation as the displayed preview (for our current `PreviewView` settings), so we draw them directly in normalized coordinates.


## Where each part lives in our app (code map)

### CameraX pipeline

**Bind Preview + ImageAnalysis**:

```448:545:/media/hrc/T7_UBUNTU_ONLY/android_humanoid_all_files/PhoneBot/APP_workspace/app/src/main/java/com/example/phonebot_app_android/MainActivity.kt
DisposableEffect(...) {
  val previewBuilder = Preview.Builder()
  val analysisBuilder = ImageAnalysis.Builder().setBackpressureStrategy(KEEP_ONLY_LATEST)
  previewBuilder.build().setSurfaceProvider(previewView.surfaceProvider)
  analysisBuilder.build().setAnalyzer(analysisExec) { imageProxy -> ... }
  provider.bindToLifecycle(lifecycleOwner, selector, preview, analysis)
}
```

**Camera selection by Camera2 cameraId** (CameraX + Camera2 interop):

```1048:1067:/media/hrc/T7_UBUNTU_ONLY/android_humanoid_all_files/PhoneBot/APP_workspace/app/src/main/java/com/example/phonebot_app_android/MainActivity.kt
CameraSelector.Builder()
  .addCameraFilter { infos -> infos.filter { Camera2CameraInfo.from(it).cameraId == camId } }
  .build()
```

**Setting FPS range** (CameraX + Camera2Interop):

```494:499:/media/hrc/T7_UBUNTU_ONLY/android_humanoid_all_files/PhoneBot/APP_workspace/app/src/main/java/com/example/phonebot_app_android/MainActivity.kt
val extPrev = Camera2Interop.Extender(previewBuilder)
extPrev.setCaptureRequestOption(CONTROL_AE_TARGET_FPS_RANGE, fpsRange)
```


### ML Kit tasks (model loading + inference)

**Key idea**: ML Kit is “service style” — you ask for a detector, and call `process(...)` asynchronously.

- **Create** the detector once (lazy) and store it in an `AtomicReference`.
- **Inference** is asynchronous; results come through listeners.

Example (face):

```551:618:/media/hrc/T7_UBUNTU_ONLY/android_humanoid_all_files/PhoneBot/APP_workspace/app/src/main/java/com/example/phonebot_app_android/MainActivity.kt
val input = InputImage.fromMediaImage(img, rotDeg)
val detector = FaceDetection.getClient(...)
detector.process(input)
  .addOnSuccessListener { faces -> ... }
  .addOnCompleteListener { imageProxy.close() }
```

Example (pose):

```620:702:/media/hrc/T7_UBUNTU_ONLY/android_humanoid_all_files/PhoneBot/APP_workspace/app/src/main/java/com/example/phonebot_app_android/MainActivity.kt
val detector =
  poseDetectorRef.get()
    ?: PoseDetection.getClient(
         PoseDetectorOptions.Builder()
           .setDetectorMode(PoseDetectorOptions.STREAM_MODE)
           .build()
       ).also { poseDetectorRef.set(it) }

detector.process(input)
  .addOnSuccessListener { pose -> /* landmarks -> skeleton overlay */ }
  .addOnCompleteListener { imageProxy.close() }
```

Example (object):

```704:777:/media/hrc/T7_UBUNTU_ONLY/android_humanoid_all_files/PhoneBot/APP_workspace/app/src/main/java/com/example/phonebot_app_android/MainActivity.kt
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
  .addOnSuccessListener { objs -> /* bounding boxes + label text */ }
  .addOnCompleteListener { imageProxy.close() }
```


### MediaPipe Tasks (model loading + inference)

**Key idea**: MediaPipe Tasks is “model instance style” — you build options with a `.task` asset, create a landmarker, then call `detect(...)`.

We load models from assets:

- `mediapipe_models/face_landmarker.task`
- `mediapipe_models/hand_landmarker.task`
- `mediapipe_models/pose_landmarker_{lite|full|heavy}.task`

Example (face):

```811:870:/media/hrc/T7_UBUNTU_ONLY/android_humanoid_all_files/PhoneBot/APP_workspace/app/src/main/java/com/example/phonebot_app_android/MainActivity.kt
val base = BaseOptions.builder().setModelAssetPath("mediapipe_models/face_landmarker.task").build()
val opts = FaceLandmarker.FaceLandmarkerOptions.builder()
  .setBaseOptions(base)
  .setRunningMode(RunningMode.IMAGE)
  .build()
val landmarker = FaceLandmarker.createFromOptions(context, opts)
val res = landmarker.detect(mpImage)
```

Example (hand):

```872:938:/media/hrc/T7_UBUNTU_ONLY/android_humanoid_all_files/PhoneBot/APP_workspace/app/src/main/java/com/example/phonebot_app_android/MainActivity.kt
val base = BaseOptions.builder().setModelAssetPath("mediapipe_models/hand_landmarker.task").build()
val opts = HandLandmarker.HandLandmarkerOptions.builder()
  .setBaseOptions(base)
  .setRunningMode(RunningMode.IMAGE)
  .build()
val landmarker = HandLandmarker.createFromOptions(context, opts)
val res = landmarker.detect(mpImage)
```

Example (pose):

```940:1014:/media/hrc/T7_UBUNTU_ONLY/android_humanoid_all_files/PhoneBot/APP_workspace/app/src/main/java/com/example/phonebot_app_android/MainActivity.kt
val base = BaseOptions.builder().setModelAssetPath(modelPath).build()
val opts = PoseLandmarker.PoseLandmarkerOptions.builder()
  .setBaseOptions(base)
  .setRunningMode(RunningMode.IMAGE)
  .build()
val landmarker = PoseLandmarker.createFromOptions(context, opts)
val res = landmarker.detect(mpImage)
```

Notes:

- We currently run MediaPipe in `RunningMode.IMAGE` per frame.
- The official samples often use `LIVE_STREAM + detectAsync(...)` for best realtime behavior; that’s a next optimization step.
- MediaPipe **OBJECT** is not implemented in our app yet (we don’t have `mediapipe_models/object_detector.task`).

Sample reference (MediaPipe “rotate + mirror before inference” pattern, LIVE_STREAM):

```165:199:/media/hrc/T7_UBUNTU_ONLY/android_humanoid_all_files/PhoneBot/reference_repo/mediapipe-samples/examples/pose_landmarker/android/app/src/main/java/com/google/mediapipe/examples/poselandmarker/PoseLandmarkerHelper.kt
val matrix = Matrix().apply {
  postRotate(imageProxy.imageInfo.rotationDegrees.toFloat())
  if (isFrontCamera) {
    postScale(-1f, 1f, imageProxy.width.toFloat(), imageProxy.height.toFloat())
  }
}
val rotatedBitmap = Bitmap.createBitmap(bitmapBuffer, 0, 0, bitmapBuffer.width, bitmapBuffer.height, matrix, true)
val mpImage = BitmapImageBuilder(rotatedBitmap).build()
detectAsync(mpImage, frameTime)
```


## Practical checklist (when adding another task)

### CameraX

- Use `ImageAnalysis.STRATEGY_KEEP_ONLY_LATEST`.
- Always `imageProxy.close()` exactly once.
- Use a single dedicated `ExecutorService` for analyzers.

### ML Kit

- Use `InputImage.fromMediaImage(imageProxy.image, rotationDegrees)`.
- Keep detectors as singletons and reuse.
- Do overlay mapping using CameraX transforms if you draw on top of `PreviewView`.

### MediaPipe Tasks

- If you see **RGBA/ARGB errors**, convert to `Bitmap.Config.ARGB_8888` and use `BitmapImageBuilder`.
- For correct overlay:
  - Either map coordinates through `PreviewView.outputTransform` (advanced), OR
  - Follow sample behavior: rotate/mirror bitmap **before** inference and draw using normalized coords in that same space.


## Common symptoms → cause → fix

- **`android media image must use rgba_8888 config`**
  - **Cause**: passing YUV frames to a MediaPipe path expecting RGBA/ARGB.
  - **Fix**: `ImageProxy -> Bitmap(ARGB_8888) -> BitmapImageBuilder`.

- **Boxes / skeleton shifted or scaled**
  - **Cause**: ignoring cropRect + PreviewView scaling/letterboxing.
  - **Fix**: map coordinates using `OutputTransform` + `CoordinateTransform`, or use the rotate+mirror-bitmap approach consistently.

- **Front camera looks mirrored**
  - **Cause**: missing horizontal mirror for front lens.
  - **Fix**: `postScale(-1, 1, centerX, centerY)` before inference (MediaPipe sample style).

