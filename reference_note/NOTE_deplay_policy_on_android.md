# NOTE: Deploy policy on Android (recommended paths + perf tips)

This note summarizes practical ways to run an RL / navigation policy on Android (phone) for a robot, with a bias toward **performance on a constrained device**.

## Key decision: where does inference run?

- **On-device inference (phone runs the policy)**
  - **Pros**: low latency, works offline, simpler control loop (no network jitter).
  - **Cons**: limited compute / thermal throttling / battery drain; model must be mobile-friendly.

- **Off-device inference (PC runs the policy, phone streams observations + receives actions)**
  - **Pros**: fastest iteration; can run larger PyTorch/JAX/Brax models; easier debugging.
  - **Cons**: network latency/jitter/dropouts; video bandwidth; needs robust buffering/timeouts.

If your end goal is an autonomous robot that works reliably without Wi‑Fi, you usually end at **on-device inference**.

## How OpenBot does it (reference)

OpenBot’s Android “robot” app runs inference with **TensorFlow Lite**:
- Preprocess camera frames into a `ByteBuffer`
- Run `Interpreter.runForMultipleInputsOutputs(...)`
- Decode outputs into motor control commands
- Optional acceleration: **GPU delegate** or **NNAPI**

This architecture is a good template for your own on-device policy.

## Recommended on-device path (best performance in practice)

### Recommendation
If you train in **Brax/JAX**, a strong mobile path is:

**Brax/JAX policy → `jax2tf` → TensorFlow SavedModel → TFLite (`.tflite`) → Android TFLite Interpreter (NNAPI/GPU/CPU)**

Reason: TFLite is the most mature “mobile-first” inference runtime on Android, and it matches OpenBot’s pipeline.

### When this works best
- Fixed input/output shapes (no dynamic shapes at runtime)
- “Simple” ops (MLP/CNN, standard activations, no exotic control-flow)
- You can quantize (INT8 or FP16)

### Common blockers / gotchas
- **Unsupported ops during conversion** (`jax2tf` or TFLite converter fails)
- **Dynamic shapes** or Python-side control flow that can’t be exported cleanly
- Post-processing that relies on NumPy/Python (must be reimplemented on device)

If you hit these, consider the “PC inference first” approach (below), then distill/compress/export later.

## Concrete steps: Brax/JAX → TFLite

### 1) Freeze the policy interface
Decide exactly what runs on device:
- Inputs (examples):
  - State-only: IMU + goal + wheel odom + previous action (fastest)
  - Vision: RGB image (heavier; reduce resolution aggressively)
- Outputs:
  - Differential drive: `left`, `right`
  - Or `throttle`, `steering`

Keep the policy’s input/output **small and stable**.

### 2) Export using `jax2tf`
High-level goal: produce a TF function / SavedModel with concrete signatures.

You typically want:
- `tf.function` with an **input_signature**
- known dtypes (usually `float32`)
- no Python control flow inside forward pass

### 3) Convert SavedModel → `.tflite`
Choose a precision:
- **INT8**: best latency/power when supported
- **FP16**: easier than INT8, often good on mobile GPUs
- **FP32**: simplest, often slowest

For INT8 you’ll usually need a **representative dataset** for calibration.

### 4) Android inference runtime
Use TensorFlow Lite on Android:
- Load the `.tflite` model from **assets** or **file**
- Create `Interpreter` with `Interpreter.Options`
- Choose acceleration:
  - **NNAPI**: best if your model’s ops are supported by the phone’s NPU/DSP
  - **GPU delegate**: often good for FP16 / float models on many devices
  - **CPU**: baseline; enable multiple threads

In OpenBot terms, this corresponds to the `Network.Device` enum (CPU/GPU/NNAPI) and the `Interpreter` created in their `Network` base class.

### 5) Preprocess + postprocess efficiently
- Avoid per-frame allocations (reuse `ByteBuffer`/arrays).
- Keep camera input resolution low (start with ~`160x120` or `224x224`, not 1080p).
- Minimize expensive image conversions; use a direct YUV → model input pipeline when possible.

## Alternative path (fast iteration): PC inference + phone IO

If export is slow/painful early on:
- Run the policy on PC (JAX/PyTorch/Brax)
- Phone publishes observations (camera frames + IMU) to PC
- PC returns actions (left/right motor targets)

Transport options:
- **ROS 2** (recommended if you already want ROS tooling; Android can run ROS2 via `ros2_java`/`ros2_android` style stacks)
- **WebSocket / TCP** (simpler for a custom pipeline)

Then later:
- distill/compress the policy
- export to TFLite for on-device autonomy

## Performance checklist (what to measure)

- **Model latency**: median + p95 inference time (ms)
- **End-to-end control loop**: camera frame timestamp → action applied
- **Thermals**: watch throttling after 1–5 minutes
- **CPU/GPU/NNAPI comparison**: same model, same input resolution

Targets depend on your robot, but rough guidelines:
- 10 Hz control loop: keep inference comfortably < 50–70 ms including preprocessing
- 30 Hz control loop: keep inference comfortably < 15–20 ms including preprocessing

## Practical recommendation (summary)

- For final on-device autonomy on Android: **Brax/JAX → `jax2tf` → TFLite (INT8/FP16) → NNAPI/GPU**
- For fastest early iteration: **PC inference** (ROS2/WebSocket), then export later


