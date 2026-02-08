# NOTE: Android project files tutorial (what to read + where our code is)

This note is a quick guide to common Android project files and **exactly where to look** in this repo for the IMU + battery app code we added.

## Big picture: what each file type is for

### Kotlin code (`.kt`)
- **What it is**: your app’s actual code (logic + UI).
- **What you change most often**: sensors, networking, ROS2 nodes, UI screens, services.
- **In this project**: Compose UI + IMU monitor + battery monitor are all Kotlin.

### Manifest (`AndroidManifest.xml`) — app “identity” + permissions + components
- **What it is**: a declarative file that tells Android:
  - permissions your app needs
  - what components exist (Activities, Services, BroadcastReceivers, Providers)
  - some app-wide flags
- **When to edit**:
  - you get a runtime error that mentions a missing permission
  - you add a `Service` (e.g., a Foreground Service for robot control)

### Gradle build scripts (`build.gradle.kts`)
- **What it is**: build configuration + dependencies for a module.
- **Two common levels**:
  - project-level: `APP_workspace/build.gradle.kts` (shared build plugins/config)
  - module-level: `APP_workspace/app/build.gradle.kts` (app dependencies + android config)
- **When to edit**:
  - you add/remove libraries (CameraX, ROS2 Java, TFLite, OkHttp, etc.)
  - you change `minSdk`, `targetSdk`, buildTypes, packaging options

### Version catalog (`libs.versions.toml`)
- **What it is**: a centralized list of dependency versions + names, used by Gradle.
- **Why it exists**: instead of hardcoding versions in `build.gradle.kts`, you reference `libs.some.library`.
- **When to edit**:
  - you want to add a library version once and reuse it
  - you want to upgrade dependencies cleanly

### Android resources (`res/…`) — UI strings, themes, icons, layouts
- **What it is**: non-code assets:
  - `res/values/strings.xml`: user-visible strings
  - `res/values/themes.xml`: theme definitions
  - `res/drawable/…`: icons/shapes
- **In Compose apps**: you may have fewer XML layouts, but resources are still used for themes/strings/icons.

## Where to look in *this* project (IMU + battery dashboard)

### UI screen (Compose)
- **File**: `APP_workspace/app/src/main/java/com/example/phonebot_app_android/MainActivity.kt`
- **What it does**:
  - sets the Compose UI via `setContent { ... }`
  - shows a simple dashboard with Battery + IMU values
  - starts/stops monitors using `DisposableEffect` so they don’t leak when the screen closes

### IMU / orientation code
- **File**: `APP_workspace/app/src/main/java/com/example/phonebot_app_android/sensors/ImuMonitor.kt`
- **What it does**:
  - registers sensors (`TYPE_ROTATION_VECTOR`, accelerometer, gyroscope)
  - computes:
    - **quaternion** `[w,x,y,z]` using `SensorManager.getQuaternionFromVector`
    - **yaw/pitch/roll** using `getRotationMatrixFromVector` → `getOrientation`
  - requests fast sampling
    - tries `SENSOR_DELAY_FASTEST`
    - falls back to `SENSOR_DELAY_GAME` if Android blocks the 0µs sampling rate
  - throttles UI updates to ~20Hz to avoid overwhelming Compose

### Battery info code
- **File**: `APP_workspace/app/src/main/java/com/example/phonebot_app_android/system/BatteryMonitor.kt`
- **What it does**:
  - listens to `ACTION_BATTERY_CHANGED` (sticky broadcast)
  - computes battery percent and charging/plugged status
  - exposes state as a `StateFlow` so the UI updates automatically

### Permission that fixed the “keep stopping” crash
- **File**: `APP_workspace/app/src/main/AndroidManifest.xml`
- **Why**:
  - When we request `SENSOR_DELAY_FASTEST` (0 microseconds), newer Android versions enforce that you declare:
    - `android.permission.HIGH_SAMPLING_RATE_SENSORS`

## How to debug: “build works, but app keeps stopping”

1. If **Build is successful** but it crashes on run, it’s usually a **runtime exception**.
2. Open **Logcat** in Android Studio.
3. Search for `FATAL EXCEPTION` and read the `Caused by:` section.
4. The stack trace will point to a specific Kotlin line like:
   - `... ImuMonitor.startFast(ImuMonitor.kt:XX)`

## Practical workflow: what to open first

- **Behavior/UI bug**: open `.kt` files first.
- **Permission or component issue**: open `AndroidManifest.xml`.
- **Missing library / dependency / SDK level**: open `app/build.gradle.kts` and maybe `libs.versions.toml`.


