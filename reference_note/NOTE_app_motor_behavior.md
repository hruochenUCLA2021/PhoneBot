# PhoneBot Android App — Motor Behavior Notes

This note documents how the Android app turns Dynamixel torque on/off, how `kp/kd` are applied, and how ROS2 motor commands end up moving the motors.

Relevant files:
- `APP_workspace/app/src/main/java/com/example/phonebot_app_android/MainActivity.kt`
- `APP_workspace/app/src/main/java/com/example/phonebot_app_android/motors/dynamixel/DynamixelController.kt`
- `Ros2_bridge/src/phonebot_bridge/phonebot_bridge/udp_bridge_immediate.py`
- `Ros2_bridge/src/phonebot_bridge/phonebot_bridge/example_motor_pub.py`

## What moves the motors (end-to-end)

- **PC side**
  - A ROS2 node publishes `sensor_msgs/JointState` to **`/phonebot/motor_cmd`**.
    - Example publisher: `example_motor_pub.py` publishes 13 joint positions as sine waves.
  - `udp_bridge_immediate.py` subscribes to **`/phonebot/motor_cmd`** and forwards that message **to the phone via UDP** (to `android_ip:android_port`).

- **Phone side**
  - The app listens on the “Motor listen port” (default `6006`) and updates `latestMotorRef` whenever a UDP motor packet arrives.
  - If **Motor HW is ON**, a 100 Hz coroutine loop:
    - Sends goal positions to the motors using the latest received UDP packet (`pkt.pos`).
    - Reads back present position/velocity (sync read) for feedback.

Important: **ROS2 motor_cmd does not directly toggle torque.** It only provides target arrays (pos/vel/tau). Torque is controlled by the app’s Motor HW enable/disable path.

## What the example motor publisher does

`example_motor_pub.py` publishes a 13-DOF `JointState`:
- Default `hz = 50`
- Default `amp_pos = 0.25` rad
- The “motion” is **all joints oscillating** with different phases (not a walking gait; more like a multi-joint wiggle).
- It does **not** contain any explicit “torque enable” field.

## Motor HW ON/OFF (what triggers torque)

### Torque ON trigger (Motor HW ON)

When you turn Motor HW ON in the app UI, `MainActivity.kt` runs:
- `ok = dxlController.enable(dxlConfig)`
- If `ok == false`, it immediately forces the UI state back to OFF:
  - `motorHwEnabled = false`

Inside `DynamixelController.enable()` the sequence is:
1. Find USB-serial device + request permission
2. Open port, set baudrate
3. Configure motors:
   - Set Status Return Level (so reads can work)
   - **Torque OFF** (to safely change mode/gains)
   - Set **Position Control mode**
   - Write **P gain (`kp`)** and **D gain (`kd`)**
   - Send initial goal position (default 2048 ticks ≈ 0 rad)
   - **Torque ON**

### Torque OFF trigger (Motor HW OFF)

Turning Motor HW OFF runs `dxlController.disable()` which:
- Sends **Torque Enable = 0** (best-effort) for all configured IDs
- Closes the serial port and marks the controller as not running

## Where `kp` / `kd` come from

They are defined in `DynamixelController.Config` defaults:
- `kp: Int = 5`
- `kd: Float = 0.1f` → converted to the motor register integer by `kdToReg()`
  - current policy: `0.1` rounds to `0`, then is clamped to **reg=1** when `kd > 0`

### Are `kp/kd` persistent after power-off?

With the current code:
- The app writes gains during `enable()` every time Motor HW turns ON.
- The code does **not** perform any “save to EEPROM / flash” operation.
- On typical Dynamixel X-series behavior, these runtime gain writes are **not guaranteed to persist across power cycles**.

Practical expectation:
- **After power cycle**, motors revert to their default gains.
- **When Motor HW is enabled again**, the app re-applies `kp/kd`.

## Why Motor HW flips back OFF when nothing is connected

If there is no usable USB-serial connection, `enable()` returns `false` early (examples):
- No USB serial driver detected (no adapter connected)
- USB permission denied
- `openDevice()` failed

