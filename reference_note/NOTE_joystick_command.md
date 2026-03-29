# NOTE_joystick_command.md

This note records how to drive **PhoneBot cmd vx/vy/wz** from a PC joystick or keyboard, through ROS2, to the Android app.

## Topic pipeline (high level)

- **Joystick**: `/joy` → `teleop_twist_joy` → `/cmd_vel`
- **Repeater node (this repo)**: `/cmd_vel` → (timer @ constant Hz, default 50Hz) → `/phonebot/cmd_vel`
- **ROS2↔UDP bridge (this repo)**: `/phonebot/cmd_vel` → UDP (`CMD_VEL` message) → **Android**
- **Android**: receives cmd vel and uses it as policy command \(v_x, v_y, w_z\)

## Packages/nodes added in this repo

- **Python**: `joystick_python`
  - executable: `ros2 run joystick_python cmd_vel_repeater`
- **C++**: `joystick_cpp`
  - executable: `ros2 run joystick_cpp cmd_vel_repeater`

Both support:
- `mode:=joystick` (default): subscribe `/cmd_vel`, repeat last cmd at fixed rate to `/phonebot/cmd_vel`
- `mode:=keyboard`: generate cmd vel from keyboard, publish at fixed rate to `/phonebot/cmd_vel`

## Default velocity limits (match `joystick_config.yaml`)

- \(v_x^{max}=0.5\)
- \(v_y^{max}=0.25\)
- \(w_z^{max}=1.0\)

They are parameters on the repeater node:
- `vx_max`, `vy_max`, `wz_max`

## How to run (copy/paste)

### 0) Build workspace (once)

From `PhoneBot/Ros2_bridge/`:

```bash
colcon build
source install/setup.bash
```

Make sure you are using the system ROS environment (avoid a Python venv while building if it breaks ROS tooling).

### 1) Joystick → `/cmd_vel` (optional but common)

Terminal A:

```bash
source /opt/ros/jazzy/setup.bash
ros2 run joy joy_node
```

Terminal B:

```bash
source /opt/ros/jazzy/setup.bash
ros2 run teleop_twist_joy teleop_node --ros-args --params-file /media/hrc/T7_UBUNTU_ONLY/android_humanoid_all_files/PhoneBot/Ros2_bridge/joystick_config.yaml
```

### 2) Repeat `/cmd_vel` at fixed 50Hz → `/phonebot/cmd_vel`

Python repeater:

```bash
source /opt/ros/jazzy/setup.bash
source /media/hrc/T7_UBUNTU_ONLY/android_humanoid_all_files/PhoneBot/Ros2_bridge/install/setup.bash
ros2 run joystick_python cmd_vel_repeater --ros-args -p mode:=joystick -p publish_hz:=50.0
```

C++ repeater:

```bash
source /opt/ros/jazzy/setup.bash
source /media/hrc/T7_UBUNTU_ONLY/android_humanoid_all_files/PhoneBot/Ros2_bridge/install/setup.bash
ros2 run joystick_cpp cmd_vel_repeater --ros-args -p mode:=joystick -p publish_hz:=50.0
```

### 3) Bridge `/phonebot/cmd_vel` → UDP → Android

Python bridge example:

```bash
source /opt/ros/jazzy/setup.bash
source /media/hrc/T7_UBUNTU_ONLY/android_humanoid_all_files/PhoneBot/Ros2_bridge/install/setup.bash
ros2 run phonebot_bridge udp_bridge_immediate --ros-args \
  -p android_ip:=<YOUR_ANDROID_IP> -p android_port:=6006 \
  -p cmd_vel_topic:=/phonebot/cmd_vel
```

### 4) Keyboard mode (instead of joystick)

Python keyboard mode:

```bash
source /opt/ros/jazzy/setup.bash
source /media/hrc/T7_UBUNTU_ONLY/android_humanoid_all_files/PhoneBot/Ros2_bridge/install/setup.bash
ros2 run joystick_python cmd_vel_repeater --ros-args -p mode:=keyboard -p publish_hz:=50.0
```

C++ keyboard mode:

```bash
source /opt/ros/jazzy/setup.bash
source /media/hrc/T7_UBUNTU_ONLY/android_humanoid_all_files/PhoneBot/Ros2_bridge/install/setup.bash
ros2 run joystick_cpp cmd_vel_repeater --ros-args -p mode:=keyboard -p publish_hz:=50.0
```

## Keyboard mapping (keyboard mode)

In keyboard mode, the node tracks **vx**, **vy**, and **wz** separately, so you can combine directions (e.g., **Up+Left** → diagonal motion).
For opposite directions on the same axis (e.g., **Up vs Down**), the **latest key event wins**.

### Linear velocity \(v_x\)

- **Up**: \(+0.5 \cdot v_x^{max}\)
- **Shift + Up**: \(+1.0 \cdot v_x^{max}\)
- **Down**: \(-0.5 \cdot v_x^{max}\)
- **Shift + Down**: \(-1.0 \cdot v_x^{max}\)

### Linear velocity \(v_y\)

- **Left**: \(+0.5 \cdot v_y^{max}\)
- **Shift + Left**: \(+1.0 \cdot v_y^{max}\)
- **Right**: \(-0.5 \cdot v_y^{max}\)
- **Shift + Right**: \(-1.0 \cdot v_y^{max}\)

### Angular velocity \(w_z\)

- **a**: \(+0.5 \cdot w_z^{max}\)
- **Shift + a** (capital **A**): \(+1.0 \cdot w_z^{max}\)
- **d**: \(-0.5 \cdot w_z^{max}\)
- **Shift + d** (capital **D**): \(-1.0 \cdot w_z^{max}\)

### Stop / quit

- **Space**: stop (zero)
- **q** or **Ctrl+C**: quit the keyboard node

### Important terminal note (Shift+Arrow)

Shift+Arrow support depends on the terminal sending distinct escape sequences.
If your terminal doesn’t distinguish Shift+Arrow, you will still get the **non-shift** arrow behavior (half speed).

## Debug commands

- Check constant publish rate:

```bash
ros2 topic hz /phonebot/cmd_vel
```

- Inspect values:

```bash
ros2 topic echo /phonebot/cmd_vel
```

