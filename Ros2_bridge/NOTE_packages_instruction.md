# NOTE_packages_instruction.md

This note records **how to build and run** the ROS2 packages in `PhoneBot/Ros2_bridge/src/`.

## Build workspace (IMPORTANT)

Always run `colcon build` from the **workspace root**:

`/media/hrc/T7_UBUNTU_ONLY/android_humanoid_all_files/PhoneBot/Ros2_bridge`

so that `build/`, `install/`, and `log/` folders are created in the correct place.

```bash
source /opt/ros/jazzy/setup.bash
cd /media/hrc/T7_UBUNTU_ONLY/android_humanoid_all_files/PhoneBot/Ros2_bridge
colcon build
source install/setup.bash
```

If you hit Python ROS tooling errors while building (e.g. missing `catkin_pkg`), **deactivate any external venv**
so `colcon/ament` uses the system Python that matches ROS2.

## `phonebot_bridge` (Python UDP ↔ ROS2 bridge)

Build:

```bash
cd /media/hrc/T7_UBUNTU_ONLY/android_humanoid_all_files/PhoneBot/Ros2_bridge
colcon build --packages-select phonebot_bridge
source install/setup.bash
```

Run (immediate bridge, recommended):

```bash
ros2 run phonebot_bridge phonebot_udp_bridge_immediate --ros-args \
  -p bind_ip:=0.0.0.0 -p bind_port:=5005 \
  -p android_ip:=192.168.20.6 -p android_port:=6006
```

Notes:
- `bind_port` must be free on the PC (common error: “Address already in use”).
- `android_ip/android_port` is the **phone** IP and the phone’s **UDP listen port** for PC→phone packets.

## `phonebot_bridge_cpp` (C++ UDP ↔ ROS2 bridge)

Build:

```bash
cd /media/hrc/T7_UBUNTU_ONLY/android_humanoid_all_files/PhoneBot/Ros2_bridge
colcon build --packages-select phonebot_bridge_cpp
source install/setup.bash
```

Run:

```bash
ros2 run phonebot_bridge_cpp phonebot_udp_bridge_immediate_cpp --ros-args \
  -p bind_ip:=0.0.0.0 -p bind_port:=5005 \
  -p android_ip:=192.168.20.6 -p android_port:=6006
```

## `joystick_python` (repeat `/cmd_vel` at constant rate)

Build:

```bash
cd /media/hrc/T7_UBUNTU_ONLY/android_humanoid_all_files/PhoneBot/Ros2_bridge
colcon build --packages-select joystick_python
source install/setup.bash
```

Joystick mode (default): subscribe `/cmd_vel`, publish `/phonebot/cmd_vel` at fixed rate:

```bash
ros2 run joystick_python cmd_vel_repeater --ros-args -p mode:=joystick -p publish_hz:=50.0
```

Keyboard mode:

```bash
ros2 run joystick_python cmd_vel_repeater --ros-args -p mode:=keyboard -p publish_hz:=50.0
```

## `joystick_cpp` (repeat `/cmd_vel` at constant rate, C++)

Build:

```bash
cd /media/hrc/T7_UBUNTU_ONLY/android_humanoid_all_files/PhoneBot/Ros2_bridge
colcon build --packages-select joystick_cpp
source install/setup.bash
```

Run:

```bash
ros2 run joystick_cpp cmd_vel_repeater --ros-args -p mode:=joystick -p publish_hz:=50.0
```

## `robot_visualizer` (MuJoCo visualizer)

Build:

```bash
cd /media/hrc/T7_UBUNTU_ONLY/android_humanoid_all_files/PhoneBot/Ros2_bridge
colcon build --packages-select robot_visualizer
source install/setup.bash
```

Run:

```bash
ros2 run robot_visualizer phonebot_visualizer --ros-args \
  -p mode:=normal \
  -p imu_topic:=/phonebot/imu_game \
  -p motor_state_topic:=/phonebot/motor_state_full


ros2 run robot_visualizer phonebot_visualizer --ros-args \
  -p mode:=alter \
  -p imu_topic:=/phonebot/imu_game \
  -p motor_state_topic:=/phonebot/motor_state_full
```

Notes:
- `mode:=normal` means IMU site attached to base link; `mode:=alter` means IMU site attached to trunk link.

### Official viewer version (arrows supported)

If your environment has a `mujoco_viewer` incompatibility (e.g. `MjvGeom.texid` crash), use the official viewer node:

```bash
ros2 run robot_visualizer phonebot_visualizer_official --ros-args \
  -p mode:=normal \
  -p imu_topic:=/phonebot/imu_game \
  -p motor_state_topic:=/phonebot/motor_state_full

ros2 run robot_visualizer phonebot_visualizer_official --ros-args \
  -p mode:=alter \
  -p imu_topic:=/phonebot/imu_game \
  -p motor_state_topic:=/phonebot/motor_state_full

# Use fred_v2_torque_version model explicitly:
ros2 run robot_visualizer phonebot_visualizer_official --ros-args \
  -p model_path:=/media/hrc/T7_UBUNTU_ONLY/android_humanoid_all_files/PhoneBot/Ros2_bridge/install/robot_visualizer/share/robot_visualizer/model/model_phonebot_fred_v2_torque_version/scene_joystick_flat_terrain_alter_v2_full_collision.xml \
  -p mode:=alter \
  -p imu_topic:=/phonebot/imu_game \
  -p motor_state_topic:=/phonebot/motor_state_full
```

## `pc_policy_runner` (CMake package, PC runs TFLite policy)

Build:

```bash
cd /media/hrc/T7_UBUNTU_ONLY/android_humanoid_all_files/PhoneBot/Ros2_bridge
colcon build --packages-select pc_policy_runner
source install/setup.bash
```

Run:

```bash
ros2 run pc_policy_runner pc_policy_runner_node.py --ros-args \
  --params-file src/pc_policy_runner/config/pc_policy_runner.yaml
```

## `pc_policy_runner_python` (Python package, PC runs TFLite policy)

Build:

```bash
cd /media/hrc/T7_UBUNTU_ONLY/android_humanoid_all_files/PhoneBot/Ros2_bridge
colcon build --packages-select pc_policy_runner_python
source install/setup.bash
```

Run:

```bash
ros2 run pc_policy_runner_python pc_policy_runner --ros-args \
  --params-file src/pc_policy_runner_python/config/pc_policy_runner.yaml
```

### PC-side Python deps for policy runners

These policy-runner nodes require:
- A TFLite interpreter:
  - `pip install tflite-runtime` (recommended) **or** `pip install tensorflow`
- `numpy`:
  - `pip install numpy`

## Suggested “full stack” run order (PC + phone + Pi)

1) Start Pi HWdriver (on Raspberry Pi): `motor_driver_*` publishes `/phonebot/motor_state_full`, subscribes `/phonebot/motor_cmd`.
2) Start PC bridge (`phonebot_bridge` or `phonebot_bridge_cpp`) so phone IMU/motor packets become ROS2 topics and motor state returns to phone.
3) Start joystick (optional): `teleop_twist_joy` → `/cmd_vel`, then `joystick_*` → `/phonebot/cmd_vel`.
4) Start policy runner:
   - **PC policy**: `pc_policy_runner(_python)` publishes `/phonebot/motor_cmd`, and keep phone policy OFF.
   - **Phone policy**: use Android “Policy: ON”, and keep PC policy runner OFF.

