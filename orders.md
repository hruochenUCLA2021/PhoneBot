# PhoneBot — How to Run

## Network Setup

| Device  | Default IP      | Notes                              |
|---------|-----------------|------------------------------------|
| PC      | `192.168.20.11` | Set in Android app as UDP target   |
| Phone   | `192.168.20.2`  | Set in bridge node as `android_ip` |

- PC listens on port `5005` (sensor packets from phone)
- Phone listens on port `6006` (motor commands from PC)
- Both must be on the same WiFi network

---

## 1. Build (PC side)

```bash
cd /media/hrc/T7_UBUNTU_ONLY/android_humanoid_all_files/PhoneBot/Ros2_bridge
colcon build --symlink-install
source install/setup.bash
```

Build a single package only:

```bash
colcon build --symlink-install --packages-select phonebot_bridge
colcon build --symlink-install --packages-select robot_visualizer
```

---

## 2. Run the UDP Bridge (PC side)

Choose one of the two bridge modes. **Immediate** is recommended for lowest latency.

### Immediate bridge (publishes as soon as UDP arrives)

```bash
ros2 run phonebot_bridge phonebot_udp_bridge_immediate
```

With custom phone IP:

```bash
ros2 run phonebot_bridge phonebot_udp_bridge_immediate --ros-args \
  -p android_ip:=192.168.20.2 \
  -p android_port:=6006 \
  -p bind_port:=5005
```

### Periodic bridge (publishes at fixed rate)

```bash
ros2 run phonebot_bridge phonebot_udp_bridge_periodic
```

With custom settings:

```bash
ros2 run phonebot_bridge phonebot_udp_bridge_periodic --ros-args \
  -p android_ip:=192.168.20.2 \
  -p android_port:=6006 \
  -p bind_port:=5005 \
  -p publish_hz:=50.0 \
  -p motor_send_hz:=50.0
```

### Bridge parameters (both modes)

| Parameter      | Default           | Description                              |
|----------------|-------------------|------------------------------------------|
| `bind_ip`      | `0.0.0.0`         | IP to bind the UDP receive socket        |
| `bind_port`    | `5005`            | Port to listen for sensor packets        |
| `topic_ns`     | `phonebot`        | ROS2 topic namespace                     |
| `motor_topic`  | `/phonebot/motor_cmd` | ROS2 topic to subscribe for motor cmds |
| `android_ip`   | `192.168.20.2`    | Phone's IP (for sending motor commands)  |
| `android_port` | `6006`            | Phone's listen port for motor commands   |

### ROS2 topics published by the bridge

| Topic                    | Type                       | Description                   |
|--------------------------|----------------------------|-------------------------------|
| `/phonebot/imu`          | `sensor_msgs/Imu`          | IMU (rotation vector)         |
| `/phonebot/imu_game`     | `sensor_msgs/Imu`          | IMU (game rotation vector)    |
| `/phonebot/battery`      | `sensor_msgs/BatteryState` | Battery level and status      |
| `/phonebot/motor_state`  | `sensor_msgs/JointState`   | Motor pos/vel (13 motors)     |

### ROS2 topic subscribed by the bridge

| Topic                    | Type                       | Description                   |
|--------------------------|----------------------------|-------------------------------|
| `/phonebot/motor_cmd`    | `sensor_msgs/JointState`   | Motor commands sent to phone  |

---

## 3. Run the MuJoCo Visualizer (PC side)

### With launch file

```bash
ros2 launch robot_visualizer phonebot_visualizer.launch.py
```

Custom render rate:

```bash
ros2 launch robot_visualizer phonebot_visualizer.launch.py render_hz:=60.0
```

### With ros2 run

```bash
ros2 run robot_visualizer phonebot_visualizer
```

With custom model path:

```bash
ros2 run robot_visualizer phonebot_visualizer --ros-args \
  -p model_path:=/media/hrc/T7_UBUNTU_ONLY/android_humanoid_all_files/PhoneBot/reference_repo/model_phonebot/phonebot_general_alternative_imu.xml \
  -p render_hz:=30.0
```

### Visualizer parameters

| Parameter            | Default                                              | Description                   |
|----------------------|------------------------------------------------------|-------------------------------|
| `model_path`         | `.../model_phonebot/phonebot_general_alternative_imu.xml` | MuJoCo XML model path    |
| `render_hz`          | `30.0`                                               | Viewer render rate            |
| `imu_topic`          | `/phonebot/imu_game`                                 | IMU topic for body orientation|
| `motor_state_topic`  | `/phonebot/motor_state`                              | Joint state topic             |

---

## 4. Example Motor Publisher (for testing without phone)

```bash
ros2 run phonebot_bridge phonebot_example_motor_pub
```

Publishes test motor commands to `/phonebot/motor_cmd`.

---

## 5. Android App (Phone side)

1. Open `APP_workspace` in Android Studio
2. Build and install on phone
3. On the app screen:
   - Set **PC IP** to `192.168.20.11` (or your PC's WiFi IP)
   - Set **UDP port** to `5005`
   - Select **REMOTE_UDP** mode
   - Enable **Motor HW** if motors are connected via USB-C serial adapter

---

## 6. Typical Full Session

Terminal 1 — bridge:

```bash
cd /media/hrc/T7_UBUNTU_ONLY/android_humanoid_all_files/PhoneBot/Ros2_bridge
source install/setup.bash
ros2 run phonebot_bridge phonebot_udp_bridge_immediate
```

Terminal 2 — visualizer:

```bash
cd /media/hrc/T7_UBUNTU_ONLY/android_humanoid_all_files/PhoneBot/Ros2_bridge
source install/setup.bash
ros2 launch robot_visualizer phonebot_visualizer.launch.py
```

Terminal 3 — check topics:

```bash
ros2 topic list
ros2 topic hz /phonebot/imu_game
ros2 topic echo /phonebot/motor_state
```

---

## 7. Useful Debug Commands

```bash
# Check all active topics
ros2 topic list

# Check IMU rate (should be ~90-100 Hz)
ros2 topic hz /phonebot/imu_game

# Check motor state rate
ros2 topic hz /phonebot/motor_state

# See latest IMU data
ros2 topic echo /phonebot/imu_game --once

# See latest motor positions
ros2 topic echo /phonebot/motor_state --once

# See battery
ros2 topic echo /phonebot/battery --once
```
