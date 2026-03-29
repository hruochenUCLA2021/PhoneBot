# PhoneBot — How to Run

## Network Setup (current)

| Device  | Default IP      | Notes                              |
|---------|-----------------|------------------------------------|
| PC      | `192.168.20.15` | Set in Android app as UDP target   |
| Phone   | `192.168.20.2`  | Set in bridge node as `android_ip` |

- PC listens on UDP port `5005` (packets from phone)
- Phone listens on UDP port `6006` (packets from PC)
- Both must be on the same WiFi network

---

## Architecture (current)

- **Phone (`APP_workspace`)**:
  - Always sends **SENSOR** UDP to PC (IMU + battery)
  - Sends **MOTOR** UDP (goal position) and **TORQUE** UDP (enable/disable) to PC
  - Receives **MOTOR** UDP back from PC for **motor status display** (UI throttled ~20 Hz)
- **PC (`Ros2_bridge`)**:
  - Runs UDP↔ROS2 bridge (`phonebot_bridge` Python OR `phonebot_bridge_cpp` C++)
  - Subscribes to Pi motor state topic and forwards it to phone via UDP
- **Raspberry Pi Zero 2W (`PhoneBot_HWdriver`)**:
  - Runs motor driver, subscribes:
    - `/phonebot/motor_cmd` (`sensor_msgs/JointState`)
    - `/phonebot/torque_enable` (`std_msgs/Bool`)
  - Publishes:
    - `/phonebot/motor_state_full` (`motor_interfaces/msg/MotorState`)

---

## UDP packet format reference

The exact binary layout is documented in:
- `reference_note/NOTE_protocal.md`

Packet types used now:
- **SENSOR** (Android → PC) `msg_type=1`
- **MOTOR** (Android ↔ PC) `msg_type=2` (pos/vel/tau float32[13])
- **TORQUE** (Android → PC) `msg_type=3` (enable u8)

---

## 1. Build (PC side: Ros2_bridge)

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

## 2. Run the UDP Bridge (PC side: Ros2_bridge)

Choose one of the two bridge modes. **Immediate** is recommended for lowest latency.

### Immediate bridge (publishes as soon as UDP arrives)

```bash
ros2 run phonebot_bridge phonebot_udp_bridge_immediate
```

Or the C++ version:

```bash
ros2 run phonebot_bridge_cpp phonebot_udp_bridge_immediate_cpp
```

With custom phone IP:

```bash
ros2 run phonebot_bridge phonebot_udp_bridge_immediate --ros-args \
  -p android_ip:=192.168.20.2 \
  -p android_port:=6006 \
  -p bind_port:=5005
```

Or C++:

```bash
ros2 run phonebot_bridge_cpp phonebot_udp_bridge_immediate_cpp --ros-args \
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
| `motor_topic`  | `/phonebot/motor_cmd` | ROS2 motor command topic (pub/sub depends on bridge impl) |
| `android_ip`   | `192.168.20.2`    | Phone's IP (for sending motor commands)  |
| `android_port` | `6006`            | Phone's listen port for motor commands   |

### ROS2 topics published by the bridge

| Topic                    | Type                       | Description                   |
|--------------------------|----------------------------|-------------------------------|
| `/phonebot/imu`          | `sensor_msgs/Imu`          | IMU (rotation vector)         |
| `/phonebot/imu_game`     | `sensor_msgs/Imu`          | IMU (game rotation vector)    |
| `/phonebot/battery`      | `sensor_msgs/BatteryState` | Battery level and status      |
| `/phonebot/motor_state`  | `sensor_msgs/JointState`   | (Legacy) motor pos/vel inside SENSOR v2 packets |
| `/phonebot/motor_cmd`    | `sensor_msgs/JointState`   | Motor command from phone (UDP) to Pi HWdriver |
| `/phonebot/torque_enable`| `std_msgs/Bool`            | Torque enable from phone (UDP) to Pi HWdriver |

### ROS2 topics subscribed by the bridge

| Topic                    | Type                       | Description                   |
|--------------------------|----------------------------|-------------------------------|
| `/phonebot/motor_state_full` | `motor_interfaces/msg/MotorState` | Motor state from Pi HWdriver forwarded to phone via UDP |

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
   - Set **PC IP** to `192.168.20.15` (or your PC's WiFi IP)
   - Set **UDP port** to `5005`
   - Use buttons:
     - **Torque ON/OFF**
     - **Zero position**
     - **Test swing**

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
ros2 topic echo /phonebot/motor_state_full
```

---

## 7. Useful Debug Commands

```bash
# Check all active topics
ros2 topic list

# Check IMU rate (should be ~90-100 Hz)
ros2 topic hz /phonebot/imu_game

# Check motor state rate
ros2 topic hz /phonebot/motor_state_full

# See latest IMU data
ros2 topic echo /phonebot/imu_game --once

# See latest motor positions
ros2 topic echo /phonebot/motor_state_full --once

# See battery
ros2 topic echo /phonebot/battery --once
```



source install/setup.bash
ros2 run phonebot_bridge_cpp phonebot_udp_bridge_immediate_cpp  --ros-args -p android_ip:=192.168.20.21
ros2 run phonebot_bridge_cpp phonebot_udp_bridge_immediate_cpp  --ros-args -p android_ip:=192.168.20.31




to test the ai backend::::
(RL_env_3_12) hrc@hrc-Nitro-ANV15-51:~$ curl -s -X POST http://127.0.0.1:8088/api/llm/text   -H "Content-Type: application/json"   -d '{"text":"hello"}'
{"transcript":"hello","reply":"Hello! How can I assist you today with your robotics project?"}(RL_env_3_12) hrc@hrc-Nitro-ANV15-51:~$ ^C
(RL_env_3_12) hrc@hrc-Nitro-ANV15-51:~$ 



(RL_env_3_12) hrc@hrc-Nitro-ANV15-51:/media/hrc/T7_UBUNTU_ONLY/android_humanoid_all_files/PhoneBot/AI_backend$ python ./server.py 
INFO:     Started server process [711244]
INFO:     Waiting for application startup.
INFO:     Application startup complete.
INFO:     Uvicorn running on http://0.0.0.0:8088 (Press CTRL+C to quit)
INFO:     127.0.0.1:33854 - "POST /api/llm/text HTTP/1.1" 200 OK






ros2 run joy joy_node

ros2 topic echo /joy
ros2 topic hz /joy --window 50



ros2 run teleop_twist_joy teleop_node   --ros-args --params-file joystick_config.yaml
ros2 topic hz /cmd_vel --window 50



