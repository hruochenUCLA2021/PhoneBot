# NOTE: Android UDP → PC ROS2 bridge (PhoneBot)

This setup avoids `ros2_android`/`ros2_java` by sending phone data over **UDP** to a PC, then republishing to ROS2 topics in Python.

## Android app (`APP_workspace`)

### Modes
- **Local**: reads sensors and shows them on screen only
- **Remote UDP**: sends the latest sensor + battery snapshot to PC via UDP (high-rate)

### UDP settings
On the screen, set:
- `PC IP / Host`
- `UDP Port` (default `5005`)
check the pc ip by ifconfig, it is right now 192.168.20.11 



### UDP payload format (binary)
Each UDP datagram is a **fixed-layout binary packet** (float32, little-endian).

See the exact field order + data types in:
- `reference_note/NOTE_protocal.md`

Notes:
- Quaternion is **Android order** `[w,x,y,z]` inside the UDP packet; the bridge converts to ROS `(x,y,z,w)`.
- The sender uses a **latest-only** policy: if it can’t keep up, it drops older packets and sends the most recent.

## PC ROS2 bridge (`Ros2_bridge/phonebot_bridge`)

### What it publishes
The bridge publishes:
- `/phonebot/imu` (`sensor_msgs/Imu`) from `rotvec`
- `/phonebot/imu_game` (`sensor_msgs/Imu`) from `game_rotvec`
- `/phonebot/battery` (`sensor_msgs/BatteryState`)
- `/phonebot/motor_state` (`sensor_msgs/JointState`) from motor present state in the sensor UDP packet (v2)

Notes:
- In ROS `sensor_msgs/Imu`, quaternion is **ROS order** `(x,y,z,w)`; the bridge converts from Android `[w,x,y,z]`.
- Accel/gyro are reused for both topics.
- Motor state:
  - `position[]` and `velocity[]` are published if the Android packet includes them (protocol v2).

### PC → Android motor command (UDP return path)

The bridge also supports taking a ROS2 motor command topic from the PC and sending it back to Android via UDP.

#### ROS2 motor command topic
- Topic: `/phonebot/motor_cmd`
- Message type: `sensor_msgs/JointState`
  - `name[]` = joint names (13 motors)
  - `position[]` = commanded position
  - `velocity[]` = commanded velocity
  - `effort[]` = commanded torque (Nm) (JointState calls it “effort”)

#### UDP payload sent to Android
Binary datagram (float32, little-endian), containing:
- 13 positions
- 13 velocities
- 13 torques

See: `reference_note/NOTE_protocal.md` (packet type `MOTOR`)

#### Bridge parameters (motor UDP target)
Both bridges accept:
- `android_ip` (default `"192.168.20.2"`)
- `android_port` (default `6006`)
- `motor_topic` (default `"/phonebot/motor_cmd"`)

Periodic bridge also accepts:
- `motor_send_hz` (default `50.0`)

### Build
From the workspace root:

```bash
cd /media/hrc/T7_UBUNTU_ONLY/android_humanoid_all_files/PhoneBot/Ros2_bridge
colcon build --packages-select phonebot_bridge
source install/setup.bash
```

Tip: if you change Python entry points or want live-edit behavior, use:

```bash
colcon build --symlink-install --packages-select phonebot_bridge
```

### Run (two modes)

#### A) Periodic bridge (default 50 Hz publish)
Receives UDP continuously, but republishes to ROS2 on a fixed timer. This is stable and avoids publishing faster than you want.

```bash
ros2 run phonebot_bridge phonebot_udp_bridge_periodic --ros-args \
  -p bind_port:=5005 -p publish_hz:=50.0 \
  -p android_ip:=192.168.20.21 -p android_port:=6006 -p motor_send_hz:=50.0
```

#### B) Immediate bridge (publish on every UDP packet)
Publishes as soon as a UDP packet is received/parsed. This lets you see the *actual* incoming UDP rate.

```bash
ros2 run phonebot_bridge phonebot_udp_bridge_immediate --ros-args \
  -p bind_port:=5005 \
  -p android_ip:=192.168.20.21 -p android_port:=6006
```

Optional parameters (both modes):
- `bind_ip` (default `0.0.0.0`): which interface to listen on
- `bind_port` (default `5005`)
- `topic_ns` (default `phonebot`): topics become `/${topic_ns}/imu`, etc.

### Verify
```bash
ros2 topic list | grep phonebot
ros2 topic echo /phonebot/imu
ros2 topic echo /phonebot/battery
```

### Example motor publisher (PC side)
This mimics an RL policy sending actions at 50 Hz:

```bash
ros2 run phonebot_bridge phonebot_example_motor_pub --ros-args \
  -p publish_hz:=50.0 -p num_motors:=13 -p topic_name:=/phonebot/motor_cmd
```


