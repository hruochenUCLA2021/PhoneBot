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

### UDP payload format (JSON)
Each UDP datagram is a UTF-8 JSON object:

```json
{
  "ts_ns": 123,
  "accel": [ax, ay, az],
  "gyro": [gx, gy, gz],
  "rotvec": { "hz": 100.0, "quat": [w,x,y,z], "ypr_deg": [yaw,pitch,roll] },
  "game_rotvec": { "hz": 100.0, "quat": [w,x,y,z], "ypr_deg": [yaw,pitch,roll] },
  "battery": { "percent": 80, "is_charging": true, "plugged": "USB", "status": "CHARGING" }
}
```

Notes:
- Quaternion is **Android order** `[w,x,y,z]`.
- The sender uses a **latest-only** policy: if it can’t keep up, it drops older packets and sends the most recent.

## PC ROS2 bridge (`Ros2_bridge/phonebot_bridge`)

### What it publishes
The node `phonebot_udp_bridge` publishes:
- `/phonebot/imu` (`sensor_msgs/Imu`) from `rotvec`
- `/phonebot/imu_game` (`sensor_msgs/Imu`) from `game_rotvec`
- `/phonebot/battery` (`sensor_msgs/BatteryState`)

Notes:
- In ROS `sensor_msgs/Imu`, quaternion is **ROS order** `(x,y,z,w)`; the bridge converts from Android `[w,x,y,z]`.
- Accel/gyro are reused for both topics.

### Run
In your ROS2 workspace:

```bash
colcon build --packages-select phonebot_bridge
source install/setup.bash
ros2 run phonebot_bridge phonebot_udp_bridge --ros-args -p bind_port:=5005
```

### Verify
```bash
ros2 topic list | grep phonebot
ros2 topic echo /phonebot/imu
ros2 topic echo /phonebot/battery
```


