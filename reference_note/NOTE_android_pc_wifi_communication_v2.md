# NOTE_android_pc_wifi_communication.md
_Last updated: 2026‑02‑05_

This note summarizes **Android ↔ PC WiFi communication for robotics**, especially:
- ROS2 on Android (DDS over WiFi)
- ROS bridge approaches
- Custom UDP communication + PC ROS gateway
- QoS considerations for high‑rate robotics communication (100–300 Hz)
- Reference GitHub repos

------------------------------------------------------------

# 1. Recommended Method — ROS2 directly on Android (DDS over WiFi)

This is the modern approach.

Phone becomes a real ROS2 node:
- publishes IMU / camera / state
- subscribes to commands
- communicates directly with ROS2 PC over WiFi

DDS internally typically uses:

- UDP multicast → discovery
- UDP unicast → data

## Core repos

ROS2 Android bindings:
https://github.com/ros2-java/ros2_android

ROS2 Java base layer:
https://github.com/ros2-java/ros2_java

Android ROS2 examples:
https://github.com/ros2-java/ros2_android_examples

Community tutorial:
https://github.com/songshan0321/ros2-android-tutorial

### Advantages

- No bridge needed
- Full ROS2 tooling compatibility
- Good performance for most robotics
- Supports QoS tuning

### Typical performance

With BEST_EFFORT QoS:

- 100–300 Hz sensor/control communication feasible over WiFi
- Latency typically a few milliseconds

------------------------------------------------------------

# 2. ROS Bridge Approach (WebSocket/TCP JSON)

Used when the phone app is NOT a native ROS2 node.

## Repos

ROS1 rosbridge:
https://github.com/RobotWebTools/rosbridge_suite

ROS2 web bridge:
https://github.com/RobotWebTools/ros2-web-bridge

Python bridge client:
https://github.com/gramaziokohler/roslibpy

### Notes

Transport:
- WebSocket
- TCP JSON messaging

Pros:
- Easy mobile/web integration
- Good for dashboards, teleoperation

Cons:
- Higher overhead
- Not ideal for tight control loops

------------------------------------------------------------

# 3. Custom UDP + ROS Gateway (Low latency option)

Architecture:

Phone <-> UDP <-> PC gateway node <-> ROS/ROS2

Used when:
- strict latency required
- custom protocol needed
- DDS discovery problematic

DDS already uses UDP internally, but adds:
- discovery
- QoS tracking
- serialization layers

Custom UDP removes those layers.

------------------------------------------------------------

## UDP ↔ ROS Bridge Repositories (REFERENCE)

### ROS2‑focused bridges

network_bridge (UDP/TCP ROS2 bridge):
https://github.com/brow1633/network_bridge

simple_udp (ROS2 UDP bridge):
https://github.com/sgrsn/simple_udp


### ROS1 bridges (still useful references)

ros_udp_bridge (bidirectional):
https://github.com/Brazilian-Institute-of-Robotics/ros_udp_bridge

ros_udp_bridge simple example:
https://github.com/plusangel/ros_udp_bridge

ros_UDP project:
https://github.com/wenbowen123/ros_UDP

udp_bridge advanced networking:
https://github.com/rolker/udp_bridge

vicon_udp motion capture example:
https://github.com/BristolFlightLab/vicon_udp


### ROS1 ↔ ROS2 compatibility bridge

ros1_bridge:
https://github.com/ros2/ros1_bridge

------------------------------------------------------------

# 4. QoS Guidance for WiFi Robots

Default when passing "10" queue depth:

- reliability = RELIABLE
- history = KEEP_LAST
- depth = 10
- durability = VOLATILE

## Recommended

Sensors (IMU/camera):
- BEST_EFFORT
- depth 1–5

Control commands:
- BEST_EFFORT
- depth 1

Critical data:
- RELIABLE

Small depth reduces latency.

------------------------------------------------------------

# 5. ADB (Android Debug Bridge)

ADB is Android developer tool for:

- installing apps
- logs
- shell access
- debugging

Included automatically with Android Studio.

Not used for robot runtime communication.

------------------------------------------------------------

# 6. Which method should I choose?

## Best general choice

ROS2 Android DDS directly.

## Use rosbridge if:

- web/mobile UI
- rapid prototyping

## Use custom UDP if:

- ultra‑low latency required
- experimental networking
- special network environments

------------------------------------------------------------

# 7. Typical Robotics Architecture

Phone:
- perception
- IMU
- planning

Microcontroller:
- motor control

PC (optional):
- mapping
- training
- visualization

Communication:

Phone <-> PC:
- DDS (recommended)
or
- Custom UDP

Phone <-> MCU:
- USB / UART / BLE

------------------------------------------------------------

# 8. Key Takeaway

ROS2 DDS already uses UDP.

Therefore:

- Often no custom UDP bridge needed.
- But custom UDP gateway is still common in research when latency matters.

