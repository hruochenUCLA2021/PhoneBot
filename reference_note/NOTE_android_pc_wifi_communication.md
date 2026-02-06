# NOTE_android_pc_wifi_communication.md
_Last updated: 2026-02-05 (America/Los_Angeles)_

This note summarizes **Android ↔ PC communication for robotics** with an emphasis on:
- **ROS 2 on Android** (DDS over Wi‑Fi)
- **Bridge-based approaches** (WebSocket/TCP JSON via rosbridge)
- **Custom UDP** (fastest / lowest overhead) and *UDP-to-ROS* bridges on the PC side
- Practical notes: **ADB**, QoS defaults, and when to choose each method

---

## 1) Goal & mental model

You want a phone to act like a robot computer:
- Phone publishes sensors (IMU, camera, etc.)
- PC runs heavier compute (mapping, planning, ML training)
- Optionally PC sends commands back to phone (or to a microcontroller)
- You prefer communication that can support **100–300 Hz** command streams with low latency/jitter

---

## 2) Method A (Recommended): ROS 2 node running on Android (DDS directly over Wi‑Fi)

### What it means
If your Android app is a real **ROS 2 node**, it can do:
- Topic pub/sub over Wi‑Fi to a ROS 2 PC
- Discovery + transport handled by DDS middleware
- DDS typically uses **UDP multicast** for discovery and **UDP unicast** for data

### Why it’s attractive
- No special “bridge” needed
- You get full ROS tooling on the PC side (rviz, rosbag, TF, etc.)
- With correct QoS (often BEST_EFFORT for sensors), it’s typically fast enough for high-rate topics

### Key repos
- **ros2_android** (Android bindings / build system)
  - https://github.com/ros2-java/ros2_android
- **ros2_java** (Java/JVM ROS 2 bindings used by Android)
  - https://github.com/ros2-java/ros2_java
- **ros2_android_examples**
  - https://github.com/ros2-java/ros2_android_examples
- **ros2-android-tutorial** (community tutorial repo)
  - https://github.com/songshan0321/ros2-android-tutorial

### Practical tips
- DDS discovery can be sensitive to Wi‑Fi networks that block multicast.
- If discovery fails, try:
  - Same subnet / simpler router
  - Different DDS implementation (CycloneDDS vs FastDDS) on the PC
  - Unicast discovery configuration (advanced)

---

## 3) Method B: rosbridge / web bridge (WebSocket or TCP, JSON protocol)

### What it means
You run a “bridge server” on the ROS side. The phone (or a web/mobile app) connects via:
- **WebSocket** (most common)
- sometimes **TCP**

Messages are encoded (often JSON) and forwarded to ROS topics.

### Strengths
- Easy to integrate with mobile/web stacks
- Great for dashboards, teleop UIs, quick prototypes

### Weaknesses
- Extra overhead (JSON serialization + WebSocket framing)
- Often not ideal for tight 100–300 Hz control loops (though it can work at lower rates)

### Key repos
- **ROS1 rosbridge_suite**
  - https://github.com/RobotWebTools/rosbridge_suite
- **ROS2 web bridge**
  - https://github.com/RobotWebTools/ros2-web-bridge
- **roslibpy** (client library; commonly used from Python)
  - https://github.com/gramaziokohler/roslibpy

---

## 4) Method C: Custom UDP (fastest) + PC-side bridge to ROS/ROS2

### What it means
You implement your own UDP protocol:
- Phone sends UDP packets to PC (e.g., IMU, commands, status)
- PC runs a small “gateway node” that converts UDP <-> ROS topics
- (Optional) PC bridges ROS2 <-> ROS1 if needed

This is the “lowest overhead” approach and can hit high rates easily.

### Why do this if DDS already uses UDP?
ROS 2 DDS uses UDP internally, but it also includes:
- discovery
- QoS bookkeeping (reliability, history, liveliness)
- serialization formats and middleware layers

Custom UDP can be **lower latency and simpler** for very specific real-time streams, but you lose:
- ROS tooling simplicity
- standardized message typing unless you implement it

### UDP-to-ROS bridge repos (useful references)
#### ROS 2 / general bridges
- **network_bridge** (supports UDP/TCP; bridges ROS 2 topics across networks)
  - https://github.com/brow1633/network_bridge
- **simple_udp** (ROS 2 package for UDP-based bridging)
  - https://github.com/sgrsn/simple_udp

#### ROS 1 bridges (still useful patterns; can run on PC and then ROS1↔ROS2 bridge)
- **udp_bridge** (ROS topic forwarding over UDP, compression/rate limiting options)
  - https://github.com/rolker/udp_bridge
- **ros_udp_bridge** (ROS <-> UDP nodes)
  - https://github.com/Brazilian-Institute-of-Robotics/ros_udp_bridge
- **ros_udp_bridge** (simple example, odom -> UDP JSON)
  - https://github.com/plusangel/ros_udp_bridge

#### ROS1↔ROS2 bridge (if your PC needs ROS1 tools)
- **ros1_bridge**
  - https://github.com/ros2/ros1_bridge

### Packet transport choices (for high-rate commands)
- **UDP**: best latency/overhead; accept packet loss or implement lightweight ACKs if needed
- **TCP**: reliable but may add jitter under loss (retransmits, congestion control)
- **WebSocket**: convenient but heavier; best for UI/teleop, not strict control

---

## 5) ADB: What it is and why it matters

**ADB (Android Debug Bridge)** is the standard Android developer tool that lets your PC communicate with an Android device for:
- installing/debugging apps
- reading logs (`adb logcat`)
- shell access (`adb shell`)
- file push/pull

ADB comes with **Android Studio** (Android SDK platform-tools).
If you have Android Studio installed, you almost certainly already have ADB.

ADB is for **debugging/dev workflow**, not your robot’s runtime comms.

---

## 6) ROS 2 QoS essentials for Wi‑Fi robots

### Default when you pass an integer like `10`
In rclpy/rclcpp, passing `10` usually means:
- history: KEEP_LAST
- depth: 10
- reliability: RELIABLE
- durability: VOLATILE

### Recommended patterns
#### High-rate sensors (IMU, camera)
- reliability: **BEST_EFFORT**
- history: KEEP_LAST
- depth: **1–5** (to avoid stale backlog)

#### Commands (velocity/joint targets)
- reliability: often **BEST_EFFORT**
- depth: **1** (only latest matters)

#### Critical low-rate state / configuration
- reliability: **RELIABLE**
- (sometimes durability TRANSIENT_LOCAL for latched-like behavior)

### Important gotcha
Publisher/subscriber QoS must be compatible; otherwise they may not connect.

---

## 7) Choosing a method (quick guide)

### Use Method A (ROS 2 on Android via DDS) if:
- You want “real ROS 2 nodes” on the phone
- You want minimal custom networking code
- You want PC tools (rviz, rosbag, TF) easily
- Your network allows DDS discovery

### Use Method B (rosbridge / WebSocket) if:
- You’re building a dashboard/teleop UI quickly
- You’re okay with moderate rates (often < ~100 Hz)

### Use Method C (custom UDP + gateway) if:
- You need very tight latency/jitter control at 100–300 Hz+
- You want full control over packet formats and loss-handling
- You’re okay writing/maintaining a gateway node

---

## 8) Suggested “phone-as-robot-brain” architecture (common in labs)

Even if the phone is the “brain”, low-level motor control is usually on a microcontroller:

- Phone: vision + IMU + high-level planning
- Microcontroller: PWM/torque control loops
- PC (optional): mapping, training, visualization

Communication:
- Phone <-> PC: ROS 2 DDS over Wi‑Fi (Method A) or UDP custom (Method C)
- Phone <-> MCU: USB / UART / BLE (depending on hardware)

---

## 9) Handy checklist for 100–300 Hz command streams

- Prefer BEST_EFFORT for high-rate streams
- Keep depth small (1–5)
- Measure latency/jitter:
  - include timestamp in messages
  - log receive time on PC and phone
- Avoid RELIABLE for high-rate sensors over lossy Wi‑Fi unless you *really* need delivery guarantees
- Consider wired (USB/Ethernet) if you need deterministic timing

---
