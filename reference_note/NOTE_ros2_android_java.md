# NOTE_ros2_android_java.md
_Last updated: 2026‑02‑05_

This note clarifies the relationship between **ros2_android** and **ros2_java**, and gives a **practical Kotlin/Android example** of how ROS 2 is typically used on a phone.

---

## 1) The 10‑second explanation

- **ros2_java** = the **ROS 2 client library API for Java/JVM** (like `rclcpp` for C++ or `rclpy` for Python, but for Java).
- **ros2_android** = the **Android platform packaging/build/runtime** that makes ROS 2 (and DDS) actually run on Android, and exposes the Java APIs on Android.

✅ On Android you typically use **both**:
- You **code against** the `ros2_java` APIs
- You **depend on** `ros2_android` to provide Android‑built native ROS 2 + DDS libraries and Gradle integration

---

## 2) Mental model / layering

Think of it like this:

```
Your Kotlin/Java Android app code
    ↓ (calls)
ros2_java  (Java API: Node, Publisher, Subscriber, QoS, messages)
    ↓ (uses native libs)
ros2_android (Android build + packaging of ROS2 core + DDS + JNI/JVM glue)
    ↓
DDS middleware (CycloneDDS / FastDDS etc., using UDP typically)
    ↓
Wi‑Fi network
    ↓
PC running ROS 2 (rclcpp/rclpy nodes)
```

### Analogy
- On Ubuntu: you write Python with **rclpy**, and ROS2 is installed by apt.
- On Android: you write Kotlin/Java with **ros2_java**, and ROS2 is provided/packaged by **ros2_android**.

---

## 3) Which one should you use?

### If you build an Android app that is a ROS 2 node
✅ Use **ros2_android** as your dependency/platform.
You will still write code using **ros2_java** APIs.

### If you build a Java program on desktop/server (not Android)
✅ Use **ros2_java** directly (no need for ros2_android).

---

## 4) Kotlin vs Java on Android

Kotlin is fully interoperable with Java.

✅ You can write your Android app in **Kotlin** and call **ros2_java** APIs directly.
The ROS2 Android stack is Java‑API based, but Kotlin works fine.

---

## 5) Practical workflow (what you actually do)

### Step A — Build/obtain Android ROS2 artifacts (handled by ros2_android)
Typical approach:
1. Clone ros2_android
2. Use its scripts/build instructions to produce Android AARs + native libs
3. Add those AARs / Gradle dependencies to your Android Studio project

Key repo:
https://github.com/ros2-java/ros2_android

### Step B — In your Android app, initialize ROS, create node, publishers/subscribers
Use ros2_java APIs:
https://github.com/ros2-java/ros2_java

---

## 6) Minimal Kotlin example (Publisher + Subscriber)

> Notes:
> - The exact package/class names can vary depending on the ros2_java version and how you import message types.
> - Treat this as a “shape of the code” reference.
> - The important part is: you create a node, create publisher/subscriber with QoS, spin in a background thread.

### 6.1 QoS helper presets (recommended)
- Sensors: BEST_EFFORT, depth 5
- Commands: BEST_EFFORT, depth 1
- Critical state: RELIABLE

### 6.2 Kotlin skeleton

```kotlin
import android.os.Bundle
import androidx.appcompat.app.AppCompatActivity
import java.util.concurrent.Executors

// ROS2 Java imports (exact paths depend on your ros2_java version)
import org.ros2.rcljava.RCLJava
import org.ros2.rcljava.node.Node
import org.ros2.rcljava.publisher.Publisher
import org.ros2.rcljava.subscription.Subscription
import org.ros2.rcljava.qos.QoSProfile
import org.ros2.rcljava.qos.QoSReliabilityPolicy
import org.ros2.rcljava.qos.QoSHistoryPolicy

// Example message types (replace with what you need)
// Many projects use std_msgs, geometry_msgs, sensor_msgs
import std_msgs.msg.String as StringMsg
// import sensor_msgs.msg.Imu
// import geometry_msgs.msg.Twist

class MainActivity : AppCompatActivity() {

  private val executor = Executors.newSingleThreadExecutor()
  private var node: Node? = null
  private var chatterPub: Publisher<StringMsg>? = null
  private var chatterSub: Subscription<StringMsg>? = null

  override fun onCreate(savedInstanceState: Bundle?) {
    super.onCreate(savedInstanceState)
    setContentView(R.layout.activity_main)

    // Initialize ROS2
    RCLJava.rclJavaInit()

    // Create node
    node = RCLJava.createNode("android_node")

    // QoS: default-like but explicitly set (example)
    val qos = QoSProfile(
      QoSHistoryPolicy.KEEP_LAST,
      10
    ).apply {
      reliability = QoSReliabilityPolicy.RELIABLE
    }

    // Publisher
    chatterPub = node!!.createPublisher(StringMsg::class.java, "chatter", qos)

    // Subscriber
    chatterSub = node!!.createSubscription(
      StringMsg::class.java,
      "chatter",
      qos
    ) { msg ->
      // This callback runs when a message arrives
      android.util.Log.i("ROS2", "Received: ${msg.data}")
    }

    // Spin node in background thread
    executor.execute {
      while (!Thread.currentThread().isInterrupted) {
        RCLJava.spinOnce(node)
        try { Thread.sleep(5) } catch (_: InterruptedException) { break }
      }
    }

    // Publish something
    publishHello()
  }

  private fun publishHello() {
    val msg = StringMsg()
    msg.data = "Hello from Android ROS2"
    chatterPub?.publish(msg)
  }

  override fun onDestroy() {
    super.onDestroy()
    executor.shutdownNow()
    node?.dispose()
    RCLJava.shutdown()
  }
}
```

---

## 7) Example: IMU publisher QoS for Wi‑Fi (recommended)

For IMU at 100–400 Hz, use BEST_EFFORT to avoid stalls:

```kotlin
val imuQos = QoSProfile(
  QoSHistoryPolicy.KEEP_LAST,
  5
).apply {
  reliability = QoSReliabilityPolicy.BEST_EFFORT
}
```

Then create publisher:

```kotlin
imuPub = node!!.createPublisher(Imu::class.java, "imu/data", imuQos)
```

---

## 8) Example architecture for phone ↔ PC

### Phone (Android)
- Read IMU via Android Sensor APIs
- Read camera via Camera2 / CameraX
- Publish to ROS2 topics

### PC (Ubuntu)
- Subscribe, run localization / mapping / planning
- Publish control commands back to phone or microcontroller

Communication:
- ROS2 DDS over Wi‑Fi (UDP internally)

---

## 9) Troubleshooting checklist (common)

### DDS discovery issues
Symptoms: phone and PC don’t “see” each other.

Try:
- same Wi‑Fi network/subnet
- disable client isolation on router
- ensure multicast not blocked (common on campus networks)
- try a phone hotspot (simple network)

### QoS mismatch
Symptoms: topics exist but no messages received.

Ensure:
- publisher/subscriber reliability compatible (BEST_EFFORT vs RELIABLE)
- history/depth reasonable

---

## 10) Repos (links)

- ros2_android:
  https://github.com/ros2-java/ros2_android
- ros2_java:
  https://github.com/ros2-java/ros2_java
- ros2_android_examples:
  https://github.com/ros2-java/ros2_android_examples
- ros2 android tutorial:
  https://github.com/songshan0321/ros2-android-tutorial

---

## 11) Key takeaway

✅ On Android, you typically depend on **ros2_android** (Android packaging/runtime) and write code using **ros2_java** (Java API).

Kotlin works because it can call Java APIs directly.

---
