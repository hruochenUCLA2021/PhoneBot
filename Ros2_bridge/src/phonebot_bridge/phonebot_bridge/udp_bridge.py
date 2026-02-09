import json
import socket
import threading
from dataclasses import dataclass
from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Quaternion, Vector3
from sensor_msgs.msg import BatteryState, Imu


@dataclass
class LatestPacket:
    data: Optional[Dict[str, Any]] = None
    bytes_len: int = 0
    packets: int = 0


class UdpToRos2Bridge(Node):
    """
    Receives JSON-over-UDP from the Android app and republishes as ROS2 topics.

    Topics (default):
      - /phonebot/imu        (sensor_msgs/Imu)          from TYPE_ROTATION_VECTOR
      - /phonebot/imu_game   (sensor_msgs/Imu)          from TYPE_GAME_ROTATION_VECTOR
      - /phonebot/battery    (sensor_msgs/BatteryState)
    """

    def __init__(self) -> None:
        super().__init__("phonebot_udp_bridge")

        self.declare_parameter("bind_ip", "0.0.0.0")
        self.declare_parameter("bind_port", 5005)
        self.declare_parameter("topic_ns", "phonebot")
        self.declare_parameter("publish_hz", 50.0)

        bind_ip = self.get_parameter("bind_ip").get_parameter_value().string_value
        bind_port = self.get_parameter("bind_port").get_parameter_value().integer_value
        topic_ns = self.get_parameter("topic_ns").get_parameter_value().string_value.strip("/")
        publish_hz = self.get_parameter("publish_hz").get_parameter_value().double_value

        self._latest = LatestPacket()
        self._lock = threading.Lock()

        self.pub_imu = self.create_publisher(Imu, f"/{topic_ns}/imu", 10)
        self.pub_imu_game = self.create_publisher(Imu, f"/{topic_ns}/imu_game", 10)
        self.pub_batt = self.create_publisher(BatteryState, f"/{topic_ns}/battery", 10)

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind((bind_ip, int(bind_port)))
        self._sock.settimeout(1.0)

        self.get_logger().info(f"Listening UDP on {bind_ip}:{bind_port}")

        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

        period = 1.0 / max(1.0, publish_hz)
        self.create_timer(period, self._publish_latest)

    def destroy_node(self):
        try:
            self._sock.close()
        except Exception:
            pass
        super().destroy_node()

    def _rx_loop(self) -> None:
        while rclpy.ok():
            try:
                payload, _addr = self._sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                return

            try:
                data = json.loads(payload.decode("utf-8", errors="replace"))
            except Exception as e:
                self.get_logger().warn(f"Bad JSON packet: {e}")
                continue

            with self._lock:
                self._latest.data = data
                self._latest.bytes_len = len(payload)
                self._latest.packets += 1

    def _publish_latest(self) -> None:
        with self._lock:
            data = self._latest.data

        if not isinstance(data, dict):
            return

        now = self.get_clock().now().to_msg()

        accel = data.get("accel")
        gyro = data.get("gyro")
        rotvec = data.get("rotvec") or {}
        game = data.get("game_rotvec") or {}
        batt = data.get("battery") or {}

        # /imu (TYPE_ROTATION_VECTOR)
        msg = Imu()
        msg.header.stamp = now
        msg.header.frame_id = "phone"
        msg.orientation = _quat_from_list(rotvec.get("quat"))
        msg.angular_velocity = _vec3_from_list(gyro)
        msg.linear_acceleration = _vec3_from_list(accel)
        msg.orientation_covariance[0] = -1.0
        msg.angular_velocity_covariance[0] = -1.0
        msg.linear_acceleration_covariance[0] = -1.0
        self.pub_imu.publish(msg)

        # /imu_game (TYPE_GAME_ROTATION_VECTOR)
        msg2 = Imu()
        msg2.header.stamp = now
        msg2.header.frame_id = "phone_game"
        msg2.orientation = _quat_from_list(game.get("quat"))
        msg2.angular_velocity = _vec3_from_list(gyro)
        msg2.linear_acceleration = _vec3_from_list(accel)
        msg2.orientation_covariance[0] = -1.0
        msg2.angular_velocity_covariance[0] = -1.0
        msg2.linear_acceleration_covariance[0] = -1.0
        self.pub_imu_game.publish(msg2)

        # /battery
        bmsg = BatteryState()
        bmsg.header.stamp = now
        bmsg.header.frame_id = "phone"
        percent = batt.get("percent")
        if isinstance(percent, (int, float)):
            bmsg.percentage = float(percent) / 100.0
        else:
            bmsg.percentage = float("nan")

        status = batt.get("status")
        bmsg.power_supply_status = _map_battery_status(status)
        self.pub_batt.publish(bmsg)


def _vec3_from_list(v) -> Vector3:
    msg = Vector3()
    if isinstance(v, list) and len(v) >= 3:
        msg.x = float(v[0])
        msg.y = float(v[1])
        msg.z = float(v[2])
    return msg


def _quat_from_list(q) -> Quaternion:
    msg = Quaternion()
    # Android app sends [w,x,y,z]. ROS expects x,y,z,w.
    if isinstance(q, list) and len(q) >= 4:
        w, x, y, z = [float(q[i]) for i in range(4)]
        msg.x = x
        msg.y = y
        msg.z = z
        msg.w = w
    return msg


def _map_battery_status(status: Any) -> int:
    if status == "CHARGING":
        return BatteryState.POWER_SUPPLY_STATUS_CHARGING
    if status == "DISCHARGING":
        return BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
    if status == "FULL":
        return BatteryState.POWER_SUPPLY_STATUS_FULL
    if status == "NOT_CHARGING":
        return BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING
    return BatteryState.POWER_SUPPLY_STATUS_UNKNOWN


def main() -> None:
    rclpy.init()
    node = UdpToRos2Bridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


