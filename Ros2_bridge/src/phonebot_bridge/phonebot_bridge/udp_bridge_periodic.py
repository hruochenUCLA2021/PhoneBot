import socket
import threading
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Quaternion, Vector3
from sensor_msgs.msg import BatteryState, Imu
from sensor_msgs.msg import JointState

from . import udp_protocol


@dataclass
class LatestPacket:
    data: Optional[udp_protocol.SensorPacket] = None
    bytes_len: int = 0
    packets: int = 0


class UdpToRos2Bridge(Node):
    """
    Receives binary UDP from the Android app and republishes as ROS2 topics.

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
        self.declare_parameter("motor_topic", "/phonebot/motor_cmd")
        self.declare_parameter("android_ip", "192.168.20.2")
        self.declare_parameter("android_port", 6006)
        self.declare_parameter("motor_send_hz", 50.0)

        bind_ip = self.get_parameter("bind_ip").get_parameter_value().string_value
        bind_port = self.get_parameter("bind_port").get_parameter_value().integer_value
        topic_ns = self.get_parameter("topic_ns").get_parameter_value().string_value.strip("/")
        publish_hz = self.get_parameter("publish_hz").get_parameter_value().double_value
        motor_topic = self.get_parameter("motor_topic").get_parameter_value().string_value
        self._android_ip = self.get_parameter("android_ip").get_parameter_value().string_value
        self._android_port = int(self.get_parameter("android_port").get_parameter_value().integer_value)
        motor_send_hz = self.get_parameter("motor_send_hz").get_parameter_value().double_value

        self._latest = LatestPacket()
        self._latest_motor: Optional[JointState] = None
        self._lock = threading.Lock()

        self.pub_imu = self.create_publisher(Imu, f"/{topic_ns}/imu", 10)
        self.pub_imu_game = self.create_publisher(Imu, f"/{topic_ns}/imu_game", 10)
        self.pub_batt = self.create_publisher(BatteryState, f"/{topic_ns}/battery", 10)

        self._motor_sub = self.create_subscription(JointState, motor_topic, self._on_motor_cmd, 10)

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind((bind_ip, int(bind_port)))
        self._sock.settimeout(1.0)

        self.get_logger().info(f"Listening UDP on {bind_ip}:{bind_port}")

        self._tx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.get_logger().info(f"Motor UDP target (periodic): {self._android_ip}:{self._android_port}")

        self._motor_seq: int = 0

        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

        period = 1.0 / max(1.0, publish_hz)
        self.create_timer(period, self._publish_latest)

        motor_period = 1.0 / max(1.0, motor_send_hz)
        self.create_timer(motor_period, self._send_latest_motor)

    def destroy_node(self):
        try:
            self._sock.close()
        except Exception:
            pass
        try:
            self._tx_sock.close()
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

            pkt = udp_protocol.try_parse_sensor(payload)
            if pkt is None:
                continue

            with self._lock:
                self._latest.data = pkt
                self._latest.bytes_len = len(payload)
                self._latest.packets += 1

    def _publish_latest(self) -> None:
        with self._lock:
            data = self._latest.data

        if data is None:
            return

        now = self.get_clock().now().to_msg()

        accel = data.accel
        gyro = data.gyro

        # /imu (TYPE_ROTATION_VECTOR)
        msg = Imu()
        msg.header.stamp = now
        msg.header.frame_id = "phone"
        msg.orientation = _quat_from_wxyz(data.rot_quat_wxyz)
        msg.angular_velocity = _vec3_from_xyz(gyro)
        msg.linear_acceleration = _vec3_from_xyz(accel)
        msg.orientation_covariance[0] = -1.0
        msg.angular_velocity_covariance[0] = -1.0
        msg.linear_acceleration_covariance[0] = -1.0
        self.pub_imu.publish(msg)

        # /imu_game (TYPE_GAME_ROTATION_VECTOR)
        msg2 = Imu()
        msg2.header.stamp = now
        msg2.header.frame_id = "phone_game"
        msg2.orientation = _quat_from_wxyz(data.game_quat_wxyz)
        msg2.angular_velocity = _vec3_from_xyz(gyro)
        msg2.linear_acceleration = _vec3_from_xyz(accel)
        msg2.orientation_covariance[0] = -1.0
        msg2.angular_velocity_covariance[0] = -1.0
        msg2.linear_acceleration_covariance[0] = -1.0
        self.pub_imu_game.publish(msg2)

        # /battery
        bmsg = BatteryState()
        bmsg.header.stamp = now
        bmsg.header.frame_id = "phone"
        if data.batt_percent >= 0.0:
            bmsg.percentage = float(data.batt_percent) / 100.0
        else:
            bmsg.percentage = float("nan")

        bmsg.power_supply_status = _map_battery_status_code(data.batt_status)
        self.pub_batt.publish(bmsg)

    def _on_motor_cmd(self, msg: JointState) -> None:
        with self._lock:
            self._latest_motor = msg

    def _send_latest_motor(self) -> None:
        with self._lock:
            msg = self._latest_motor
        if msg is None:
            return
        ts_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        pos = list(msg.position) if msg.position else []
        vel = list(msg.velocity) if msg.velocity else []
        tau = list(msg.effort) if msg.effort else []

        self._motor_seq = (self._motor_seq + 1) & 0xFFFF_FFFF
        payload = udp_protocol.pack_motor(self._motor_seq, ts_ns, pos, vel, tau)
        try:
            self._tx_sock.sendto(payload, (self._android_ip, self._android_port))
        except Exception as e:
            self.get_logger().warn(f"Motor UDP send failed: {e}")


def _vec3_from_xyz(v) -> Vector3:
    msg = Vector3()
    if v is not None:
        msg.x = float(v[0])
        msg.y = float(v[1])
        msg.z = float(v[2])
    return msg


def _quat_from_wxyz(q) -> Quaternion:
    msg = Quaternion()
    # Android app sends [w,x,y,z]. ROS expects x,y,z,w.
    w, x, y, z = [float(q[i]) for i in range(4)]
    msg.x = x
    msg.y = y
    msg.z = z
    msg.w = w
    return msg


def _map_battery_status_code(code: int) -> int:
    if code == 1:
        return BatteryState.POWER_SUPPLY_STATUS_CHARGING
    if code == 2:
        return BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
    if code == 3:
        return BatteryState.POWER_SUPPLY_STATUS_FULL
    if code == 4:
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


