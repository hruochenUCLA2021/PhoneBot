import socket
import threading
from typing import Any, Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Quaternion, Vector3
from sensor_msgs.msg import BatteryState, Imu
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from motor_interfaces.msg import MotorState

from . import udp_protocol


class UdpToRos2ImmediateBridge(Node):
    """
    Immediate bridge: publish ROS2 messages as soon as a UDP packet is received and parsed.

    This is useful to measure the *actual* end-to-end UDP receive + publish rate, without
    throttling via a periodic timer.
    """

    def __init__(self) -> None:
        super().__init__("phonebot_udp_bridge_immediate")

        self.declare_parameter("bind_ip", "0.0.0.0")
        self.declare_parameter("bind_port", 5005)
        self.declare_parameter("topic_ns", "phonebot")
        # ROS topic to publish motor commands to (from Android -> UDP -> this bridge)
        self.declare_parameter("motor_topic", "/phonebot/motor_cmd")
        # ROS topic to publish torque enable to (from Android -> UDP -> this bridge)
        self.declare_parameter("torque_topic", "/phonebot/torque_enable")
        # ROS topic to subscribe motor state from (Pi HWdriver -> ROS2 -> this bridge -> UDP -> Android)
        self.declare_parameter("motor_state_full_topic", "/phonebot/motor_state_full")
        self.declare_parameter("android_ip", "192.168.20.2")
        self.declare_parameter("android_port", 6006)

        bind_ip = self.get_parameter("bind_ip").get_parameter_value().string_value
        bind_port = self.get_parameter("bind_port").get_parameter_value().integer_value
        topic_ns = self.get_parameter("topic_ns").get_parameter_value().string_value.strip("/")
        motor_topic = self.get_parameter("motor_topic").get_parameter_value().string_value
        torque_topic = self.get_parameter("torque_topic").get_parameter_value().string_value
        motor_state_full_topic = self.get_parameter("motor_state_full_topic").get_parameter_value().string_value
        self._android_ip = self.get_parameter("android_ip").get_parameter_value().string_value
        self._android_port = int(self.get_parameter("android_port").get_parameter_value().integer_value)

        self.pub_imu = self.create_publisher(Imu, f"/{topic_ns}/imu", 10)
        self.pub_imu_game = self.create_publisher(Imu, f"/{topic_ns}/imu_game", 10)
        self.pub_batt = self.create_publisher(BatteryState, f"/{topic_ns}/battery", 10)
        # (Legacy) `motor_state` from SENSOR packets is no longer used in the current architecture.
        # self.pub_motor_state = self.create_publisher(JointState, f"/{topic_ns}/motor_state", 10)

        # Android -> ROS2
        self.pub_motor_cmd = self.create_publisher(JointState, motor_topic, 10)
        self.pub_torque = self.create_publisher(Bool, torque_topic, 10)

        # Pi -> Android
        self._motor_state_sub = self.create_subscription(
            MotorState, motor_state_full_topic, self._on_motor_state_full, 10
        )

        # (Legacy) Subscribe to motor commands and forward to Android via UDP.
        # self._motor_sub = self.create_subscription(JointState, motor_topic, self._on_motor_cmd, 10)

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind((bind_ip, int(bind_port)))
        self._sock.settimeout(1.0)
        self.get_logger().info(f"Listening UDP (immediate) on {bind_ip}:{bind_port}")

        self._tx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.get_logger().info(f"Motor UDP target (immediate): {self._android_ip}:{self._android_port}")

        self._tx_seq: int = 0

        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

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
            if pkt is not None:
                self._publish_from_sensor(pkt)
                continue

            m = udp_protocol.try_parse_motor(payload)
            if m is not None:
                self._publish_motor_cmd(m)
                continue

            t = udp_protocol.try_parse_torque(payload)
            if t is not None:
                self._publish_torque(t)
                continue

    def _publish_from_sensor(self, pkt: udp_protocol.SensorPacket) -> None:
        now = self.get_clock().now().to_msg()

        accel = pkt.accel
        gyro = pkt.gyro

        msg = Imu()
        msg.header.stamp = now
        msg.header.frame_id = "phone"
        msg.orientation = _quat_from_wxyz(pkt.rot_quat_wxyz)
        msg.angular_velocity = _vec3_from_xyz(gyro)
        msg.linear_acceleration = _vec3_from_xyz(accel)
        msg.orientation_covariance[0] = -1.0
        msg.angular_velocity_covariance[0] = -1.0
        msg.linear_acceleration_covariance[0] = -1.0
        self.pub_imu.publish(msg)

        msg2 = Imu()
        msg2.header.stamp = now
        msg2.header.frame_id = "phone_game"
        msg2.orientation = _quat_from_wxyz(pkt.game_quat_wxyz)
        msg2.angular_velocity = _vec3_from_xyz(gyro)
        msg2.linear_acceleration = _vec3_from_xyz(accel)
        msg2.orientation_covariance[0] = -1.0
        msg2.angular_velocity_covariance[0] = -1.0
        msg2.linear_acceleration_covariance[0] = -1.0
        self.pub_imu_game.publish(msg2)

        bmsg = BatteryState()
        bmsg.header.stamp = now
        bmsg.header.frame_id = "phone"
        if pkt.batt_percent >= 0.0:
            bmsg.percentage = float(pkt.batt_percent) / 100.0
        else:
            bmsg.percentage = float("nan")

        bmsg.power_supply_status = _map_battery_status_code(pkt.batt_status)
        self.pub_batt.publish(bmsg)

        # No motor state is carried inside SENSOR packets in the current architecture.

    def _publish_motor_cmd(self, pkt: udp_protocol.MotorPacket) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [f"motor_{i+1}" for i in range(len(pkt.pos))]
        msg.position = [float(x) for x in pkt.pos]
        msg.velocity = [float(x) for x in pkt.vel]
        msg.effort = [float(x) for x in pkt.tau]
        self.pub_motor_cmd.publish(msg)

    def _publish_torque(self, pkt: udp_protocol.TorquePacket) -> None:
        msg = Bool()
        msg.data = bool(pkt.enable)
        self.pub_torque.publish(msg)

    def _on_motor_state_full(self, msg: MotorState) -> None:
        # Forward present motor state to Android via UDP using MOTOR_STATUS packet format.
        ts_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        self._tx_seq = (self._tx_seq + 1) & 0xFFFF_FFFF
        payload = udp_protocol.pack_motor_status(
            self._tx_seq,
            ts_ns,
            pwm_percent=list(msg.present_pwm_percent),
            load_percent=list(msg.present_load_percent),
            pos_rad=list(msg.present_position_rad),
            vel_rad_s=list(msg.present_velocity_rad_s),
            vin_v=list(msg.present_input_voltage_v),
            temp_c=list(msg.present_temperature_c),
        )
        try:
            self._tx_sock.sendto(payload, (self._android_ip, self._android_port))
        except Exception as e:
            self.get_logger().warn(f"MotorState UDP send failed: {e}")


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
    node = UdpToRos2ImmediateBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


