import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState


DEFAULT_JOINT_NAMES = [
    # Left leg (6)
    "left_hip_yaw",
    "left_hip_roll",
    "left_hip_pitch",
    "left_knee",
    "left_ankle_pitch",
    "left_ankle_roll",
    # Right leg (6)
    "right_hip_yaw",
    "right_hip_roll",
    "right_hip_pitch",
    "right_knee",
    "right_ankle_pitch",
    "right_ankle_roll",
    # Waist (1)
    "waist_yaw",
]


class ExampleMotorPublisher(Node):
    """
    Periodically publishes a 13-DOF motor command as sensor_msgs/JointState:
      - position[]  (rad or your unit)
      - velocity[]  (rad/s)
      - effort[]    (torque Nm or your unit)

    This mimics an RL policy producing an action vector.
    """

    def __init__(self) -> None:
        super().__init__("phonebot_example_motor_pub")

        self.declare_parameter("topic", "/phonebot/motor_cmd")
        self.declare_parameter("hz", 50.0)
        self.declare_parameter("amp_pos", 0.25)
        self.declare_parameter("amp_vel", 0.0)
        self.declare_parameter("amp_tau", 0.0)

        topic = self.get_parameter("topic").get_parameter_value().string_value
        hz = self.get_parameter("hz").get_parameter_value().double_value
        self.amp_pos = self.get_parameter("amp_pos").get_parameter_value().double_value
        self.amp_vel = self.get_parameter("amp_vel").get_parameter_value().double_value
        self.amp_tau = self.get_parameter("amp_tau").get_parameter_value().double_value

        self.pub = self.create_publisher(JointState, topic, 10)
        self.names = list(DEFAULT_JOINT_NAMES)

        self._t = 0.0
        self._dt = 1.0 / max(1.0, hz)
        self.create_timer(self._dt, self._tick)

        self.get_logger().info(f"Publishing motor commands to {topic} at {1.0/self._dt:.1f} Hz")

    def _tick(self) -> None:
        self._t += self._dt

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.names

        # Simple demo policy: sin waves with different phases.
        pos = []
        vel = []
        tau = []
        for i in range(len(self.names)):
            phase = 0.5 * i
            pos.append(self.amp_pos * math.sin(2.0 * math.pi * 0.5 * self._t + phase))
            vel.append(self.amp_vel * math.cos(2.0 * math.pi * 0.5 * self._t + phase))
            tau.append(self.amp_tau * math.sin(2.0 * math.pi * 0.5 * self._t + phase))

        msg.position = pos
        msg.velocity = vel
        msg.effort = tau

        self.pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = ExampleMotorPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()



