import os

import numpy as np
import mujoco

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from ament_index_python.packages import get_package_share_directory

from .simulator_mujoco_hrc import simulator_mujoco_v1


# MuJoCo joint order from phonebot_general_alternative_imu.xml.
# qpos layout: [0:3]=base_pos, [3:7]=base_quat(wxyz), [7:20]=joints.
# Remap this list if the Android motor IDs (1..13) don't match this order.
JOINT_NAMES = [
    "l_hip_pitch_joint",   # motor 1
    "l_hip_roll_joint",    # motor 2
    "l_hip_thigh_joint",   # motor 3
    "l_hip_calf_joint",    # motor 4
    "l_ankle_pitch_joint", # motor 5
    "l_ankle_roll_joint",  # motor 6
    "r_hip_pitch_joint",   # motor 7
    "r_hip_roll_joint",    # motor 8
    "r_hip_thigh_joint",   # motor 9
    "r_hip_calf_joint",    # motor 10
    "r_ankle_pitch_joint", # motor 11
    "r_ankle_roll_joint",  # motor 12
    "base_to_trunk",       # motor 13
]

NUM_JOINTS = len(JOINT_NAMES)
BASE_QPOS_SIZE = 7  # 3 pos + 4 quat
DEFAULT_BASE_POS = np.array([0.0, 0.0, 0.33])
DEFAULT_BASE_QUAT = np.array([1.0, 0.0, 0.0, 0.0])  # wxyz identity


def _quat_mul(a, b):
    """Hamilton product of two quaternions in [w,x,y,z] order."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw*bw - ax*bx - ay*by - az*bz,
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
    ])


def _quat_conj(q):
    """Conjugate (inverse for unit quaternion) in [w,x,y,z] order."""
    return np.array([q[0], -q[1], -q[2], -q[3]])


def _quat_from_axis_angle_z(theta):
    """Quaternion for rotation of `theta` radians about the Z axis, [w,x,y,z]."""
    return np.array([np.cos(theta / 2.0), 0.0, 0.0, np.sin(theta / 2.0)])


# --- Frame conversion ---
#
# Phone mounting: screen faces forward, phone top (+Y) points up.
#
# Phone body axes:  X = right (of screen), Y = up (along screen), Z = out-of-screen
# Robot base axes:  X = right (sideways),   Y = forward,           Z = up
#   (from XML: "base_motor_link frame is (z up, y forward, x sideways)")
#   +X_robot = right leg side.
#
# When looking at the back of the phone (standing behind the robot, both facing forward):
#   phone +X points to YOUR left = robot's LEFT = -X_robot
#   phone +Z comes out of the screen = forward = +Y_robot
#   phone +Y points up = +Z_robot  (by right-hand rule: base_Z = (-phone_X) x phone_Z = phone_Y)
#
# Rotation matrix R (v_base = R * v_phone):
#   [-1   0   0 ]
#   [ 0   0   1 ]
#   [ 0   1   0 ]
#
# Quaternion (wxyz): [0, 0, sqrt2/2, sqrt2/2]
_s = np.sqrt(2.0) / 2.0
Q_PHONE_TO_BASE = np.array([0.0, 0.0, _s, _s])

# IMU site orientation relative to trunk_link body frame (from XML).
# <site name="imu" quat="0.707107 0.000000 0.000000 0.707107"/>  -> 90 deg about Z
Q_TRUNK_TO_IMU_SITE = np.array([_s, 0.0, 0.0, _s])
Q_IMU_SITE_TO_TRUNK = _quat_conj(Q_TRUNK_TO_IMU_SITE)


class PhonebotVisualizer(Node):
    def __init__(self):
        super().__init__("phonebot_visualizer")

        _default_model = os.path.join(
            get_package_share_directory("robot_visualizer"),
            "model", "model_phonebot",
            "scene_joystick_flat_terrain_alternative_imu.xml",
        )
        self.declare_parameter("model_path", _default_model)
        self.declare_parameter("render_hz", 30.0)
        self.declare_parameter("imu_topic", "/phonebot/imu_game")
        self.declare_parameter("motor_state_topic", "/phonebot/motor_state")

        model_path = self.get_parameter("model_path").get_parameter_value().string_value
        render_hz = self.get_parameter("render_hz").get_parameter_value().double_value
        imu_topic = self.get_parameter("imu_topic").get_parameter_value().string_value
        motor_topic = self.get_parameter("motor_state_topic").get_parameter_value().string_value

        self._base_quat = DEFAULT_BASE_QUAT.copy()
        self._q_trunk = DEFAULT_BASE_QUAT.copy()  # latest trunk orientation in world
        self._joint_pos = np.zeros(NUM_JOINTS)

        self.get_logger().info(f"Loading MuJoCo model: {model_path}")
        self.sim = simulator_mujoco_v1()
        self.sim.load_model(model_path)
        self.sim.create_viewer()

        # Build joint-name -> qpos-index lookup from the loaded model
        self._joint_qpos_idx = {}
        for name in JOINT_NAMES:
            jid = mujoco.mj_name2id(self.sim.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid < 0:
                self.get_logger().warn(f"Joint '{name}' not found in model")
            else:
                self._joint_qpos_idx[name] = self.sim.model.jnt_qposadr[jid]

        # Freejoint qpos address
        fj_id = mujoco.mj_name2id(self.sim.model, mujoco.mjtObj.mjOBJ_JOINT, "floating_base")
        self._base_qpos_adr = self.sim.model.jnt_qposadr[fj_id] if fj_id >= 0 else 0

        self.create_subscription(Imu, imu_topic, self._on_imu, 10)
        self.create_subscription(JointState, motor_topic, self._on_motor_state, 10)

        render_period = 1.0 / render_hz
        self._render_timer = self.create_timer(render_period, self._render_callback)

        self.get_logger().info(
            f"Visualizer ready — render {render_hz:.0f} Hz, "
            f"IMU: {imu_topic}, motors: {motor_topic}"
        )

    def _on_imu(self, msg: Imu):
        q = msg.orientation
        # ROS Quaternion is (x, y, z, w) -> convert to [w,x,y,z]
        q_phone = np.array([q.w, q.x, q.y, q.z])
        # Phone is mounted on trunk. Convert phone body frame to trunk/base body frame.
        # q_trunk_in_world = q_phone_in_world * Q_PHONE_TO_BASE
        self._q_trunk[:] = _quat_mul(q_phone, Q_PHONE_TO_BASE)

    def _on_motor_state(self, msg: JointState):
        n = min(NUM_JOINTS, len(msg.position))
        for i in range(n):
            self._joint_pos[i] = msg.position[i]

    def _render_callback(self):
        if not self.sim.viewer.is_alive:
            self.get_logger().info("Viewer closed — shutting down")
            self.destroy_node()
            rclpy.shutdown()
            return

        d = self.sim.data
        a = self._base_qpos_adr

        d.qpos[a : a + 3] = DEFAULT_BASE_POS

        for idx, name in enumerate(JOINT_NAMES):
            qi = self._joint_qpos_idx.get(name)
            if qi is not None:
                d.qpos[qi] = self._joint_pos[idx]

        # Compute base orientation from trunk orientation.
        # Kinematic chain: q_trunk_in_world = q_base * Rz(theta)
        #   where theta = base_to_trunk hinge joint angle (motor 13).
        # Solve: q_base = q_trunk_in_world * Rz(-theta)
        trunk_theta = self._joint_pos[12]  # base_to_trunk = motor 13
        q_base = _quat_mul(self._q_trunk, _quat_from_axis_angle_z(-trunk_theta))
        d.qpos[a + 3 : a + 7] = q_base

        mujoco.mj_forward(self.sim.model, d)
        self.sim.viewer.render()

    def destroy_node(self):
        try:
            self.sim.viewer.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PhonebotVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
