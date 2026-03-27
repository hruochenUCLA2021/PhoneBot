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


def _rotate_vec_by_quat(q_wxyz, v_xyz):
    """Rotate 3D vector by quaternion q (wxyz): v' = q * (0,v) * conj(q)."""
    p = np.array([0.0, float(v_xyz[0]), float(v_xyz[1]), float(v_xyz[2])])
    qp = _quat_mul(q_wxyz, p)
    qpq = _quat_mul(qp, _quat_conj(q_wxyz))
    return qpq[1:4]


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
Q_PHONE_TO_TRUNK = np.array([0.0, 0.0, _s, _s])

# --- IMU site frame ---
#
# Both XMLs use the SAME imu-site local rotation:
#   quat="0.707107 0.000000 0.000000 0.707107"  -> 90 deg about +Z
#
# What changes across modes is which BODY the site is attached to:
#
# - normal mode  (PhonebotJoystickFlatTerrain):
#     imu site is under base_motor_link in `phonebot_general.xml`
# - alter mode   (PhonebotJoystickFlatTerrainAlter):
#     imu site is under trunk_link in `phonebot_general_alternative_imu.xml`
#
# We keep two names for clarity even though the numeric quaternion is identical.
Q_BASE_TO_IMU_SITE = np.array([_s, 0.0, 0.0, _s])
Q_TRUNK_TO_IMU_SITE = np.array([_s, 0.0, 0.0, _s])


class PhonebotVisualizer(Node):
    def __init__(self):
        super().__init__("phonebot_visualizer")

        _default_model = os.path.join(
            get_package_share_directory("robot_visualizer"),
            "model", "model_phonebot",
            "scene_joystick_flat_terrain.xml",
        )
        self.declare_parameter("model_path", _default_model)
        # "normal" => imu site on base link; "alter" => imu site on trunk link.
        self.declare_parameter("mode", "normal")
        self.declare_parameter("render_hz", 30.0)
        self.declare_parameter("imu_topic", "/phonebot/imu_game")
        self.declare_parameter("motor_state_topic", "/phonebot/motor_state")

        model_path = self.get_parameter("model_path").get_parameter_value().string_value
        self._mode = self.get_parameter("mode").get_parameter_value().string_value.strip().lower() or "normal"
        render_hz = self.get_parameter("render_hz").get_parameter_value().double_value
        imu_topic = self.get_parameter("imu_topic").get_parameter_value().string_value
        motor_topic = self.get_parameter("motor_state_topic").get_parameter_value().string_value

        self._base_quat = DEFAULT_BASE_QUAT.copy()
        self._q_trunk = DEFAULT_BASE_QUAT.copy()  # latest trunk orientation in world
        self._gyro_phone = np.zeros(3)
        self._accel_phone = np.zeros(3)
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

        # IMU site id (exists in both normal and alter XMLs with the same name "imu")
        self._imu_site_id = mujoco.mj_name2id(self.sim.model, mujoco.mjtObj.mjOBJ_SITE, "imu")

        self.create_subscription(Imu, imu_topic, self._on_imu, 10)
        self.create_subscription(JointState, motor_topic, self._on_motor_state, 10)

        render_period = 1.0 / render_hz
        self._render_timer = self.create_timer(render_period, self._render_callback)

        self.get_logger().info(
            f"Visualizer ready — render {render_hz:.0f} Hz, "
            f"mode={self._mode}, IMU: {imu_topic}, motors: {motor_topic}"
        )

    def _on_imu(self, msg: Imu):
        q = msg.orientation
        # ROS Quaternion is (x, y, z, w) -> convert to [w,x,y,z]
        q_phone = np.array([q.w, q.x, q.y, q.z])
        # Phone is mounted on trunk. Convert phone body frame to trunk/base body frame.
        # q_trunk_in_world = q_phone_in_world * Q_PHONE_TO_TRUNK
        self._q_trunk[:] = _quat_mul(q_phone, Q_PHONE_TO_TRUNK)
        self._gyro_phone[:] = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        self._accel_phone[:] = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])

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

        # --- Debug draw: gyro/accel axes at the policy's IMU site frame ---
        # Clear old markers if mujoco_viewer keeps them.
        for attr in ("markers", "_markers"):
            m = getattr(self.sim.viewer, attr, None)
            if isinstance(m, list):
                m.clear()

        if self._imu_site_id >= 0:
            site_pos = d.site_xpos[self._imu_site_id].copy()
            site_mat = d.site_xmat[self._imu_site_id].reshape(3, 3).copy()

            # Convert phone gyro/accel into the IMU site local frame used by the policy obs.
            # 1) phone -> trunk (mounting): v_trunk = R(q_phone_to_trunk) * v_phone
            # 2) normal mode only: trunk -> base: v_base = Rz(-theta) * v_trunk
            # 3) body -> imu_site: v_imu = Rz(-90deg) * v_body  (because imu_site is rotated +90deg about +Z)
            #
            # Using quaternion conventions in this file, step (3) is implemented as:
            #   v_imu = R(q_body_to_imu)^T * v_body
            # because q_body_to_imu maps imu vectors into body frame.
            v_trunk_gyro = _rotate_vec_by_quat(Q_PHONE_TO_TRUNK, self._gyro_phone)
            v_trunk_acc = _rotate_vec_by_quat(Q_PHONE_TO_TRUNK, self._accel_phone)

            if self._mode == "alter":
                v_body_gyro = v_trunk_gyro
                v_body_acc = v_trunk_acc
                q_body_to_imu = Q_TRUNK_TO_IMU_SITE
            else:
                q_base_to_trunk = _quat_from_axis_angle_z(trunk_theta)
                v_body_gyro = _rotate_vec_by_quat(_quat_conj(q_base_to_trunk), v_trunk_gyro)
                v_body_acc = _rotate_vec_by_quat(_quat_conj(q_base_to_trunk), v_trunk_acc)
                q_body_to_imu = Q_BASE_TO_IMU_SITE

            gyro_imu = _rotate_vec_by_quat(_quat_conj(q_body_to_imu), v_body_gyro)
            acc_imu = _rotate_vec_by_quat(_quat_conj(q_body_to_imu), v_body_acc)

            # Draw components along IMU axes (x=red, y=green, z=blue).
            # Convert IMU-local components to world directions using site_xmat.
            def draw_vec_components(origin, v_local, scale, alpha):
                colors = [
                    (1.0, 0.0, 0.0, alpha),
                    (0.0, 1.0, 0.0, alpha),
                    (0.0, 0.0, 1.0, alpha),
                ]
                for ax in range(3):
                    comp = float(v_local[ax])
                    if abs(comp) < 1e-6:
                        continue
                    dir_world = site_mat[:, ax] * np.sign(comp)
                    self.sim.add_axis_arrow(origin, dir_world, length=abs(comp) * scale, color=colors[ax])

            # Slight Z offset so gyro vs accel don't overlap perfectly.
            up = site_mat[:, 2]
            draw_vec_components(site_pos + 0.00 * up, gyro_imu, scale=0.10, alpha=0.8)   # rad/s
            draw_vec_components(site_pos + 0.03 * up, acc_imu, scale=0.02, alpha=0.8)    # m/s^2

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
