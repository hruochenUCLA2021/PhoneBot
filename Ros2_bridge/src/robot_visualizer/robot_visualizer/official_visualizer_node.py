import os

import numpy as np
import mujoco

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from ament_index_python.packages import get_package_share_directory
from motor_interfaces.msg import MotorState

from .official_simulator_mujoco_hrc import official_simulator_mujoco_v1


# MuJoCo joint order from phonebot_general_alternative_imu.xml.
# qpos layout: [0:3]=base_pos, [3:7]=base_quat(wxyz), [7:20]=joints.
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
    return np.array(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ]
    )


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


# Phone body -> trunk correction (mounting), in [w,x,y,z].
_s = np.sqrt(2.0) / 2.0
Q_PHONE_TO_TRUNK = np.array([0.0, 0.0, _s, _s])

# IMU site local rotation relative to its body frame: +90deg about +Z.
Q_BASE_TO_IMU_SITE = np.array([_s, 0.0, 0.0, _s])
Q_TRUNK_TO_IMU_SITE = np.array([_s, 0.0, 0.0, _s])


class PhonebotOfficialVisualizer(Node):
    def __init__(self):
        super().__init__("phonebot_visualizer_official")

        _default_model = os.path.join(
            get_package_share_directory("robot_visualizer"),
            "model",
            "model_phonebot_fred_v2_torque_version",
            "scene_joystick_flat_terrain_alter_v2_full_collision.xml",
        )
        self.declare_parameter("model_path", _default_model)
        # "normal" => imu site on base link; "alter" => imu site on trunk link.
        self.declare_parameter("mode", "normal")
        self.declare_parameter("render_hz", 30.0)
        self.declare_parameter("imu_topic", "/phonebot/imu_game")
        # Prefer MotorState from HWdriver.
        self.declare_parameter("motor_state_topic", "/phonebot/motor_state_full")
        self.declare_parameter("gyro_arrow_scale", 0.05)
        self.declare_parameter("acc_arrow_scale", 0.01)
        self.declare_parameter("arrow_max_len", 0.4)

        model_path = self.get_parameter("model_path").get_parameter_value().string_value
        self._mode = self.get_parameter("mode").get_parameter_value().string_value.strip().lower() or "normal"
        render_hz = self.get_parameter("render_hz").get_parameter_value().double_value
        imu_topic = self.get_parameter("imu_topic").get_parameter_value().string_value
        motor_topic = self.get_parameter("motor_state_topic").get_parameter_value().string_value
        self._gyro_arrow_scale = float(self.get_parameter("gyro_arrow_scale").value)
        self._acc_arrow_scale = float(self.get_parameter("acc_arrow_scale").value)
        self._arrow_max_len = float(self.get_parameter("arrow_max_len").value)

        self._q_trunk = DEFAULT_BASE_QUAT.copy()  # latest trunk orientation in world
        self._gyro_phone = np.zeros(3)
        self._accel_phone = np.zeros(3)
        self._joint_pos = np.zeros(NUM_JOINTS)

        self.get_logger().info(f"Loading MuJoCo model (official viewer): {model_path}")
        self.sim = official_simulator_mujoco_v1()
        self.sim.load_model(model_path)
        self.sim.create_viewer()

        # Build joint-name -> qpos-index lookup
        self._joint_qpos_idx = {}
        for name in JOINT_NAMES:
            jid = mujoco.mj_name2id(self.sim.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid < 0:
                self.get_logger().warn(f"Joint '{name}' not found in model")
            else:
                self._joint_qpos_idx[name] = self.sim.model.jnt_qposadr[jid]

        fj_id = mujoco.mj_name2id(self.sim.model, mujoco.mjtObj.mjOBJ_JOINT, "floating_base")
        self._base_qpos_adr = self.sim.model.jnt_qposadr[fj_id] if fj_id >= 0 else 0

        self._imu_site_id = mujoco.mj_name2id(self.sim.model, mujoco.mjtObj.mjOBJ_SITE, "imu")

        self.create_subscription(Imu, imu_topic, self._on_imu, 10)
        self.create_subscription(MotorState, motor_topic, self._on_motor_state_full, 10)

        render_period = 1.0 / render_hz
        self._render_timer = self.create_timer(render_period, self._render_callback)

        self.get_logger().info(
            f"Official visualizer ready — render {render_hz:.0f} Hz, "
            f"mode={self._mode}, IMU: {imu_topic}, motors: {motor_topic}"
        )

    def _on_imu(self, msg: Imu):
        q = msg.orientation
        # ROS Quaternion is (x, y, z, w) -> convert to [w,x,y,z]
        q_phone = np.array([q.w, q.x, q.y, q.z])
        self._q_trunk[:] = _quat_mul(q_phone, Q_PHONE_TO_TRUNK)
        self._gyro_phone[:] = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        self._accel_phone[:] = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])

    def _on_motor_state_full(self, msg: MotorState):
        ids = list(msg.ids)
        pos = list(msg.present_position_rad)
        n = min(len(ids), len(pos))
        if n <= 0:
            return
        for i in range(n):
            mid = int(ids[i])
            if 1 <= mid <= NUM_JOINTS:
                self._joint_pos[mid - 1] = float(pos[i])

    def _render_callback(self):
        if not self.sim.is_alive:
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

        trunk_theta = self._joint_pos[12]  # base_to_trunk = motor 13
        q_base = _quat_mul(self._q_trunk, _quat_from_axis_angle_z(-trunk_theta))
        d.qpos[a + 3 : a + 7] = q_base

        mujoco.mj_forward(self.sim.model, d)

        # Clear previous overlays and redraw debug arrows.
        self.sim.clear_overlays()

        if self._imu_site_id >= 0:
            site_pos = d.site_xpos[self._imu_site_id].copy()
            site_mat = d.site_xmat[self._imu_site_id].reshape(3, 3).copy()

            # Convert phone gyro/accel into the IMU site local frame used by the policy obs.
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
                    L = abs(comp) * float(scale)
                    if self._arrow_max_len > 0.0:
                        L = min(L, self._arrow_max_len)
                    self.sim.add_axis_arrow(origin, dir_world, length=L, color=colors[ax], radius=0.015)

            up = site_mat[:, 2]
            draw_vec_components(site_pos + 0.00 * up, gyro_imu, scale=self._gyro_arrow_scale, alpha=0.8)  # rad/s
            draw_vec_components(site_pos + 0.03 * up, acc_imu, scale=self._acc_arrow_scale, alpha=0.8)   # m/s^2

        self.sim.render()

    def destroy_node(self):
        try:
            self.sim.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PhonebotOfficialVisualizer()
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

