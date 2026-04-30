import json
import math
import os
import threading
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, JointState
from motor_interfaces.msg import MotorState


def _quat_mul(a_wxyz, b_wxyz):
    aw, ax, ay, az = a_wxyz
    bw, bx, by, bz = b_wxyz
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def _quat_conj(q_wxyz):
    return (q_wxyz[0], -q_wxyz[1], -q_wxyz[2], -q_wxyz[3])


def _quat_from_axis_angle_z(theta: float):
    h = 0.5 * float(theta)
    return (math.cos(h), 0.0, 0.0, math.sin(h))


def _quat_rotate_vec(q_wxyz, v_xyz):
    # v' = q * (0,v) * conj(q)
    p = (0.0, float(v_xyz[0]), float(v_xyz[1]), float(v_xyz[2]))
    qp = _quat_mul(q_wxyz, p)
    qpq = _quat_mul(qp, _quat_conj(q_wxyz))
    return (qpq[1], qpq[2], qpq[3])


@dataclass
class PolicyMeta:
    env_name: str
    state_dim: int
    action_dim: int
    default_pose: Tuple[float, ...]

    @property
    def mode(self) -> str:
        return "alter" if ("alter" in (self.env_name or "").lower()) else "normal"


def _load_policy_meta(meta_path: str) -> PolicyMeta:
    with open(meta_path, "r", encoding="utf-8") as f:
        j = json.load(f)
    env = str(j.get("env_name", ""))
    state_dim = int(j.get("state_dim", 52))
    action_dim = int(j.get("action_dim", 13))
    dp = j.get("default_pose", None)
    if not isinstance(dp, list):
        dp = [0.0] * 13
    dp = [float(x) for x in dp][:13]
    if len(dp) < 13:
        dp = dp + [0.0] * (13 - len(dp))
    return PolicyMeta(env_name=env, state_dim=state_dim, action_dim=action_dim, default_pose=tuple(dp))


class TfliteActor:
    def __init__(self, tflite_path: str, state_dim: int, action_dim: int, num_threads: int = 2):
        self.state_dim = int(state_dim)
        self.action_dim = int(action_dim)

        interpreter = None
        try:
            from tflite_runtime.interpreter import Interpreter  # type: ignore

            interpreter = Interpreter(model_path=tflite_path, num_threads=num_threads)
        except Exception:
            try:
                import tensorflow as tf  # type: ignore

                interpreter = tf.lite.Interpreter(model_path=tflite_path, num_threads=num_threads)
            except Exception as e:
                raise RuntimeError(
                    "No TFLite interpreter available. Install one of:\n"
                    "- pip install tflite-runtime\n"
                    "- pip install tensorflow\n"
                    f"Original error: {e}"
                ) from e

        self._interpreter = interpreter
        self._interpreter.allocate_tensors()
        self._in = self._interpreter.get_input_details()[0]
        self._out = self._interpreter.get_output_details()[0]

    def run(self, state_1d):
        if len(state_1d) != self.state_dim:
            raise ValueError(f"state size {len(state_1d)} != expected {self.state_dim}")
        import numpy as np

        x = np.asarray(state_1d, dtype=np.float32).reshape((1, self.state_dim))
        self._interpreter.set_tensor(self._in["index"], x)
        self._interpreter.invoke()
        y = self._interpreter.get_tensor(self._out["index"]).reshape((-1,))
        if int(y.size) != self.action_dim:
            raise ValueError(f"action size {int(y.size)} != expected {self.action_dim}")
        return y.astype(np.float32)


def build_phonebot_obs52(
    *,
    q_phone_wxyz,
    gyro_phone_xyz,
    joint_pos_rad,
    joint_vel_rad_s,
    last_act,
    default_pose,
    cmd_vx: float,
    cmd_vy: float,
    cmd_wz: float,
    base_to_trunk_rad: float,
    mode: str,
    phase_l: float,
):
    # Matches Android buildPhonebotObs52:
    # [gyro(3), gravity(3), command(3), (q - q_default)(13), qd(13), last_act(13), phase_feat(4)]
    out = [0.0] * 52

    s = math.sqrt(2.0) / 2.0
    q_phone_to_trunk = (0.0, 0.0, s, s)
    q_body_to_imu_site = (s, 0.0, 0.0, s)  # 90deg about +Z

    gyro_trunk = _quat_rotate_vec(q_phone_to_trunk, gyro_phone_xyz)

    q_base_to_trunk = _quat_from_axis_angle_z(base_to_trunk_rad)
    if str(mode).lower() == "alter":
        gyro_imu = _quat_rotate_vec(_quat_conj(q_body_to_imu_site), gyro_trunk)
    else:
        gyro_base = _quat_rotate_vec(_quat_conj(q_base_to_trunk), gyro_trunk)
        gyro_imu = _quat_rotate_vec(_quat_conj(q_body_to_imu_site), gyro_base)

    q_trunk_world = _quat_mul(q_phone_wxyz, q_phone_to_trunk)
    q_base_world = _quat_mul(q_trunk_world, _quat_from_axis_angle_z(-base_to_trunk_rad))
    if str(mode).lower() == "alter":
        q_imu_world = _quat_mul(q_trunk_world, q_body_to_imu_site)
    else:
        q_imu_world = _quat_mul(q_base_world, q_body_to_imu_site)
    gravity_world = (0.0, 0.0, -1.0)
    gravity_imu = _quat_rotate_vec(_quat_conj(q_imu_world), gravity_world)

    i = 0
    out[i : i + 3] = list(gyro_imu)
    i += 3
    out[i : i + 3] = list(gravity_imu)
    i += 3
    out[i] = float(cmd_vx)
    out[i + 1] = float(cmd_vy)
    out[i + 2] = float(cmd_wz)
    i += 3

    for k in range(13):
        q = float(joint_pos_rad[k]) if k < len(joint_pos_rad) else 0.0
        q0 = float(default_pose[k]) if k < len(default_pose) else 0.0
        out[i] = q - q0
        i += 1
    for k in range(13):
        qd = float(joint_vel_rad_s[k]) if k < len(joint_vel_rad_s) else 0.0
        out[i] = qd
        i += 1
    for k in range(13):
        out[i] = float(last_act[k])
        i += 1

    phase_r = float(phase_l) + math.pi
    out[i] = math.cos(phase_l)
    out[i + 1] = math.cos(phase_r)
    out[i + 2] = math.sin(phase_l)
    out[i + 3] = math.sin(phase_r)
    return out


class PcPolicyRunner(Node):
    def __init__(self):
        super().__init__("pc_policy_runner_python")

        default_tflite = (
            "/media/hrc/T7_UBUNTU_ONLY/android_humanoid_all_files/PhoneBot/"
            "APP_workspace/app/src/main/assets/exported_tflite/"
            "phonebot_flat_alter_fv2_torque_awared_home_straight_v1_actor.tflite"
        )
        default_meta = (
            "/media/hrc/T7_UBUNTU_ONLY/android_humanoid_all_files/PhoneBot/"
            "APP_workspace/app/src/main/assets/exported_tflite/"
            "phonebot_flat_alter_fv2_torque_awared_home_straight_v1_actor_metadata.json"
        )
        self.declare_parameter("tflite_path", default_tflite)
        self.declare_parameter("meta_path", default_meta)
        self.declare_parameter("num_threads", 2)
        self.declare_parameter("publish_hz", 50.0)
        self.declare_parameter("gait_hz", 1.5)

        self.declare_parameter("imu_topic", "/phonebot/imu_game")
        self.declare_parameter("motor_state_topic", "/phonebot/motor_state_full")
        self.declare_parameter("cmd_vel_topic", "/phonebot/cmd_vel")
        self.declare_parameter("motor_cmd_topic", "/phonebot/motor_cmd")

        self._tflite_path = str(self.get_parameter("tflite_path").value)
        self._meta_path = str(self.get_parameter("meta_path").value)
        self._num_threads = int(self.get_parameter("num_threads").value)
        self._publish_hz = float(self.get_parameter("publish_hz").value)
        self._gait_hz = float(self.get_parameter("gait_hz").value)

        imu_topic = str(self.get_parameter("imu_topic").value)
        motor_state_topic = str(self.get_parameter("motor_state_topic").value)
        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        motor_cmd_topic = str(self.get_parameter("motor_cmd_topic").value)

        if not os.path.exists(self._tflite_path):
            raise RuntimeError(f"tflite_path does not exist: {self._tflite_path}")
        if not os.path.exists(self._meta_path):
            raise RuntimeError(f"meta_path does not exist: {self._meta_path}")

        self._meta = _load_policy_meta(self._meta_path)
        self.get_logger().info(f"Policy env_name={self._meta.env_name} mode={self._meta.mode}")
        self.get_logger().info(f"Loading TFLite actor: {self._tflite_path}")
        self._actor = TfliteActor(self._tflite_path, self._meta.state_dim, self._meta.action_dim, self._num_threads)

        self._lock = threading.Lock()
        self._latest_imu: Optional[Imu] = None
        self._latest_motor: Optional[MotorState] = None
        self._latest_cmd: Twist = Twist()
        self._cmd_rx_t = 0.0

        self._last_act = [0.0] * 13
        self._phase_l = 0.0
        self._last_tick_ns = time.time_ns()

        self.create_subscription(Imu, imu_topic, self._on_imu, 10)
        self.create_subscription(MotorState, motor_state_topic, self._on_motor_state, 10)
        self.create_subscription(Twist, cmd_vel_topic, self._on_cmd_vel, 10)
        self._pub_motor_cmd = self.create_publisher(JointState, motor_cmd_topic, 10)

        period = 1.0 / max(1.0, self._publish_hz)
        self.create_timer(period, self._on_timer)
        self.get_logger().info(
            f"pc_policy_runner_python ready: publish_hz={self._publish_hz:.1f} "
            f"imu={imu_topic} motor_state={motor_state_topic} cmd_vel={cmd_vel_topic} -> {motor_cmd_topic}"
        )

    def _on_imu(self, msg: Imu):
        with self._lock:
            self._latest_imu = msg

    def _on_motor_state(self, msg: MotorState):
        with self._lock:
            self._latest_motor = msg

    def _on_cmd_vel(self, msg: Twist):
        with self._lock:
            self._latest_cmd = msg
            self._cmd_rx_t = time.time()

    def _on_timer(self):
        with self._lock:
            imu = self._latest_imu
            ms = self._latest_motor
            cmd = self._latest_cmd
            cmd_age = time.time() - self._cmd_rx_t if self._cmd_rx_t > 0.0 else 1e9

        if imu is None or ms is None:
            return

        if cmd_age > 1.0:
            cmd_vx = 0.0
            cmd_vy = 0.0
            cmd_wz = 0.0
        else:
            cmd_vx = float(cmd.linear.x)
            cmd_vy = float(cmd.linear.y)
            cmd_wz = float(cmd.angular.z)

        q = imu.orientation
        q_phone = (float(q.w), float(q.x), float(q.y), float(q.z))
        gyro_phone = (float(imu.angular_velocity.x), float(imu.angular_velocity.y), float(imu.angular_velocity.z))

        joint_pos = list(ms.present_position_rad)
        joint_vel = list(ms.present_velocity_rad_s)
        base_to_trunk = float(joint_pos[12]) if len(joint_pos) > 12 and math.isfinite(joint_pos[12]) else 0.0

        now_ns = time.time_ns()
        dt = max(0.0, (now_ns - self._last_tick_ns) * 1e-9)
        self._last_tick_ns = now_ns
        self._phase_l = (self._phase_l + 2.0 * math.pi * self._gait_hz * dt) % (2.0 * math.pi)

        obs = build_phonebot_obs52(
            q_phone_wxyz=q_phone,
            gyro_phone_xyz=gyro_phone,
            joint_pos_rad=joint_pos,
            joint_vel_rad_s=joint_vel,
            last_act=self._last_act,
            default_pose=self._meta.default_pose,
            cmd_vx=cmd_vx,
            cmd_vy=cmd_vy,
            cmd_wz=cmd_wz,
            base_to_trunk_rad=base_to_trunk,
            mode=self._meta.mode,
            phase_l=self._phase_l,
        )

        try:
            act = self._actor.run(obs)
        except Exception as e:
            self.get_logger().warn(f"policy inference failed: {e}", throttle_duration_sec=1.0)
            return

        tgt = [0.0] * 13
        for i in range(13):
            a = float(act[i]) if i < len(act) else 0.0
            self._last_act[i] = a
            v = float(self._meta.default_pose[i]) + a
            if v < -2.8:
                v = -2.8
            if v > 2.8:
                v = 2.8
            tgt[i] = v

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [f"motor_{i+1}" for i in range(13)]
        msg.position = [float(x) for x in tgt]
        self._pub_motor_cmd.publish(msg)


def main():
    rclpy.init()
    node = None
    try:
        node = PcPolicyRunner()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

