import select
import sys
import termios
import threading
import time
import tty
from dataclasses import dataclass
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


@dataclass
class Cmd:
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0


def _clamp(x: float, lo: float, hi: float) -> float:
    return min(max(x, lo), hi)


def _make_twist(cmd: Cmd) -> Twist:
    m = Twist()
    m.linear.x = float(cmd.vx)
    m.linear.y = float(cmd.vy)
    m.angular.z = float(cmd.wz)
    return m


class _StdinRaw:
    def __init__(self) -> None:
        self._fd = sys.stdin.fileno()
        self._old: Optional[Tuple] = None

    def __enter__(self) -> "_StdinRaw":
        self._old = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        if self._old is not None:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old)


class CmdVelRepeater(Node):
    """
    Publishes `/phonebot/cmd_vel` at a constant rate (default 50Hz).

    Modes:
    - joystick: subscribe `/cmd_vel` (Twist), repeat last command at fixed rate
    - keyboard: read keyboard from stdin, generate command, publish at fixed rate
    """

    def __init__(self) -> None:
        super().__init__("phonebot_cmd_vel_repeater_py")

        self.declare_parameter("mode", "joystick")  # joystick | keyboard
        self.declare_parameter("publish_hz", 50.0)
        self.declare_parameter("cmd_vel_in_topic", "/cmd_vel")
        self.declare_parameter("cmd_vel_out_topic", "/phonebot/cmd_vel")

        # Defaults match `teleop_twist_joy` params in `joystick_config.yaml`.
        self.declare_parameter("vx_max", 0.5)
        self.declare_parameter("vy_max", 0.25)
        self.declare_parameter("wz_max", 1.0)

        self._mode = str(self.get_parameter("mode").value)
        self._hz = float(self.get_parameter("publish_hz").value)
        self._in_topic = str(self.get_parameter("cmd_vel_in_topic").value)
        self._out_topic = str(self.get_parameter("cmd_vel_out_topic").value)
        self._vx_max = float(self.get_parameter("vx_max").value)
        self._vy_max = float(self.get_parameter("vy_max").value)
        self._wz_max = float(self.get_parameter("wz_max").value)

        self._lock = threading.Lock()
        self._latest = Cmd()
        self._latest_rx = 0.0

        self._kbd_thread: Optional[threading.Thread] = None
        self._kbd_stop = threading.Event()
        self._kbd_vx_ts = 0.0
        self._kbd_vy_ts = 0.0
        self._kbd_wz_ts = 0.0
        self._kbd_timeout_s = 0.35

        if self._mode == "joystick":
            self._sub = self.create_subscription(Twist, self._in_topic, self._on_cmd_vel, 10)
            self.get_logger().info(f"Mode=joystick, repeating {self._in_topic} -> {self._out_topic} @ {self._hz} Hz")
        elif self._mode == "keyboard":
            self._sub = None
            self.get_logger().info(f"Mode=keyboard, publishing {self._out_topic} @ {self._hz} Hz")
            self.get_logger().info("Keyboard: arrows for vx/vy, a/d for wz, Shift gives max. Space stops.")
            self._start_keyboard()
        else:
            raise RuntimeError(f"Unknown mode: {self._mode}")

        self._pub = self.create_publisher(Twist, self._out_topic, 10)
        period = 1.0 / max(self._hz, 1e-3)
        self._timer = self.create_timer(period, self._on_timer)

    def destroy_node(self) -> bool:
        self._stop_keyboard()
        return super().destroy_node()

    def _on_cmd_vel(self, msg: Twist) -> None:
        with self._lock:
            self._latest = Cmd(vx=float(msg.linear.x), vy=float(msg.linear.y), wz=float(msg.angular.z))
            self._latest_rx = time.time()

    def _on_timer(self) -> None:
        if self._mode == "keyboard":
            # Per-axis timeout so vx/vy/wz can combine (diagonal), but each axis returns to 0
            # if it hasn't been updated recently.
            now = time.time()
            with self._lock:
                if self._kbd_vx_ts > 0.0 and (now - self._kbd_vx_ts) > self._kbd_timeout_s:
                    self._latest.vx = 0.0
                if self._kbd_vy_ts > 0.0 and (now - self._kbd_vy_ts) > self._kbd_timeout_s:
                    self._latest.vy = 0.0
                if self._kbd_wz_ts > 0.0 and (now - self._kbd_wz_ts) > self._kbd_timeout_s:
                    self._latest.wz = 0.0

        with self._lock:
            cmd = self._latest

        self._pub.publish(_make_twist(cmd))

    def _start_keyboard(self) -> None:
        self._kbd_stop.clear()
        self._kbd_thread = threading.Thread(target=self._keyboard_loop, name="phonebot-kbd", daemon=True)
        self._kbd_thread.start()

    def _stop_keyboard(self) -> None:
        self._kbd_stop.set()
        t = self._kbd_thread
        if t is not None and t.is_alive():
            t.join(timeout=0.5)
        self._kbd_thread = None

    def _keyboard_loop(self) -> None:
        buf = b""

        def set_vx(vx: float) -> None:
            with self._lock:
                self._latest.vx = float(vx)
                self._kbd_vx_ts = time.time()

        def set_vy(vy: float) -> None:
            with self._lock:
                self._latest.vy = float(vy)
                self._kbd_vy_ts = time.time()

        def set_wz(wz: float) -> None:
            with self._lock:
                self._latest.wz = float(wz)
                self._kbd_wz_ts = time.time()

        def stop_all() -> None:
            with self._lock:
                self._latest = Cmd()
                now = time.time()
                self._kbd_vx_ts = now
                self._kbd_vy_ts = now
                self._kbd_wz_ts = now

        with _StdinRaw():
            while rclpy.ok() and not self._kbd_stop.is_set():
                r, _, _ = select.select([sys.stdin], [], [], 0.05)
                if not r:
                    continue
                chunk = sys.stdin.buffer.read1(32)
                if not chunk:
                    continue
                buf += chunk

                # Handle single-char commands.
                while buf:
                    b0 = buf[:1]
                    if b0 == b" ":
                        stop_all()
                        buf = buf[1:]
                        continue
                    if b0 in (b"q", b"\x03"):  # q or Ctrl+C
                        self.get_logger().info("Keyboard quit requested.")
                        rclpy.shutdown()
                        return
                    if b0 in (b"a", b"A", b"d", b"D"):
                        half = 0.5
                        full = 1.0
                        is_full = (b0 in (b"A", b"D"))
                        scale = full if is_full else half
                        wz = (scale * self._wz_max) * (1.0 if b0 in (b"a", b"A") else -1.0)
                        set_wz(wz)
                        buf = buf[1:]
                        continue

                    # Escape sequences (arrows)
                    if b0 != b"\x1b":
                        buf = buf[1:]
                        continue

                    # Need more bytes.
                    if len(buf) < 3:
                        break

                    # Standard arrows: ESC [ A/B/C/D
                    if buf.startswith(b"\x1b[A"):
                        set_vx(0.5 * self._vx_max)
                        buf = buf[3:]
                        continue
                    if buf.startswith(b"\x1b[B"):
                        set_vx(-0.5 * self._vx_max)
                        buf = buf[3:]
                        continue
                    if buf.startswith(b"\x1b[D"):
                        set_vy(0.5 * self._vy_max)  # left -> +vy
                        buf = buf[3:]
                        continue
                    if buf.startswith(b"\x1b[C"):
                        set_vy(-0.5 * self._vy_max)  # right -> -vy
                        buf = buf[3:]
                        continue

                    # Common shifted arrows in terminals: ESC [ 1 ; 2 A/B/C/D
                    if buf.startswith(b"\x1b[1;2A"):
                        set_vx(1.0 * self._vx_max)
                        buf = buf[len(b"\x1b[1;2A") :]
                        continue
                    if buf.startswith(b"\x1b[1;2B"):
                        set_vx(-1.0 * self._vx_max)
                        buf = buf[len(b"\x1b[1;2B") :]
                        continue
                    if buf.startswith(b"\x1b[1;2D"):
                        set_vy(1.0 * self._vy_max)
                        buf = buf[len(b"\x1b[1;2D") :]
                        continue
                    if buf.startswith(b"\x1b[1;2C"):
                        set_vy(-1.0 * self._vy_max)
                        buf = buf[len(b"\x1b[1;2C") :]
                        continue

                    # Unrecognized escape; drop one byte and retry.
                    buf = buf[1:]


def main() -> None:
    rclpy.init()
    node = None
    try:
        node = CmdVelRepeater()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

