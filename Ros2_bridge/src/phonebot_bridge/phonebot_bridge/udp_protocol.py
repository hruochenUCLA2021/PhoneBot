"""
PhoneBot binary UDP protocol helpers (little-endian).

This file must stay in sync with the Android app implementation.
See: reference_note/NOTE_protocal.md
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple


MAGIC = b"PBOT"
VERSION = 1

MSG_TYPE_SENSOR = 1
MSG_TYPE_MOTOR = 2

MOTOR_COUNT = 13

# Header:
#   magic[4] + version[u8] + msg_type[u8] + flags[u16] + seq[u32] + ts_ns[u64]
HEADER_FMT = "<4sBBHIQ"
HEADER_SIZE = struct.calcsize(HEADER_FMT)  # 20

# Sensor packet total: 116 bytes
SENSOR_FMT = "<4sBBHIQ3f3ff4f3ff4f3ff4B"
SENSOR_SIZE = struct.calcsize(SENSOR_FMT)  # 116

# Motor packet total: 176 bytes
MOTOR_FMT = "<4sBBHIQ39f"
MOTOR_SIZE = struct.calcsize(MOTOR_FMT)  # 176


@dataclass(frozen=True)
class SensorPacket:
    seq: int
    ts_ns: int
    accel: Tuple[float, float, float]
    gyro: Tuple[float, float, float]
    rot_hz: float
    rot_quat_wxyz: Tuple[float, float, float, float]
    rot_ypr_deg: Tuple[float, float, float]
    game_hz: float
    game_quat_wxyz: Tuple[float, float, float, float]
    game_ypr_deg: Tuple[float, float, float]
    batt_percent: float
    batt_is_charging: int
    batt_plugged: int
    batt_status: int


@dataclass(frozen=True)
class MotorPacket:
    seq: int
    ts_ns: int
    pos: Tuple[float, ...]  # len 13
    vel: Tuple[float, ...]  # len 13
    tau: Tuple[float, ...]  # len 13


def try_parse_sensor(payload: bytes) -> Optional[SensorPacket]:
    if len(payload) < SENSOR_SIZE:
        return None
    try:
        unpacked = struct.unpack_from(SENSOR_FMT, payload, 0)
    except struct.error:
        return None

    magic, version, msg_type, _flags, seq, ts_ns = unpacked[0:6]
    if magic != MAGIC or version != VERSION or msg_type != MSG_TYPE_SENSOR:
        return None

    # Unpacked layout after header:
    # accel(3), gyro(3),
    # rot_hz(1), rot_quat(4), rot_ypr(3),
    # game_hz(1), game_quat(4), game_ypr(3),
    # batt_percent(1), batt_bytes(4)
    i = 6
    accel = tuple(float(x) for x in unpacked[i : i + 3])
    i += 3
    gyro = tuple(float(x) for x in unpacked[i : i + 3])
    i += 3

    rot_hz = float(unpacked[i])
    i += 1
    rot_quat = tuple(float(x) for x in unpacked[i : i + 4])
    i += 4
    rot_ypr = tuple(float(x) for x in unpacked[i : i + 3])
    i += 3

    game_hz = float(unpacked[i])
    i += 1
    game_quat = tuple(float(x) for x in unpacked[i : i + 4])
    i += 4
    game_ypr = tuple(float(x) for x in unpacked[i : i + 3])
    i += 3

    batt_percent = float(unpacked[i])
    i += 1
    batt_is_charging = int(unpacked[i])
    batt_plugged = int(unpacked[i + 1])
    batt_status = int(unpacked[i + 2])

    return SensorPacket(
        seq=int(seq),
        ts_ns=int(ts_ns),
        accel=accel,
        gyro=gyro,
        rot_hz=rot_hz,
        rot_quat_wxyz=rot_quat,
        rot_ypr_deg=rot_ypr,
        game_hz=game_hz,
        game_quat_wxyz=game_quat,
        game_ypr_deg=game_ypr,
        batt_percent=batt_percent,
        batt_is_charging=batt_is_charging,
        batt_plugged=batt_plugged,
        batt_status=batt_status,
    )


def pack_motor(seq: int, ts_ns: int, pos: Sequence[float], vel: Sequence[float], tau: Sequence[float]) -> bytes:
    # Enforce exactly 13 float32 values each; pad with 0.0 / truncate.
    def _fix(a: Sequence[float]) -> Tuple[float, ...]:
        out = [0.0] * MOTOR_COUNT
        n = min(MOTOR_COUNT, len(a))
        for j in range(n):
            out[j] = float(a[j])
        return tuple(out)

    p = _fix(pos)
    v = _fix(vel)
    t = _fix(tau)

    # Flatten 39 floats (pos, vel, tau)
    floats = p + v + t
    return struct.pack(MOTOR_FMT, MAGIC, VERSION, MSG_TYPE_MOTOR, 0, int(seq) & 0xFFFFFFFF, int(ts_ns), *floats)


