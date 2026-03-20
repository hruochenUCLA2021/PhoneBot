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
# Single protocol version for the current architecture.
# Must match Android `PhonebotProtocol.VERSION` (currently 3).
VERSION = 3

MSG_TYPE_SENSOR = 1
MSG_TYPE_MOTOR = 2
MSG_TYPE_TORQUE = 3
MSG_TYPE_MOTOR_STATUS = 4

MOTOR_COUNT = 13

# Header:
#   magic[4] + version[u8] + msg_type[u8] + flags[u16] + seq[u32] + ts_ns[u64]
HEADER_FMT = "<4sBBHIQ"
HEADER_SIZE = struct.calcsize(HEADER_FMT)  # 20

# Sensor packet total: 116 bytes (no motor arrays in SENSOR packets)
SENSOR_FMT = "<4sBBHIQ3f3ff4f3ff4f3ff4B"
SENSOR_SIZE = struct.calcsize(SENSOR_FMT)  # 116

# Motor packet total: 176 bytes
MOTOR_FMT = "<4sBBHIQ39f"
MOTOR_SIZE = struct.calcsize(MOTOR_FMT)  # 176

# Motor status packet: header(20) + 6*13 float32 = 332 bytes
MOTOR_STATUS_FMT = "<4sBBHIQ78f"
MOTOR_STATUS_SIZE = struct.calcsize(MOTOR_STATUS_FMT)  # 332

# Torque packet total: 21 bytes (header 20 + enable u8)
TORQUE_FMT = "<4sBBHIQB"
TORQUE_SIZE = struct.calcsize(TORQUE_FMT)  # 21


@dataclass(frozen=True)
class SensorPacket:
    seq: int
    ts_ns: int
    flags: int
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


@dataclass(frozen=True)
class TorquePacket:
    seq: int
    ts_ns: int
    enable: bool


@dataclass(frozen=True)
class MotorStatusPacket:
    seq: int
    ts_ns: int
    pwm_percent: Tuple[float, ...]  # len 13
    load_percent: Tuple[float, ...]  # len 13
    pos_rad: Tuple[float, ...]  # len 13
    vel_rad_s: Tuple[float, ...]  # len 13
    vin_v: Tuple[float, ...]  # len 13
    temp_c: Tuple[float, ...]  # len 13


def try_parse_sensor(payload: bytes) -> Optional[SensorPacket]:
    if len(payload) < SENSOR_SIZE:
        return None
    try:
        hdr = struct.unpack_from(HEADER_FMT, payload, 0)
    except struct.error:
        return None

    magic, version, msg_type, flags, seq, ts_ns = hdr
    if magic != MAGIC or msg_type != MSG_TYPE_SENSOR:
        return None

    if version != VERSION or len(payload) < SENSOR_SIZE:
        return None

    try:
        unpacked = struct.unpack_from(SENSOR_FMT, payload, 0)
    except struct.error:
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
    i += 4

    return SensorPacket(
        seq=int(seq),
        ts_ns=int(ts_ns),
        flags=int(flags),
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


def try_parse_motor(payload: bytes) -> Optional[MotorPacket]:
    if len(payload) < MOTOR_SIZE:
        return None
    try:
        unpacked = struct.unpack_from(MOTOR_FMT, payload, 0)
    except struct.error:
        return None

    magic, version, msg_type, flags, seq, ts_ns, *floats = unpacked
    if magic != MAGIC or version != VERSION or msg_type != MSG_TYPE_MOTOR:
        return None
    pos = tuple(float(x) for x in floats[0:13])
    vel = tuple(float(x) for x in floats[13:26])
    tau = tuple(float(x) for x in floats[26:39])
    return MotorPacket(seq=int(seq), ts_ns=int(ts_ns), pos=pos, vel=vel, tau=tau)


def try_parse_torque(payload: bytes) -> Optional[TorquePacket]:
    if len(payload) < TORQUE_SIZE:
        return None
    try:
        magic, version, msg_type, flags, seq, ts_ns, enable_u8 = struct.unpack_from(TORQUE_FMT, payload, 0)
    except struct.error:
        return None
    if magic != MAGIC or version != VERSION or msg_type != MSG_TYPE_TORQUE:
        return None
    return TorquePacket(seq=int(seq), ts_ns=int(ts_ns), enable=(int(enable_u8) != 0))


def try_parse_motor_status(payload: bytes) -> Optional[MotorStatusPacket]:
    if len(payload) < MOTOR_STATUS_SIZE:
        return None
    try:
        unpacked = struct.unpack_from(MOTOR_STATUS_FMT, payload, 0)
    except struct.error:
        return None
    magic, version, msg_type, flags, seq, ts_ns, *floats = unpacked
    if magic != MAGIC or version != VERSION or msg_type != MSG_TYPE_MOTOR_STATUS:
        return None
    pwm = tuple(float(x) for x in floats[0:13])
    load = tuple(float(x) for x in floats[13:26])
    pos = tuple(float(x) for x in floats[26:39])
    vel = tuple(float(x) for x in floats[39:52])
    vin = tuple(float(x) for x in floats[52:65])
    temp = tuple(float(x) for x in floats[65:78])
    return MotorStatusPacket(
        seq=int(seq),
        ts_ns=int(ts_ns),
        pwm_percent=pwm,
        load_percent=load,
        pos_rad=pos,
        vel_rad_s=vel,
        vin_v=vin,
        temp_c=temp,
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


def pack_torque(seq: int, ts_ns: int, enable: bool) -> bytes:
    return struct.pack(
        TORQUE_FMT,
        MAGIC,
        VERSION,
        MSG_TYPE_TORQUE,
        0,
        int(seq) & 0xFFFFFFFF,
        int(ts_ns),
        1 if enable else 0,
    )


def pack_motor_status(
    seq: int,
    ts_ns: int,
    pwm_percent: Sequence[float],
    load_percent: Sequence[float],
    pos_rad: Sequence[float],
    vel_rad_s: Sequence[float],
    vin_v: Sequence[float],
    temp_c: Sequence[float],
) -> bytes:
    def _fix(a: Sequence[float]) -> Tuple[float, ...]:
        out = [0.0] * MOTOR_COUNT
        n = min(MOTOR_COUNT, len(a))
        for j in range(n):
            out[j] = float(a[j])
        return tuple(out)

    p_pwm = _fix(pwm_percent)
    p_load = _fix(load_percent)
    p_pos = _fix(pos_rad)
    p_vel = _fix(vel_rad_s)
    p_vin = _fix(vin_v)
    p_temp = _fix(temp_c)
    floats = p_pwm + p_load + p_pos + p_vel + p_vin + p_temp
    return struct.pack(
        MOTOR_STATUS_FMT,
        MAGIC,
        VERSION,
        MSG_TYPE_MOTOR_STATUS,
        0,
        int(seq) & 0xFFFFFFFF,
        int(ts_ns),
        *floats,
    )


