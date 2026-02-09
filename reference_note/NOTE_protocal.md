# PhoneBot UDP Binary Protocol (v1)

This file defines the **exact byte order** and **data types** for PhoneBot UDP packets.

- **Transport**: UDP datagrams (each datagram = one whole packet; no partial reads)
- **Endianness**: **little-endian**
- **Floats**: **float32** (IEEE-754)
- **Integers**: unsigned semantics (but packed as standard binary fields)

All packets share the same 20-byte header:

## Common header (20 bytes)
| Field | Type | Bytes | Notes |
|---|---:|---:|---|
| `magic` | `char[4]` | 4 | ASCII `"PBOT"` |
| `version` | `u8` | 1 | currently `1` |
| `msg_type` | `u8` | 1 | `1=sensor`, `2=motor` |
| `flags` | `u16` | 2 | reserved, set `0` |
| `seq` | `u32` | 4 | increasing packet counter (wrap allowed) |
| `ts_ns` | `u64` | 8 | timestamp in nanoseconds |

Binary struct (Python `struct`): `<4sBBHIQ`

---

## Packet type 1: SENSOR (Android → PC) — 220 bytes total (Protocol v2)

This is sent from the Android app to the PC on each rotation-vector update (and includes latest accel/gyro/battery snapshot).

### Layout after header (200 bytes payload)
| Field | Type | Count | Bytes |
|---|---:|---:|---:|
| `accel_xyz` | `float32` | 3 | 12 |
| `gyro_xyz` | `float32` | 3 | 12 |
| `rotvec_hz` | `float32` | 1 | 4 |
| `rotvec_quat_wxyz` | `float32` | 4 | 16 |
| `rotvec_ypr_deg` | `float32` | 3 | 12 |
| `game_rotvec_hz` | `float32` | 1 | 4 |
| `game_quat_wxyz` | `float32` | 4 | 16 |
| `game_ypr_deg` | `float32` | 3 | 12 |
| `battery_percent` | `float32` | 1 | 4 |
| `battery_is_charging` | `u8` | 1 | 1 |
| `battery_plugged` | `u8` | 1 | 1 |
| `battery_status` | `u8` | 1 | 1 |
| `reserved` | `u8` | 1 | 1 |
| `motor_pos_rad` | `float32` | 13 | 52 |
| `motor_vel_rad_s` | `float32` | 13 | 52 |

Binary struct (Python `struct`): `<4sBBHIQ3f3ff4f3ff4f3ff4B26f`

### Header flags (`flags` u16)
- bit0 (`1`): motor state valid (`motor_pos_rad` / `motor_vel_rad_s` contain real values)

### Enumerations
`battery_is_charging`:
- `0` = false
- `1` = true

`battery_plugged`:
- `0` = UNPLUGGED/UNKNOWN
- `1` = AC
- `2` = USB
- `3` = WIRELESS

`battery_status`:
- `0` = UNKNOWN
- `1` = CHARGING
- `2` = DISCHARGING
- `3` = FULL
- `4` = NOT_CHARGING

Notes:
- Quaternion is **Android order**: `[w, x, y, z]`
- `battery_percent` uses `-1` when unknown.

---

## Packet type 2: MOTOR (PC → Android) — 176 bytes total

This is sent from the PC to the Android app, typically originating from ROS2 topic `/phonebot/motor_cmd` (`sensor_msgs/JointState`).

### Layout after header (156 bytes payload)
| Field | Type | Count | Bytes |
|---|---:|---:|---:|
| `pos` | `float32` | 13 | 52 |
| `vel` | `float32` | 13 | 52 |
| `tau` | `float32` | 13 | 52 |

Binary struct (Python `struct`): `<4sBBHIQ39f`

Notes:
- The bridge **pads/truncates** to 13 elements if the ROS2 `JointState` arrays are shorter/longer.

