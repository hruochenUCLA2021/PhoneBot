# NOTE: PhoneBot UDP packet header (20 bytes)

This note explains the **common binary header** that appears at the start of *every* PhoneBot UDP packet (SENSOR / MOTOR / TORQUE).

## Key idea

The header is **raw binary bytes** (not JSON, not a hex string).

- You might *display* it as hex for debugging, but on the network it is just bytes.
- All multi-byte numbers are **little-endian**.

## Header layout (20 bytes total)

Order on the wire:

1. `magic[4]`  (4 bytes)  ASCII bytes `"PBOT"`
2. `version`   (1 byte)   unsigned integer (currently `3`)
3. `msg_type`  (1 byte)   unsigned integer:
   - `1` = SENSOR
   - `2` = MOTOR
   - `3` = TORQUE
4. `flags`     (2 bytes)  unsigned integer, reserved (currently `0`)
5. `seq`       (4 bytes)  unsigned integer sequence counter (wrap allowed)
6. `ts_ns`     (8 bytes)  unsigned integer timestamp in nanoseconds

So the total is \(4 + 1 + 1 + 2 + 4 + 8 = 20\) bytes.

## Python `struct` format

In `Ros2_bridge/src/phonebot_bridge/phonebot_bridge/udp_protocol.py`:

- Format string: `"<4sBBHIQ"`
- Meaning:
  - `<` = **little-endian**
  - `4s` = 4-byte raw string (bytes)
  - `B`  = uint8
  - `H`  = uint16
  - `I`  = uint32
  - `Q`  = uint64

## Kotlin (Android) equivalent

Android code uses `ByteBuffer.order(ByteOrder.LITTLE_ENDIAN)` and then writes fields in the same order:

- `put(byteArrayOf('P','B','O','T'))`
- `put(version)`
- `put(msgType)`
- `putShort(flags)`
- `putInt(seq)`
- `putLong(tsNs)`

## C++ equivalent

C++ code reads raw bytes and decodes little-endian values.

Typical approach:
- validate first 4 bytes are `"PBOT"`
- check `data[4] == version`
- check `data[5] == msg_type`
- read:
  - `flags` from bytes `[6..7]`
  - `seq` from bytes `[8..11]`
  - `ts_ns` from bytes `[12..19]`

## Why this header exists (practical reasons)

- **`magic`**: quickly reject unrelated UDP traffic (wrong port, stale sender, etc.)
- **`version`**: lets you change layouts later while detecting mismatches immediately
- **`msg_type`**: allows multiple packet kinds on the same UDP port
- **`flags`**: reserved for future use (feature bits, etc.)
- **`seq`**: helps detect packet loss / reordering / duplicates
- **`ts_ns`**: allows latency measurement and timeline alignment in the bridge

## Common debugging tips

- If parsing fails, check these first:
  - magic mismatch (`"PBOT"`)
  - version mismatch (Android vs PC bridge not rebuilt / not synced)
  - msg_type mismatch (sending a different packet type than you expect)
  - payload size mismatch (SENSOR/MOTOR/TORQUE have different fixed sizes)

See also:
- `reference_note/NOTE_protocal.md` for full packet layouts after the header.

