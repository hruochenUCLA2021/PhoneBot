# Android → USB‑Serial → Dynamixel XL430 (Protocol 2.0)

This project adds basic motor control directly from the Android phone using:

- Phone USB‑C port (USB host mode / OTG)
- USB → Serial adapter (FTDI / CH340 / CP210x, etc.)
- Dynamixel TTL bus via adapter + proper power

## Key assumptions (current implementation)

- **Protocol**: Dynamixel **Protocol 2.0**
- **Motors**: XL430‑W250
- **Motor IDs**: `1..13` (hard-coded default)
- **Baudrate**: `1,000,000`
- **Operating Mode**: Position Control (`Operating Mode` = `3`)
- **Gains**:
  - `Position P Gain` = `5` (register units)
  - `kd` (user) = `0.1` (float) → mapped to `Position D Gain` register (integer)
    - current mapping: `kd>0` ⇒ `D_gain_reg >= 1`, so default `0.1` maps to `1`
- **Initial Goal Position**: `0 rad` ≈ `2048 ticks`
- **Command source**: UDP motor packet `pos[13]` (interpreted as radians), sent to motors at **50 Hz**.
- **Command loop**: **100 Hz** (measured in-app)

## XL430 control table addresses used
From `reference_repo/DynamixelSDK/control_table/xl430_w250.model`:

- `Operating Mode` @ `11` (1 byte)
- `Torque Enable` @ `64` (1 byte)
- `Status Return Level` @ `68` (1 byte) — we set to `0` to reduce reply traffic
- `Position D Gain` @ `80` (2 bytes)
- `Position P Gain` @ `84` (2 bytes)
- `Goal Position` @ `116` (4 bytes)

## Radians → ticks mapping

We map radians in \([-π, +π]\) to ticks in \([0, 4095]\) with:

- \(0\) rad → \(2048\) ticks
- \(+π\) rad → \(4095\) ticks (clamped)
- \(-π\) rad → \(0\) ticks

## Android UI

The app has a **Motor HW (USB → Dynamixel XL430)** section:
- **OFF → ON**: requests USB permission, opens serial, configures all motors once, enables torque.
- **ON → OFF**: disables torque for all motors and closes the serial port.

## Important safety notes

- **Power**: XL430 motors need external power; do not power from the phone.
- **Wiring**: ensure correct TTL wiring + common ground.
- **Torque**: enabling torque can cause sudden motion; test with legs off the ground first.


