# NOTE: Android phone coordinate frames (IMU / orientation)

This note summarizes the coordinate frames Android uses for IMU sensors and orientation.

## 1) Device frame (phone/body frame)

Android defines a **right-handed** coordinate system fixed to the device (screen):

- **+X**: points to the **right** side of the screen
- **+Y**: points to the **top** of the screen
- **+Z**: points **out of the screen toward you** (perpendicular to the display)

This frame is intended to be consistent across devices.

## 2) World/reference frame used by “orientation” sensors

Some sensors represent orientation relative to a world-like reference:

- **Gravity** provides “down”, so pitch/roll are usually stable.
- **Magnetic field** (magnetometer) can provide “north”, which stabilizes yaw/heading, but can be noisy indoors/near motors.

Important: different sensor types use different references:

### `TYPE_ROTATION_VECTOR`
- Intended to give **absolute orientation** (uses magnetometer when available).
- Yaw (azimuth) tries to align with a world heading (north).
- More sensitive to magnetic disturbances.

### `TYPE_GAME_ROTATION_VECTOR`
- Orientation **without magnetometer**.
- Yaw is **relative** and can drift over time.
- Often smoother and less affected by magnetic noise.

## 3) Yaw / Pitch / Roll meaning in Android

When you compute Euler angles via:
- `getRotationMatrixFromVector(...)` then `getOrientation(...)`

You get (in radians) an array:
- `o[0]`: **azimuth / yaw**
- `o[1]`: **pitch**
- `o[2]`: **roll**

Interpretation depends on your mounting. For robots, you usually remap/define your own:
- robot forward/left/up axes
- then convert phone orientation into robot orientation.

## 4) Mounting reminder (robot use)

Once you decide how the phone is mounted (screen facing forward? sideways? flat?),
you must define a transform from:

**phone device frame (X/Y/Z)** → **robot body frame (forward/left/up)**.

That mapping is often the #1 source of sign/axis confusion.

## 5) UI rotation vs IMU/device frame (important clarification)

- The **IMU/device body frame** is fixed to the *physical phone*:
  - **+X** = right side of the screen
  - **+Y** = top of the screen (toward the earpiece)
  - **+Z** = out of the screen toward you
- The phone’s **auto-rotate setting** only changes the **UI/layout orientation** (portrait/landscape).
  - Turning auto-rotate off does **not** change the IMU coordinate definition.
  - If you **physically rotate the phone**, the IMU/device frame rotates with it (because it’s attached to the phone).


