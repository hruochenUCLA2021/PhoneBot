# Phone IMU → Robot Base Frame Transformation

## 1. Coordinate Frames

### Android Phone Body Frame
(from Android sensor documentation, see `smartphone_axes.png`)

- **+X**: right side of the screen (when facing the screen)
- **+Y**: top of the phone (along the screen, towards earpiece)
- **+Z**: out of the screen (towards the viewer)
- Right-handed: X × Y = Z

### Robot Base Frame (`base_motor_link`)
(from `phonebot_general_alternative_imu.xml` comment)

- **+X**: sideways (towards the right leg, `r_hip_pitch` at pos `+0.05`)
- **+Y**: forward (the direction the robot walks)
- **+Z**: up (towards the sky)
- Right-handed: X × Y = Z

### MuJoCo IMU Site Frame (on `trunk_link`)
(from XML: `<site name="imu" quat="0.707107 0.000000 0.000000 0.707107"/>`)

- This is a 90-degree rotation about Z relative to the trunk body frame.
- The IMU site is used only by MuJoCo's simulated sensors — it does NOT
  affect the real phone measurement. The real phone measures its own body
  frame orientation directly.

## 2. Phone Mounting on the Robot

The phone is mounted on the **trunk** with:
- **Screen faces forward** (away from robot's back, in the walking direction)
- **Phone top (+Y) points up** (towards the sky)

## 3. Deriving the Rotation

Standing **behind the robot**, both you and the robot face forward.
Looking at the **back of the phone** (since the screen faces away from you):

| Phone axis | Direction (from behind) | Robot base axis |
|------------|------------------------|-----------------|
| +X (right of screen) | points to YOUR left = robot's LEFT | **−X_robot** |
| +Y (top of phone) | points up | **+Z_robot** |
| +Z (out of screen) | points forward | **+Y_robot** |

Verify right-handedness:
- base_Z = base_X × base_Y = (−phone_X) × (phone_Z) = −(phone_X × phone_Z) = −(−phone_Y) = **phone_Y** ✓

### Rotation matrix (v_base = R · v_phone)

```
R = [-1   0   0]
    [ 0   0   1]
    [ 0   1   0]
```

det(R) = −1·(0−1) − 0 + 0 = **+1** ✓ (proper rotation)

### Quaternion (wxyz)

Using scipy: `Rotation.from_matrix(R).as_quat()` → `[x,y,z,w] = [0, 0.7071, 0.7071, 0]`

Converted to MuJoCo [w,x,y,z] order:

```
Q_PHONE_TO_BASE = [0, 0, 0.7071, 0.7071]
```

## 4. Full Pipeline

```
q_phone_sensor    (from Android TYPE_GAME_ROTATION_VECTOR, via ROS2 /phonebot/imu_game)
        |
        |  post-multiply by Q_PHONE_TO_BASE
        v
q_trunk_in_world  (orientation of trunk_link body in MuJoCo world frame)
        |
        |  post-multiply by Rz(−theta),  where theta = base_to_trunk joint angle
        v
q_base_in_world   (orientation of base_motor_link, written to freejoint qpos[3:7])
```

### Why Rz(−theta)?

The `base_to_trunk` hinge joint rotates about the Z axis. In the kinematic chain:

```
q_trunk = q_base · Rz(theta)
```

Solving for q_base:

```
q_base = q_trunk · Rz(−theta)
```

## 5. Previous Errors and Why They Were Wrong

### Attempt 1: `Q_PHONE_TO_ROBOT = Rx(−90) · Ry(−90)`
- Quaternion: `[0.5, −0.5, −0.5, 0.5]`
- This assumed phone_X → base_Y (forward) and phone_Z → base_X, which was
  geometrically incorrect for the screen-forward mounting.
- Also incorrectly applied `Q_IMU_SITE_TO_TRUNK` (the MuJoCo simulated sensor
  site rotation), which is irrelevant for real phone measurements.

### Attempt 2: `Rx(−90)` only → `[0.7071, −0.7071, 0, 0]`
- This correctly mapped phone_Y → base_Z (up/down was correct).
- But it assumed phone_X → +base_X (right = right), which is wrong.
- When viewing the phone from behind (back of phone visible), phone +X
  actually points to the robot's LEFT (= −X_robot), not right.
- This caused forward/sideways to be swapped or inverted.

### Correct: `[0, 0, 0.7071, 0.7071]`
- Correctly maps: phone_X → −base_X, phone_Z → +base_Y, phone_Y → +base_Z.
- All three axes verified.

## 6. Key Lesson

When reasoning about phone axis directions on a robot, always think from
the **back of the phone** (the side you see when standing behind the robot),
not from the screen side. The phone's +X axis (right of screen) reverses
when viewed from behind.
