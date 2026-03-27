# NOTE: PhoneBot frame transformations (normal vs alter IMU)

This note documents the coordinate / frame transforms used to:

- convert **phone IMU** signals (orientation + gyro + accel) into the **policy IMU-site frame**
- keep Android on-device policy inference consistent with the MuJoCo training setup
- keep `robot_visualizer/visualizer_node.py` consistent with the above

The policy is trained on an IMU **site** named `imu` inside the MuJoCo XML. The key difference between modes is **which body that site is attached to**.

## Frames (high level)

- **Phone body frame** (Android sensors):
  - \(+X\): right of screen
  - \(+Y\): top of screen
  - \(+Z\): out of screen (toward viewer)

- **Robot trunk/body frame** (MuJoCo):
  - From comments in `phonebot_general.xml`: trunk/base body frame is \((z \uparrow,\; y \rightarrow,\; x \text{ sideways})\).

- **Robot base body frame**:
  - Same axis convention as trunk body frame; base and trunk are connected by the hinge joint `base_to_trunk` (motor 13) which is a **rotation about +Z**.

- **Policy IMU site frame**:
  - The site `imu` has a fixed local quaternion relative to its parent body:
    - `quat="0.707107 0 0 0.707107"` (wxyz) = **+90° about +Z**

## The two modes

You can detect the mode from metadata:

- **normal**: `env_name="PhonebotJoystickFlatTerrain"`
  - `imu` site is attached to **base_motor_link** (base body)
  - XML: `APP_workspace/app/src/main/assets/exported_tflite/phonebot_general.xml`

- **alter**: `env_name="PhonebotJoystickFlatTerrainAlter"`
  - `imu` site is attached to **trunk_link** (trunk body)
  - XML: `APP_workspace/app/src/main/assets/exported_tflite/phonebot_general_alternative_imu.xml`

Even though the site rotation is the same quaternion in both XMLs, the **parent body differs**, which changes how phone → policy IMU signals must be interpreted.

## Quaternion conventions used in this repo

Both:
- `Ros2_bridge/src/robot_visualizer/robot_visualizer/visualizer_node.py`
- Android policy code (`MainActivity.kt` helper `buildPhonebotObs52`)

use quaternions in **wxyz** order and apply composition like:

\[
q_{A\leftarrow C} = q_{A\leftarrow B} \otimes q_{B\leftarrow C}
\]

Vector rotation is implemented as:

\[
v' = q \otimes (0, v) \otimes q^*
\]

## Step 1: phone mounting correction (phone → trunk)

The phone is mounted on the robot so the phone body axes do not match the robot trunk axes. A fixed correction quaternion is used:

- `Q_PHONE_TO_TRUNK = [0, 0, s, s]` with \(s=\sqrt{2}/2\)

This corresponds to the rotation matrix (from `visualizer_node.py`):

\[
R =
\begin{bmatrix}
-1 & 0 & 0 \\
0 & 0 & 1 \\
0 & 1 & 0
\end{bmatrix}
\]

So vectors are converted as:

\[
v_\text{trunk} = R \, v_\text{phone}
\]

And orientations are converted as:

\[
q_\text{trunk\_in\_world} = q_\text{phone\_in\_world} \otimes Q_\text{PHONE\_TO\_TRUNK}
\]

## Step 2: trunk ↔ base relation via motor 13

The joint `base_to_trunk` is motor 13. In the visualizer code the relationship is treated as:

\[
q_\text{trunk\_in\_world} = q_\text{base\_in\_world} \otimes R_z(\theta)
\]

So:

\[
q_\text{base\_in\_world} = q_\text{trunk\_in\_world} \otimes R_z(-\theta)
\]

For vectors:

\[
v_\text{base} = R_z(-\theta)\, v_\text{trunk}
\]

## Step 3: body → IMU site frame

Both XMLs define the IMU site rotation relative to its parent body as:

- `Q_BODY_TO_IMU_SITE = [s, 0, 0, s]` (wxyz), \(s=\sqrt{2}/2\)

This is a **+90° rotation about +Z**.

To convert a vector from body frame into IMU-site local coordinates:

\[
v_\text{imu} = R_\text{BODY\_TO\_IMU}^\top\, v_\text{body}
\]

and the IMU site orientation in world is:

\[
q_\text{imu\_in\_world} = q_\text{body\_in\_world} \otimes Q_\text{BODY\_TO\_IMU\_SITE}
\]

## Putting it together

### normal mode (policy IMU on **base**)

- `body = base`

1. \(q_\text{trunk} = q_\text{phone} \otimes Q_\text{PHONE\_TO\_TRUNK}\)
2. \(q_\text{base} = q_\text{trunk} \otimes R_z(-\theta)\)
3. \(q_\text{imu} = q_\text{base} \otimes Q_\text{BASE\_TO\_IMU\_SITE}\)

Vectors:

1. \(v_\text{trunk} = R_\text{PHONE\_TO\_TRUNK}\, v_\text{phone}\)
2. \(v_\text{base} = R_z(-\theta)\, v_\text{trunk}\)
3. \(v_\text{imu} = R_\text{BASE\_TO\_IMU}^\top\, v_\text{base}\)

### alter mode (policy IMU on **trunk**)

- `body = trunk`

1. \(q_\text{trunk} = q_\text{phone} \otimes Q_\text{PHONE\_TO\_TRUNK}\)
2. \(q_\text{imu} = q_\text{trunk} \otimes Q_\text{TRUNK\_TO\_IMU\_SITE}\)

Vectors:

1. \(v_\text{trunk} = R_\text{PHONE\_TO\_TRUNK}\, v_\text{phone}\)
2. \(v_\text{imu} = R_\text{TRUNK\_TO\_IMU}^\top\, v_\text{trunk}\)

## Gravity vector used by the policy

Training computes `gravity` in IMU-local coordinates as:

\[
g_\text{imu} = R_\text{imu}^\top \,[0,0,-1]
\]

On Android we do the same using the reconstructed IMU-site world quaternion \(q_\text{imu\_in\_world}\):

\[
g_\text{imu} = \text{rotate}(q_\text{imu\_in\_world}^*, [0,0,-1])
\]

## Where this is implemented

- **Android inference**: `APP_workspace/app/src/main/java/com/example/phonebot_app_android/MainActivity.kt`
  - helper: `buildPhonebotObs52(...)`
- **Visualizer**: `Ros2_bridge/src/robot_visualizer/robot_visualizer/visualizer_node.py`
  - parameters:
    - `mode:=normal|alter`
    - `model_path:=...scene_joystick_flat_terrain*.xml`

