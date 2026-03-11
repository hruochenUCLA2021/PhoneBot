"""
Interactive 3D visualization of the phone-to-robot frame transformation.

Uses a COMMON world convention for all panels:
    world X = right,  world Y = forward,  world Z = up

This matches the robot base frame, so the robot axes are identity.
The phone axes are drawn IN the world frame according to the physical mounting
(screen forward, phone top up, standing behind the robot).

Panels:
  1. Phone body axes drawn in the world frame (as physically mounted)
  2. Robot base axes drawn in the world frame (identity)
  3. Phone axes transformed by R_PHONE_TO_BASE — should overlap with panel 2
  4. Mapping diagram showing which phone axis becomes which robot axis

Usage:
    python check_frame_transformation.py
"""

import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from scipy.spatial.transform import Rotation


# ──────────────────────────────────────────────
# Transformation to verify
# ──────────────────────────────────────────────

R_PHONE_TO_BASE = np.array([
    [-1,  0,  0],
    [ 0,  0,  1],
    [ 0,  1,  0],
], dtype=float)

_s = np.sqrt(2.0) / 2.0
Q_PHONE_TO_BASE_WXYZ = np.array([0.0, 0.0, _s, _s])
Q_PHONE_TO_BASE_XYZW = np.array([Q_PHONE_TO_BASE_WXYZ[1],
                                  Q_PHONE_TO_BASE_WXYZ[2],
                                  Q_PHONE_TO_BASE_WXYZ[3],
                                  Q_PHONE_TO_BASE_WXYZ[0]])

R_from_quat = Rotation.from_quat(Q_PHONE_TO_BASE_XYZW).as_matrix()

print("R_PHONE_TO_BASE (defined):")
print(R_PHONE_TO_BASE)
print()
print("R from quaternion [0, 0, 0.7071, 0.7071] (wxyz):")
print(np.round(R_from_quat, 4))
print()
print("Match:", np.allclose(R_PHONE_TO_BASE, R_from_quat))
print()

# Phone axes expressed in the WORLD frame (world = robot base convention).
# Physical mounting: screen forward, phone top up, standing behind robot.
#   phone +X (right of screen) → world -X (robot's left)
#   phone +Y (top of phone)    → world +Z (up)
#   phone +Z (out of screen)   → world +Y (forward)
# So the columns of R_PHONE_IN_WORLD are where phone X/Y/Z end up in world coords.
R_PHONE_IN_WORLD = np.array([
    [-1,  0,  0],   # phone_X in world
    [ 0,  0,  1],   # phone_Z in world → world Y
    [ 0,  1,  0],   # phone_Y in world → world Z
], dtype=float).T  # transpose: columns = phone axes in world

# Actually, let's be explicit.
# Column 0 = where phone +X goes in world = [-1, 0, 0]
# Column 1 = where phone +Y goes in world = [0, 0, 1]
# Column 2 = where phone +Z goes in world = [0, 1, 0]
R_PHONE_IN_WORLD = np.array([
    [-1,  0,  0],
    [ 0,  0,  1],
    [ 0,  1,  0],
], dtype=float)

R_ROBOT_IN_WORLD = np.eye(3)

# Verify: R_PHONE_TO_BASE applied to phone axes in phone frame (identity)
# should give phone axes in robot base frame = R_PHONE_IN_WORLD
print("Phone axis mappings (v_base = R * v_phone):")
for label, v in [("phone_X=[1,0,0]", [1,0,0]),
                 ("phone_Y=[0,1,0]", [0,1,0]),
                 ("phone_Z=[0,0,1]", [0,0,1])]:
    result = R_PHONE_TO_BASE @ np.array(v)
    print(f"  {label} → base {np.round(result, 4)}")
print()


# ──────────────────────────────────────────────
# Drawing helpers
# ──────────────────────────────────────────────

def draw_frame(ax, origin, axes_cols, labels, title, scale=1.0,
               colors=("r", "g", "b"), alpha=1.0, lw=2.5):
    """
    Draw a coordinate frame.
    axes_cols: 3x3 array whose COLUMNS are the three axis directions in plot coords.
    """
    for i in range(3):
        d = axes_cols[:, i] * scale
        ax.quiver(
            origin[0], origin[1], origin[2],
            d[0], d[1], d[2],
            color=colors[i], arrow_length_ratio=0.12, linewidth=lw, alpha=alpha,
        )
        tip = origin + d * 1.15
        ax.text(tip[0], tip[1], tip[2], labels[i],
                color=colors[i], fontsize=10, fontweight="bold", alpha=alpha)
    if title:
        ax.text(origin[0], origin[1], origin[2] - 0.35 * scale, title,
                fontsize=10, ha="center", color="k", fontweight="bold")


def setup_world_axes(ax, title=""):
    """Configure a 3D axes with world convention labels."""
    ax.set_xlabel("world X (right)")
    ax.set_ylabel("world Y (forward)")
    ax.set_zlabel("world Z (up)")
    if title:
        ax.set_title(title, fontsize=11)
    lim = 1.5
    ax.set_xlim([-lim, lim])
    ax.set_ylim([-lim, lim])
    ax.set_zlim([-lim, lim])
    ax.view_init(elev=25, azim=-60)


# ──────────────────────────────────────────────
# Figure 1: Three panels
# ──────────────────────────────────────────────

fig = plt.figure(figsize=(20, 7))
fig.suptitle(
    "Phone → Robot Base Transformation   "
    "(Q_PHONE_TO_BASE = [0, 0, 0.707, 0.707] wxyz)\n"
    "All panels use the SAME world frame: X=right, Y=forward, Z=up",
    fontsize=13, fontweight="bold",
)

# --- Panel 1: Phone axes as physically mounted, drawn in world coords ---
ax1 = fig.add_subplot(131, projection="3d")
draw_frame(ax1, np.zeros(3), R_PHONE_IN_WORLD,
           ["ph+X\n(right of screen\n= world -X)",
            "ph+Y\n(top of phone\n= world +Z)",
            "ph+Z\n(out of screen\n= world +Y)"],
           "Phone (mounted)", scale=1.0)
setup_world_axes(ax1, "Phone axes in world\n(physical mounting)")

# --- Panel 2: Robot base frame = world identity ---
ax2 = fig.add_subplot(132, projection="3d")
draw_frame(ax2, np.zeros(3), R_ROBOT_IN_WORLD,
           ["+X (right/leg)", "+Y (forward)", "+Z (up)"],
           "Robot base", scale=1.0)
setup_world_axes(ax2, "Robot base axes in world\n(= world identity)")

# --- Panel 3: Apply R_PHONE_TO_BASE to phone-frame identity, show result ---
# In phone's own frame, its axes are [1,0,0], [0,1,0], [0,0,1].
# After transformation: each column of R_PHONE_TO_BASE is where that phone axis
# ends up in the robot base frame.
R_transformed = R_PHONE_TO_BASE  # columns = transformed phone axes in base coords
ax3 = fig.add_subplot(133, projection="3d")

# Ghost: original phone axes in world (faint)
draw_frame(ax3, np.zeros(3), R_PHONE_IN_WORLD,
           ["ph_X", "ph_Y", "ph_Z"],
           "", scale=0.5, alpha=0.25, lw=1.5)

# Transformed result
draw_frame(ax3, np.zeros(3), R_transformed,
           ["→ -X_base", "→ +Z_base", "→ +Y_base"],
           "Transformed", scale=1.0)

# Also draw robot base axes as dashed reference
for i, (c, lbl) in enumerate(zip(["r", "g", "b"],
                                   ["ref X", "ref Y", "ref Z"])):
    d = R_ROBOT_IN_WORLD[:, i] * 1.0
    ax3.plot([0, d[0]], [0, d[1]], [0, d[2]],
             color=c, linestyle="--", linewidth=1.0, alpha=0.35)

setup_world_axes(ax3, "R_PHONE_TO_BASE applied\n(solid = result, dashed = robot ref)")


# ──────────────────────────────────────────────
# Figure 2: Mapping diagram
# ──────────────────────────────────────────────

fig2 = plt.figure(figsize=(12, 8))
fig2.suptitle("Axis Mapping: Phone → Robot Base", fontsize=13, fontweight="bold")
ax4 = fig2.add_subplot(111, projection="3d")

origin_phone = np.array([-2.0, 0, 0])
origin_base  = np.array([ 2.0, 0, 0])

# Draw phone axes in world coords (shifted to origin_phone)
phone_cols_in_world = R_PHONE_IN_WORLD
phone_labels = ["+X_phone\n(screen right → world -X)",
                "+Y_phone\n(phone top → world +Z)",
                "+Z_phone\n(out screen → world +Y)"]
draw_frame(ax4, origin_phone, phone_cols_in_world,
           phone_labels, "PHONE\n(mounted)", scale=1.0,
           colors=["#ff6666", "#66ff66", "#6666ff"])

# Draw robot base axes (shifted to origin_base)
base_labels = ["+X_base (right/leg)", "+Y_base (forward)", "+Z_base (up)"]
draw_frame(ax4, origin_base, R_ROBOT_IN_WORLD,
           base_labels, "ROBOT BASE", scale=1.0,
           colors=["#cc0000", "#00cc00", "#0000cc"])

# Mapping arrows
mappings = [
    (0, "ph_X → -base_X", "#cc0000"),
    (1, "ph_Y → +base_Z", "#0000cc"),
    (2, "ph_Z → +base_Y", "#00cc00"),
]
for ph_idx, label, color in mappings:
    start = origin_phone + phone_cols_in_world[:, ph_idx] * 0.6
    # Where does it map? R_PHONE_TO_BASE column ph_idx
    target_in_base = R_PHONE_TO_BASE[:, ph_idx]
    end = origin_base + target_in_base * 0.6
    ax4.plot([start[0], end[0]], [start[1], end[1]], [start[2], end[2]],
             color=color, linestyle="--", linewidth=1.5, alpha=0.6)
    mid = (start + end) / 2
    ax4.text(mid[0], mid[1], mid[2] + 0.15, label,
             fontsize=9, ha="center", alpha=0.8, color=color)

ax4.set_xlabel("world X (right)")
ax4.set_ylabel("world Y (forward)")
ax4.set_zlabel("world Z (up)")
ax4.set_xlim([-4, 4])
ax4.set_ylim([-2, 2])
ax4.set_zlim([-2, 2])
ax4.view_init(elev=25, azim=-60)

plt.tight_layout()
plt.show()
