import numpy as np
import math
import cv2
import os
import matplotlib.pyplot as plt

# --------- Load dataset ---------
npz_path = "calib_3.npz"
if not os.path.isfile(npz_path):
    raise FileNotFoundError(f"File not found: {npz_path}")
data = np.load(npz_path)
angles = data["angles"]   # shape (N, 4) in degrees
rvecs  = data["rvecs"]    # shape (N, 3)
tvecs  = data["tvecs"]    # shape (N, 3)

num_samples = angles.shape[0]

# --------- Display all joint angles numerically ---------
print("\nAll Joint Angles (in degrees):\n")
print(f"{'Idx':>3}   {'Joint1':>8}   {'Joint2':>8}   {'Joint3':>8}   {'Joint4':>8}")
print("----  --------  --------  --------  --------")
for i, q in enumerate(angles, start=1):
    print(f"{i:>3}   {q[0]:8.3f}   {q[1]:8.3f}   {q[2]:8.3f}   {q[3]:8.3f}")

# Also create a separate matplotlib table of those angles
fig2, ax2 = plt.subplots(figsize=(5, max(2, num_samples*0.25)))
ax2.axis('off')

cell_text = [
    [str(i), f"{q[0]:.3f}", f"{q[1]:.3f}", f"{q[2]:.3f}", f"{q[3]:.3f}"]
    for i, q in enumerate(angles, start=1)
]
col_labels = ["Idx", "Joint1", "Joint2", "Joint3", "Joint4"]

table = ax2.table(
    cellText=cell_text,
    colLabels=col_labels,
    loc='center',
    cellLoc='center'
)
table.auto_set_font_size(False)
table.set_fontsize(8)
table.scale(1, 1.5)

ax2.set_title("Joint Angles for All Samples", pad=20)
plt.tight_layout()
plt.show()


# --------- Uncalibrated DH and hand-eye ---------
# Each row = [theta_offset, d, a, alpha]
DH_UNCAL = np.array([
    [math.radians(0),        0.093, 0.0,      math.pi/2],
    [math.radians(90-9.505), 0.0,   0.126,    0.0      ],
    [math.radians(9.505),    0.0,   0.130,    0.0      ],
    [math.radians(0),        0.0,   0.147,    math.pi/2],
], dtype=float)

HE_R0 = np.array([1.208, 1.208, 1.208])   # Rodrigues (uncalibrated)
HE_T0 = np.array([-0.06, 0.0, -0.05])     # translation (uncalibrated)

def dh(theta, d, a, alpha):
    """
    Build a 4×4 classic DH transform from (theta, d, a, alpha).
    """
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return np.array([
        [ ct, -st*ca,  st*sa, a*ct ],
        [ st,  ct*ca, -ct*sa, a*st ],
        [  0,      sa,     ca,    d ],
        [  0,       0,      0,    1 ],
    ], dtype=float)

def get_handeye_matrix(rvec, tvec):
    """
    Convert Rodrigues vector + translation into a 4×4 transform.
    """
    R_mat, _ = cv2.Rodrigues(rvec)
    T = np.eye(4)
    T[:3, :3] = R_mat
    T[:3, 3] = tvec
    return T

# Precompute the (constant) uncalibrated hand‐eye matrix:
T_he_uncal = get_handeye_matrix(HE_R0, HE_T0)

# --------- Containers for all samples ---------
cb_uncal_list     = []    # Checkerboard centers (uncalibrated)
ee_pos_list       = []    # Flange (end‐effector) origins
joint_origins     = []    # shape→ list of (4×3) per sample
camera_origins    = []    # camera origin in base frame for each sample

for i in range(num_samples):
    q_deg = angles[i]  # [q1, q2, q3, q4]
    rvec  = rvecs[i]
    tvec  = tvecs[i]

    T_cumulative = np.eye(4)
    this_joints   = []  # will store 4 joint origins for sample i

    # 1) Compute joint origins in base frame (link‐by‐link)
    for j in range(4):
        θ = math.radians(q_deg[j]) + DH_UNCAL[j, 0]
        d, a, α = DH_UNCAL[j, 1:]
        T_link = dh(θ, d, a, α)
        T_cumulative = T_cumulative @ T_link
        joint_origin = T_cumulative[:3, 3].copy()
        this_joints.append(joint_origin)

    joint_origins.append(np.stack(this_joints))  # shape (4,3)

    # 2) Flange (end‐effector) origin = T_cumulative translation after link 4
    flange_origin = T_cumulative[:3, 3].copy()
    ee_pos_list.append(flange_origin)

    # 3) Camera origin in base frame = result of multiplying flange→camera mount
    T_base_to_camera = T_cumulative @ T_he_uncal
    camera_origin = T_base_to_camera[:3, 3].copy()
    camera_origins.append(camera_origin)

    # 4) Checkerboard center in base frame:
    R_cb, _ = cv2.Rodrigues(rvec)
    T_cb = np.eye(4)
    T_cb[:3, :3] = R_cb
    T_cb[:3, 3]  = tvec
    cb_uncal = (T_base_to_camera @ T_cb)[:3, 3]
    cb_uncal_list.append(cb_uncal)

# Stack into NumPy arrays
cb_uncal_arr     = np.stack(cb_uncal_list)     # (N, 3)
ee_pos_arr       = np.stack(ee_pos_list)       # (N, 3)
joint_orig_arr   = np.stack(joint_origins)     # (N, 4, 3)
camera_orig_arr  = np.stack(camera_origins)    # (N, 3)
# joint_orig_arr[i, j, :] = (x,y,z) of joint j+1 in sample i
# camera_orig_arr[i]     = (x,y,z) of camera frame origin for sample i

# --------- 3D Plot with Kinematic Chains + Camera & “Projection Ray” ---------
fig = plt.figure(figsize=(10, 10))
ax = fig.add_subplot(111, projection='3d')
ax.set_title("Uncalibrated Kinematic Chains + Camera Origins + CB Projections")
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.set_zlabel("Z (m)")

# 1) Plot all CB centers (red)
ax.scatter(
    cb_uncal_arr[:, 0],
    cb_uncal_arr[:, 1],
    cb_uncal_arr[:, 2],
    c='red', s=20, alpha=0.6,
    label='CB Centers'
)

# 2) Plot all flange (end‐effector) origins (blue)
ax.scatter(
    ee_pos_arr[:, 0],
    ee_pos_arr[:, 1],
    ee_pos_arr[:, 2],
    c='blue', s=20, alpha=0.6,
    label='Flange Origins'
)

# 3) Plot all camera origins (magenta)
ax.scatter(
    camera_orig_arr[:, 0],
    camera_orig_arr[:, 1],
    camera_orig_arr[:, 2],
    c='magenta', s=20, alpha=0.6,
    label='Camera Origins'
)

# 4) Plot trajectories of each joint (distinct colors)
joint_colors = ['green', 'orange', 'purple', 'brown']
joint_labels = ['Joint 1', 'Joint 2', 'Joint 3', 'Joint 4']
for j in range(4):
    ax.scatter(
        joint_orig_arr[:, j, 0],
        joint_orig_arr[:, j, 1],
        joint_orig_arr[:, j, 2],
        c=joint_colors[j], s=15, alpha=0.6,
        label=joint_labels[j] + " Origins"
    )

# 5) For each sample i, draw a thin gray line connecting Joint1→Joint2→Joint3→Joint4→Flange
for i in range(num_samples):
    joints_xyz = joint_orig_arr[i]         # shape (4,3)
    flange_xyz = ee_pos_arr[i]             # shape (3,)
    xs = np.concatenate([joints_xyz[:, 0], [flange_xyz[0]]])
    ys = np.concatenate([joints_xyz[:, 1], [flange_xyz[1]]])
    zs = np.concatenate([joints_xyz[:, 2], [flange_xyz[2]]])
    ax.plot(xs, ys, zs,
            c='gray', linewidth=0.5, alpha=0.4)

# 6) For each sample i, draw a thin black line (“projection ray”) from camera origin → CB center
for i in range(num_samples):
    cam_xyz = camera_orig_arr[i]
    cb_xyz  = cb_uncal_arr[i]
    ax.plot(
        [cam_xyz[0], cb_xyz[0]],
        [cam_xyz[1], cb_xyz[1]],
        [cam_xyz[2], cb_xyz[2]],
        c='black', linestyle='--', linewidth=0.5, alpha=0.5
    )

# 7) Draw small coordinate axes at the base (0,0,0)
axis_len = 0.05
ax.quiver(0, 0, 0, axis_len, 0, 0, color='black', arrow_length_ratio=0.1)  # X
ax.quiver(0, 0, 0, 0, axis_len, 0, color='black', arrow_length_ratio=0.1)  # Y
ax.quiver(0, 0, 0, 0, 0, axis_len, color='black', arrow_length_ratio=0.1)  # Z
ax.text(0, 0, 0, "Base", color='black')

# 8) Legend (use “ncol=2” if overlapping)
ax.legend(loc='upper left', fontsize='small', ncol=1)
plt.tight_layout()
plt.show()
