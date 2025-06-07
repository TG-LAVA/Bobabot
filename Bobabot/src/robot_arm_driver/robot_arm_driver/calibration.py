#!/usr/bin/env python3
import numpy as np, cv2, math, os, sys, argparse, pyceres as ceres
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# ───────────── Parameters ──────────────────────────
DH0 = np.array([[math.radians(0),        0.093, 0.0,  math.pi/2],
                [math.radians(90-9.505), 0.0,   0.126, 0.0],
                [math.radians(9.505),    0.0,   0.130, 0.0],
                [math.radians(0),        0.0, 0.147,  math.pi/2]], float)

HE_R0 = np.array([1.208, 1.208, 1.208])
HE_T0 = np.array([-0.06, 0.0, -0.05])

DLIM = 0.005
ALIM = math.radians(2)
THLIM = math.radians(2)

B0_GROUND_TRUTH = np.array([0.59, -0.038, 0.038])

# ───────────── Helper Functions ────────────────────
def dh(th, d, a, al):
    ct, st, ca, sa = math.cos(th), math.sin(th), math.cos(al), math.sin(al)
    return np.array([[ct,-st*ca, st*sa, a*ct],
                     [st, ct*ca,-ct*sa, a*st],
                     [0 ,    sa,    ca,    d],
                     [0 ,     0,     0,    1]])

# ───────────── Visualisation Function ──────────────
def visualize_calibration(dh_opt, he_r, he_t, B0, ang, tv):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Observed positions
    obs_pts = np.array(tv)
    ax.scatter(obs_pts[:,0], obs_pts[:,1], obs_pts[:,2], c='r', marker='o', label='Observed')

    # Predicted positions
    R_he, _ = cv2.Rodrigues(he_r)
    T_he = np.eye(4); T_he[:3,:3] = R_he; T_he[:3,3] = he_t

    pred_pts = []
    for q in ang:
        T = np.eye(4)
        q_rad = np.radians(q)
        for i in range(4):
            th = q_rad[i] + dh_opt[i,0]
            d,a,al = dh_opt[i,1:]
            T = T @ dh(th,d,a,al)
        T_cam = T @ T_he
        pred_pt = (np.linalg.inv(T_cam) @ np.append(B0,1))[:3]
        pred_pts.append(pred_pt)

    pred_pts = np.array(pred_pts)
    ax.scatter(pred_pts[:,0], pred_pts[:,1], pred_pts[:,2], c='g', marker='^', label='Predicted')

    # Ground truth
    ax.scatter([B0[0]], [B0[1]], [B0[2]], c='b', marker='*', s=200, label='Ground Truth')

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()
    plt.title("Calibration Results")
    plt.show()

# ───────────── Main ────────────────────────────────
def main(npz_path):
    if not os.path.isfile(npz_path):
        sys.exit(f"File not found: {npz_path}")

    D = np.load(npz_path)
    ang, tv = D['angles'], D['tvecs']

    # Example optimized values (replace with real results from optimization)
    dh_opt = DH0.copy()
    he_r, he_t = HE_R0, HE_T0
    B0 = B0_GROUND_TRUTH

    visualize_calibration(dh_opt, he_r, he_t, B0, ang, tv)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("npz", nargs="?", default="/home/yinuo/BubbleBot/calib.npz",
                    help="calibration capture file (.npz)")
    args = parser.parse_args()
    main(args.npz)

