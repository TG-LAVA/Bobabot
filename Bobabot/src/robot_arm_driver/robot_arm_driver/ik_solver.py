import numpy as np
# import matplotlib.pyplot as plt
from casadi import SX, vertcat, nlpsol, Function, horzcat, mtimes

class FiveDOFIKSolver:
    def __init__(self):
        self.dh_params = [
            [0.091,    -0.054555,   -0.010, 1.565351],
            [0.009441, 1.489953,    0.127451, 0.008170],
            [-0.010,   0.121548,    0.123048, 0.016081],
            [0.008789, 0.038747,    0.156341, 1.573905],
            [0.0,      0,    0.02,     0.0]
        ]
        self.joint_limits = [
            (-np.radians(150), np.radians(150)),
            (-np.radians(80),  np.radians(135)),
            (-np.radians(175), np.radians(40)),
            (-np.radians(120), np.radians(125)),
            (-np.radians(90),  np.radians(90))
        ]
        self.angle_offset_deg = np.array([0, 90, 0, 0])

    def dh_transform(self, d, theta, a, alpha):
        ct, st = np.cos(theta), np.sin(theta)
        ca, sa = np.cos(alpha), np.sin(alpha)
        return np.array([
            [ct, -st * ca, st * sa, a * ct],
            [st,  ct * ca, -ct * sa, a * st],
            [0,   sa,      ca,      d],
            [0,   0,       0,       1]
        ])

    def forward_kinematics(self, q):
        T = np.eye(4)
        for i in range(len(q)):
            d, _, a, alpha = self.dh_params[i]
            T = T @ self.dh_transform(d, q[i], a, alpha)
        return T
    
    def compensate_gravity_drop(self, x, y, z, k=0.08):
        r = np.sqrt(x**2 + y**2)
        delta_z = k * r**2
        print(f"[Compensation] Horizontal reach: {r:.3f} m → Δz (compensation): {delta_z:.4f} m")
        return x, y, z + delta_z

    def check_reachability(self, x, y, z, current_angles, tolerance=0.04):
        current_deg = np.array(current_angles) + self.angle_offset_deg
        q_start = np.radians(current_deg)
        q_start_full = np.append(q_start, 0.0)
        target = np.array([x, y, z])

        q = SX.sym('q', 5)
        T = SX.eye(4)
        for i in range(5):
            d, _, a, alpha = self.dh_params[i]
            ct = SX.cos(q[i])
            st = SX.sin(q[i])
            ca = np.cos(alpha)
            sa = np.sin(alpha)
            A = vertcat(
                horzcat(ct, -st * ca, st * sa, a * ct),
                horzcat(st,  ct * ca, -ct * sa, a * st),
                horzcat(0,   sa,      ca,      d),
                horzcat(0,   0,       0,       1)
            )
            T = mtimes(T, A)

        pos = T[:3, 3]
        cost = sum((pos[i] - target[i]) ** 2 for i in range(3))
        solver = nlpsol('solver', 'ipopt', {'x': q, 'f': cost}, {
            'ipopt.tol': 1e-6,
            'ipopt.print_level': 0,
            'print_time': False
        })

        lbq = [l[0] for l in self.joint_limits]
        ubq = [l[1] for l in self.joint_limits]

        try:
            sol = solver(x0=q_start_full, lbx=lbq, ubx=ubq)
            q_solution = np.array(sol['x'].full()).flatten()
            ee_pos = self.forward_kinematics(np.append(q_solution[:4], 0.0))[:3, 3]
            error = np.linalg.norm(ee_pos - target)
            print(f"[Reachability] IK-solved position: {ee_pos.round(4)} | Target: {target.round(4)} | Error: {error:.4f} m")
            return error < tolerance
        except Exception as e:
            print("[Reachability] IK failed:", str(e))
            return False

    def solve(self, x, y, z, current_angles=None):
        if current_angles is None:
            current_angles = [0, 0, 0, 0]

        current_deg = np.array(current_angles) + self.angle_offset_deg
        q_start = np.radians(current_deg)
        q_start_full = np.append(q_start, 0.0)
        target_pos = np.array([x, y, z])
        keyframes = []

        # === Orientation threshold check ===
        base_deg = current_angles[0]
        target_angle_rad = np.arctan2(y, x)
        base_rotated_deg = np.degrees(target_angle_rad)
        orientation_change = abs(base_rotated_deg - base_deg)

        if orientation_change >= 40.0:
            # Add Contracted Pose if orientation change > 40 degrees
            contraction_angles_deg = [base_deg, 60, -90, -60]
            contracted_rad = np.radians(np.array(contraction_angles_deg) + self.angle_offset_deg[:4])
            fk_contracted = self.forward_kinematics(np.append(contracted_rad, 0.0))
            contracted_pos = fk_contracted[:3, 3]
            print(f"[Keyframe 1] Contracted EE Position (m): {contracted_pos.round(4)}")
            keyframes.append(contraction_angles_deg)

            # === Then Rotate Base ===
            keyframe2 = [base_rotated_deg] + contraction_angles_deg[1:]
            rotated_rad = np.radians(np.array(keyframe2) + self.angle_offset_deg[:4])
            keyframes.append(keyframe2)
            print(f"[Keyframe 2] Orientation change {orientation_change:.2f}° > 40°, adding contracted + rotated keyframes.")
        else:
            # No contracted pose, go directly
            rotated_rad = np.radians(np.array([base_rotated_deg, 60, -90, -60]) + self.angle_offset_deg[:4])
            print(f"[Keyframe 1] Orientation change {orientation_change:.2f}° < 40°, skipping contraction.")

        # === CasADi symbolic FK for IK solving ===
        q = SX.sym('q', 5)
        T = SX.eye(4)
        joint_positions = []
        T_joint = SX.eye(4)
        for i in range(5):
            d, _, a, alpha = self.dh_params[i]
            ct = SX.cos(q[i])
            st = SX.sin(q[i])
            ca = np.cos(alpha)
            sa = np.sin(alpha)
            A = vertcat(
                horzcat(ct, -st * ca, st * sa, a * ct),
                horzcat(st,  ct * ca, -ct * sa, a * st),
                horzcat(0,   sa,      ca,      d),
                horzcat(0,   0,       0,       1)
            )
            T = mtimes(T, A)
            if i < 4:  # collect positions for self-collision
                T_joint = mtimes(T_joint, A)
                joint_positions.append(T_joint[:3, 3])

        pos = T[:3, 3]
        cost = sum((pos[i] - target_pos[i]) ** 2 for i in range(3))
        lambda_reg = 0.01
        cost += lambda_reg * sum((q[i] - q_start_full[i]) ** 2 for i in range(5))

        # === Self-collision constraints ===
        min_dist = 0.03
        g_list = []
        for i in range(len(joint_positions)):
            for j in range(i + 2, len(joint_positions)):
                diff = joint_positions[i] - joint_positions[j]
                dist_sq = sum(diff[k] ** 2 for k in range(3))
                g_list.append(dist_sq)

        g_constraints = vertcat(*g_list)
        lbg = [min_dist ** 2] * g_constraints.shape[0]
        ubg = [1e5] * g_constraints.shape[0]

        nlp = {'x': q, 'f': cost, 'g': g_constraints}
        solver = nlpsol('solver', 'ipopt', nlp, {
            'ipopt.tol': 1e-10,
            'ipopt.max_iter': 500,
            'ipopt.print_level': 0,
            'print_time': False
        })

        lbq = [l[0] for l in self.joint_limits]
        ubq = [l[1] for l in self.joint_limits]

        sol = solver(x0=q_start_full, lbx=lbq, ubx=ubq, lbg=lbg, ubg=ubg)
        q_solution = np.array(sol['x'].full()).flatten()

        # === Check if camera is inverted ===
        if q_solution[3] > 0:
            print("[Recovery] Camera inverted! Applying contraction recovery...")
            contraction_angles_deg = [base_deg, 60, -90, -60]
            contracted_rad = np.radians(np.array(contraction_angles_deg) + self.angle_offset_deg[:4])
            keyframes.append(contraction_angles_deg)  # Add contracted pose
            rotated_rad = contracted_rad  # Reset interpolation base

        # === Keyframes 3–5: interpolate ===
        interp1 = rotated_rad + 0.33 * (q_solution[:4] - rotated_rad)
        interp2 = rotated_rad + 0.66 * (q_solution[:4] - rotated_rad)
        final = q_solution[:4]

        for q_rad in [interp1, interp2, final]:
            keyframes.append(np.degrees(q_rad) - self.angle_offset_deg[:4])

        final_pos = self.forward_kinematics(np.append(np.radians(keyframes[-1] + self.angle_offset_deg[:4]), 0.0))[:3, 3]
        error = np.linalg.norm(final_pos - target_pos)

        print("Final EE Position (m):", final_pos)
        print("Target Position (m):  ", target_pos)
        print(f"[POST-IK] Error: {error:.4f} m")
        print("Final Joint Angles (deg, adjusted):", np.round(keyframes[-1], 2))
        print(f"Total Keyframes Generated: {len(keyframes)}")

        return keyframes


if __name__ == "__main__":
    solver = FiveDOFIKSolver()
    x, y, z = 0.0, -0.1, 0.25
    current_angles = [50, 0, 0, 0]

    # === Apply gravity compensation before checking reachability ===
    x, y, z = solver.compensate_gravity_drop(x, y, z, k=0.08)

    # === Iterative reachability check with nudging ===
    max_tries = 20
    delta = 0.005
    success = False

    for attempt in range(max_tries):
        print(f"[Attempt {attempt+1}] Checking reachability at ({x:.3f}, {y:.3f}, {z:.3f})")
        if solver.check_reachability(x, y, z, current_angles, tolerance=0.04):
            success = True
            break
        else:
            print(f"[Attempt {attempt+1}] Target unreachable. Nudging position...")
            x += delta
            y += delta
            z += delta

    if success:
        keyframes = solver.solve(x, y, z, current_angles)
        if keyframes is not None:
            print("\nGenerated Keyframes (deg, servo-ready):")
            for i, frame in enumerate(keyframes):
                print(f"Keyframe {i+1}: {np.round(frame, 2)}")
    else:
        print("[Recovery] Max attempts reached. Target still unreachable.")