#!/usr/bin/env python3
"""
Forward and inverse kinematics for a 3-DOF robotic arm (base yaw, shoulder pitch, elbow pitch).
Computes joint positions given joint angles, or solves joint angles from end-effector position.
"""

import numpy as np
from dataclasses import dataclass
from typing import List, Tuple, Optional
from backend.logger import get_logger

log = get_logger(__name__)


@dataclass
class ArmConfig:
    """Configuration for the arm model."""
    base_height: float = 0.1      # Height of shoulder base above ground (meters)
    upper_arm_length: float = 0.3 # Length from shoulder to elbow
    lower_arm_length: float = 0.25 # Length from elbow to wrist (gripper)
    gripper_offset: float = 0.05  # Small offset beyond wrist for gripper tip

    joint_colors: List[str] = None

    def __post_init__(self):
        if self.joint_colors is None:
            self.joint_colors = ['#666666', '#3498db', '#e67e22', '#2ecc71', '#e74c3c', '#e74c3c']


def compute_arm_positions(q1: float, q2: float, q3: float,
                          q4: float = 0.0, q5: float = 0.0, q6: float = 0.0,
                          config: ArmConfig = None) -> np.ndarray:
    """
    Forward kinematics for 6-DOF arm: compute joint positions from joint angles (degrees).

    Convention:
        q2 > 0 lifts the upper arm UP (positive Z)
        q2 < 0 lowers the upper arm DOWN (toward ground)
        q3 > 0 bends the elbow so the forearm points upward relative to upper arm

    Joints:
        q1: base yaw (rotation about Z, CCW looking down)
        q2: shoulder pitch (rotation about Y, positive = up)
        q3: elbow pitch (rotation about Y, positive = bend up relative to upper arm)
        q4: wrist roll (rotation about X)
        q5: wrist pitch (rotation about Y)
        q6: wrist yaw (rotation about Z)

    Returns:
        positions: (6, 3) numpy array for:
            0: base (0,0,0)
            1: shoulder (0,0,base_height)
            2: elbow
            3: wrist center
            4: gripper tip
            5: duplicate tip (for 6-point chain compatibility)
    """
    if config is None:
        config = ArmConfig()

    # Convert to radians
    q1r = np.radians(q1)
    q2r = np.radians(q2)
    q3r = np.radians(q3)
    q4r = np.radians(q4)
    q5r = np.radians(q5)
    q6r = np.radians(q6)

    L1 = config.upper_arm_length
    L2 = config.lower_arm_length
    Lg = config.gripper_offset
    h0 = config.base_height

    # Shoulder position
    O1 = np.array([0.0, 0.0, h0])

    # Base yaw rotation Rz(q1)
    c1, s1 = np.cos(q1r), np.sin(q1r)
    R1 = np.array([[c1, -s1, 0],
                   [s1,  c1, 0],
                   [0,    0,  1]])

    # Shoulder pitch Ry(q2) — with sign convention: positive q2 = up
    # Standard Ry matrix: [cos, 0, sin; 0, 1, 0; -sin, 0, cos]
    # We use the transpose (equivalent to Ry(-q2)) so that positive q2 lifts the arm UP
    c2, s2 = np.cos(q2r), np.sin(q2r)
    R2 = np.array([[ c2, 0, -s2],
                   [ 0,  1,  0 ],
                   [ s2, 0,  c2]])

    # Elbow pitch Ry(q3) — same convention as q2
    c3, s3 = np.cos(q3r), np.sin(q3r)
    R3 = np.array([[ c3, 0, -s3],
                   [ 0,  1,  0 ],
                   [ s3, 0,  c3]])

    # Upper arm tip (elbow) position: shoulder + R1 * (R2 * [L1, 0, 0])
    # R2 @ [L1,0,0] = [cos(q2)*L1, 0, sin(q2)*L1]  →  positive q2 lifts UP ✓
    O2 = O1 + R1 @ (R2 @ np.array([L1, 0, 0]))

    # Forearm tip (wrist center) position: elbow + R1 * R2 * (R3 * [L2, 0, 0])
    O3 = O2 + R1 @ R2 @ (R3 @ np.array([L2, 0, 0]))

    # Wrist rotations
    c4, s4 = np.cos(q4r), np.sin(q4r)
    R4 = np.array([[1,  0,   0],
                   [0,  c4, -s4],
                   [0,  s4,  c4]])
    c5, s5 = np.cos(q5r), np.sin(q5r)
    R5 = np.array([[ c5, 0, -s5],
                   [ 0,  1,  0 ],
                   [ s5, 0,  c5]])
    c6, s6 = np.cos(q6r), np.sin(q6r)
    R6 = np.array([[c6, -s6, 0],
                   [s6,  c6, 0],
                   [0,    0,  1]])
    R36 = R4 @ R5 @ R6

    # Full rotation base→tool
    R06 = R1 @ R2 @ R3 @ R36

    # Tool tip along X axis of tool frame
    O4 = O3 + R06[:, 0] * Lg

    base = np.array([0.0, 0.0, 0.0])

    positions = np.array([
        base,
        O1,
        O2,
        O3,
        O4,
        O4
    ])
    return positions


def inverse_kinematics_3dof(x: float, y: float, z: float, config: ArmConfig = None, elbow_down: bool = True) -> Optional[Tuple[float, float, float]]:
    """
    Compute joint angles (q1, q2, q3) to place the gripper tip at target (x, y, z).

    Uses geometric 2-link IK in the plane defined by base yaw.

    Args:
        x, y, z: target position in meters.
        config: ArmConfig.
        elbow_down: if True, prefers elbow-down configuration; else elbow-up.

    Returns:
        (q1, q2, q3) in degrees if reachable, else None (or clamped to nearest reachable).
    """
    if config is None:
        config = ArmConfig()

    L1 = config.upper_arm_length
    L2 = config.lower_arm_length + config.gripper_offset
    h0 = config.base_height

    # Base yaw
    q1 = np.degrees(np.arctan2(y, x))

    # Distance from shoulder projection to target in XY plane
    r = np.sqrt(x**2 + y**2)
    # Vertical distance from shoulder to target
    dz = z - h0

    # Distance from shoulder to target
    d = np.sqrt(r**2 + dz**2)

    # Reach limits
    max_reach = L1 + L2
    min_reach = abs(L1 - L2)

    # Clamp target to be within reach (you could also return None)
    if d > max_reach:
        # Scale towards shoulder direction to max reach
        scale = max_reach / d
        r *= scale
        dz *= scale
        d = max_reach
    elif d < min_reach:
        scale = min_reach / d
        r *= scale
        dz *= scale
        d = min_reach

    # Solve for elbow position using law of cosines
    # Angle at shoulder between vector to target and vector to elbow
    cos_theta = (L1**2 + d**2 - L2**2) / (2 * L1 * d)
    cos_theta = np.clip(cos_theta, -1, 1)
    theta = np.arccos(cos_theta)  # always positive

    # Angle of target vector from shoulder (in the vertical plane)
    phi = np.arctan2(dz, r)

    # Two elbow configurations: elbow-down (phi - theta) or elbow-up (phi + theta)
    if elbow_down:
        elbow_sign = -1
    else:
        elbow_sign = 1

    # Elbow angle offset from target direction
    elbow_angle_offset = elbow_sign * theta

    # Elbow (shoulder to elbow) vector angle in the plane
    alpha = phi + elbow_angle_offset  # angle from horizontal (r axis) up to upper arm

    # Elbow position (in the plane)
    ex = L1 * np.cos(alpha)
    ey = 0  # not used; we are in 2D plane
    ez = L1 * np.sin(alpha)

    # Shoulder pitch (q2) is alpha (angle from horizontal plane? Actually q2 is pitch: 0 means arm horizontal? Wait our convention: q2=0 gives horizontal arm? Let's check: in compute_arm_positions, when q2=0, cos=1, sin=0 -> elbow at (L1,0,h0). So arm is horizontal along X. That's q2=0 meaning arm parallel to ground. That's typical. So q2 = degrees(alpha) (since alpha = angle from horizontal). So:
    q2 = np.degrees(alpha)

    # Elbow angle q3: angle between upper arm and lower arm.
    # Lower arm direction relative to upper arm:
    # The lower arm vector in the plane has angle beta = ??? We can compute from triangle:
    # The angle at elbow between upper arm (pointing from elbow to shoulder) and lower arm (pointing to target) is supplementary to theta?
    # Actually we have triangle: shoulder -> elbow (length L1), elbow -> wrist (L2), shoulder -> target (d). The interior angle at elbow (between elbow->shoulder and elbow->target) is, using law of cosines:
    # cos(angle_elbow) = (L1^2 + L2^2 - d^2) / (2*L1*L2)
    # That's the interior angle between the lines from elbow to shoulder and from elbow to target.
    # But our q3 is the joint rotation relative to upper arm. In our forward kinematics: lower arm direction = rotation by q3 relative to upper arm direction. If arm is straight, q3=0. If we want to bend, q3 negative for elbow-down (since we add q3 to q2 for lower arm direction). Let's derive:

    # In FK: lower arm angle = q2 + q3 (in radians). So q3 = (lower arm angle) - q2.
    # Lower arm angle = angle of vector from elbow to wrist in the plane. That vector is from elbow to target (since wrist at target). So its angle from horizontal is:
    # lower_angle = np.arctan2(dz, r)  ??? Actually from elbow, the vector to target is (r - ex, dz - ez). But our ex, ez are computed such that elbow is at (L1*cos(alpha), L1*sin(alpha)). And target is at (r, dz) in the plane. So vector from elbow to target is (r - L1*cos(alpha), dz - L1*sin(alpha)). Its angle = arctan2(dz - L1*sin(alpha), r - L1*cos(alpha)). That angle should equal alpha + q3 (since lower arm direction = q2+q3 = alpha+q3). So:
    q3 = np.degrees(np.arctan2(dz - ez, r - ex) - alpha)

    # Normalize q3 to reasonable range (e.g., -180 to 180). Could also clamp to limits.
    # For simplicity, return as is.

    return q1, q2, q3


# ============================================
# Denavit-Hartenberg (DH) Parameter Kinematics
# ============================================

class DHJoint:
    """Represents a single joint using Denavit-Hartenberg parameters."""
    def __init__(self, joint_type: str, theta: float, d: float, a: float, alpha: float, 
                 name: str = "", q_min: float = None, q_max: float = None):
        """
        Args:
            joint_type: 'revolute' or 'prismatic' or 'fixed'
            theta: joint angle (degrees) for revolute, or fixed value if prismatic
            d: link offset (meters) along z-axis; for prismatic this is the variable displacement
            a: link length (meters) along x-axis
            alpha: twist angle (degrees) about x-axis
            name: optional joint name for UI
            q_min, q_max: optional joint limits (degrees for revolute, meters for prismatic)
        """
        self.type = joint_type  # 'revolute', 'prismatic', 'fixed'
        self.theta = float(theta)  # degrees
        self.d = float(d)          # meters
        self.a = float(a)          # meters
        self.alpha = float(alpha)  # degrees
        self.name = name or f"Joint {id(self) % 1000}"
        self.q_min = q_min
        self.q_max = q_max

    def to_dict(self):
        return {
            'type': self.type,
            'theta': self.theta,
            'd': self.d,
            'a': self.a,
            'alpha': self.alpha,
            'name': self.name,
            'q_min': self.q_min,
            'q_max': self.q_max,
        }

    @staticmethod
    def from_dict(data: dict):
        return DHJoint(**data)

    def __repr__(self):
        return f"DHJoint({self.type}, θ={self.theta}°, d={self.d}m, a={self.a}m, α={self.alpha}°)"


class KinematicChain:
    """A chain of DH joints representing a robotic manipulator."""
    def __init__(self, joints: List[DHJoint] = None, base_height: float = 0.0, name: str = ""):
        self.joints = joints if joints is not None else []
        self.base_height = base_height
        self.name = name

    def add_joint(self, joint: DHJoint):
        self.joints.append(joint)

    def remove_joint(self, index: int):
        if 0 <= index < len(self.joints):
            del self.joints[index]

    def move_joint(self, from_idx: int, to_idx: int):
        """Reorder joint: move joint at from_idx to position to_idx."""
        if 0 <= from_idx < len(self.joints) and 0 <= to_idx < len(self.joints):
            joint = self.joints.pop(from_idx)
            self.joints.insert(to_idx, joint)

    def get_joint(self, index: int) -> DHJoint:
        return self.joints[index]

    def update_joint(self, index: int, **kwargs):
        joint = self.joints[index]
        for k, v in kwargs.items():
            if hasattr(joint, k):
                setattr(joint, k, float(v) if k in ['theta', 'd', 'a', 'alpha'] else v)

    def forward_kinematics(self, joint_angles: List[float] = None) -> List[np.ndarray]:
        """
        Compute positions of all joint frames (including end-effector) using DH parameters.
        Returns list of 4x4 homogeneous transformation matrices for each joint frame.
        The base frame (0) is at (0,0,base_height) with identity orientation.
        """
        import numpy.linalg as LA

        log.debug(
            "FK called | joints=%d | angles=%s",
            len(self.joints),
            [round(a, 3) for a in joint_angles] if joint_angles else "from joint.theta/d",
        )

        T_base = np.eye(4)
        T_base[2, 3] = self.base_height  # base origin at (0,0,base_height)
        transforms = [T_base]

        for i, joint in enumerate(self.joints):
            # Joint displacement q (0.0 if not provided)
            q = 0.0
            if joint_angles is not None and i < len(joint_angles):
                q = joint_angles[i]
            
            if joint.type == 'revolute':
                # theta_final = theta_base + q_revolute
                theta = np.radians(q + joint.theta)
                d = joint.d
            elif joint.type == 'prismatic':
                # d_final = d_base + q_prismatic
                theta = np.radians(joint.theta)
                d = q + joint.d
            else:
                # Fixed: T is constant from DH params
                theta = np.radians(joint.theta)
                d = joint.d

            a = joint.a
            alpha = np.radians(joint.alpha)

            # DH transformation matrix (standard DH)
            cth, sth = np.cos(theta), np.sin(theta)
            cal, sal = np.cos(alpha), np.sin(alpha)
            T = np.array([
                [cth, -sth * cal,  sth * sal, a * cth],
                [sth,  cth * cal, -cth * sal, a * sth],
                [0,    sal,        cal,       d],
                [0,    0,          0,         1]
            ])
            T_new = transforms[-1] @ T
            transforms.append(T_new)
            log.debug(
                "  J%d (%s) θ=%.2f d=%.3f a=%.3f α=%.2f → pos=(%.3f, %.3f, %.3f)",
                i + 1, joint.type,
                np.degrees(theta) if joint.type != 'prismatic' else joint.theta,
                d, joint.a, joint.alpha,
                T_new[0, 3], T_new[1, 3], T_new[2, 3],
            )

        ee = transforms[-1][:3, 3]
        log.debug("FK result EE=(%.4f, %.4f, %.4f)", ee[0], ee[1], ee[2])
        return transforms

    def joint_positions(self, joint_angles: List[float] = None) -> np.ndarray:
        """
        Compute 3D positions (x,y,z) of each joint origin from the transforms.
        Returns an (N+1, 3) array where N = number of joints.
        """
        transforms = self.forward_kinematics(joint_angles)
        positions = np.array([T[:3, 3] for T in transforms])
        return positions

    def to_dict(self) -> dict:
        """Serialize chain to dict for saving."""
        return {
            'base_height': self.base_height,
            'joints': [j.to_dict() for j in self.joints]
        }

    @staticmethod
    def from_dict(data: dict):
        joints = [DHJoint.from_dict(j) for j in data['joints']]
        return KinematicChain(joints, base_height=data.get('base_height', 0.0))

    def get_max_reach(self) -> float:
        """
        Calculate the maximum possible distance the end-effector can reach from the base.
        This is a conservative upper bound (sum of all link lengths and prismatic strokes).
        """
        max_reach = 0.0
        for i, j in enumerate(self.joints):
            contribution = 0.0
            if j.type == 'prismatic':
                d_max = abs(j.d)
                if j.q_max is not None:
                    d_max = max(d_max, abs(j.q_max))
                contribution = abs(j.a) + d_max
            else:
                contribution = abs(j.a) + abs(j.d)
            max_reach += contribution
            print(f"DEBUG: Joint {i+1} ({j.name}) reach contribution: {contribution:.4f}")
        return max_reach

    def inverse_kinematics(self, target_position: np.ndarray, initial_angles: List[float] = None, max_iter: int = 1000, tol: float = 0.005) -> Optional[List[float]]:
        """
        Numerical IK using damped least-squares (Levenberg-Marquardt) Jacobian.
        Solves for joint angles (in degrees for revolute, meters for prismatic) that place
        the end-effector at target_position.
        Returns list of angles (one per joint). Fixed joints are ignored.
        """
        n = len(self.joints)
        log.info(
            "IK requested | target=(%.4f, %.4f, %.4f) | joints=%d | base_height=%.3f",
            target_position[0], target_position[1], target_position[2], n, self.base_height,
        )

        if n == 0:
            log.warning("IK aborted — chain has no joints")
            return None

        # Determine which joints are variable (revolute or prismatic).
        var_indices = [i for i, j in enumerate(self.joints) if j.type in ('revolute', 'prismatic')]
        if not var_indices:
            log.warning("IK aborted — no variable joints (all fixed)")
            return None

        log.debug(
            "IK variable joints: %s",
            [(i, self.joints[i].type, self.joints[i].name) for i in var_indices],
        )

        # Quick reachability check
        max_reach = self.get_max_reach()
        dist = np.linalg.norm(target_position - np.array([0.0, 0.0, self.base_height]))
        log.debug("IK reachability | max_reach=%.4f | dist_from_base=%.4f", max_reach, dist)
        if dist > max_reach * 1.05:
            log.warning("IK aborted — target out of reach (dist=%.4f > max_reach*1.05=%.4f)", dist, max_reach * 1.05)
            return None

        def _run_ik(start_full_angles, run_label=""):
            angles = list(start_full_angles)
            best_angles = angles[:]
            best_error = float('inf')

            # Finite-difference delta: 0.01° for revolute, 0.1mm for prismatic
            delta_deg = 0.01
            # Damping factor — start small, increase if stuck
            damping = 0.01
            max_step_deg = 10.0
            max_step_m = 0.02

            log.debug(
                "IK run%s | start=%s",
                f" [{run_label}]" if run_label else "",
                [round(a, 3) for a in start_full_angles],
            )

            for it in range(max_iter):
                transforms = self.forward_kinematics(angles)
                ee_pos = transforms[-1][:3, 3]
                error = target_position - ee_pos
                err_norm = np.linalg.norm(error)

                if err_norm < best_error:
                    best_error = err_norm
                    best_angles = angles[:]

                # Log every 50 iterations to track convergence without flooding
                if it % 50 == 0 or err_norm < tol:
                    log.debug(
                        "  iter %3d | err=%.5f | ee=(%.4f,%.4f,%.4f) | angles=%s",
                        it, err_norm,
                        ee_pos[0], ee_pos[1], ee_pos[2],
                        [round(a, 2) for a in angles],
                    )

                if err_norm < tol:
                    log.info(
                        "IK converged%s at iter %d | err=%.5f | solution=%s",
                        f" [{run_label}]" if run_label else "",
                        it, err_norm,
                        [round(a, 3) for a in angles],
                    )
                    return angles, err_norm

                # Build Jacobian J (3 x len(var_indices)) via finite differences
                J = np.zeros((3, len(var_indices)))
                for col, idx in enumerate(var_indices):
                    perturbed = angles[:]
                    perturbed[idx] += delta_deg
                    pos_plus = self.forward_kinematics(perturbed)[-1][:3, 3]
                    J[:, col] = (pos_plus - ee_pos) / delta_deg

                # Damped least-squares: delta_q = J^T (J J^T + lambda^2 I)^{-1} error
                JJT = J @ J.T
                try:
                    delta_vars = J.T @ np.linalg.solve(JJT + damping**2 * np.eye(3), error)
                except np.linalg.LinAlgError as exc:
                    log.error("IK Jacobian solve failed at iter %d: %s", it, exc)
                    break

                # Apply step with clamping to prevent divergence
                for col, idx in enumerate(var_indices):
                    step = delta_vars[col]
                    if self.joints[idx].type == 'revolute':
                        step = np.clip(step, -max_step_deg, max_step_deg)
                    else:
                        step = np.clip(step, -max_step_m, max_step_m)
                    angles[idx] += step

            log.debug(
                "IK run%s ended | best_err=%.5f | best_angles=%s",
                f" [{run_label}]" if run_label else "",
                best_error,
                [round(a, 3) for a in best_angles],
            )
            return best_angles, best_error

        # Try multiple random restarts to escape local minima
        import random
        best_overall_angles = None
        best_overall_error = float('inf')

        # First try: use current joint angles as initial guess
        if initial_angles is None:
            initial_full = []
            for j in self.joints:
                if j.type == 'revolute':
                    initial_full.append(j.theta)
                elif j.type == 'prismatic':
                    initial_full.append(j.d)
                else:
                    initial_full.append(0.0)
        else:
            # If initial_angles is provided, it might be just variable joints or full
            if len(initial_angles) == len(var_indices):
                initial_full = []
                var_idx = 0
                for j in self.joints:
                    if j.type in ('revolute', 'prismatic'):
                        initial_full.append(initial_angles[var_idx])
                        var_idx += 1
                    else:
                        initial_full.append(0.0)
            else:
                initial_full = list(initial_angles)

        result_angles, result_error = _run_ik(initial_full, run_label="initial")
        if result_error < best_overall_error:
            best_overall_error = result_error
            best_overall_angles = result_angles

        if best_overall_error < tol:
            log.info("IK solved on initial guess | final_err=%.5f", best_overall_error)
            return best_overall_angles

        # Random restarts
        rng = np.random.default_rng(42)
        for restart_idx in range(10):
            rand_start = []
            for j in self.joints:
                if j.type == 'revolute':
                    lo = j.q_min if j.q_min is not None else -180.0
                    hi = j.q_max if j.q_max is not None else 180.0
                    rand_start.append(float(rng.uniform(lo, hi)))
                elif j.type == 'prismatic':
                    lo = j.q_min if j.q_min is not None else 0.0
                    hi = j.q_max if j.q_max is not None else 1.0
                    rand_start.append(float(rng.uniform(lo, hi)))
                else:
                    rand_start.append(0.0)
            result_angles, result_error = _run_ik(rand_start, run_label=f"restart-{restart_idx+1}")
            if result_error < best_overall_error:
                best_overall_error = result_error
                best_overall_angles = result_angles
            if best_overall_error < tol:
                log.info(
                    "IK solved on restart %d | final_err=%.5f",
                    restart_idx + 1, best_overall_error,
                )
                break

        # Return best solution found, or None if too far off
        if best_overall_error < 0.05:  # within 5 cm — acceptable near-solution
            log.info(
                "IK returning near-solution | best_err=%.5f | angles=%s",
                best_overall_error,
                [round(a, 3) for a in best_overall_angles],
            )
            # Return only the variable joint values for the animation
            return [best_overall_angles[i] for i in var_indices]

        log.warning(
            "IK FAILED | best_err=%.5f (threshold 0.05 m) | target=(%.4f,%.4f,%.4f)",
            best_overall_error,
            target_position[0], target_position[1], target_position[2],
        )
        return None

    @staticmethod
    def create_3dof_arm(config: ArmConfig) -> 'KinematicChain':
        """Create a 3-DOF arm (yaw, pitch, elbow) matching the existing compute_arm_positions convention."""
        # Base yaw (revolute around Z, at base)
        j1 = DHJoint('revolute', theta=0.0, d=config.base_height, a=0.0, alpha=0.0, name="Base Yaw")
        # Shoulder pitch (revolute around Y, link length = upper_arm_length)
        j2 = DHJoint('revolute', theta=0.0, d=0.0, a=config.upper_arm_length, alpha=0.0, name="Shoulder Pitch")
        # Elbow pitch (revolute around Y, link length = lower_arm_length)
        j3 = DHJoint('revolute', theta=0.0, d=0.0, a=config.lower_arm_length, alpha=0.0, name="Elbow Pitch")
        # Wrist offset (fixed, gripper length along X)
        j4 = DHJoint('fixed', theta=0.0, d=0.0, a=config.gripper_offset, alpha=0.0, name="Gripper Offset")
        return KinematicChain([j1, j2, j3, j4], base_height=0.0)  # base_height already in j1.d

# Presets for common robots
def preset_3dof_arm() -> KinematicChain:
    """Return a standard 3-DOF arm with default dimensions."""
    return KinematicChain.create_3dof_arm(ArmConfig())

def preset_2dof_arm() -> KinematicChain:
    """2-DOF planar arm (shoulder pitch, elbow pitch) on a fixed base."""
    j1 = DHJoint('fixed', theta=0.0, d=0.1, a=0.0, alpha=0.0, name="Base Fixed")
    j2 = DHJoint('revolute', theta=0.0, d=0.0, a=0.3, alpha=0.0, name="Shoulder")
    j3 = DHJoint('revolute', theta=0.0, d=0.0, a=0.25, alpha=0.0, name="Elbow")
    j4 = DHJoint('fixed', theta=0.0, d=0.0, a=0.05, alpha=0.0, name="End Link")
    return KinematicChain([j1, j2, j3, j4], base_height=0.0)
