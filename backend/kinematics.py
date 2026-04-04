#!/usr/bin/env python3
"""
Forward and inverse kinematics for a 3-DOF robotic arm (base yaw, shoulder pitch, elbow pitch).
Computes joint positions given joint angles, or solves joint angles from end-effector position.
"""

import numpy as np
from dataclasses import dataclass
from typing import List, Tuple, Optional


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

    Joints:
        q1: base yaw (rotation about Z)
        q2: shoulder pitch (rotation about Y)
        q3: elbow pitch (rotation about Y)
        q4: wrist roll (rotation about X)
        q5: wrist pitch (rotation about Y)
        q6: wrist yaw (rotation about Z)

    Returns:
        positions: (6, 3) numpy array for:
            0: base (0,0,0)
            1: shoulder
            2: elbow
            3: wrist center
            4: gripper tip
            5: duplicate tip
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

    # Shoulder position (origin of frame 1)
    O1 = np.array([0.0, 0.0, h0])

    # Rotation matrices (intrinsic order: yaw, pitch, pitch, then wrist roll/pitch/yaw)
    c1, s1 = np.cos(q1r), np.sin(q1r)
    R1 = np.array([[c1, -s1, 0],
                   [s1, c1, 0],
                   [0, 0, 1]])

    c2, s2 = np.cos(q2r), np.sin(q2r)
    R2 = np.array([[c2, 0, s2],
                   [0, 1, 0],
                   [-s2, 0, c2]])

    c3, s3 = np.cos(q3r), np.sin(q3r)
    R3 = np.array([[c3, 0, s3],
                   [0, 1, 0],
                   [-s3, 0, c3]])

    # Combined rotation to wrist center (frame 3)
    R03 = R1 @ R2 @ R3

    # Positions: shoulder -> elbow along X axis of frame2
    O2 = O1 + R1 @ R2 @ np.array([L1, 0, 0])
    # Elbow -> wrist along X axis of frame3
    O3 = O2 + R1 @ R2 @ R3 @ np.array([L2, 0, 0])  # wrist center

    # Wrist rotations: roll about X, pitch about Y, yaw about Z (all intrinsic)
    c4, s4 = np.cos(q4r), np.sin(q4r)
    R4 = np.array([[1, 0, 0],
                   [0, c4, -s4],
                   [0, s4, c4]])
    c5, s5 = np.cos(q5r), np.sin(q5r)
    R5 = np.array([[c5, 0, s5],
                   [0, 1, 0],
                   [-s5, 0, c5]])
    c6, s6 = np.cos(q6r), np.sin(q6r)
    R6 = np.array([[c6, -s6, 0],
                   [s6, c6, 0],
                   [0, 0, 1]])
    R36 = R4 @ R5 @ R6

    # Full rotation from base to tool frame
    R06 = R03 @ R36

    # Tool tip: from wrist center along X axis of tool frame
    tool_offset = R06[:, 0] * Lg
    O4 = O3 + tool_offset

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
    def __init__(self, joints: List[DHJoint] = None, base_height: float = 0.0):
        self.joints = joints or []
        self.base_height = base_height  # base origin offset in Z

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

        T_base = np.eye(4)
        T_base[2, 3] = self.base_height  # base origin at (0,0,base_height)
        transforms = [T_base]

        for i, joint in enumerate(self.joints):
            # Get variable for this joint
            if joint.type == 'revolute':
                theta = np.radians(joint_angles[i] if joint_angles else joint.theta)
                d = joint.d
            elif joint.type == 'prismatic':
                theta = np.radians(joint.theta)
                d = joint.d + (joint_angles[i] if joint_angles else 0.0)
            else:  # fixed
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

    def inverse_kinematics(self, target_position: np.ndarray, initial_angles: List[float] = None, max_iter: int = 100, tol: float = 0.01) -> Optional[List[float]]:
        """
        Numerical IK using Jacobian pseudoinverse.
        Solves for joint angles (in degrees) that place the end-effector at target_position.
        Returns list of angles (one per joint). Fixed joints are ignored (their value won't affect result).
        """
        n = len(self.joints)
        if n == 0:
            return None
        # Determine which joints are variable (revolute or prismatic). We'll solve for all that change.
        variable_mask = []
        for j in self.joints:
            variable_mask.append(j.type in ('revolute', 'prismatic'))
        var_indices = [i for i, v in enumerate(variable_mask) if v]
        if not var_indices:
            return None

        # Initial guess
        if initial_angles is None:
            angles = []
            for j in self.joints:
                if j.type == 'revolute':
                    angles.append(j.theta)
                elif j.type == 'prismatic':
                    angles.append(j.d)
                else:
                    angles.append(0.0)
        else:
            angles = list(initial_angles)

        # Small delta in degrees for finite differences
        delta_deg = 0.1

        for it in range(max_iter):
            # Forward kinematics with current angles
            transforms = self.forward_kinematics(angles)
            ee_pos = transforms[-1][:3, 3]
            error = target_position - ee_pos
            if np.linalg.norm(error) < tol:
                return angles

            # Compute Jacobian J (3 x n) via finite differences
            J = np.zeros((3, n))
            for idx in var_indices:
                perturbed = angles.copy()
                perturbed[idx] += delta_deg
                pos_plus = self.forward_kinematics(perturbed)[-1][:3, 3]
                J[:, idx] = (pos_plus - ee_pos) / delta_deg
            # Pseudoinverse
            try:
                J_pinv = np.linalg.pinv(J)
            except np.linalg.LinAlgError:
                return None
            delta_angles = J_pinv @ error
            # Update angles
            for i in var_indices:
                angles[i] += delta_angles[i]

        # Did not converge
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
