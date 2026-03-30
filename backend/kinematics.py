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
