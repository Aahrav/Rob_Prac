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


def compute_arm_positions(q1: float, q2: float, q3: float, config: ArmConfig = None) -> np.ndarray:
    """
    Forward kinematics: compute 6 joint positions from joint angles (degrees).

    Joint 1 (q1): base yaw (rotation around Z)
    Joint 2 (q2): shoulder pitch (rotation around Y)
    Joint 3 (q3): elbow pitch (rotation around Y, relative to upper arm)

    Args:
        q1, q2, q3: joint angles in degrees.
        config: ArmConfig with segment lengths.

    Returns:
        positions: (6, 3) numpy array of (x, y, z) coordinates for:
            0: base (ground center)
            1: shoulder (at base_height)
            2: elbow
            3: wrist
            4: gripper tip
            5: extra point (duplicate of tip for 6-point chain)
    """
    if config is None:
        config = ArmConfig()

    # Convert to radians
    q1r = np.radians(q1)
    q2r = np.radians(q2)
    q3r = np.radians(q3)

    L1 = config.upper_arm_length
    L2 = config.lower_arm_length
    Lg = config.gripper_offset
    h0 = config.base_height

    # Shoulder position
    shoulder = np.array([0.0, 0.0, h0])

    # Compute wrist offset from elbow
    # Upper arm vector from shoulder to elbow:
    # After Rz(q1) and Ry(q2), the local X axis direction in world coords is:
    # dir_upper = [cos(q2)*cos(q1), cos(q2)*sin(q1), sin(q2)]
    # Elbow position = shoulder + L1 * dir_upper
    elbow_x = L1 * np.cos(q2r) * np.cos(q1r)
    elbow_y = L1 * np.cos(q2r) * np.sin(q1r)
    elbow_z = h0 + L1 * np.sin(q2r)
    elbow = np.array([elbow_x, elbow_y, elbow_z])

    # Lower arm direction: after additional Ry(q3) from elbow
    # So lower arm direction is: rotate by (q2+q3) in the same scheme:
    dir_lower_x = np.cos(q2r + q3r) * np.cos(q1r)
    dir_lower_y = np.cos(q2r + q3r) * np.sin(q1r)
    dir_lower_z = np.sin(q2r + q3r)

    wrist_x = elbow_x + L2 * dir_lower_x
    wrist_y = elbow_y + L2 * dir_lower_y
    wrist_z = elbow_z + L2 * dir_lower_z
    wrist = np.array([wrist_x, wrist_y, wrist_z])

    # Gripper tip extends a bit further along same direction
    tip_x = wrist_x + Lg * dir_lower_x
    tip_y = wrist_y + Lg * dir_lower_y
    tip_z = wrist_z + Lg * dir_lower_z
    tip = np.array([tip_x, tip_y, tip_z])

    base = np.array([0.0, 0.0, 0.0])

    positions = np.array([
        base,
        shoulder,
        elbow,
        wrist,
        tip,
        tip  # duplicate to fill 6th point
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
