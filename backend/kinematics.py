#!/usr/bin/env python3
"""
Forward kinematics for a 6-DOF robotic arm.
Computes joint positions given roll, pitch, yaw (for the glove) or individual joint angles.
"""

import numpy as np
from dataclasses import dataclass
from typing import List


@dataclass
class ArmConfig:
    """Configuration for the arm model."""
    # Segment lengths (meters)
    base_height: float = 0.1
    upper_arm_length: float = 0.3
    lower_arm_length: float = 0.25
    wrist_length: float = 0.1
    gripper_length: float = 0.05

    # Joint colors (for visualization)
    joint_colors: List[str] = None

    def __post_init__(self):
        if self.joint_colors is None:
            self.joint_colors = ['#666666', '#3498db', '#e67e22', '#2ecc71', '#e74c3c', '#e74c3c']


def compute_arm_positions(roll: float, pitch: float, yaw: float, config: ArmConfig = None) -> np.ndarray:
    """
    Compute 6 joint positions from glove orientation angles (Euler angles in degrees).

    Simplified arm model:
    - Base: at (0,0,0) with rotation yaw (horizontal rotation)
    - Shoulder: at base height + small offset, rotation pitch (vertical)
    - Elbow: at upper arm length along shoulder's direction
    - Wrist: at lower arm length
    - Gripper: at wrist length (end effector)
    - Optional: maybe an extra joint for rotation

    Args:
        roll: rotation around X-axis (radial, side to side) in degrees
        pitch: rotation around Y-axis (vertical up/down) in degrees
        yaw: rotation around Z-axis (horizontal left/right) in degrees

    Returns:
        positions: (6, 3) numpy array of (x, y, z) coordinates in meters.
    """
    if config is None:
        config = ArmConfig()

    # Convert to radians
    r = np.radians(roll)
    p = np.radians(pitch)
    y = np.radians(yaw)

    # Base position
    base = np.array([0.0, 0.0, 0.0])

    # Shoulder position (up from base)
    shoulder = base + np.array([0.0, 0.0, config.base_height])

    # Simple simplification: The angles directly control the direction of the upper arm.
    # Upper arm segment direction determined by pitch and yaw; roll is wrist rotation.
    # For MVP, let's treat it as: upper arm extends from shoulder with direction based on (pitch, yaw)
    # Then elbow, wrist follow in chain; we'll add some wrist roll effect.

    # Upper arm direction vector (spherical angles: elevation = pitch, azimuth = yaw)
    # Normalize direction
    dir_vec = np.array([
        np.sin(p) * np.cos(y),
        np.cos(p) * np.sin(y),
        np.cos(p)  # Z positive up? We'll use Z up.
    ])
    if np.linalg.norm(dir_vec) > 0:
        dir_vec = dir_vec / np.linalg.norm(dir_vec)
    else:
        dir_vec = np.array([0, 0, 1])

    # Elbow position
    elbow = shoulder + dir_vec * config.upper_arm_length

    # Lower arm continues in same direction (for simplicity)
    wrist = elbow + dir_vec * config.lower_arm_length

    # Gripper extends further with a slight offset based on roll
    # Add some offset based on roll (rotation around the arm axis)
    gripper = wrist + dir_vec * config.gripper_length

    # Create array of 6 points. For simplicity, we'll include base, shoulder,
    # elbow, wrist, gripper tip, and maybe gripper tip again or an intermediate point.
    positions = np.array([
        base,
        shoulder,
        elbow,
        wrist,
        gripper,
        gripper + np.array([0, 0, 0])  # duplicate for visualization (or add a small tip offset)
    ])

    return positions


# Alternative: for free-body rotation, we just rotate a single point/cube
def rotation_matrix_from_euler(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """
    Create 3x3 rotation matrix from Euler angles (XYZ order) in degrees.
    """
    r = np.radians(roll)
    p = np.radians(pitch)
    y = np.radians(yaw)

    Rx = np.array([[1, 0, 0],
                   [0, np.cos(r), -np.sin(r)],
                   [0, np.sin(r), np.cos(r)]])
    Ry = np.array([[np.cos(p), 0, np.sin(p)],
                   [0, 1, 0],
                   [-np.sin(p), 0, np.cos(p)]])
    Rz = np.array([[np.cos(y), -np.sin(y), 0],
                   [np.sin(y), np.cos(y), 0],
                   [0, 0, 1]])

    return Rz @ Ry @ Rx  # Combined rotation: first roll (X), then pitch (Y), then yaw (Z)
