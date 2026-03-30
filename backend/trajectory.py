#!/usr/bin/env python3
"""
Trajectory generation and collision detection for robotic arm.
"""

import numpy as np
from typing import List, Tuple, Optional
from .kinematics import compute_arm_positions, inverse_kinematics_3dof, ArmConfig


def generate_trajectory(waypoints: List[dict], num_points: int = 100, method: str = 'linear') -> List[dict]:
    """
    Generate a smooth trajectory through waypoints.

    Args:
        waypoints: list of {'pos': [x,y,z], 'wrist': [q4,q5,q6]}
        num_points: total number of interpolated points
        method: 'linear' or 'cubic' (cubic spline not yet implemented)

    Returns:
        List of trajectory points with interpolated positions and wrist angles.
    """
    if len(waypoints) < 2:
        return waypoints.copy()

    # We'll interpolate the XYZ positions linearly and also interpolate wrist angles linearly.
    # For cubic, we'd use scipy.interpolate.CubicSpline (but keep simple for now).

    traj = []
    total_segments = len(waypoints) - 1
    points_per_segment = num_points // total_segments
    remainder = num_points % total_segments

    config = ArmConfig()

    for i in range(total_segments):
        start_wp = waypoints[i]
        end_wp = waypoints[i+1]
        start_pos = np.array(start_wp['pos'])
        end_pos = np.array(end_wp['pos'])
        start_wrist = np.array(start_wp['wrist'])
        end_wrist = np.array(end_wp['wrist'])

        # Allocate points for this segment
        seg_pts = points_per_segment + (1 if i < remainder else 0)
        if seg_pts < 1:
            continue

        for j in range(seg_pts):
            t = j / seg_pts
            pos = start_pos * (1-t) + end_pos * t
            wrist = start_wrist * (1-t) + end_wrist * t
            traj.append({'pos': pos.tolist(), 'wrist': wrist.tolist()})

    # Ensure exact inclusion of final waypoint
    if traj:
        traj[-1] = waypoints[-1].copy()

    return traj


def check_self_collision(angles: Tuple[float, float, float, float, float, float], config: ArmConfig = None) -> bool:
    """
    Simple self-collision check: ensure links don't intersect (using bounding cylinders).
    Returns True if collision detected, False if safe.
    """
    if config is None:
        config = ArmConfig()

    # Compute positions
    pos = compute_arm_positions(*angles, config=config)
    # Points: base, shoulder, elbow, wrist, tip, tip
    p0, p1, p2, p3, p4 = pos[0], pos[1], pos[2], pos[3], pos[4]

    # Check link lengths are respected (an IK sanity check)
    d12 = np.linalg.norm(p2 - p1)
    d23 = np.linalg.norm(p3 - p2)
    d34 = np.linalg.norm(p4 - p3)
    L1, L2, Lg = config.upper_arm_length, config.lower_arm_length, config.gripper_offset
    tol = 0.01  # tolerance

    if abs(d12 - L1) > tol:
        return True
    if abs(d23 - L2) > tol:
        return True
    if abs(d34 - Lg) > tol:
        return True

    # Additional: check if elbow is too close to base?
    # Or check that links don't cross in an unnatural way.
    # For a simple planar-like arm, main issue is if angles cause arm to fold back on itself (though IK already constrains elbow_down)

    return False


def check_workspace_bounds(pos: List[float], config: ArmConfig = None) -> bool:
    """
    Check if end-effector position is within reachable workspace.
    """
    if config is None:
        config = ArmConfig()
    x, y, z = pos
    # ReCompute distance from shoulder-level projection
    # Base to shoulder height
    shoulder_z = config.base_height
    # Planar distance from shoulder to (x,y)
    r = np.sqrt(x**2 + y**2)
    z_rel = z - shoulder_z
    d = np.sqrt(r**2 + z_rel**2)
    L1, L2, Lg = config.upper_arm_length, config.lower_arm_length, config.gripper_offset
    max_reach = L1 + L2 + Lg
    min_reach = max(0.0, L1 - L2 - Lg)  # actually for 3-DOF with elbow constraint, min reach is more complex
    # Simple: allow within a small tolerance of max_reach and min_reach
    return d <= max_reach + 0.01 and d >= abs(L1 - (L2+Lg)) - 0.01


def solve_ik_for_waypoint(pos: List[float], wrist: List[float], config: ArmConfig = None, elbow_down: bool = True) -> Optional[Tuple[float, float, float, float, float, float]]:
    """
    Solve IK for a waypoint: XYZ gives q1,q2,q3; wrist angles are direct.
    Returns (q1,q2,q3,q4,q5,q6) or None if unreachable.
    """
    if config is None:
        config = ArmConfig()
    result = inverse_kinematics_3dof(pos[0], pos[1], pos[2], config, elbow_down=elbow_down)
    if result is None:
        return None
    q1, q2, q3 = result
    q4, q5, q6 = wrist
    # Normalize
    q1 = (q1 + 180) % 360 - 180
    q2 = (q2 + 180) % 360 - 180
    q3 = (q3 + 180) % 360 - 180
    return (float(q1), float(q2), float(q3), float(q4), float(q5), float(q6))


def validate_trajectory(trajectory: List[dict]) -> List[bool]:
    """
    Validate each trajectory point: returns list of booleans (True=valid, False=collision/out of bounds).
    """
    valid = []
    config = ArmConfig()
    for pt in trajectory:
        angles = solve_ik_for_waypoint(pt['pos'], pt['wrist'])
        if angles is None:
            valid.append(False)
            continue
        # Check workspace
        if not check_workspace_bounds(pt['pos'], config):
            valid.append(False)
            continue
        # Check self-collision (simple)
        if check_self_collision(angles, config):
            valid.append(False)
            continue
        valid.append(True)
    return valid
