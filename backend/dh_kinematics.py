#!/usr/bin/env python3
"""
DH-based kinematics for arbitrary serial manipulators.
Supports both standard DH parameters (θ, d, a, α) and transformation composition.
"""

import numpy as np
from typing import List, Dict, Tuple, Optional

def dh_transform(theta: float, d: float, a: float, alpha: float) -> np.ndarray:
    """
    Compute homogeneous transformation from DH parameters.
    Uses standard DH convention: Rot_z(θ) * Trans_z(d) * Trans_x(a) * Rot_x(α)
    Angles in radians, distances in meters.
    Returns 4x4 transformation matrix.
    """
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)

    T = np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0,   sa,     ca,    d   ],
        [0,   0,      0,     1   ]
    ])
    return T

def forward_kinematics_dh(joints: List[Dict]) -> Tuple[List[np.ndarray], np.ndarray]:
    """
    Compute forward kinematics for a serial chain given DH parameters.
    joints: list of dicts with keys: type ('revolute' or 'prismatic'), theta, d, a, alpha
    Returns:
      - transforms: list of 4x4 homogeneous transforms for each joint frame (including base)
      - end_effector: 4x4 transform of end-effector frame
    """
    T = np.eye(4)  # base transform
    transforms = [T.copy()]

    for joint in joints:
        if joint['type'] == 'revolute':
            theta = joint['theta']
            d = joint['d']
        elif joint['type'] == 'prismatic':
            theta = joint['theta']  # fixed theta for prismatic
            d = joint['d']          # variable d
        elif joint['type'] == 'fixed':
            theta = joint['theta']
            d = joint['d']
        else:
            raise ValueError(f"Unknown joint type: {joint['type']}")

        T_joint = dh_transform(theta, d, joint['a'], joint['alpha'])
        T = T @ T_joint
        transforms.append(T.copy())

    return transforms, T

def joint_angles_to_dh_params(joints: List[Dict], variable_vals: List[float]) -> List[Dict]:
    """
    Convert a list of joints with variable values into DH parameter sets.
    variable_vals: list of values for each joint (θ for revolute, d for prismatic)
    Returns a new list of DH dicts with updated variable parameters.
    """
    dh_list = []
    for i, joint in enumerate(joints):
        dh = joint.copy()
        if joint['type'] == 'revolute':
            dh['theta'] = variable_vals[i]
        elif joint['type'] == 'prismatic':
            dh['d'] = variable_vals[i]
        else:  # fixed
            pass  # no change
        dh_list.append(dh)
    return dh_list

def compute_joint_positions(transforms: List[np.ndarray]) -> np.ndarray:
    """
    From a list of joint frames ( transforms ), extract origins (x,y,z).
    Returns array of shape (N, 3).
    """
    positions = np.array([T[:3, 3] for T in transforms])
    return positions

def dh_to_denavit_hartenberg_string(joints: List[Dict]) -> str:
    """Format joints as a DH table string for display."""
    lines = ["i | type   | θ (rad) | d (m) | a (m) | α (rad)"]
    lines.append("-" * 50)
    for i, j in enumerate(joints):
        typ = j['type'][:3]
        theta = j.get('theta', 0)
        d = j.get('d', 0)
        a = j.get('a', 0)
        alpha = j.get('alpha', 0)
        lines.append(f"{i} | {typ:6} | {theta:7.3f} | {d:5.3f} | {a:5.3f} | {alpha:7.3f}")
    return "\n".join(lines)
