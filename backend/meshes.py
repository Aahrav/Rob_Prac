#!/usr/bin/env python3
"""
3D mesh generation for robotic arm visualization.
Creates cylinder and sphere meshes for Poly3DCollection.
"""

import numpy as np
from typing import Tuple, List
from functools import lru_cache


def cylinder_mesh(start: np.ndarray, end: np.ndarray, radius: float, resolution: int = 12) -> Tuple[np.ndarray, List[List[int]]]:
    """
    Generate vertices and faces for a cylinder between two 3D points.
    Vertices are computed with fully vectorized NumPy.

    Args:
        start: (3,) start point
        end: (3,) end point
        radius: cylinder radius
        resolution: number of segments around circumference

    Returns:
        vertices: (N, 3) array of vertex positions
        faces: list of lists of vertex indices (each face is a polygon)
    """
    # Direction vector and length
    axis = end - start
    length = np.linalg.norm(axis)
    if length < 1e-9:
        return np.zeros((0, 3)), []
    axis = axis / length

    # Build orthonormal basis for the cross-section
    if abs(axis[2]) < 0.9:
        up = np.array([0.0, 0.0, 1.0])
    else:
        up = np.array([0.0, 1.0, 0.0])
    right = np.cross(axis, up)
    right /= np.linalg.norm(right)
    forward = np.cross(axis, right)

    # Vectorized: generate both rings at once
    angles = np.linspace(0, 2 * np.pi, resolution, endpoint=False)
    cos_a = np.cos(angles)[:, None]  # (R, 1)
    sin_a = np.sin(angles)[:, None]  # (R, 1)
    offsets = radius * (cos_a * right + sin_a * forward)  # (R, 3)

    vertices = np.empty((2 * resolution, 3))
    vertices[:resolution] = start + offsets
    vertices[resolution:] = end + offsets

    # Faces — use cached face index list
    faces = _cylinder_faces(resolution)
    return vertices, faces


@lru_cache(maxsize=8)
def _cylinder_faces(resolution: int) -> List[List[int]]:
    """Pre-compute and cache face index lists for a given resolution."""
    faces = []
    for i in range(resolution):
        ni = (i + 1) % resolution
        faces.append([i, ni, ni + resolution])
        faces.append([i, ni + resolution, i + resolution])
    faces.append(list(range(resolution)))
    faces.append(list(range(resolution + resolution - 1, resolution - 1, -1)))
    return faces


def sphere_mesh(center: np.ndarray, radius: float, resolution: int = 8) -> Tuple[np.ndarray, List[List[int]]]:
    """
    Generate vertices and faces for a sphere (fully vectorized).
    Uses latitude-longitude grid computed with NumPy — no Python loops in vertex generation.
    Face indices for a given resolution are cached via lru_cache.

    Args:
        center: (3,) sphere center
        radius: sphere radius
        resolution: number of latitude/longitude segments

    Returns:
        vertices: (N, 3) array
        faces: list of triangle index lists
    """
    lats = resolution
    lons = resolution

    # ── Vectorized vertex generation ──────────────────────────────────────────
    lat_angles = np.pi * np.arange(1, lats) / lats          # (lats-1,)
    lon_angles = 2.0 * np.pi * np.arange(lons) / lons       # (lons,)

    ring_r = radius * np.sin(lat_angles)                     # (lats-1,)
    z_vals = center[2] + radius * np.cos(lat_angles)        # (lats-1,)

    # Broadcast to (lats-1, lons) — single vectorized operation
    x_mid = center[0] + ring_r[:, None] * np.cos(lon_angles[None, :])
    y_mid = center[1] + ring_r[:, None] * np.sin(lon_angles[None, :])
    z_mid = np.broadcast_to(z_vals[:, None], x_mid.shape).copy()

    mid_verts = np.stack([x_mid.ravel(), y_mid.ravel(), z_mid.ravel()], axis=1)

    north = (center + np.array([0.0, 0.0, radius]))[None, :]
    south = (center + np.array([0.0, 0.0, -radius]))[None, :]
    vertices = np.vstack([north, mid_verts, south])

    # ── Cached face indices ────────────────────────────────────────────────────
    faces = _sphere_faces(resolution)
    return vertices, faces


@lru_cache(maxsize=8)
def _sphere_faces(resolution: int) -> List[List[int]]:
    """Pre-compute and cache face index lists for a sphere of given resolution."""
    lats = resolution
    lons = resolution
    south_idx = 1 + (lats - 1) * lons  # north(1) + mid rings + south

    faces = []

    # North pole fan
    for j in range(lons):
        faces.append([0, 1 + j, 1 + (j + 1) % lons])

    # Middle quads (two triangles each)
    for i in range(lats - 2):
        for j in range(lons):
            a = 1 + i * lons + j
            b = 1 + i * lons + (j + 1) % lons
            c = 1 + (i + 1) * lons + (j + 1) % lons
            d = 1 + (i + 1) * lons + j
            faces.append([a, b, c])
            faces.append([a, c, d])

    # South pole fan
    last_ring_start = 1 + (lats - 2) * lons
    for j in range(lons):
        a = last_ring_start + j
        b = last_ring_start + (j + 1) % lons
        faces.append([south_idx, a, b])

    return faces


def cuboid_mesh(center: np.ndarray, size: Tuple[float, float, float]) -> Tuple[np.ndarray, List[List[int]]]:
    """
    Generate vertices and faces for a cuboid (box) centered at given point.
    """
    dx, dy, dz = size[0] / 2, size[1] / 2, size[2] / 2
    cx, cy, cz = center[0], center[1], center[2]
    vertices = np.array([
        [cx - dx, cy - dy, cz - dz],
        [cx + dx, cy - dy, cz - dz],
        [cx + dx, cy + dy, cz - dz],
        [cx - dx, cy + dy, cz - dz],
        [cx - dx, cy - dy, cz + dz],
        [cx + dx, cy - dy, cz + dz],
        [cx + dx, cy + dy, cz + dz],
        [cx - dx, cy + dy, cz + dz],
    ])
    faces = [
        [0, 1, 2], [0, 2, 3],  # bottom
        [4, 5, 6], [4, 6, 7],  # top
        [0, 1, 5], [0, 5, 4],  # front
        [1, 2, 6], [1, 6, 5],  # right
        [2, 3, 7], [2, 7, 6],  # back
        [3, 0, 4], [3, 4, 7],  # left
    ]
    return vertices, faces
