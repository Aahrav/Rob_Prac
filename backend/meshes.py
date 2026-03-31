#!/usr/bin/env python3
"""
3D mesh generation for robotic arm visualization.
Creates cylinder and sphere meshes for Poly3DCollection.
"""

import numpy as np
from typing import Tuple, List


def cylinder_mesh(start: np.ndarray, end: np.ndarray, radius: float, resolution: int = 12) -> Tuple[np.ndarray, List[List[int]]]:
    """
    Generate vertices and faces for a cylinder between two 3D points.

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

    # Create a basis for the cylinder's circular cross-section
    # Find two perpendicular vectors to axis
    if abs(axis[2]) < 0.9:
        up = np.array([0, 0, 1])
    else:
        up = np.array([0, 1, 0])
    right = np.cross(axis, up)
    right = right / np.linalg.norm(right)
    forward = np.cross(axis, right)

    # Generate circle points at start and end
    angles = np.linspace(0, 2*np.pi, resolution, endpoint=False)
    circle = np.stack([np.cos(angles), np.sin(angles), np.zeros(resolution)], axis=1)  # (R, 3)

    # Vertices: we have resolution vertices on start ring, resolution on end ring
    vertices = np.zeros((2*resolution, 3))
    vertices[:resolution] = start + radius * (circle[:, 0:1] * right + circle[:, 1:2] * forward)
    vertices[resolution:] = end + radius * (circle[:, 0:1] * right + circle[:, 1:2] * forward)

    # Faces: side quads (two triangles each)
    faces = []
    for i in range(resolution):
        next_i = (i + 1) % resolution
        # two triangles per quad: (i, next_i, next_i + R) and (i, next_i + R, i + R)
        faces.append([i, next_i, next_i + resolution])
        faces.append([i, next_i + resolution, i + resolution])

    # Caps (optional, for closed cylinder)
    # Start cap
    faces.append(list(range(resolution)))
    # End cap (reversed winding for correct normal)
    faces.append([resolution + i for i in range(resolution)][::-1])

    return vertices, faces


def sphere_mesh(center: np.ndarray, radius: float, resolution: int = 8) -> Tuple[np.ndarray, List[List[int]]]:
    """
    Generate vertices and faces for a sphere.
    Uses latitude-longitude grid.

    Args:
        center: (3,) sphere center
        radius: sphere radius
        resolution: number of latitude segments

    Returns:
        vertices, faces
    """
    # Latitude (rings)
    lats = resolution
    # Longitude (slices)
    lons = resolution

    vertices = []
    faces = []

    # North pole
    vertices.append(center + np.array([0, 0, radius]))
    north_idx = 0

    # South pole
    south_idx = None

    # Create latitude rings
    for i in range(1, lats):
        lat_angle = np.pi * i / lats
        z = center[2] + radius * np.cos(lat_angle)
        ring_radius = radius * np.sin(lat_angle)
        for j in range(lons):
            lon_angle = 2 * np.pi * j / lons
            x = center[0] + ring_radius * np.cos(lon_angle)
            y = center[1] + ring_radius * np.sin(lon_angle)
            vertices.append(np.array([x, y, z]))
    south_idx = len(vertices) - 1
    vertices.append(center + np.array([0, 0, -radius]))
    south_idx_last = len(vertices) - 1

    # Build faces
    # North pole triangles
    for j in range(lons):
        next_j = (j + 1) % lons
        faces.append([north_idx, 1+j, 1+next_j])

    # Middle quads
    for i in range(lats - 2):
        for j in range(lons):
            next_j = (j + 1) % lons
            a = 1 + i*lons + j
            b = 1 + i*lons + next_j
            c = 1 + (i+1)*lons + next_j
            d = 1 + (i+1)*lons + j
            faces.append([a, b, c])
            faces.append([a, c, d])

    # South pole triangles
    last_ring_start = 1 + (lats - 2)*lons
    for j in range(lons):
        next_j = (j + 1) % lons
        a = last_ring_start + j
        b = last_ring_start + next_j
        faces.append([south_idx_last, a, b])

    return np.array(vertices), faces


def cuboid_mesh(center: np.ndarray, size: Tuple[float, float, float]) -> Tuple[np.ndarray, List[List[int]]]:
    """
    Generate vertices and faces for a cuboid (box) centered at given point.
    """
    dx, dy, dz = size[0]/2, size[1]/2, size[2]/2
    # 8 corners
    corners = np.array([
        [center[0]-dx, center[1]-dy, center[2]-dz],
        [center[0]+dx, center[1]-dy, center[2]-dz],
        [center[0]+dx, center[1]+dy, center[2]-dz],
        [center[0]-dx, center[1]+dy, center[2]-dz],
        [center[0]-dx, center[1]-dy, center[2]+dz],
        [center[0]+dx, center[1]-dy, center[2]+dz],
        [center[0]+dx, center[1]+dy, center[2]+dz],
        [center[0]-dx, center[1]+dy, center[2]+dz],
    ])
    vertices = corners
    # Faces defined by vertex indices (each quad -> two triangles)
    face_indices = [
        [0, 1, 2, 3],  # bottom
        [4, 5, 6, 7],  # top
        [0, 1, 5, 4],  # front
        [1, 2, 6, 5],  # right
        [2, 3, 7, 6],  # back
        [3, 0, 4, 7],  # left
    ]
    faces = []
    for quad in face_indices:
        faces.append([quad[0], quad[1], quad[2]])
        faces.append([quad[0], quad[2], quad[3]])

    return vertices, faces
