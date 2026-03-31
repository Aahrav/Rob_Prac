#!/usr/bin/env python3
"""
ArmCanvas - 3D Robotic Arm visualization using Matplotlib with realistic cylinders.
"""

from PyQt6.QtWidgets import QVBoxLayout, QWidget
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
from backend.meshes import cylinder_mesh, sphere_mesh, cuboid_mesh
from backend.kinematics import ArmConfig


class ArmCanvas(FigureCanvas):
    """Matplotlib 3D canvas for rendering the robotic arm with 3D meshes."""

    def __init__(self, parent=None):
        self.fig = Figure(figsize=(8, 6), facecolor='#2d2d2d')
        super().__init__(self.fig)
        self.setParent(parent)

        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_facecolor('#2d2d2d')
        self.ax.grid(True, color='#444')

        # Styling
        self.ax.xaxis.label.set_color('#aaa')
        self.ax.yaxis.label.set_color('#aaa')
        self.ax.zaxis.label.set_color('#aaa')
        self.ax.tick_params(colors='#aaa')

        # Arm segment colors
        self.config = ArmConfig()
        self.colors = {
            'base': '#666666',
            'shoulder': '#3498db',
            'elbow': '#e67e22',
            'wrist': '#2ecc71',
            'gripper': '#e74c3c',
            'cylinders': {
                'upper': '#95a5a6',
                'lower': '#7f8c8d',
                'gripper': '#e74c3c',
            }
        }

        # Store mesh collections for updating
        self.mesh_collections = []
        self.joint_spheres = []
        self.base_mesh = None

        self._init_empty_plot()

    def _init_empty_plot(self):
        self.ax.cla()
        self._setup_axes()
        self.draw()

    def _setup_axes(self):
        self.ax.set_xlim([-1, 1])
        self.ax.set_ylim([-1, 1])
        self.ax.set_zlim([0, 2])
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_zlabel('Z (m)')

    def draw_arm(self, positions):
        """
        Draw the arm with 3D meshes.
        positions: (6, 3) numpy array of joint coordinates.
        """
        # Remove previous meshes
        for coll in self.mesh_collections:
            coll.remove()
        self.mesh_collections.clear()
        for sphere in self.joint_spheres:
            sphere.remove()
        self.joint_spheres.clear()
        if self.base_mesh:
            self.base_mesh.remove()
            self.base_mesh = None

        # Extract points
        base_pt = positions[0]
        shoulder_pt = positions[1]
        elbow_pt = positions[2]
        wrist_pt = positions[3]
        tip_pt = positions[4]

        # Draw base cuboid
        base_size = (0.15, 0.15, self.config.base_height)
        base_vertices, base_faces = cuboid_mesh(np.array([0,0, self.config.base_height/2]), base_size)
        self.base_mesh = Poly3DCollection([base_vertices[face] for face in base_faces], facecolors=self.colors['base'], edgecolors='#444', linewidths=0.5)
        self.ax.add_collection3d(self.base_mesh)

        # Draw upper arm cylinder (shoulder -> elbow)
        upper_radius = 0.02
        upper_verts, upper_faces = cylinder_mesh(shoulder_pt, elbow_pt, upper_radius)
        upper_coll = Poly3DCollection([upper_verts[face] for face in upper_faces],
                                      facecolors=self.colors['cylinders']['upper'],
                                      edgecolors='#444', linewidths=0.5)
        self.ax.add_collection3d(upper_coll)
        self.mesh_collections.append(upper_coll)

        # Draw lower arm cylinder (elbow -> wrist)
        lower_radius = 0.015
        lower_verts, lower_faces = cylinder_mesh(elbow_pt, wrist_pt, lower_radius)
        lower_coll = Poly3DCollection([lower_verts[face] for face in lower_faces],
                                      facecolors=self.colors['cylinders']['lower'],
                                      edgecolors='#444', linewidths=0.5)
        self.ax.add_collection3d(lower_coll)
        self.mesh_collections.append(lower_coll)

        # Draw gripper cylinder (wrist -> tip)
        gripper_radius = 0.01
        gripper_verts, gripper_faces = cylinder_mesh(wrist_pt, tip_pt, gripper_radius)
        gripper_coll = Poly3DCollection([gripper_verts[face] for face in gripper_faces],
                                        facecolors=self.colors['cylinders']['gripper'],
                                        edgecolors='#444', linewidths=0.5)
        self.ax.add_collection3d(gripper_coll)
        self.mesh_collections.append(gripper_coll)

        # Draw joints as spheres
        joint_radius = 0.02
        for idx, (pt, color_key) in enumerate([
            (shoulder_pt, 'shoulder'),
            (elbow_pt, 'elbow'),
            (wrist_pt, 'wrist')
        ]):
            j_verts, j_faces = sphere_mesh(pt, joint_radius)
            face_arrays = [j_verts[face] for face in j_faces]
            joint_coll = Poly3DCollection(face_arrays, facecolors=self.colors[color_key], edgecolors='#333', linewidths=0.3)
            self.ax.add_collection3d(joint_coll)
            self.joint_spheres.append(joint_coll)

        # Draw gripper tip as small sphere
        tip_radius = 0.015
        tip_verts, tip_faces = sphere_mesh(tip_pt, tip_radius)
        tip_face_arrays = [tip_verts[face] for face in tip_faces]
        tip_coll = Poly3DCollection(tip_face_arrays, facecolors=self.colors['gripper'], edgecolors='#333', linewidths=0.3)
        self.ax.add_collection3d(tip_coll)
        self.joint_spheres.append(tip_coll)

        # Adjust axis limits to fit arm
        self._adjust_limits(positions)
        self.draw_idle()

    def _adjust_limits(self, positions):
        xs = positions[:, 0]
        ys = positions[:, 1]
        zs = positions[:, 2]
        xs_all = np.concatenate([xs, [0]])
        ys_all = np.concatenate([ys, [0]])
        zs_all = np.concatenate([zs, [0]])

        max_range = np.array([xs_all.max()-xs_all.min(), ys_all.max()-ys_all.min(), zs_all.max()-zs_all.min()]).max() / 2.0
        if max_range < 0.1:
            max_range = 0.5
        mid_x = (xs_all.max()+xs_all.min()) * 0.5
        mid_y = (ys_all.max()+ys_all.min()) * 0.5
        mid_z = (zs_all.max()+zs_all.min()) * 0.5
        range_pad = max_range * 1.3
        self.ax.set_xlim(mid_x - range_pad, mid_x + range_pad)
        self.ax.set_ylim(mid_y - range_pad, mid_y + range_pad)
        self.ax.set_zlim(max(0, mid_z - range_pad), mid_z + range_pad)

    def set_trajectory(self, points):
        """Set trajectory trace (simple line, not mesh)."""
        self.trajectory_points = points if points else []
        if hasattr(self, 'joint_positions') and self.joint_positions is not None:
            self.draw_arm(self.joint_positions)
