#!/usr/bin/env python3
"""
ArmCanvas - 3D Robotic Arm visualization using Matplotlib.
Simple line-based rendering (not realistic meshes) for stability.
"""

from PyQt6.QtWidgets import QVBoxLayout, QWidget
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import numpy as np


class ArmCanvas(FigureCanvas):
    """Matplotlib 3D canvas for rendering the robotic arm."""

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

        # Arm segment styling
        self.colors = {
            'base': '#666666',
            'shoulder': '#3498db',
            'elbow': '#e67e22',
            'wrist': '#2ecc71',
            'gripper': '#e74c3c',
            'links': '#ffffff'
        }

        # Joint positions (6 points)
        self.joint_positions = np.zeros((6, 3))

        # Trajectory trace
        self.trajectory_points = []

        self._init_empty_plot()

    def _init_empty_plot(self):
        """Initialize with empty/invalid arm."""
        self.ax.cla()
        self.ax.set_xlim([-1, 1])
        self.ax.set_ylim([-1, 1])
        self.ax.set_zlim([0, 2])
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_zlabel('Z (m)')
        self.fig.tight_layout()
        self.draw()

    def draw_arm(self, positions):
        """
        Draw the arm given 6 joint positions.
        positions: (6, 3) numpy array of (x, y, z) coordinates.
        """
        self.joint_positions = positions.copy()
        self.ax.cla()

        # Draw trajectory trace if available
        if self.trajectory_points:
            traj_arr = np.array(self.trajectory_points)
            self.ax.plot(traj_arr[:, 0], traj_arr[:, 1], traj_arr[:, 2],
                         '-', linewidth=2, color='#f39c12', alpha=0.7)

        xs = positions[:, 0]
        ys = positions[:, 1]
        zs = positions[:, 2]

        # Draw arm segments as connected lines
        self.ax.plot(xs, ys, zs, 'o-', linewidth=4, markersize=8,
                     color=self.colors['links'], markerfacecolor='#fff', markeredgecolor='#333')

        # Draw joints with distinct colors
        joint_colors = [self.colors['base'], self.colors['shoulder'],
                        self.colors['elbow'], self.colors['wrist'],
                        self.colors['gripper'], self.colors['gripper']]
        for i, (x, y, z) in enumerate(positions):
            self.ax.scatter([x], [y], [z], s=120, c=joint_colors[i],
                            edgecolors='#fff', linewidth=1.5, depthshade=False)

        # Set axis limits based on arm extents with padding
        xs_all = xs
        ys_all = ys
        zs_all = zs
        if self.trajectory_points:
            traj_arr = np.array(self.trajectory_points)
            xs_all = np.concatenate([xs_all, traj_arr[:, 0]])
            ys_all = np.concatenate([ys_all, traj_arr[:, 1]])
            zs_all = np.concatenate([zs_all, traj_arr[:, 2]])

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

        self.fig.tight_layout()
        self.draw_idle()

    def set_trajectory(self, points):
        """Set trajectory points to display."""
        self.trajectory_points = points if points else []
        if hasattr(self, 'joint_positions') and self.joint_positions is not None:
            self.draw_arm(self.joint_positions)
