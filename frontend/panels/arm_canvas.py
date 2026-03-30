#!/usr/bin/env python3
"""
ArmCanvas - 3D Robotic Arm visualization using Matplotlib.
Step 4: Static arm with simple lines and joint spheres.
Step 5: Live updates.
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

        # Joint positions (6 points: base, shoulder, elbow, wrist, gripper tip, extra)
        self.joint_positions = np.zeros((6, 3))

        # Initialize plot
        self._init_empty_plot()

    def _init_empty_plot(self):
        """Initialize with empty/invalid arm."""
        self.ax.cla()
        self.ax.set_xlim([-1, 1])
        self.ax.set_ylim([-1, 1])
        self.ax.set_zlim([0, 2])
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.fig.tight_layout()
        self.draw()

    def draw_arm(self, positions):
        """
        Draw the arm given 6 joint positions.
        positions: (6, 3) numpy array of (x, y, z) coordinates.
        """
        self.joint_positions = positions

        self.ax.cla()

        # Draw arm segments (lines connecting consecutive joints)
        xs = positions[:, 0]
        ys = positions[:, 1]
        zs = positions[:, 2]

        # Draw connecting lines
        self.ax.plot(xs, ys, zs, 'o-', linewidth=4, markersize=8,
                     color=self.colors['links'], markerfacecolor='#fff')

        # Draw joints with distinct colors
        joint_colors = [self.colors['base'], self.colors['shoulder'],
                        self.colors['elbow'], self.colors['wrist'],
                        self.colors['gripper'], self.colors['gripper']]
        for i, (x, y, z) in enumerate(positions):
            self.ax.scatter([x], [y], [z], s=120, c=joint_colors[i], edgecolors='#fff', linewidth=1.5)

        # Set axis limits based on arm extents with padding
        max_range = np.array([xs.max()-xs.min(), ys.max()-ys.min(), zs.max()-zs.min()]).max() / 2.0
        if max_range < 0.1:
            max_range = 0.5
        mid_x = (xs.max()+xs.min()) * 0.5
        mid_y = (ys.max()+ys.min()) * 0.5
        mid_z = (zs.max()+zs.min()) * 0.5
        range_pad = max_range * 1.3
        self.ax.set_xlim(mid_x - range_pad, mid_x + range_pad)
        self.ax.set_ylim(mid_y - range_pad, mid_y + range_pad)
        self.ax.set_zlim(max(0, mid_z - range_pad), mid_z + range_pad)

        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_zlabel('Z (m)')

        self.fig.tight_layout()
        self.draw_idle()
