#!/usr/bin/env python3
"""
ArmCanvas - 3D Robotic Arm visualization with realistic meshes, ground plane, and collision detection.
"""

from PyQt6.QtWidgets import QVBoxLayout, QWidget
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
from backend.meshes import cylinder_mesh, sphere_mesh, cuboid_mesh
from backend.kinematics import ArmConfig


class ArmCanvas(FigureCanvas):
    """Matplotlib 3D canvas for rendering the robotic arm with 3D meshes and collision detection."""

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

        self.config = ArmConfig()

        # Colors
        self.default_colors = {
            'base': '#666666',
            'upper': '#95a5a6',
            'lower': '#7f8c8d',
            'gripper': '#e74c3c',
            'joint_shoulder': '#3498db',
            'joint_elbow': '#e67e22',
            'joint_wrist': '#2ecc71',
            'joint_tip': '#e74c3c',
        }
        self.collision_color = '#ff0000'  # bright red for colliding parts

        # Mesh collections (cleared and rebuilt each frame)
        self.meshes = {
            'base': None,
            'upper': None,
            'lower': None,
            'gripper': None,
            'joints': [],
            'ground': None,
        }

        # Collision state
        self.colliding_segments = set()

        # Trajectory trace
        self.trajectory_points = []
        self._traj_line = None

        self._init_empty_plot()

    def _init_empty_plot(self):
        self.ax.cla()
        self._setup_axes()
        self._add_static_ground()
        self.draw()

    def _setup_axes(self):
        self.ax.set_xlim([-1, 1])
        self.ax.set_ylim([-1, 1])
        self.ax.set_zlim([0, 2])
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_zlabel('Z (m)')

    def _add_static_ground(self):
        """Create a ground plane with grid (static, added once)."""
        size = 2.0
        # Ground plane
        corners = np.array([
            [-size, -size, 0],
            [ size, -size, 0],
            [ size,  size, 0],
            [-size,  size, 0],
        ])
        faces = [[0, 1, 2], [0, 2, 3]]
        ground = Poly3DCollection([corners[face] for face in faces], facecolors='#444', edgecolors='#555', linewidths=0.5, alpha=0.3)
        self.ax.add_collection3d(ground)
        self.meshes['ground'] = ground
        # Grid lines
        step = 0.5
        # X grid lines (vary Y)
        for x in np.arange(-size, size + step, step):
            self.ax.plot([x, x], [-size, size], [0, 0], color='#555', linewidth=0.5, alpha=0.5)
        # Y grid lines (vary X)
        for y in np.arange(-size, size + step, step):
            self.ax.plot([-size, size], [y, y], [0, 0], color='#555', linewidth=0.5, alpha=0.5)

    def _detect_collisions(self, positions):
        """
        Detect collisions.
        Returns a set of colliding segment names: 'base', 'upper', 'lower', 'gripper', 'joints'
        """
        collisions = set()
        # positions indices: 0=base (should be at height), 1=shoulder, 2=elbow, 3=wrist, 4=tip
        base_z = self.config.base_height
        # Ground collision: if any point (except base origin) goes near or below ground
        for i, name in [(1, 'shoulder'), (2, 'elbow'), (3, 'wrist'), (4, 'tip')]:
            if positions[i, 2] <= 0.05:  # 5cm above ground
                collisions.add(name)
        # Segment collisions: if a segment intersects the ground or base volume
        # Simple: if both endpoints of a segment are near ground, mark the segment
        def endpoint_collision(p):
            return p[2] <= 0.05
        # Upper arm: shoulder->elbow
        if endpoint_collision(positions[1]) or endpoint_collision(positions[2]):
            collisions.add('upper')
        # Lower arm: elbow->wrist
        if endpoint_collision(positions[2]) or endpoint_collision(positions[3]):
            collisions.add('lower')
        # Gripper: wrist->tip
        if endpoint_collision(positions[3]) or endpoint_collision(positions[4]):
            collisions.add('gripper')
        # Base collision: if elbow or lower arm goes into the base cuboid volume (x,y within 0.15/2, z < base_height)
        base_half = 0.075
        for i in [2, 3]:  # elbow, wrist
            p = positions[i]
            if abs(p[0]) <= base_half and abs(p[1]) <= base_half and p[2] < base_z + 0.05:
                collisions.add('base')
                break
        self.colliding_segments = collisions
        return collisions

    def draw_arm(self, positions):
        """
        Draw the arm with 3D meshes.
        positions: (5, 3) numpy array of joint coordinates: [shoulder, elbow, wrist, tip, ?]
        We expect positions from compute_arm_positions with 6 points? We'll adapt to use first 5.
        """
        # Clear previous arm meshes (ground stays)
        for key in ['base', 'upper', 'lower', 'gripper']:
            if self.meshes[key]:
                self.meshes[key].remove()
                self.meshes[key] = None
        for coll in self.meshes['joints']:
            coll.remove()
        self.meshes['joints'].clear()

        # Ensure positions shape
        if positions.shape[0] >= 5:
            pts = positions[:5]  # 0:base?, actually compute_arm_positions returns 6 points; we want index 1..4? Let's define:
            # Our expected ordering: 0: base (at shoulder base), 1: shoulder, 2: elbow, 3: wrist, 4: tip
            # But compute_arm_positions returns: 0: base (0,0,0), 1: shoulder, 2: elbow, 3: wrist, 4: tip, 5: duplicate tip.
            # We'll use positions[1:5] as the actual arm joints, and base as separate.
            # However we also need the shoulder point which is at (0,0,base_height). The base of arm is at (0,0,base_height).
            shoulder_pt = positions[1] if positions.shape[0] >= 5 else positions[0]
            elbow_pt = positions[2] if positions.shape[0] >= 5 else positions[1]
            wrist_pt = positions[3] if positions.shape[0] >= 5 else positions[2]
            tip_pt = positions[4] if positions.shape[0] >= 5 else positions[3]
            base_pt = np.array([0.0, 0.0, self.config.base_height])
        else:
            # Not enough points, cannot draw
            return

        # Detect collisions
        coll = self._detect_collisions(np.array([base_pt, shoulder_pt, elbow_pt, wrist_pt, tip_pt]))

        # Draw base cuboid (static, centered at base_pt but base_pt is at shoulder level? Actually base cuboid sits on ground up to base_height)
        base_size = (0.15, 0.15, self.config.base_height)
        base_center = np.array([0.0, 0.0, self.config.base_height/2])
        base_verts, base_faces = cuboid_mesh(base_center, base_size)
        base_color = self.collision_color if 'base' in coll else self.default_colors['base']
        base_coll = Poly3DCollection([base_verts[face] for face in base_faces], facecolors=base_color, edgecolors='#444', linewidths=0.5)
        self.ax.add_collection3d(base_coll)
        self.meshes['base'] = base_coll

        # Draw upper arm cylinder (shoulder -> elbow)
        upper_radius = 0.025
        upper_verts, upper_faces = cylinder_mesh(shoulder_pt, elbow_pt, upper_radius, resolution=12)
        upper_color = self.collision_color if 'upper' in coll else self.default_colors['upper']
        upper_coll = Poly3DCollection([upper_verts[face] for face in upper_faces], facecolors=upper_color, edgecolors='#444', linewidths=0.5)
        self.ax.add_collection3d(upper_coll)
        self.meshes['upper'] = upper_coll

        # Draw lower arm cylinder (elbow -> wrist)
        lower_radius = 0.02
        lower_verts, lower_faces = cylinder_mesh(elbow_pt, wrist_pt, lower_radius, resolution=10)
        lower_color = self.collision_color if 'lower' in coll else self.default_colors['lower']
        lower_coll = Poly3DCollection([lower_verts[face] for face in lower_faces], facecolors=lower_color, edgecolors='#444', linewidths=0.5)
        self.ax.add_collection3d(lower_coll)
        self.meshes['lower'] = lower_coll

        # Draw gripper cylinder (wrist -> tip)
        gripper_radius = 0.015
        gripper_verts, gripper_faces = cylinder_mesh(wrist_pt, tip_pt, gripper_radius, resolution=8)
        gripper_color = self.collision_color if 'gripper' in coll else self.default_colors['gripper']
        gripper_coll = Poly3DCollection([gripper_verts[face] for face in gripper_faces], facecolors=gripper_color, edgecolors='#444', linewidths=0.5)
        self.ax.add_collection3d(gripper_coll)
        self.meshes['gripper'] = gripper_coll

        # Draw joint spheres (shoulder, elbow, wrist)
        joint_radius = 0.03
        for idx, (pt, color_key) in enumerate([
            (shoulder_pt, 'joint_shoulder'),
            (elbow_pt, 'joint_elbow'),
            (wrist_pt, 'joint_wrist')
        ]):
            j_verts, j_faces = sphere_mesh(pt, joint_radius, resolution=6)
            face_arrays = [j_verts[face] for face in j_faces]
            joint_color = self.collision_color if ['shoulder','elbow','wrist'][idx] in coll else self.default_colors[color_key]
            joint_coll = Poly3DCollection(face_arrays, facecolors=joint_color, edgecolors='#333', linewidths=0.3)
            self.ax.add_collection3d(joint_coll)
            self.meshes['joints'].append(joint_coll)

        # Draw gripper tip sphere (small)
        tip_radius = 0.02
        tip_verts, tip_faces = sphere_mesh(tip_pt, tip_radius, resolution=6)
        tip_face_arrays = [tip_verts[face] for face in tip_faces]
        tip_color = self.collision_color if 'tip' in coll else self.default_colors['joint_tip']
        tip_coll = Poly3DCollection(tip_face_arrays, facecolors=tip_color, edgecolors='#333', linewidths=0.3)
        self.ax.add_collection3d(tip_coll)
        self.meshes['joints'].append(tip_coll)

        # Adjust axis limits to fit arm and ground
        self._adjust_limits(np.array([base_pt, shoulder_pt, elbow_pt, wrist_pt, tip_pt]))
        self.draw_idle()

    def _adjust_limits(self, positions):
        xs = positions[:, 0]
        ys = positions[:, 1]
        zs = positions[:, 2]
        xs_all = np.concatenate([xs, [-1, 1]])  # include ground extents
        ys_all = np.concatenate([ys, [-1, 1]])
        zs_all = np.concatenate([zs, [0, 2]])

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
        """Set trajectory trace (simple line)."""
        self.trajectory_points = points if points else []
        # we would redraw arm; caller will do draw_arm anyway
        # but we could draw trace separately
        if hasattr(self, '_traj_line') and self._traj_line is not None:
            self._traj_line.remove()
        if self.trajectory_points:
            pts = np.array(self.trajectory_points)
            self._traj_line = self.ax.plot(pts[:,0], pts[:,1], pts[:,2], 'r-', linewidth=1, alpha=0.7)[0]
        else:
            self._traj_line = None
        self.draw_idle()
