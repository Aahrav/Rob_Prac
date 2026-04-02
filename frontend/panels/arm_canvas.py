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
        self.fig = Figure(figsize=(9, 7), facecolor='#2d2d2d', dpi=100)
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

        # Obstacles
        self.obstacles = []
        self.obstacle_meshes = []

        # Workspace reference elements (ground + grid + boundary)
        self._ground_elements = []  # list of artists to toggle

        self._init_empty_plot()

    def _init_empty_plot(self):
        self.ax.cla()
        self._setup_axes()
        self._add_static_ground()
        self.draw()

    def _setup_axes(self):
        # Workspace bounds (meters)
        self.workspace_radius = 1.2  # XY range: ±1.2m
        self.workspace_z_max = 1.44   # Z max: 1.44m (aspect ~0.6)

        # Set fixed limits (shoulder at origin, works within these bounds)
        self.ax.set_xlim([-self.workspace_radius, self.workspace_radius])
        self.ax.set_ylim([-self.workspace_radius, self.workspace_radius])
        self.ax.set_zlim([0, self.workspace_z_max])

        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_zlabel('Z (m)')

        # Lock aspect ratio (prevents distortion)
        self.ax.set_box_aspect((1, 1, 0.6))

        # Preset views (elev, azim) for educational purposes
        self.views = {
            'front':   {'elev': 0,   'azim': 0},    # Look along -Y, X right, Z up
            'side':    {'elev': 0,   'azim': 90},   # Look along -X, Y right, Z up
            'top':     {'elev': 90,  'azim': 0},    # Top-down, all axes in plane
            'iso':     {'elev': 30,  'azim': -45},  # Classic 3/4 isometric
            'back':    {'elev': 0,   'azim': 180},  # Back view
        }
        self.default_view = self.views['iso']

        # Start with isometric view
        self.ax.view_init(**self.default_view)

        # Enable interactive navigation (mouse rotate, zoom, pan) — default

    def _add_static_ground(self):
        """Create a ground plane with grid and workspace boundary (static, added once)."""
        size = self.workspace_radius  # ground spans full XY workspace

        # Ground plane (very transparent)
        corners = np.array([
            [-size, -size, 0],
            [ size, -size, 0],
            [ size,  size, 0],
            [-size,  size, 0],
        ])
        faces = [[0, 1, 2], [0, 2, 3]]
        ground = Poly3DCollection([corners[face] for face in faces],
                                   facecolors='#3a3a3a', edgecolors='#444', linewidths=0.3, alpha=0.12)
        self.ax.add_collection3d(ground)
        self.meshes['ground'] = ground
        self._ground_elements.append(ground)

        # Grid lines every 0.25m (subtle)
        step = 0.25
        for x in np.arange(-size, size + step, step):
            line = self.ax.plot([x, x], [-size, size], [0, 0], color='#555', linewidth=0.3, alpha=0.2)[0]
            self._ground_elements.append(line)
        for y in np.arange(-size, size + step, step):
            line = self.ax.plot([-size, size], [y, y], [0, 0], color='#555', linewidth=0.3, alpha=0.2)[0]
            self._ground_elements.append(line)

        # Workspace boundary: faint circle at ground (max XY reach 0.6m)
        max_reach = 0.6
        theta = np.linspace(0, 2*np.pi, 64)
        x_circle = max_reach * np.cos(theta)
        y_circle = max_reach * np.sin(theta)
        z_circle = np.zeros_like(x_circle)
        circle_line = self.ax.plot(x_circle, y_circle, z_circle, color='#999', linewidth=0.8, alpha=0.25, linestyle='--')[0]
        self._ground_elements.append(circle_line)
        self._workspace_circle = circle_line  # store for later updates

    def _detect_collisions(self, positions):
        """
        Detect collisions.
        Returns a set of colliding segment names: 'base', 'upper', 'lower', 'gripper', 'shoulder', 'elbow', 'wrist', 'tip'
        """
        collisions = set()
        base_z = self.config.base_height
        point_names = ['shoulder', 'elbow', 'wrist', 'tip']
        # Map positions: positions = [base_pt, shoulder, elbow, wrist, tip] (length 5)
        # Ground collision
        for i, name in enumerate(point_names, start=1):
            if positions[i, 2] <= 0.05:
                collisions.add(name)
        # Segment collisions (based on endpoints)
        def endpoint_near_ground(p):
            return p[2] <= 0.05
        # Upper arm (shoulder->elbow)
        if endpoint_near_ground(positions[1]) or endpoint_near_ground(positions[2]):
            collisions.add('upper')
        # Lower arm (elbow->wrist)
        if endpoint_near_ground(positions[2]) or endpoint_near_ground(positions[3]):
            collisions.add('lower')
        # Gripper (wrist->tip)
        if endpoint_near_ground(positions[3]) or endpoint_near_ground(positions[4]):
            collisions.add('gripper')
        # Base intrusion (elbow or wrist entering base volume)
        base_half = 0.075
        for i in [2, 3]:
            p = positions[i]
            if abs(p[0]) <= base_half and abs(p[1]) <= base_half and p[2] < base_z + 0.05:
                collisions.add('base')
                break
        # Obstacle collisions: if any joint point lies inside an obstacle cuboid
        for obs in self.obstacles:
            c = obs['center']
            sx, sy, sz = obs['size']
            min_bounds = c - np.array([sx/2, sy/2, sz/2])
            max_bounds = c + np.array([sx/2, sy/2, sz/2])
            for i, name in enumerate(point_names, start=1):
                p = positions[i]
                if (min_bounds[0] <= p[0] <= max_bounds[0] and
                    min_bounds[1] <= p[1] <= max_bounds[1] and
                    min_bounds[2] <= p[2] <= max_bounds[2]):
                    collisions.add(name)
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
        # Don't change limits during animation - fixed workspace keeps aspect ratio stable
        # Only set once in _setup_axes
        pass

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

    def set_view(self, name: str = None, elev: float = None, azim: float = None):
        """Set camera view.
        Args:
            name: preset view name ('front', 'side', 'top', 'iso', 'back')
            elev, azim: explicit angles (overrides name if provided)
        """
        if name and name in self.views:
            self.ax.view_init(**self.views[name])
        elif elev is not None and azim is not None:
            self.ax.view_init(elev=elev, azim=azim)
        else:
            self.ax.view_init(**self.default_view)
        self.draw_idle()

    def reset_view(self):
        """Reset to default isometric view."""
        self.set_view(name='iso')

    def update_workspace_boundary(self, radius):
        """Update the dashed workspace boundary circle to reflect current max reach."""
        if hasattr(self, '_workspace_circle'):
            theta = np.linspace(0, 2*np.pi, 64)
            x = radius * np.cos(theta)
            y = radius * np.sin(theta)
            self._workspace_circle.set_data(x, y)
            self._workspace_circle.set_3d_properties(np.zeros_like(x))
            self.draw_idle()

    def toggle_ground(self, visible: bool):
        """Show or hide ground, grid, and workspace boundary."""
        for artist in self._ground_elements:
            artist.set_visible(visible)
        self.draw_idle()

    def add_obstacle(self, center, size, color='#c0392b'):
        """Add a static cuboid obstacle to the scene."""
        center_arr = np.array(center)
        verts, faces = cuboid_mesh(center_arr, size)
        coll = Poly3DCollection([verts[face] for face in faces], facecolors=color, edgecolors='#333', linewidths=0.5, alpha=0.7)
        self.ax.add_collection3d(coll)
        self.obstacles.append({'center': center_arr, 'size': size})
        self.obstacle_meshes.append(coll)
        self.draw_idle()
