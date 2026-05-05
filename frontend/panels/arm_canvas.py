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
            'custom_links': [],  # for generic DH chain cylinders
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

        # ── Idle overlay (P3-T4) ───────────────────────────────────────────
        # Rendered at figure-level coordinates so it floats above the 3D axes
        # regardless of viewport rotation.  MainWindow calls set_idle_message();
        # draw_arm / draw_chain also hide it automatically as a safety net.
        self._idle_visible: bool = False   # will be set True by _init_empty_plot
        self._idle_overlay = self.fig.text(
            0.5, 0.5,
            "No data\nChoose  Simulate  or  Replay",
            transform=self.fig.transFigure,
            ha='center', va='center',
            fontsize=16,
            fontfamily='monospace',
            color='#e5e2e1',
            alpha=0.0,          # hidden until _init_empty_plot enables it
            zorder=10,
            bbox=dict(
                boxstyle='round,pad=0.8',
                facecolor='#131313',
                edgecolor='#3498db',
                alpha=0.0,      # also hidden; set together via set_idle_message
                linewidth=1.5,
            ),
        )

        self._init_empty_plot()

    def _init_empty_plot(self):
        self.ax.cla()
        # Reset all mesh references to avoid stale artists after clear
        self.meshes = {
            'base': None,
            'base_plate': None, # new mounting plate mesh
            'upper': None,
            'lower': None,
            'gripper': None,
            'joints': [],
            'ground': None,
            'custom_links': [],
        }
        self.obstacle_meshes = []
        self.obstacles = []
        self.trajectory_points = []
        self._traj_line = None
        self.colliding_segments = set()
        self._ground_elements = []  # will be repopulated by _add_static_ground
        self._setup_axes()
        self._add_static_ground()
        # Show the idle overlay whenever the canvas is reset (disconnect / startup)
        self.set_idle_message(True)
        self.draw()

    def _setup_axes(self):
        # Initial tight bounds (will be updated by presets)
        self.workspace_radius = 0.5
        self.workspace_z_max = 0.6
        
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

        # Dynamic grid step based on size
        if size <= 0.35:
            step = 0.05
        elif size <= 0.7:
            step = 0.1
        else:
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

    # ── Idle overlay public API (P3-T4) ─────────────────────────────────────

    def set_idle_message(self, visible: bool) -> None:
        """Show or hide the 'No data' idle overlay.

        Called by MainWindow:
          • set_idle_message(True)  — on disconnect / stop / startup
          • set_idle_message(False) — when the first valid sample arrives

        draw_arm() and draw_chain() also call set_idle_message(False)
        automatically so callers need not remember to do so.
        """
        if visible == self._idle_visible:
            return
        self._idle_visible = visible
        alpha_text = 0.92 if visible else 0.0
        alpha_box  = 0.88 if visible else 0.0
        self._idle_overlay.set_alpha(alpha_text)
        self._idle_overlay.get_bbox_patch().set_alpha(alpha_box)
        self.draw_idle()

    # ────────────────────────────────────────────────────────────────────────

    def draw_arm(self, positions):
        """
        Draw the arm with 3D meshes.
        positions: (5, 3) numpy array of joint coordinates: [shoulder, elbow, wrist, tip, ?]
        We expect positions from compute_arm_positions with 6 points? We'll adapt to use first 5.
        """
        # Hide idle overlay — data has arrived
        self.set_idle_message(False)

        # We no longer clear everything every frame.
        # Instead, we'll update vertices of existing collections.
        pass

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

        # Proportional scaling based on arm dimensions
        # We use upper_arm_length as the primary scale reference
        scale = max(0.1, self.config.upper_arm_length)
        base_w = max(0.06, scale * 0.4)
        joint_r_large = max(0.015, scale * 0.1)
        joint_r_small = joint_r_large * 0.7
        link_r_upper = joint_r_large * 0.8
        link_r_lower = joint_r_large * 0.6
        link_r_gripper = joint_r_large * 0.4

        # Proportional scaling with tapering
        scale = max(0.1, self.config.upper_arm_length)
        base_w = max(0.06, scale * 0.4)
        base_plate_w = base_w * 1.5
        
        # Tapered radii (Shoulder > Elbow > Wrist > Tip)
        r_shoulder = max(0.015, scale * 0.12)
        r_elbow    = r_shoulder * 0.85
        r_wrist    = r_shoulder * 0.70
        r_tip      = r_shoulder * 0.50
        
        link_r_upper   = r_shoulder * 0.8
        link_r_lower   = r_elbow * 0.8
        link_r_gripper = r_wrist * 0.8

        # Draw / Update base plate (mounting disk)
        plate_h = 0.01
        p_start = np.array([0, 0, 0])
        p_end = np.array([0, 0, plate_h])
        p_verts, p_faces = cylinder_mesh(p_start, p_end, base_plate_w/2, resolution=24)
        self._update_mesh('base_plate', p_verts, p_faces, '#444')

        # Draw / Update base cylinder (pedestal)
        base_h = self.config.base_height
        b_start = np.array([0, 0, plate_h])
        b_end = np.array([0, 0, base_h])
        base_verts, base_faces = cylinder_mesh(b_start, b_end, base_w/2, resolution=24)
        base_color = self.collision_color if 'base' in coll else self.default_colors['base']
        self._update_mesh('base', base_verts, base_faces, base_color)
        
        # Draw / Update upper arm cylinder (shoulder -> elbow)
        upper_verts, upper_faces = cylinder_mesh(shoulder_pt, elbow_pt, link_r_upper, resolution=20)
        upper_color = self.collision_color if 'upper' in coll else self.default_colors['upper']
        self._update_mesh('upper', upper_verts, upper_faces, upper_color)

        # Draw / Update lower arm cylinder (elbow -> wrist)
        lower_verts, lower_faces = cylinder_mesh(elbow_pt, wrist_pt, link_r_lower, resolution=16)
        lower_color = self.collision_color if 'lower' in coll else self.default_colors['lower']
        self._update_mesh('lower', lower_verts, lower_faces, lower_color)

        # Draw / Update gripper cylinder (wrist -> tip)
        gripper_verts, gripper_faces = cylinder_mesh(wrist_pt, tip_pt, link_r_gripper, resolution=12)
        gripper_color = self.collision_color if 'gripper' in coll else self.default_colors['gripper']
        self._update_mesh('gripper', gripper_verts, gripper_faces, gripper_color)

        # Draw / Update joint spheres
        joint_data = [
            (shoulder_pt, r_shoulder, 'shoulder', 'joint_shoulder'),
            (elbow_pt,    r_elbow,    'elbow',    'joint_elbow'),
            (wrist_pt,    r_wrist,    'wrist',    'joint_wrist'),
            (tip_pt,      r_tip,      'tip',      'joint_tip'),
        ]
        
        # Ensure joints list has enough collections
        while len(self.meshes['joints']) < len(joint_data):
            c = Poly3DCollection([], alpha=1.0)
            self.ax.add_collection3d(c)
            self.meshes['joints'].append(c)
        while len(self.meshes['joints']) > len(joint_data):
            c = self.meshes['joints'].pop()
            c.remove()

        for i, (pt, radius, coll_name, color_key) in enumerate(joint_data):
            j_verts, j_faces = sphere_mesh(pt, radius, resolution=16)
            j_color = self.collision_color if coll_name in coll else self.default_colors[color_key]
            self.meshes['joints'][i].set_verts([j_verts[f] for f in j_faces])
            self.meshes['joints'][i].set_facecolor(j_color)
            self.meshes['joints'][i].set_edgecolor('#333')
            self.meshes['joints'][i].set_linewidth(0.3)
            self.meshes['joints'][i].set_visible(True)

        # Adjust axis limits to fit arm and ground
        self._adjust_limits(np.array([base_pt, shoulder_pt, elbow_pt, wrist_pt, tip_pt]))
        self.draw_idle()

    def _update_mesh(self, key: str, verts: np.ndarray, faces: list, color: str):
        """Update existing mesh collection or create new one."""
        poly_verts = [verts[f] for f in faces]
        if self.meshes[key] is None:
            coll = Poly3DCollection(poly_verts, facecolors=color, edgecolors='#444', linewidths=0.5)
            self.ax.add_collection3d(coll)
            self.meshes[key] = coll
        else:
            self.meshes[key].set_verts(poly_verts)
            self.meshes[key].set_facecolor(color)
            self.meshes[key].set_visible(True)

    def _adjust_limits(self, positions):
        """Automatically adjust limits if robot moves outside current view or is too small."""
        # Calculate current bounding box of the arm
        max_h = np.max(np.linalg.norm(positions[:, :2], axis=1))
        max_v = np.max(positions[:, 2])
        
        # If the robot is significantly different from current limits, reframing is needed
        # We use a 20% margin to avoid jitter
        if max_h > self.workspace_radius or max_h < self.workspace_radius * 0.5:
             self.frame_to_fit_robot(max_h, max_v)

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

    def update_ground(self, radius: float):
        """Rebuild ground plane, grid, and workspace circle to match new radius."""
        # Remove existing ground elements
        for artist in self._ground_elements:
            artist.remove()
        self._ground_elements.clear()
        self.workspace_radius = radius
        # Ground plane
        corners = np.array([
            [-radius, -radius, 0],
            [ radius, -radius, 0],
            [ radius,  radius, 0],
            [-radius,  radius, 0],
        ])
        faces = [[0,1,2],[0,2,3]]
        ground = Poly3DCollection([corners[face] for face in faces],
                                   facecolors='#3a3a3a', edgecolors='#444', linewidths=0.3, alpha=0.12)
        self.ax.add_collection3d(ground)
        self._ground_elements.append(ground)
        # Dynamic grid step
        if radius <= 0.35:
            step = 0.05
        elif radius <= 0.7:
            step = 0.1
        else:
            step = 0.25

        for x in np.arange(-radius, radius + step, step):
            line = self.ax.plot([x, x], [-radius, radius], [0,0], color='#555', linewidth=0.3, alpha=0.2)[0]
            self._ground_elements.append(line)
        for y in np.arange(-radius, radius + step, step):
            line = self.ax.plot([-radius, radius], [y, y], [0,0], color='#555', linewidth=0.3, alpha=0.2)[0]
            self._ground_elements.append(line)
        # Workspace boundary circle
        theta = np.linspace(0, 2*np.pi, 64)
        x_circle = radius * np.cos(theta)
        y_circle = radius * np.sin(theta)
        z_circle = np.zeros_like(x_circle)
        circle_line = self.ax.plot(x_circle, y_circle, z_circle, color='#999', linewidth=0.8, alpha=0.25, linestyle='--')[0]
        self._ground_elements.append(circle_line)
        self._workspace_circle = circle_line
        self.draw_idle()

    def frame_to_fit_robot(self, max_horizontal: float, max_vertical: float, base_height: float = 0.0, padding: float = 0.25):
        """Adjust camera and axis limits to fit a robot with given maximum reach extents."""
        if max_horizontal < 0.1: max_horizontal = 0.1
        if max_vertical < 0.1: max_vertical = 0.1
        
        # New radius and Z height with padding
        new_radius = max_horizontal * (1 + padding)
        new_z = max_vertical * (1 + padding)
        
        # Update internal state
        self.workspace_radius = new_radius
        self.workspace_z_max = new_z
        
        # Set limits
        self.ax.set_xlim([-new_radius, new_radius])
        self.ax.set_ylim([-new_radius, new_radius])
        self.ax.set_zlim([0.0, new_z])
        
        # Update ground and grid to new radius
        self.update_ground(new_radius)
        
        # Lock aspect ratio (keep X and Y same, Z scaled)
        # For small robots, we want a taller-looking box for better perspective
        self.ax.set_box_aspect((1, 1, 0.7))
        
        # Move camera closer (dist default is 10, smaller is closer)
        self.ax.dist = 8.5
        self.draw_idle()

    def draw_chain(self, positions, base_height=None):
        """
        Draw a generic kinematic chain from DH parameters.
        positions: (N+1, 3) numpy array where positions[0] is base origin and positions[-1] is end-effector.
        base_height: optional height of base from ground; if None, uses positions[0,2].
        """
        # Hide idle overlay — data has arrived
        self.set_idle_message(False)

        # Hide standard meshes if they are visible
        for key in ['base', 'upper', 'lower', 'gripper']:
            if self.meshes[key]:
                self.meshes[key].set_visible(False)
        # Clear custom link collections as they might change size
        for coll in self.meshes['custom_links']:
            coll.remove()
        self.meshes['custom_links'].clear()

        if positions.shape[0] < 2:
            return

        if base_height is None:
            base_height = positions[0, 2]

        # ── Scale reference: mean of all *non-degenerate* link lengths ────────
        # Using mean (not just first link) avoids the Spherical arm bug where
        # J2 has a=0,d=0 making the first useful link index unpredictable.
        link_lengths = []
        for i in range(positions.shape[0] - 1):
            ln = np.linalg.norm(positions[i+1] - positions[i])
            if ln > 1e-4:  # skip zero-length DH links
                link_lengths.append(ln)

        scale = float(np.mean(link_lengths)) if link_lengths else 0.15
        if scale < 0.03:
            scale = 0.15  # fallback for very compact robots

        num_links = positions.shape[0] - 1
        base_w        = max(0.04, scale * 0.35)
        base_plate_w  = base_w * 1.6
        r_base_joint  = max(0.012, scale * 0.12)

        # Draw / Update base plate
        plate_h = max(0.008, scale * 0.06)
        p_start = np.array([0, 0, 0])
        p_end = np.array([0, 0, plate_h])
        p_verts, p_faces = cylinder_mesh(p_start, p_end, base_plate_w / 2, resolution=24)
        if p_faces:
            if 'base_plate' not in self.meshes or self.meshes.get('base_plate') is None:
                coll = Poly3DCollection([p_verts[f] for f in p_faces],
                                        facecolors='#444', edgecolors='#555', linewidths=0.3)
                self.ax.add_collection3d(coll)
                self.meshes['base_plate'] = coll
            else:
                self.meshes['base_plate'].set_verts([p_verts[f] for f in p_faces])
                self.meshes['base_plate'].set_visible(True)

        # Draw / Update base cylinder
        b_start = np.array([0, 0, plate_h])
        b_end = np.array([0, 0, max(plate_h + 1e-4, base_height)])
        base_verts, base_faces = cylinder_mesh(b_start, b_end, base_w / 2, resolution=24)
        self._update_mesh('base', base_verts, base_faces, self.default_colors['base'])

        # Sync custom_links and joints list
        while len(self.meshes['custom_links']) < num_links:
            c = Poly3DCollection([], alpha=1.0)
            self.ax.add_collection3d(c)
            self.meshes['custom_links'].append(c)
        while len(self.meshes['custom_links']) > num_links:
            c = self.meshes['custom_links'].pop()
            c.remove()

        while len(self.meshes['joints']) < num_links:
            c = Poly3DCollection([], alpha=1.0)
            self.ax.add_collection3d(c)
            self.meshes['joints'].append(c)
        while len(self.meshes['joints']) > num_links:
            c = self.meshes['joints'].pop()
            c.remove()

        # Draw links and joints with tapering
        for i in range(num_links):
            start_pt = positions[i]
            end_pt   = positions[i + 1]

            seg_len = np.linalg.norm(end_pt - start_pt)

            # Tapering factor: 1.0 at base → 0.4 at tip
            taper       = 1.0 - (i / max(num_links, 1)) * 0.6
            curr_link_r = r_base_joint * 0.8 * taper
            curr_joint_r = r_base_joint * taper

            # ── Link cylinder ──────────────────────────────────────────────
            if seg_len > 1e-4:   # skip zero-length DH links (no mesh = no stretch)
                verts, faces = cylinder_mesh(start_pt, end_pt, curr_link_r, resolution=16)
                if faces:
                    if i == 0:
                        color = self.default_colors['upper']
                    elif i == num_links - 1:
                        color = self.default_colors['gripper']
                    else:
                        color = self.default_colors['lower']
                    self.meshes['custom_links'][i].set_verts([verts[f] for f in faces])
                    self.meshes['custom_links'][i].set_facecolor(color)
                    self.meshes['custom_links'][i].set_edgecolor('#444')
                    self.meshes['custom_links'][i].set_linewidth(0.5)
                    self.meshes['custom_links'][i].set_visible(True)
                else:
                    self.meshes['custom_links'][i].set_visible(False)
            else:
                self.meshes['custom_links'][i].set_visible(False)

            # ── Joint sphere (at end_pt, always drawn regardless of seg_len) ──
            if i == num_links - 1:
                j_color = self.default_colors['joint_tip']
                curr_joint_r *= 0.8
            else:
                color_key = ('joint_shoulder' if i == 0
                             else 'joint_elbow' if i == 1
                             else 'joint_wrist')
                j_color = self.default_colors[color_key]

            j_verts, j_faces = sphere_mesh(end_pt, curr_joint_r, resolution=16)
            if j_faces:
                self.meshes['joints'][i].set_verts([j_verts[f] for f in j_faces])
                self.meshes['joints'][i].set_facecolor(j_color)
                self.meshes['joints'][i].set_edgecolor('#333')
                self.meshes['joints'][i].set_linewidth(0.3)
                self.meshes['joints'][i].set_visible(True)
            else:
                self.meshes['joints'][i].set_visible(False)

        # Adjust viewport to fit the new chain size
        self._adjust_limits(positions)
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
