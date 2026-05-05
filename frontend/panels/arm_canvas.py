#!/usr/bin/env python3
"""
ArmCanvas - 3D Robotic Arm visualization with realistic meshes, ground plane, and collision detection.
"""

from PyQt6.QtWidgets import QVBoxLayout, QWidget
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection
import numpy as np
from backend.meshes import cylinder_mesh, sphere_mesh, cuboid_mesh
from backend.kinematics import ArmConfig


class ArmCanvas(FigureCanvas):
    """Matplotlib 3D canvas for rendering the robotic arm with 3D meshes and collision detection."""

    def __init__(self, parent=None):
        self.fig = Figure(figsize=(9, 7), facecolor='#282828', dpi=100)
        super().__init__(self.fig)
        self.setParent(parent)

        self.fig.subplots_adjust(left=0, right=1, bottom=0, top=1)
        self.ax = self.fig.add_axes([0, 0, 1, 1], projection='3d')
        self.ax.set_facecolor('#282828')
        
        # Turn off default matplotlib bounding box to mimic Blender's infinite void
        self.ax.set_axis_off()

        # ── Blender Navigation Overrides ──
        self.fig.canvas.mpl_connect('scroll_event', self._on_mouse_scroll)
        self._orig_button_press = self.ax._button_press
        self.ax._button_press = self._custom_button_press

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

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._update_aspect_ratio()

    def _update_aspect_ratio(self):
        """Dynamically adjust 3D box aspect and data limits to fill the canvas widget."""
        w = self.width()
        h = self.height()
        if h <= 0 or not hasattr(self, 'workspace_radius'):
            return

        aspect = w / h
        xy_max = self.workspace_radius
        z_max = self.ax.get_zlim()[1]
        
        # Base physical bounds
        base_x_range = 2 * xy_max
        base_y_range = 2 * xy_max
        
        if aspect > 1.0:
            # Widescreen: stretch X limits to fill horizontal space
            self.ax.set_xlim([-xy_max * aspect, xy_max * aspect])
            self.ax.set_ylim([-xy_max, xy_max])
            self.ax.set_box_aspect((2 * xy_max * aspect, 2 * xy_max, z_max))
        else:
            # Tallscreen: stretch Y limits to fill vertical space
            self.ax.set_xlim([-xy_max, xy_max])
            self.ax.set_ylim([-xy_max / aspect, xy_max / aspect])
            self.ax.set_box_aspect((2 * xy_max, 2 * xy_max / aspect, z_max))
        
        # Ensure the axes occupies the full figure on every resize
        self.ax.set_position([0, 0, 1, 1])
        self.draw_idle()



    def _custom_button_press(self, event):
        """Intercept button press to support Shift+Middle Click for panning."""
        if event.button == 2 and getattr(event, 'key', None) == 'shift':
            event.button = 3  # Trick Matplotlib into thinking pan_btn was pressed
        self._orig_button_press(event)

    def _on_mouse_scroll(self, event):
        """Zoom in/out on scroll wheel."""
        if event.inaxes != self.ax:
            return
        
        # Scroll up (positive) = zoom in = scale < 1.0
        # Scroll down (negative) = zoom out = scale > 1.0
        scale_factor = 0.9 if event.step > 0 else 1.1
        
        xlim = self.ax.get_xlim3d()
        ylim = self.ax.get_ylim3d()
        zlim = self.ax.get_zlim3d()
        
        x_center = sum(xlim) / 2
        y_center = sum(ylim) / 2
        z_center = sum(zlim) / 2
        
        x_span = (xlim[1] - xlim[0]) * scale_factor
        y_span = (ylim[1] - ylim[0]) * scale_factor
        z_span = (zlim[1] - zlim[0]) * scale_factor
        
        self.ax.set_xlim3d([x_center - x_span/2, x_center + x_span/2])
        self.ax.set_ylim3d([y_center - y_span/2, y_center + y_span/2])
        self.ax.set_zlim3d([z_center - z_span/2, z_center + z_span/2])
        
        self.draw_idle()

    def _init_empty_plot(self):
        self.ax.cla()
        # Reset all mesh references to avoid stale artists after clear
        self.meshes = {
            'base': None,
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
        # Re-apply styling after cla()
        self.ax.set_facecolor('#282828')
        self.ax.set_axis_off()

        # Workspace bounds (meters)
        self.workspace_radius = 1.2  # XY range: ±1.2m
        self.workspace_z_max = 1.44   # Z max: 1.44m (aspect ~0.6)

        # Set fixed limits (shoulder at origin, works within these bounds)
        self.ax.set_xlim([-self.workspace_radius, self.workspace_radius])
        self.ax.set_ylim([-self.workspace_radius, self.workspace_radius])
        self.ax.set_zlim([0, self.workspace_z_max])

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

        # Enable interactive navigation: Middle click = rotate, Right click (mapped via shift) = pan
        self.ax.mouse_init(rotate_btn=2, pan_btn=3, zoom_btn=None)

    def _add_static_ground(self):
        """Create an infinite floor grid with Blender-colored origin axes."""
        size = self.workspace_radius  # ground spans full XY workspace

        # Grid lines every 0.25m
        step = 0.25
        grid_color = '#3a3a3a'
        for x in np.arange(-size, size + step, step):
            if abs(x) > 1e-5:  # skip origin for axis lines
                line = self.ax.plot([x, x], [-size, size], [0, 0], color=grid_color, linewidth=0.8)[0]
                self._ground_elements.append(line)
        for y in np.arange(-size, size + step, step):
            if abs(y) > 1e-5:
                line = self.ax.plot([-size, size], [y, y], [0, 0], color=grid_color, linewidth=0.8)[0]
                self._ground_elements.append(line)

        # Origin Axes (Blender colors: X=Red, Y=Green, Z=Blue)
        # X-axis (Red)
        line_x = self.ax.plot([-size, size], [0, 0], [0, 0], color='#e74c3c', linewidth=1.5)[0]
        self._ground_elements.append(line_x)
        # Y-axis (Green)
        line_y = self.ax.plot([0, 0], [-size, size], [0, 0], color='#2ecc71', linewidth=1.5)[0]
        self._ground_elements.append(line_y)
        # Z-axis (Blue)
        line_z = self.ax.plot([0, 0], [0, 0], [0, self.workspace_z_max], color='#3498db', linewidth=1.5)[0]
        self._ground_elements.append(line_z)

        # Labels for the origin axes
        txt_x = self.ax.text(size * 1.05, 0, 0, 'X', color='#e74c3c', fontsize=10, fontweight='bold')
        txt_y = self.ax.text(0, size * 1.05, 0, 'Y', color='#2ecc71', fontsize=10, fontweight='bold')
        txt_z = self.ax.text(0, 0, self.workspace_z_max * 1.05, 'Z', color='#3498db', fontsize=10, fontweight='bold')
        self._ground_elements.extend([txt_x, txt_y, txt_z])

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

    def update_ground(self, radius: float):
        """Rebuild ground plane, grid, and workspace circle to match new radius with Blender aesthetics."""
        # Remove existing ground elements
        for artist in self._ground_elements:
            artist.remove()
        self._ground_elements.clear()
        
        self.workspace_radius = radius
        # Draw a grid large enough to cover widescreen but small enough to avoid projection issues
        size = radius * 8.0 

        # Grid lines coordinates
        segments = []
        step = 0.25
        grid_color = '#3a3a3a'
        
        # Build grid segments
        for x in np.arange(-size, size + step, step):
            if abs(x) > 1e-5:
                segments.append([[x, -size, 0], [x, size, 0]])
        for y in np.arange(-size, size + step, step):
            if abs(y) > 1e-5:
                segments.append([[-size, y, 0], [size, y, 0]])
        
        # Add as a single collection (MUCH faster)
        grid_coll = Line3DCollection(segments, colors=grid_color, linewidths=0.5, alpha=0.4)
        grid_coll.set_clip_on(False) # Prevent clipping to the xlim/ylim box
        self.ax.add_collection3d(grid_coll)
        self._ground_elements.append(grid_coll)

        # Origin Axes (Blender colors: X=Red, Y=Green, Z=Blue)
        z_max = self.ax.get_zlim()[1]
        
        axis_len = radius * 2.0
        line_x = self.ax.plot([-axis_len, axis_len], [0, 0], [0, 0], color='#e74c3c', linewidth=1.5, zorder=5)[0]
        line_x.set_clip_on(False)
        self._ground_elements.append(line_x)
        
        line_y = self.ax.plot([0, 0], [-axis_len, axis_len], [0, 0], color='#2ecc71', linewidth=1.5, zorder=5)[0]
        line_y.set_clip_on(False)
        self._ground_elements.append(line_y)
        
        line_z = self.ax.plot([0, 0], [0, 0], [0, z_max], color='#3498db', linewidth=1.5, zorder=5)[0]
        line_z.set_clip_on(False)
        self._ground_elements.append(line_z)

        # Labels for the origin axes
        txt_x = self.ax.text(axis_len * 1.05, 0, 0, 'X', color='#e74c3c', fontsize=10, fontweight='bold')
        txt_y = self.ax.text(0, axis_len * 1.05, 0, 'Y', color='#2ecc71', fontsize=10, fontweight='bold')
        txt_z = self.ax.text(0, 0, z_max * 1.05, 'Z', color='#3498db', fontsize=10, fontweight='bold')
        self._ground_elements.extend([txt_x, txt_y, txt_z])

        # Workspace boundary circle
        theta = np.linspace(0, 2*np.pi, 64)
        x_circle = radius * np.cos(theta)
        y_circle = radius * np.sin(theta)
        z_circle = np.zeros_like(x_circle)
        circle_line = self.ax.plot(x_circle, y_circle, z_circle, color='#999', linewidth=0.8, alpha=0.25, linestyle='--')[0]
        circle_line.set_clip_on(False)
        self._ground_elements.append(circle_line)
        self._workspace_circle = circle_line
        self.draw_idle()

    def frame_to_fit_robot(self, max_horizontal: float, max_vertical: float, base_height: float = 0.0, padding: float = 0.2):
        """Adjust camera and axis limits to fit a robot with given maximum reach extents."""
        if max_horizontal <= 0:
            max_horizontal = 0.5
        if max_vertical <= 0:
            max_vertical = 0.5
        xy_max = max_horizontal * (1 + padding)
        z_max = max_vertical * (1 + padding)
        # Set limits
        self.ax.set_xlim([-xy_max, xy_max])
        self.ax.set_ylim([-xy_max, xy_max])
        self.ax.set_zlim([0.0, z_max])
        # Update ground and grid to new radius
        self.update_ground(xy_max)
        # Set camera distance (perspective) to a comfortable multiple of scene size
        self.ax.dist = max(xy_max * 2, z_max) * 2.2
        
        # Apply responsive limits and box aspect to fill widget
        self._update_aspect_ratio()
        self.draw_idle()

    def draw_chain(self, positions, base_height=None):
        """
        Draw a generic kinematic chain from DH parameters.
        positions: (N+1, 3) numpy array where positions[0] is base origin and positions[-1] is end-effector.
        base_height: optional height of base from ground; if None, uses positions[0,2].
        """
        # Hide idle overlay — data has arrived
        self.set_idle_message(False)

        # Clear previous custom meshes and standard meshes
        for key in ['base', 'upper', 'lower', 'gripper']:
            if self.meshes[key]:
                self.meshes[key].remove()
                self.meshes[key] = None
        for coll in self.meshes['joints']:
            coll.remove()
        self.meshes['joints'].clear()
        for coll in self.meshes['custom_links']:
            coll.remove()
        self.meshes['custom_links'].clear()

        if positions.shape[0] < 2:
            return

        if base_height is None:
            base_height = positions[0, 2]

        # Draw base cuboid (static, from ground up to base origin)
        base_size = (0.15, 0.15, base_height)
        base_center = np.array([0.0, 0.0, base_height/2])
        base_verts, base_faces = cuboid_mesh(base_center, base_size)
        base_coll = Poly3DCollection([base_verts[face] for face in base_faces], facecolors=self.default_colors['base'], edgecolors='#444', linewidths=0.5)
        self.ax.add_collection3d(base_coll)
        self.meshes['base'] = base_coll

        # Draw cylinders for each link (between consecutive joints)
        link_radius = 0.02
        num_links = positions.shape[0] - 1
        for i in range(num_links):
            start_pt = positions[i]
            end_pt = positions[i+1]
            verts, faces = cylinder_mesh(start_pt, end_pt, link_radius, resolution=10)
            # Choose color based on position: first after base = upper, middle = lower, last = gripper
            if i == 0:
                color = self.default_colors['upper']
            elif i == num_links - 1:
                color = self.default_colors['gripper']
            else:
                color = self.default_colors['lower']
            coll = Poly3DCollection([verts[face] for face in faces], facecolors=color, edgecolors='#444', linewidths=0.5)
            self.ax.add_collection3d(coll)
            self.meshes['custom_links'].append(coll)

        # Draw joint spheres at each joint position (except base, optional)
        joint_radius = 0.03
        # For joints, we have intermediate points (positions[1:-1]) and tip (last)
        for idx, pt in enumerate(positions[1:]):  # skip base at 0
            if idx == len(positions) - 2:  # last joint (tip)
                color_key = 'joint_tip'
                radius = 0.02
            else:
                color_key = 'joint_shoulder' if idx == 0 else 'joint_elbow' if idx == 1 else 'joint_wrist'
                radius = joint_radius
            j_verts, j_faces = sphere_mesh(pt, radius, resolution=6)
            face_arrays = [j_verts[face] for face in j_faces]
            coll = Poly3DCollection(face_arrays, facecolors=self.default_colors[color_key], edgecolors='#333', linewidths=0.3)
            self.ax.add_collection3d(coll)
            self.meshes['joints'].append(coll)

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
