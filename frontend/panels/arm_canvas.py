#!/usr/bin/env python3
"""
ArmCanvas - 3D Robotic Arm visualization with realistic meshes (cylinders, spheres).
"""

from PyQt6.QtWidgets import QVBoxLayout, QWidget
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


class ArmCanvas(FigureCanvas):
    """Matplotlib 3D canvas for rendering the robotic arm with 3D meshes."""

    def __init__(self, parent=None):
        self.fig = Figure(figsize=(8, 6), facecolor='#2d2d2d')
        super().__init__(self.fig)
        self.setParent(parent)

        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_facecolor('#2d2d2d')

        # Styling
        self.ax.grid(True, color='#444')
        self.ax.xaxis.label.set_color('#aaa')
        self.ax.yaxis.label.set_color('#aaa')
        self.ax.zaxis.label.set_color('#aaa')
        self.ax.tick_params(colors='#aaa')

        # Segment styling
        self.colors = {
            'base': '#666666',
            'shoulder': '#3498db',
            'elbow': '#e67e22',
            'wrist': '#2ecc71',
            'gripper': '#e74c3c',
            'links': '#ffffff'
        }

        # Joint positions
        self.joint_positions = np.zeros((6, 3))

        # Initialize plot
        self._init_empty_plot()

    def _init_empty_plot(self):
        """Initialize with empty/invalid arm."""
        self.ax.cla()
        self._setup_axes()
        self.draw()

    def _setup_axes(self):
        """Setup axis labels and limits."""
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_zlabel('Z (m)')
        self.fig.tight_layout()

    def _create_cylinder_mesh(self, start, end, radius=0.02, resolution=16):
        """
        Generate vertices and faces for a cylinder along the vector from start to end.

        Returns:
            vertices: (N, 3) array
            faces: list of tuples (vertex indices) for Poly3DCollection
            (Simpler: return list of polygon vertices for each ring)
        """
        # Direction vector
        direction = np.array(end) - np.array(start)
        length = np.linalg.norm(direction)
        if length == 0:
            return [], []

        direction = direction / length
        # Choose an arbitrary axis not parallel to direction
        if abs(direction[0]) < 0.9:
            up = np.array([1, 0, 0])
        else:
            up = np.array([0, 1, 0])
        # Compute perpendicular vectors
        right = np.cross(direction, up)
        right = right / np.linalg.norm(right)
        up2 = np.cross(right, direction)

        # Generate circles
        theta = np.linspace(0, 2*np.pi, resolution, endpoint=False)
        circle = np.array([right * np.cos(t) + up2 * np.sin(t) for t in theta])  # (resolution, 3)

        # Create vertices along cylinder
        vertices = []
        for i in range(resolution + 1):
            t = i / resolution
            center = np.array(start) + direction * length * t
            if i == resolution:
                ring = center + circle * radius
            else:
                ring = center + circle * radius
            vertices.extend(ring)
        vertices = np.array(vertices)  # (resolution*(resolution+1), 3) ??? Actually vertices per ring: resolution vertices; we have resolution+1 rings

        # Better: vertices will be a list of rings, each ring has 'resolution' vertices
        # For Poly3DCollection, we can directly provide list of vertex arrays for each quadrilateral face.
        # Simpler: return list of polygons (each quad between two rings)
        polygons = []
        for i in range(resolution):
            for j in range(resolution):
                # Quad connecting ring i, ring i+1 at vertices j and (j+1)%resolution
                v0 = i * resolution + j
                v1 = i * resolution + (j + 1) % resolution
                v2 = (i + 1) * resolution + (j + 1) % resolution
                v3 = (i + 1) * resolution + j
                polygons.append([vertices[v0], vertices[v1], vertices[v2], vertices[v3]])
        return polygons

    def _create_sphere_mesh(self, center, radius=0.025, resolution=12):
        """Return a Poly3DCollection of sphere (approximated by icosphere-like)."""
        u = np.linspace(0, 2*np.pi, resolution)
        v = np.linspace(0, np.pi, resolution)
        verts = []
        for i in range(resolution-1):
            for j in range(resolution):
                # Quad between u[i], u[i+1], v[j], v[j+1]
                theta1 = u[j]
                theta2 = u[(j+1)%(resolution)]
                phi1 = v[i]
                phi2 = v[i+1]
                # 4 vertices
                p1 = (radius*np.sin(phi1)*np.cos(theta1) + center[0],
                      radius*np.sin(phi1)*np.sin(theta1) + center[1],
                      radius*np.cos(phi1) + center[2])
                p2 = (radius*np.sin(phi1)*np.cos(theta2) + center[0],
                      radius*np.sin(phi1)*np.sin(theta2) + center[1],
                      radius*np.cos(phi1) + center[2])
                p3 = (radius*np.sin(phi2)*np.cos(theta2) + center[0],
                      radius*np.sin(phi2)*np.sin(theta2) + center[1],
                      radius*np.cos(phi2) + center[2])
                p4 = (radius*np.sin(phi2)*np.cos(theta1) + center[0],
                      radius*np.sin(phi2)*np.sin(theta1) + center[1],
                      radius*np.cos(phi2) + center[2])
                verts.append([p1, p2, p3, p4])
        return verts

    def _create_box_mesh(self, center, size=(0.05,0.05,0.05)):
        """Create a simple cube centered at center."""
        cx, cy, cz = center
        sx, sy, sz = size
        x = cx + np.array([-sx/2, sx/2, sx/2, -sx/2, -sx/2, -sx/2, sx/2, sx/2])/2
        y = cy + np.array([-sy/2, -sy/2, sy/2, sy/2, -sy/2, sy/2, sy/2, -sy/2])/2
        z = cz + np.array([-sz/2, -sz/2, -sz/2, -sz/2, sz/2, sz/2, sz/2, sz/2])/2
        verts = [
            [ [x[0],y[0],z[0]], [x[1],y[1],z[1]], [x[2],y[2],z[2]], [x[3],y[3],z[3]] ],  # bottom
            [ [x[4],y[4],z[4]], [x[5],y[5],z[5]], [x[6],y[6],z[6]], [x[7],y[7],z[7]] ],  # top
            [ [x[0],y[0],z[0]], [x[1],y[1],z[1]], [x[6],y[6],z[6]], [x[7],y[7],z[7]] ],
            [ [x[0],y[0],z[0]], [x[3],y[3],z[3]], [x[5],y[5],z[5]], [x[4],y[4],z[4]] ],
            [ [x[1],y[1],z[1]], [x[2],y[2],z[2]], [x[6],y[6],z[6]], [x[5],y[5],z[5]] ],
            [ [x[2],y[2],z[2]], [x[3],y[3],z[3]], [x[7],y[7],z[7]], [x[6],y[6],z[6]] ]
        ]
        return verts

    def draw_arm(self, positions):
        """
        Draw the arm with realistic 3D meshes.
        positions: (6, 3) array of (x, y, z) for: base, shoulder, elbow, wrist, tip, extra.
        """
        self.joint_positions = positions
        self.ax.cla()
        self._setup_axes()

        # Extract points
        base = positions[0]
        shoulder = positions[1]
        elbow = positions[2]
        wrist = positions[3]
        tip = positions[4]

        # Config: segment lengths from ArmConfig
        from backend.kinematics import ArmConfig
        config = ArmConfig()
        L1 = config.upper_arm_length
        L2 = config.lower_arm_length
        Lg = config.gripper_offset
        base_h = config.base_height
        # Radii (in meters)
        r_base = 0.04
        r_shoulder = 0.03
        r_elbow = 0.025
        r_wrist = 0.02
        r_gripper = 0.015

        # 1. Base cylinder (vertical)
        base_cyl = self._create_cylinder_mesh([0,0,0], [0,0,base_h], radius=r_base)
        base_col = Poly3DCollection(base_cyl, facecolors=self.colors['base'], edgecolors='#333', linewidths=0.5, alpha=0.9)
        self.ax.add_collection3d(base_col)

        # 2. Upper arm cylinder (shoulder to elbow)
        upper_cyl = self._create_cylinder_mesh(shoulder, elbow, radius=r_shoulder)
        upper_col = Poly3DCollection(upper_cyl, facecolors=self.colors['links'], edgecolors='#333', linewidths=0.5, alpha=0.9)
        self.ax.add_collection3d(upper_col)

        # 3. Lower arm cylinder (elbow to wrist)
        lower_cyl = self._create_cylinder_mesh(elbow, wrist, radius=r_elbow)
        lower_col = Poly3DCollection(lower_cyl, facecolors=self.colors['links'], edgecolors='#333', linewidths=0.5, alpha=0.9)
        self.ax.add_collection3d(lower_col)

        # 4. Wrist to gripper tip (smaller cylinder)
        wrist_cyl = self._create_cylinder_mesh(wrist, tip, radius=r_wrist)
        wrist_col = Poly3DCollection(wrist_cyl, facecolors=self.colors['links'], edgecolors='#333', linewidths=0.5, alpha=0.9)
        self.ax.add_collection3d(wrist_col)

        # Joints as spheres
        shoulder_sphere = self._create_sphere_mesh(shoulder, radius=r_shoulder*1.2)
        elbow_sphere = self._create_sphere_mesh(elbow, radius=r_elbow*1.2)
        wrist_sphere = self._create_sphere_mesh(wrist, radius=r_wrist*1.2)
        gripper_sphere = self._create_sphere_mesh(tip, radius=r_gripper*1.5)

        self.ax.add_collection3d(Poly3DCollection(shoulder_sphere, facecolors=self.colors['shoulder'], edgecolors='#333', linewidths=0.5, alpha=0.9))
        self.ax.add_collection3d(Poly3DCollection(elbow_sphere, facecolors=self.colors['elbow'], edgecolors='#333', linewidths=0.5, alpha=0.9))
        self.ax.add_collection3d(Poly3DCollection(wrist_sphere, facecolors=self.colors['wrist'], edgecolors='#333', linewidths=0.5, alpha=0.9))
        self.ax.add_collection3d(Poly3DCollection(gripper_sphere, facecolors=self.colors['gripper'], edgecolors='#333', linewidths=0.5, alpha=0.9))

        # Set axis limits based on arm extents with padding
        xs, ys, zs = positions[:, 0], positions[:, 1], positions[:, 2]
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

        self.fig.tight_layout()
        self.draw_idle()
