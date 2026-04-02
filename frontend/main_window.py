#!/usr/bin/env python3
"""
MainWindow for the simulation application.
Holds the left panel (controls) and right panel (3D visualization).
"""

import sys
from PyQt6.QtWidgets import QMainWindow, QWidget, QSplitter, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QMessageBox, QCheckBox
from PyQt6.QtCore import Qt

# Panels will be imported when needed to avoid circular dependencies
# from frontend.panels.connection_panel import ConnectionPanel
# from frontend.panels.data_panel import DataPanel
# from frontend.panels.arm_canvas import ArmCanvas

# Backend kinematics (used in draw updates)
from backend.kinematics import compute_arm_positions, ArmConfig, inverse_kinematics_3dof

# Panels
from frontend.panels.robot_config_panel import RobotConfigPanel


class MainWindow(QMainWindow):
    """Main application window with two-panel layout."""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("AppName - Robotic Arm Simulation")
        self.resize(1200, 800)

        # Central widget with splitter for resizable panels
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)
        layout.setContentsMargins(0, 0, 0, 0)

        # Header with app name
        header = QLabel("AppName — Real-Time Simulation")
        header.setStyleSheet("font-size: 14px; font-weight: bold; padding: 8px; background-color: #252526; color: #ddd;")
        header.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(header)

        # Splitter for left/right panels
        splitter = QSplitter(Qt.Orientation.Horizontal)

        # Left panel container
        self.left_panel = QWidget()
        left_layout = QVBoxLayout(self.left_panel)
        left_layout.setContentsMargins(10, 10, 10, 10)
        left_layout.setSpacing(12)
        self.left_panel.setMinimumWidth(320)
        self.left_panel.setStyleSheet("background-color: #1e1e1e; border-right: 1px solid #444;")
        self.left_layout = left_layout  # keep reference to add panels

        # Right panel container
        self.right_panel = QWidget()
        self.right_layout = QVBoxLayout(self.right_panel)
        self.right_layout.setContentsMargins(0, 0, 0, 0)
        self.right_panel.setStyleSheet("background-color: #2d2d2d;")

        splitter.addWidget(self.left_panel)
        splitter.addWidget(self.right_panel)
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)

        layout.addWidget(splitter, 1)

        # Set initial splitter sizes
        splitter.setSizes([320, 880])

        # Set a reasonable default window size
        self.resize(1200, 800)
        self.setMinimumSize(1000, 600)

        # Status bar
        self.status_bar = self.statusBar()
        self.status_bar.showMessage("Disconnected")

        # Apply dark theme to the whole app
        self.setStyleSheet("""
            QMainWindow { background-color: #1e1e1e; }
            QWidget { color: #eee; font-family: Segoe UI, Arial; font-size: 9pt; }
            QScrollBar { background: #333; }
            QScrollBar::handle { background: #555; border-radius: 3px; }
        """)

        # Shared kinematics configuration (mutable)
        self.kinematics_config = ArmConfig()

        # Setup UI panels (lazy imports to avoid circular dependencies)
        self._setup_panels()

    def _setup_panels(self):
        """Instantiate and add panels to the layout."""
        from frontend.panels.connection_panel import ConnectionPanel
        from frontend.panels.data_panel import DataPanel
        from frontend.panels.arm_canvas import ArmCanvas

        # Left panel: controls

        # Connection panel (Simulator/Interactive mode)
        self.connection_panel = ConnectionPanel()
        self.left_layout.addWidget(self.connection_panel)

        # Robot configuration panel (adjustable geometry)
        self.robot_config_panel = RobotConfigPanel(self.kinematics_config)
        self.robot_config_panel.config_changed.connect(self._on_robot_config_changed)
        self.left_layout.addWidget(self.robot_config_panel)

        # Data panel (real-time feedback)
        self.data_panel = DataPanel()
        self.left_layout.addWidget(self.data_panel)

        # Trajectory panel (Cartesian control)
        from frontend.panels.trajectory_panel import TrajectoryPanel
        self.trajectory_panel = TrajectoryPanel(config=self.kinematics_config)
        self.trajectory_panel.setVisible(False)
        self.left_layout.addWidget(self.trajectory_panel)

        # Waypoint panel not implemented for 3-DOF; do not add

        # Trajectory playback controls
        self.btn_play = QPushButton("Play Trajectory")
        self.btn_play.setStyleSheet("background-color: #8e44ad; color: white; padding: 8px; margin-top: 5px;")
        self.btn_play.clicked.connect(self._play_trajectory)
        self.left_layout.addWidget(self.btn_play)
        self.btn_play.setVisible(False)
        self.btn_play.setEnabled(False)  # disabled until enough waypoints

        self.btn_clear_trace = QPushButton("Clear Trace")
        self.btn_clear_trace.setStyleSheet("background-color: #7f8c8d; color: white; padding: 8px;")
        self.btn_clear_trace.clicked.connect(self._clear_trace)
        self.left_layout.addWidget(self.btn_clear_trace)
        self.btn_clear_trace.setVisible(False)

        self.left_layout.addStretch()

        # Right panel: toolbar + canvas
        # Toolbar for view controls
        view_toolbar = QWidget()
        view_toolbar.setMaximumHeight(36)
        view_toolbar.setStyleSheet("background-color: #252526; border-bottom: 1px solid #444;")
        toolbar_layout = QHBoxLayout(view_toolbar)
        toolbar_layout.setContentsMargins(10, 4, 10, 4)
        toolbar_layout.setSpacing(6)

        # View preset buttons (small, evenly spaced)
        btn_style = """
            QPushButton { background-color: #444; color: #ddd; padding: 4px 10px; border: 1px solid #555; border-radius: 4px; font-size: 10px; }
            QPushButton:hover { background-color: #555; border-color: #666; }
            QPushButton:pressed { background-color: #333; }
        """
        for name, label in [('front','Front'), ('side','Side'), ('top','Top'), ('iso','Iso'), ('back','Back')]:
            btn = QPushButton(label)
            btn.setStyleSheet(btn_style)
            btn.clicked.connect(lambda checked, n=name: self._set_view_preset(n))
            toolbar_layout.addWidget(btn)

        toolbar_layout.addSpacing(12)

        # Ground toggle checkbox
        self.chk_ground = QCheckBox("Show Ground")
        self.chk_ground.setChecked(True)
        self.chk_ground.setStyleSheet("color: #ddd; font-size: 10px;")
        self.chk_ground.toggled.connect(self._toggle_ground)
        toolbar_layout.addWidget(self.chk_ground)

        toolbar_layout.addSpacing(8)

        # Reset view button
        self.btn_reset_view = QPushButton("Reset")
        self.btn_reset_view.setStyleSheet("""
            QPushButton { background-color: #3498db; color: white; padding: 4px 12px; border: 1px solid #2980b9; border-radius: 4px; font-size: 10px; font-weight: bold; }
            QPushButton:hover { background-color: #2980b9; }
        """)
        self.btn_reset_view.clicked.connect(self._reset_view)
        toolbar_layout.addWidget(self.btn_reset_view)

        toolbar_layout.addStretch()
        self.right_layout.addWidget(view_toolbar)

        # 3D canvas
        self.arm_canvas = ArmCanvas()
        self.arm_canvas.setMinimumSize(600, 500)
        self.right_layout.addWidget(self.arm_canvas, stretch=3)

        # Setup connection handling
        self.simulator = None
        self.interactive_controller = None
        self.connection_panel.connect_requested.connect(self._on_connect_requested)
        self.connection_panel.disconnect_requested.connect(self._on_disconnect_requested)
        self.connection_panel.mode_changed.connect(self._on_mode_changed)

        # Trajectory panel emits target joint angles (from IK)
        self.trajectory_panel.target_angles_updated.connect(self._on_target_angles)

        # Initialize UI to match the shared config
        self._on_robot_config_changed(self.kinematics_config)

    def _on_mode_changed(self, mode: str):
        """Show/hide panels based on mode."""
        visible = (mode == "interactive")
        self.trajectory_panel.setVisible(visible)
        # Waypoint panel and trajectory playback not implemented for 3-DOF yet
        self.btn_play.setVisible(False)
        self.btn_clear_trace.setVisible(False)

    def _on_connect_requested(self, port: str, baud: int):
        """Handle connection request."""
        if port == "INTERACTIVE":
            self._start_interactive()
        elif port == "SIMULATED (no hardware)":
            self._start_simulation()
        else:
            # Real serial not implemented yet
            self.connection_panel.set_status(f"Real serial not implemented yet: {port}")
            self.connection_panel.set_connected(False)

    def _on_disconnect_requested(self):
        """Handle disconnect."""
        self._stop_current_mode()
        self.connection_panel.set_connected(False)
        self.connection_panel.set_status("Disconnected")

    def _start_simulation(self):
        """Start simulated data generation."""
        from PyQt6.QtCore import QTimer, QObject, pyqtSignal
        import numpy as np

        class Simulator(QObject):
            data_updated = pyqtSignal(float, float, float)
            def __init__(self, parent=None):
                super().__init__(parent)
                self.t = 0.0
                self.timer = QTimer(parent)
                self.timer.timeout.connect(self._update)
                self.interval = 50

            def start(self):
                self.timer.start(self.interval)

            def stop(self):
                self.timer.stop()

            def _update(self):
                self.t += 0.05
                roll = 20 * np.sin(self.t * 0.5)
                pitch = 15 * np.sin(self.t * 0.7 + 1)
                yaw = 30 * np.sin(self.t * 0.3 + 2)
                self.data_updated.emit(roll, pitch, yaw)

        self.simulator = Simulator(self)
        self.simulator.data_updated.connect(self._on_data_received)
        self.simulator.start()
        self.connection_panel.set_connected(True)
        self.status_bar.showMessage("Simulation mode active")
        self.connection_panel.set_status("Simulation running")

    def _start_interactive(self):
        """Start interactive mode - target angles from trajectory panel."""

        self.interactive_controller = type('InteractiveCtrl', (), {
            'active': True,
            'target': [0.0, 0.0, 0.0]
        })  # simple namespace
        self.connection_panel.set_connected(True)
        self.status_bar.showMessage("Interactive mode active")
        self.connection_panel.set_status("Use sliders to set target")
        # Set initial target to zero
        self._apply_target_angles(0, -45, 50)

    def _stop_current_mode(self):
        """Stop whichever mode is running."""
        if self.simulator:
            self.simulator.stop()
            self.simulator = None
        if self.interactive_controller:
            self.interactive_controller.active = False
            self.interactive_controller = None
        self.arm_canvas._init_empty_plot()

    def _on_data_received(self, roll: float, pitch: float, yaw: float):
        """Update UI with data (from simulation or real source)."""
        self.data_panel.update_values(roll, pitch, yaw)
        self.trajectory_panel.set_current_angles(roll, pitch, yaw)
        positions = compute_arm_positions(roll, pitch, yaw)
        self.arm_canvas.draw_arm(positions)

    def _on_target_angles(self, q1: float, q2: float, q3: float):
        """Handle target angles from trajectory panel (interactive mode)."""
        if self.interactive_controller and self.interactive_controller.active:
            self._apply_target_angles(q1, q2, q3)

    def _apply_target_angles(self, q1: float, q2: float, q3: float):
        """Apply target joint angles to arm (3-DOF)."""
        # Update DataPanel
        self.data_panel.update_values(q1, q2, q3)
        # Compute forward kinematics
        positions = compute_arm_positions(q1, q2, q3)
        self.arm_canvas.draw_arm(positions)
        # Keep trajectory panel in sync
        self.trajectory_panel.set_current_angles(q1, q2, q3)

    def _update_play_button(self):
        """Enable Play button only if at least 2 waypoints exist."""
        waypoints = self.waypoint_panel.get_waypoints()
        self.btn_play.setEnabled(len(waypoints) >= 2)

    def _clear_trace(self):
        """Clear the trajectory trace from the canvas."""
        self.arm_canvas.set_trajectory([])
        self.status_bar.showMessage("Trace cleared")

    def _reset_view(self):
        """Reset 3D camera to default view."""
        if hasattr(self.arm_canvas, 'reset_view'):
            self.arm_canvas.reset_view()
            self.status_bar.showMessage("View reset to Isometric")

    def _set_view_preset(self, name):
        """Set camera to a preset view."""
        if hasattr(self.arm_canvas, 'set_view'):
            self.arm_canvas.set_view(name=name)
            view_names = {'front':'Front', 'side':'Side', 'top':'Top', 'iso':'Isometric', 'back':'Back'}
            self.status_bar.showMessage(f"View: {view_names.get(name, name)}")

    def _on_robot_config_changed(self, config):
        """Handle updates to robot geometry parameters."""
        self.kinematics_config = config
        max_reach = config.upper_arm_length + config.lower_arm_length + config.gripper_offset
        # Update workspace boundary circle
        self.arm_canvas.update_workspace_boundary(max_reach)
        # Update TrajectoryPanel ranges and clamp target
        self.trajectory_panel.update_config(config)
        # Recompute IK for current target (XYZ) with new config
        x, y, z = self.trajectory_panel.current_pos
        result = inverse_kinematics_3dof(x, y, z, config=config, elbow_down=True)
        if result:
            q1, q2, q3 = result
            self._apply_target_angles(q1, q2, q3)
        else:
            self.status_bar.showMessage("Current target unreachable with new dimensions", 3000)

    def _toggle_ground(self, checked):
        """Toggle ground and workspace grid visibility."""
        if hasattr(self.arm_canvas, 'toggle_ground'):
            self.arm_canvas.toggle_ground(checked)
            self.status_bar.showMessage("Ground hidden" if not checked else "Ground shown")

    def _play_trajectory(self):
        """Generate and play the trajectory through waypoints."""
        waypoints = self.waypoint_panel.get_waypoints()
        if len(waypoints) < 2:
            QMessageBox.warning(self, "Not enough waypoints", "Add at least 2 waypoints to play a trajectory.")
            return

        # Generate smooth trajectory
        try:
            from backend.trajectory import generate_trajectory, validate_trajectory
            traj = generate_trajectory(waypoints, num_points=200, method='linear')
        except Exception as e:
            QMessageBox.critical(self, "Trajectory Error", f"Failed to generate: {e}")
            return

        # Validate trajectory
        valid_mask = validate_trajectory(traj)
        if not all(valid_mask):
            invalid_count = sum(1 for v in valid_mask if not v)
            resp = QMessageBox.question(self, "Collision/Unreachable Points",
                                        f"{invalid_count} points fail IK or collision detection. Play anyway?",
                                        QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No)
            if resp == QMessageBox.StandardButton.No:
                return
            # Filter valid points only? For simplicity, we'll still play but mark invalid points in trace.
            # We'll keep all points but maybe color invalid ones red later. For now, just proceed.

        # Extract tip positions for trace
        trace_points = [pt['pos'] for pt in traj]
        self.arm_canvas.set_trajectory(trace_points)

        # Start playback using QTimer
        self._trajectory_index = 0
        self._trajectory_points = traj
        self._trajectory_playing = True
        from PyQt6.QtCore import QTimer
        self._traj_timer = QTimer()
        self._traj_timer.timeout.connect(self._trajectory_step)
        self._trajectory_interval = 30  # ms between frames (about 33fps)
        self._traj_timer.start(self._trajectory_interval)
        self.btn_play.setEnabled(False)
        self.status_bar.showMessage("Playing trajectory...")

    def _trajectory_step(self):
        """Advance one step in trajectory playback."""
        if not hasattr(self, '_trajectory_playing') or not self._trajectory_playing:
            return
        if self._trajectory_index >= len(self._trajectory_points):
            self._traj_timer.stop()
            self._trajectory_playing = False
            self.btn_play.setEnabled(True)
            self.status_bar.showMessage("Trajectory complete")
            return

        pt = self._trajectory_points[self._trajectory_index]
        pos = pt['pos']
        wrist = pt['wrist']
        # Solve IK for XYZ to get q1,q2,q3
        from backend.trajectory import solve_ik_for_waypoint
        angles = solve_ik_for_waypoint(pos, wrist)
        if angles is not None:
            q1, q2, q3, q4, q5, q6 = angles
            self._apply_target_angles(q1, q2, q3, q4, q5, q6)
        else:
            print(f"WARNING: Trajectory point {self._trajectory_index} unreachable, skipping", file=sys.stderr)
        self._trajectory_index += 1
