#!/usr/bin/env python3
"""
MainWindow for the simulation application.
Holds the left panel (controls) and right panel (3D visualization).
"""

import sys
from PyQt6.QtWidgets import QMainWindow, QWidget, QSplitter, QVBoxLayout, QLabel
from PyQt6.QtCore import Qt

# Panels will be imported when needed to avoid circular dependencies
# from frontend.panels.connection_panel import ConnectionPanel
# from frontend.panels.data_panel import DataPanel
# from frontend.panels.arm_canvas import ArmCanvas

# Backend kinematics
from backend.kinematics import compute_arm_positions


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
        self.left_panel.setMinimumWidth(300)
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
        splitter.setSizes([300, 900])

        layout.addWidget(splitter, 1)

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

        # Setup UI panels (lazy imports to avoid circular dependencies)
        self._setup_panels()

    def _setup_panels(self):
        """Instantiate and add panels to the layout."""
        from frontend.panels.connection_panel import ConnectionPanel
        from frontend.panels.data_panel import DataPanel
        from frontend.panels.arm_canvas import ArmCanvas

        # Left panel: controls
        self.connection_panel = ConnectionPanel()
        self.left_layout.addWidget(self.connection_panel)

        self.data_panel = DataPanel()
        self.left_layout.addWidget(self.data_panel)

        # Trajectory panel (hidden by default)
        from frontend.panels.trajectory_panel import TrajectoryPanel
        self.trajectory_panel = TrajectoryPanel()
        self.trajectory_panel.setVisible(False)
        self.left_layout.addWidget(self.trajectory_panel)

        self.left_layout.addStretch()

        # Right panel: 3D canvas
        self.arm_canvas = ArmCanvas()
        self.right_layout.addWidget(self.arm_canvas)

        # Setup connection handling
        self.simulator = None
        self.interactive_controller = None
        self.connection_panel.connect_requested.connect(self._on_connect_requested)
        self.connection_panel.disconnect_requested.connect(self._on_disconnect_requested)
        self.connection_panel.mode_changed.connect(self._on_mode_changed)

        # Trajectory panel emits target joint angles (from IK)
        self.trajectory_panel.target_angles_updated.connect(self._on_target_angles)

    def _on_mode_changed(self, mode: str):
        """Show/hide trajectory panel based on mode."""
        if mode == "interactive":
            self.trajectory_panel.setVisible(True)
        else:
            self.trajectory_panel.setVisible(False)

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
        print("DEBUG: Starting interactive mode", file=sys.stderr)
        self.interactive_controller = type('InteractiveCtrl', (), {
            'active': True,
            'target': [0.0, 0.0, 0.0]
        })  # simple namespace
        self.connection_panel.set_connected(True)
        self.status_bar.showMessage("Interactive mode active")
        self.connection_panel.set_status("Use sliders to set target")
        # Set initial target to zero
        self._apply_target_angles(0, 0, 0)

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

    def _on_target_angles(self, q1: float, q2: float, q3: float, q4: float = 0, q5: float = 0, q6: float = 0):
        """Handle target angles from trajectory panel (interactive mode)."""
        print(f"DEBUG: MainWindow received target angles: ({q1}, {q2}, {q3}, {q4}, {q5}, {q6})", file=sys.stderr)
        if self.interactive_controller and self.interactive_controller.active:
            self._apply_target_angles(q1, q2, q3, q4, q5, q6)
        else:
            print("DEBUG: interactive_controller not active, ignoring", file=sys.stderr)

    def _apply_target_angles(self, q1: float, q2: float, q3: float, q4: float = 0, q5: float = 0, q6: float = 0):
        """Apply target angles to arm immediately (no animation)."""
        print(f"DEBUG: _apply_target_angles({q1}, {q2}, {q3}, {q4}, {q5}, {q6})", file=sys.stderr)
        # Update DataPanel with main angles (q1,q2,q3)
        self.data_panel.update_values(q1, q2, q3)
        # Compute forward kinematics with all 6 DOF
        positions = compute_arm_positions(q1, q2, q3, q4, q5, q6)
        self.arm_canvas.draw_arm(positions)
        # Keep trajectory panel in sync
        self.trajectory_panel.set_current_angles(q1, q2, q3, q4, q5, q6)
