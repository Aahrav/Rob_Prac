#!/usr/bin/env python3
"""
Test visualization with simulated data.
Runs without hardware - generates oscillating roll/pitch/yaw.
"""

import sys
from PyQt6.QtWidgets import QApplication, QMainWindow, QWidget, QHBoxLayout, QVBoxLayout, QLabel, QPushButton, QComboBox, QGroupBox, QStatusBar
from PyQt6.QtCore import QTimer, pyqtSignal, QObject
import numpy as np

from backend.kinematics import compute_arm_positions
from frontend.panels.arm_canvas import ArmCanvas


class Simulator(QObject):
    """Generates simulated sensor data (roll, pitch, yaw)."""
    data_updated = pyqtSignal(float, float, float)  # roll, pitch, yaw

    def __init__(self, parent=None):
        super().__init__(parent)
        self.t = 0.0
        self.running = False
        self.timer = QTimer(parent)
        self.timer.timeout.connect(self._update)
        self.interval = 50  # ms (~20Hz)

    def start(self):
        self.running = True
        self.timer.start(self.interval)

    def stop(self):
        self.running = False
        self.timer.stop()

    def _update(self):
        self.t += 0.05
        # Simulate oscillating angles
        roll = 20 * np.sin(self.t * 0.5)
        pitch = 15 * np.sin(self.t * 0.7 + 1)
        yaw = 30 * np.sin(self.t * 0.3 + 2)
        self.data_updated.emit(roll, pitch, yaw)


class DataPanel(QWidget):
    """Displays current Roll/Pitch/Yaw values."""

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)

        self.lbl_roll = QLabel("Roll: 0.0°")
        self.lbl_pitch = QLabel("Pitch: 0.0°")
        self.lbl_yaw = QLabel("Yaw: 0.0°")

        for lbl in [self.lbl_roll, self.lbl_pitch, self.lbl_yaw]:
            lbl.setStyleSheet("font-size: 18px; font-weight: bold; padding: 10px;")
            layout.addWidget(lbl)

        self.setStyleSheet("background-color: #1e1e1e; color: #eee;")

    def update_values(self, roll, pitch, yaw):
        self.lbl_roll.setText(f"Roll: {roll:6.1f}°")
        self.lbl_pitch.setText(f"Pitch: {pitch:6.1f}°")
        self.lbl_yaw.setText(f"Yaw: {yaw:6.1f}°")


class TestWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("AppName - Visualization Test (Simulated)")
        self.resize(1200, 800)

        # Central layout
        central = QWidget()
        self.setCentralWidget(central)
        layout = QHBoxLayout(central)
        layout.setContentsMargins(0, 0, 0, 0)

        # Left panel: controls
        left = QWidget()
        left.setMinimumWidth(300)
        left.setStyleSheet("background-color: #1e1e1e;")
        left_layout = QVBoxLayout(left)

        # Title
        title = QLabel("AppName v0.1 (Simulator)")
        title.setStyleSheet("font-size: 16px; font-weight: bold; padding: 10px; color: #fff;")
        left_layout.addWidget(title)

        # Connection panel (simulated)
        conn_group = QGroupBox("Connection")
        conn_layout = QVBoxLayout(conn_group)
        self.combo_port = QComboBox()
        self.combo_port.addItems(["SIMULATED"])
        self.btn_connect = QPushButton("Connect")
        self.lbl_status = QLabel("Status: Simulated Mode")
        self.lbl_status.setStyleSheet("color: #2ecc71;")
        conn_layout.addWidget(QLabel("Port:"))
        conn_layout.addWidget(self.combo_port)
        conn_layout.addWidget(self.btn_connect)
        conn_layout.addWidget(self.lbl_status)
        left_layout.addWidget(conn_group)

        # Data panel
        self.data_panel = DataPanel()
        left_layout.addWidget(self.data_panel)

        left_layout.addStretch()

        # Right panel: 3D canvas
        self.arm_canvas = ArmCanvas()
        self.arm_canvas.setStyleSheet("background-color: #2d2d2d;")

        layout.addWidget(left)
        layout.addWidget(self.arm_canvas, 1)

        # Status bar
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_bar.showMessage("Simulation active")

        # Simulator
        self.simulator = Simulator(self)
        self.simulator.data_updated.connect(self.on_data)

        # Connect button just starts the simulator
        self.btn_connect.clicked.connect(self.start_simulation)

    def start_simulation(self):
        self.simulator.start()
        self.btn_connect.setEnabled(False)
        self.btn_connect.setText("Connected (Simulated)")
        self.lbl_status.setText("Status: Connected")
        self.lbl_status.setStyleSheet("color: #2ecc71;")

    def on_data(self, roll, pitch, yaw):
        # Update data panel
        self.data_panel.update_values(roll, pitch, yaw)

        # Compute arm positions
        positions = compute_arm_positions(roll, pitch, yaw)

        # Draw arm
        self.arm_canvas.draw_arm(positions)


if __name__ == "__main__":
    app = QApplication(sys.argv)

    # Apply dark theme
    app.setStyleSheet("""
        QMainWindow { background-color: #1e1e1e; }
        QWidget { color: #eee; font-family: Segoe UI, Arial; }
        QGroupBox { border: 1px solid #444; border-radius: 5px; margin-top: 10px; }
        QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; }
        QPushButton { background-color: #3498db; color: white; border: none; padding: 8px; border-radius: 4px; }
        QPushButton:hover { background-color: #2980b9; }
        QPushButton:disabled { background-color: #555; }
        QComboBox { background-color: #333; color: #eee; border: 1px solid #555; padding: 5px; }
        QLabel { color: #ddd; }
    """)

    window = TestWindow()
    window.show()
    sys.exit(app.exec())
