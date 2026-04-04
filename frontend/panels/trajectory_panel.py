#!/usr/bin/env python3
"""
TrajectoryPanel - Interactive control for moving the arm to a target position.
Set XYZ coordinates, compute IK, and optionally animate the movement.
"""

from PyQt6.QtWidgets import QGroupBox, QVBoxLayout, QLabel, QSlider, QPushButton, QDoubleSpinBox, QHBoxLayout, QGridLayout
from PyQt6.QtCore import Qt, pyqtSignal
import numpy as np
from backend.kinematics import inverse_kinematics_3dof, ArmConfig


class TrajectoryPanel(QGroupBox):
    """Panel for interactive Cartesian control."""

    target_angles_updated = pyqtSignal(float, float, float)  # q1, q2, q3

    def __init__(self, config: ArmConfig = None, parent=None):
        super().__init__("Trajectory Control (Cartesian)", parent)

        self.config = config if config is not None else ArmConfig()
        self.current_pos = [0.5, 0.0, 0.3]  # natural extended pose

        self.layout = QVBoxLayout(self)
        self.layout.setContentsMargins(10, 10, 10, 10)
        self.layout.setSpacing(12)

        # Description
        desc = QLabel("Set end-effector position (meters). Arm will compute joint angles via IK.")
        desc.setWordWrap(True)
        desc.setStyleSheet("color: #aaa; font-size: 11px; padding: 0;")
        self.layout.addWidget(desc)

        # Workspace ranges (fixed large area for testing, independent of arm size)
        self._create_position_controls()

        # Spacer
        self.layout.addSpacing(10)

        # Set Target button
        self.btn_set = QPushButton("Set Target")
        self.btn_set.setStyleSheet("""
            QPushButton { background-color: #3498db; color: white; font-weight: bold; padding: 12px; min-height: 30px; }
            QPushButton:hover { background-color: #2980b9; }
        """)
        self.btn_set.clicked.connect(self._set_target_clicked)
        self.layout.addWidget(self.btn_set)

        # Animate to Target button
        self.btn_animate = QPushButton("Animate to Target")
        self.btn_animate.setStyleSheet("""
            QPushButton { background-color: #27ae60; color: white; font-weight: bold; padding: 12px; min-height: 30px; }
            QPushButton:hover { background-color: #2ecc71; }
        """)
        self.btn_animate.clicked.connect(self._animate_clicked)
        self.layout.addWidget(self.btn_animate)

        # Stop button
        self.btn_stop = QPushButton("Stop")
        self.btn_stop.setStyleSheet("""
            QPushButton { background-color: #c0392b; color: white; padding: 12px; min-height: 30px; }
            QPushButton:hover { background-color: #e74c3c; }
        """)
        self.btn_stop.clicked.connect(self._stop_clicked)
        self.btn_stop.setEnabled(False)
        self.layout.addWidget(self.btn_stop)

        self.layout.addSpacing(10)

        # Status
        self.lbl_status = QLabel("Idle")
        self.lbl_status.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lbl_status.setStyleSheet("color: #ddd; padding: 5px; font-size: 11px;")
        self.layout.addWidget(self.lbl_status)

        # Stretch to fill remaining space
        self.layout.addStretch()

        self.setStyleSheet("""
            QGroupBox {
                border: 1px solid #444;
                border-radius: 5px;
                margin-top: 10px;
                font-weight: bold;
                padding: 10px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
            }
        """)

        # Animation state
        self.animating = False
        self.animation_timer = None
        self.animation_start_angles = [0.0, 0.0, 0.0]
        self.animation_target_angles = [0.0, 0.0, 0.0]
        # Disable Animate initially (no target set)
        self.btn_animate.setEnabled(False)

    def _create_position_controls(self):
        """Create XYZ controls using a grid layout for perfect alignment."""
        grid = QGridLayout()
        grid.setHorizontalSpacing(10)
        grid.setVerticalSpacing(8)

        # Slider style
        slider_style = """
            QSlider::groove:horizontal { background: #444; height: 6px; border-radius: 3px; }
            QSlider::handle:horizontal { background: #3498db; width: 14px; margin: -4px 0; border-radius: 7px; }
        """
        # Spinbox style
        spin_style = """
            QDoubleSpinBox { background: #333; color: #eee; padding: 4px; border: 1px solid #555; border-radius: 4px; }
        """
        # Label style
        label_style = "color: #ddd; font-weight: bold;"

        # Define axes with ranges and defaults
        axes = [
            ('X', -1.2, 1.2, 0.5),
            ('Y', -1.2, 1.2, 0.0),
            ('Z', 0.0, 1.44, 0.3),
        ]

        for row, (axis, min_val, max_val, default) in enumerate(axes):
            axis_key = axis.lower()

            # Label
            lbl = QLabel(f"{axis}:")
            lbl.setStyleSheet(label_style)
            lbl.setFixedWidth(30)
            grid.addWidget(lbl, row, 0)

            # Slider
            slider = QSlider(Qt.Orientation.Horizontal)
            slider.setRange(int(min_val * 100), int(max_val * 100))
            slider.setValue(int(default * 100))
            slider.setStyleSheet(slider_style)
            grid.addWidget(slider, row, 1)

            # Spinbox
            spinbox = QDoubleSpinBox()
            spinbox.setRange(min_val, max_val)
            spinbox.setValue(default)
            spinbox.setSingleStep(0.01)
            spinbox.setFixedWidth(80)
            spinbox.setStyleSheet(spin_style)
            grid.addWidget(spinbox, row, 2)

            # Connect signals
            slider.valueChanged.connect(lambda v, key=axis_key: self._on_slider_changed(key, v/100.0))
            spinbox.valueChanged.connect(lambda v, key=axis_key: self._on_spinbox_changed(key, v))

            # Store references
            setattr(self, f"slider_{axis_key}", slider)
            setattr(self, f"spin_{axis_key}", spinbox)

        # Add grid to main layout
        self.layout.addLayout(grid)

    def _on_slider_changed(self, axis_key, value):
        spin = getattr(self, f"spin_{axis_key}")
        spin.setValue(value)
        self._update_current_pos()

    def _on_spinbox_changed(self, axis_key, value):
        slider = getattr(self, f"slider_{axis_key}")
        slider.setValue(int(value * 100))
        self._update_current_pos()

    def _update_current_pos(self):
        self.current_pos = [
            self.spin_x.value(),
            self.spin_y.value(),
            self.spin_z.value()
        ]

    def _set_target_clicked(self):
        """Compute IK for current XYZ and emit angles."""
        x, y, z = self.current_pos
        result = inverse_kinematics_3dof(x, y, z, self.config, elbow_down=True)
        if result is None:
            self.lbl_status.setText("Target unreachable")
            self.lbl_status.setStyleSheet("color: #e74c3c;")
            return
        q1, q2, q3 = result
        # Normalize angles to -180..180 range
        q1 = (q1 + 180) % 360 - 180
        q2 = (q2 + 180) % 360 - 180
        q3 = (q3 + 180) % 360 - 180
        self.lbl_status.setText("Target Set")
        self.lbl_status.setStyleSheet("color: #2ecc71;")
        # Enable Animate button now that target is set
        self.btn_animate.setEnabled(True)
        # Emit joint angles
        self.target_angles_updated.emit(float(q1), float(q2), float(q3))

    def _animate_clicked(self):
        """Animate from current arm angles to the computed target."""
        x, y, z = self.current_pos
        result = inverse_kinematics_3dof(x, y, z, self.config, elbow_down=True)
        if result is None:
            self.lbl_status.setText("Target unreachable")
            self.lbl_status.setStyleSheet("color: #e74c3c;")
            return

        q1, q2, q3 = result
        q1, q2, q3 = float(q1), float(q2), float(q3)
        self.animation_target_angles = (q1, q2, q3)
        self.animating = True
        self.btn_animate.setEnabled(False)
        self.btn_set.setEnabled(False)
        self.btn_stop.setEnabled(True)
        self.lbl_status.setText("Animating...")
        self._start_animation()

    def _start_animation(self):
        """Start local animation timer that emits interpolated joint angles."""
        if not hasattr(self, 'current_angles') or self.current_angles is None or len(self.current_angles) < 3:
            self.current_angles = [0.0, 0.0, 0.0]

        self.animation_start_angles = self.current_angles[:]
        self.anim_duration = 2000  # ms
        self._anim_start_time = None
        from PyQt6.QtCore import QTimer
        self.animation_timer = QTimer()
        self.animation_timer.timeout.connect(self._animation_step)
        self.animation_timer.start(33)  # ~30fps

    def _animation_step(self):
        import time
        if self._anim_start_time is None:
            self._anim_start_time = time.perf_counter()

        elapsed = (time.perf_counter() - self._anim_start_time) * 1000  # ms
        t = min(elapsed / self.anim_duration, 1.0)
        # Smooth cosine ease-in-out
        import math
        t = 0.5 - 0.5 * math.cos(math.pi * t)
        start = self.animation_start_angles
        target = self.animation_target_angles
        if len(start) < 3 or len(target) < 3:
            return
        current = [start[i] + (target[i] - start[i]) * t for i in range(3)]
        current_float = [float(v) for v in current]

        self.target_angles_updated.emit(*current_float)
        if t >= 1.0:
            self.animation_timer.stop()
            self.animating = False
            self.btn_animate.setEnabled(True)
            self.btn_set.setEnabled(True)
            self.btn_stop.setEnabled(False)
            self.lbl_status.setText("Target Set")

    def _stop_clicked(self):
        if self.animation_timer:
            self.animation_timer.stop()
        self.animating = False
        self.btn_animate.setEnabled(True)
        self.btn_set.setEnabled(True)
        self.btn_stop.setEnabled(False)
        self.lbl_status.setText("Stopped")

    def set_current_angles(self, q1, q2, q3):
        """Update current arm angles (for animation)."""
        self.current_angles = [q1, q2, q3]

    def update_config(self, new_config: ArmConfig):
        """Update the arm configuration used for IK calculations."""
        self.config = new_config
