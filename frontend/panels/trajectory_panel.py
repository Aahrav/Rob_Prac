#!/usr/bin/env python3
"""
TrajectoryPanel - Interactive control for moving the arm to a target position.
Set XYZ coordinates, compute IK, and optionally animate the movement.
"""

from PyQt6.QtWidgets import QGroupBox, QVBoxLayout, QLabel, QSlider, QPushButton, QDoubleSpinBox, QHBoxLayout
from PyQt6.QtCore import Qt, pyqtSignal
import numpy as np
from backend.kinematics import inverse_kinematics_3dof, ArmConfig


class TrajectoryPanel(QGroupBox):
    """Panel for interactive Cartesian control."""

    target_angles_updated = pyqtSignal(float, float, float)  # q1, q2, q3 (degrees)

    def __init__(self, parent=None):
        super().__init__("Trajectory Control (Cartesian)", parent)

        self.config = ArmConfig()
        self.current_pos = [0.0, 0.0, self.config.base_height + self.config.upper_arm_length + self.config.lower_arm_length + self.config.gripper_offset]  # start at full reach forward
        # Actually default: (0, 0, h0 + L1+L2+Lg) along X direction because q1=0, q2=0, q3=0 gives arm straight along X. So x = L1+L2+Lg, y=0, z=h0. That's good.

        self.layout = QVBoxLayout(self)

        # Description
        desc = QLabel("Set end-effector position (meters). Arm will compute joint angles via IK.")
        desc.setWordWrap(True)
        desc.setStyleSheet("color: #aaa; font-size: 11px; padding: 5px;")
        self.layout.addWidget(desc)

        # XYZ controls with sliders and spinboxes
        self._create_position_control("X", -0.5, 0.5, 0.0, 0.01)
        self._create_position_control("Y", -0.5, 0.5, 0.0, 0.01)
        self._create_position_control("Z", 0.0, 1.0, self.config.base_height + self.config.upper_arm_length + self.config.lower_arm_length + self.config.gripper_offset, 0.01)

        # Set Target button
        self.btn_set = QPushButton("Set Target")
        self.btn_set.setStyleSheet("""
            QPushButton { background-color: #3498db; color: white; font-weight: bold; padding: 10px; }
            QPushButton:hover { background-color: #2980b9; }
        """)
        self.btn_set.clicked.connect(self._set_target_clicked)
        self.layout.addWidget(self.btn_set)

        # Animate to Target button
        self.btn_animate = QPushButton("Animate to Target")
        self.btn_animate.setStyleSheet("""
            QPushButton { background-color: #27ae60; color: white; font-weight: bold; padding: 10px; }
            QPushButton:hover { background-color: #2ecc71; }
        """)
        self.btn_animate.clicked.connect(self._animate_clicked)
        self.layout.addWidget(self.btn_animate)

        # Stop button
        self.btn_stop = QPushButton("Stop")
        self.btn_stop.setStyleSheet("""
            QPushButton { background-color: #c0392b; color: white; padding: 10px; }
            QPushButton:hover { background-color: #e74c3c; }
        """)
        self.btn_stop.clicked.connect(self._stop_clicked)
        self.btn_stop.setEnabled(False)
        self.layout.addWidget(self.btn_stop)

        # Status
        self.lbl_status = QLabel("Ready")
        self.lbl_status.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lbl_status.setStyleSheet("color: #ddd; padding: 5px;")
        self.layout.addWidget(self.lbl_status)

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

    def _create_position_control(self, axis, min_val, max_val, default, step):
        hbox = QHBoxLayout()
        lbl = QLabel(f"{axis}:")
        lbl.setFixedWidth(30)
        hbox.addWidget(lbl)

        slider = QSlider(Qt.Orientation.Horizontal)
        slider.setRange(int(min_val*100), int(max_val*100))
        slider.setValue(int(default*100))
        slider.valueChanged.connect(lambda v, a=axis: self._on_slider_changed(a, v/100.0))
        hbox.addWidget(slider)

        spinbox = QDoubleSpinBox()
        spinbox.setRange(min_val, max_val)
        spinbox.setValue(default)
        spinbox.setSingleStep(step)
        spinbox.setFixedWidth(70)
        spinbox.valueChanged.connect(lambda v, a=axis: self._on_spinbox_changed(a, v))
        hbox.addWidget(spinbox)

        setattr(self, f"slider_{axis.lower()}", slider)
        setattr(self, f"spin_{axis.lower()}", spinbox)
        self.layout.addLayout(hbox)

    def _on_slider_changed(self, axis, value):
        spin = getattr(self, f"spin_{axis.lower()}")
        spin.setValue(value)
        self._update_current_pos()

    def _on_spinbox_changed(self, axis, value):
        slider = getattr(self, f"slider_{axis.lower()}")
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
        self.lbl_status.setText(f"IK: q1={q1:.1f} q2={q2:.1f} q3={q3:.1f}")
        self.lbl_status.setStyleSheet("color: #2ecc71;")
        self.target_angles_updated.emit(q1, q2, q3)

    def _animate_clicked(self):
        """Animate from current arm angles to the computed target."""
        # We need current arm angles. We'll get them from MainWindow later via set_current_angles.
        # For now, just compute target angles and store them for animation.
        x, y, z = self.current_pos
        result = inverse_kinematics_3dof(x, y, z, self.config, elbow_down=True)
        if result is None:
            self.lbl_status.setText("Target unreachable")
            self.lbl_status.setStyleSheet("color: #e74c3c;")
            return
        self.animation_target_angles = result
        self.animating = True
        self.btn_animate.setEnabled(False)
        self.btn_stop.setEnabled(True)
        self.lbl_status.setText("Animating...")
        # Will be handled by MainWindow updating current_angles each step
        # We'll need MainWindow to drive animation or we can do it here. Simpler: MainWindow owns the animation timer.
        # For now, just emit a one-shot? Actually we need smooth interpolation. We'll have MainWindow connect to target_angles_updated and handle animation.
        # But currently the panel's target_angles_updated is emitted on _set_target_clicked. For animation, we could emit intermediate targets over time.
        # Better: Move animation logic to MainWindow when in interactive mode. Let's have MainWindow handle animation when a new target arrives if animation is active.
        # However, to keep panel independent, we could implement local QTimer here. But MainWindow needs to know to update its current_angles.
        # For simplicity, let's not have separate animate button; the Set Target already moves instantly. We'll add smooth animation later. For now, just call set.
        # But the user asked for animation. Let's implement a simple local timer that emits interpolated angles.
        self._start_animation()

    def _start_animation(self):
        """Start local animation timer that emits interpolated joint angles."""
        # Need start angles (current arm state). We'll require MainWindow to call set_current_angles to update us.
        if not hasattr(self, 'current_angles') or self.current_angles is None:
            self.lbl_status.setText("No current arm angles")
            return

        self.animation_start_angles = self.current_angles[:]
        self.anim_duration = 2000  # ms
        self._anim_start_time = None
        from PyQt6.QtCore import QTimer
        self.animation_timer = QTimer()
        self.animation_timer.timeout.connect(self._animation_step)
        self.animation_timer.start(16)  # ~60fps

    def _animation_step(self):
        import time
        if self._anim_start_time is None:
            self._anim_start_time = time.time()
        elapsed = (time.time() - self._anim_start_time) * 1000
        t = min(elapsed / self.anim_duration, 1.0)
        # Ease in-out
        t = 3*t*t - 2*t*t*t
        # Interpolate each joint angle
        start = self.animation_start_angles
        target = self.animation_target_angles
        current = [start[i] + (target[i] - start[i]) * t for i in range(3)]
        self.target_angles_updated.emit(*current)
        if t >= 1.0:
            self.animation_timer.stop()
            self.animating = False
            self.btn_animate.setEnabled(True)
            self.btn_stop.setEnabled(False)
            self.lbl_status.setText("Complete")

    def _stop_clicked(self):
        if self.animation_timer:
            self.animation_timer.stop()
        self.animating = False
        self.btn_animate.setEnabled(True)
        self.btn_stop.setEnabled(False)
        self.lbl_status.setText("Stopped")

    def set_current_angles(self, q1, q2, q3):
        """Update current arm angles (called by MainWindow)."""
        self.current_angles = [q1, q2, q3]
        # If not animating and we have a target, maybe update sliders? Not necessary.
