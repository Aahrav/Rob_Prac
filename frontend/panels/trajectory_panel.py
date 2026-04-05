#!/usr/bin/env python3
"""
TrajectoryPanel - Interactive control for moving the arm to a target position.
Set XYZ coordinates, compute IK, and optionally animate the movement.
"""

import time
import math
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
        self.target_pos = None

        # Custom DH chain support (optional)
        self.chain = None
        self.use_custom_chain = False

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

        # Workspace info
        self.lbl_workspace = QLabel()
        self.lbl_workspace.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lbl_workspace.setStyleSheet("color: #aaa; padding: 5px; font-size: 11px;")
        self.layout.addWidget(self.lbl_workspace)

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
        self.animation_start_angles = []  # will be set when animation starts
        self.animation_target_angles = []
        self._var_joint_indices = None  # for custom DH: indices of variable joints in chain
        # Disable Animate initially (no target set)
        self.btn_animate.setEnabled(False)

        # Initialize workspace ranges to match default configuration
        self.update_workspace_ranges()

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
        """Store the desired XYZ position as target. No IK solving."""
        self.target_pos = self.current_pos[:]  # copy
        self.lbl_status.setText("Target set")
        self.lbl_status.setStyleSheet("color: #2ecc71;")
        self.btn_animate.setEnabled(True)


    def _animate_clicked(self):
        """Solve IK for stored target and animate movement."""
        if self.target_pos is None:
            self.lbl_status.setText("No target set")
            self.lbl_status.setStyleSheet("color: #e74c3c;")
            return

        x, y, z = self.target_pos

        # Quick reachability validation
        if self.use_custom_chain and self.chain is not None:
            max_xy = sum(joint.a for joint in self.chain.joints)
            if np.sqrt(x*x + y*y) > max_xy * 1.05:
                self.lbl_status.setText("Target out of reach")
                self.lbl_status.setStyleSheet("color: #e74c3c;")
                return
        else:
            max_xy = self.config.upper_arm_length + self.config.lower_arm_length + self.config.gripper_offset
            if np.sqrt(x*x + y*y) > max_xy * 1.05:
                self.lbl_status.setText("Target out of reach")
                self.lbl_status.setStyleSheet("color: #e74c3c;")
                return

        if self.use_custom_chain and self.chain is not None:
            # Build initial angles for all joints (n-length list)
            initial_angles = []
            for joint in self.chain.joints:
                if joint.type == 'revolute':
                    initial_angles.append(joint.theta)
                elif joint.type == 'prismatic':
                    initial_angles.append(joint.d)
                else:
                    initial_angles.append(0.0)

            # Identify all variable joint indices (revolute or prismatic)
            var_indices = [i for i, joint in enumerate(self.chain.joints) if joint.type in ('revolute', 'prismatic')]
            if len(var_indices) < 3:
                self.lbl_status.setText("Need at least 3 variable joints")
                self.lbl_status.setStyleSheet("color: #e74c3c;")
                return

            full_solution = self.chain.inverse_kinematics(np.array([x, y, z]), initial_angles=initial_angles)
            if full_solution is None:
                self.lbl_status.setText("IK failed: unreachable")
                self.lbl_status.setStyleSheet("color: #e74c3c;")
                return

            # Extract start and target values for all variable joints
            start_vars = [initial_angles[i] for i in var_indices]
            target_vars = [full_solution[i] for i in var_indices]

            # Update panel's current_angles for first three and emit to sync UI
            self.current_angles = start_vars[:3]
            self.target_angles_updated.emit(*start_vars[:3])

            # Store for animation
            self._var_joint_indices = var_indices
            self.animation_start_angles = start_vars
            self.animation_target_angles = target_vars

            self.animating = True
            self.btn_animate.setEnabled(False)
            self.btn_set.setEnabled(False)
            self.btn_stop.setEnabled(True)
            self.lbl_status.setText("Animating...")
            self._start_animation()

        else:
            # Standard 3-DOF mode
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
            target_angles = [q1, q2, q3]
            self.animation_target_angles = target_angles
            self.animating = True
            self.btn_animate.setEnabled(False)
            self.btn_set.setEnabled(False)
            self.btn_stop.setEnabled(True)
            self.lbl_status.setText("Animating...")
            self._start_animation()


    def _start_animation(self):
        """Start local animation timer that emits interpolated joint angles."""
        # Only set default start angles if not already configured (e.g., custom mode sets them beforehand)
        if not self.animation_start_angles:
            if not hasattr(self, 'current_angles') or self.current_angles is None or len(self.current_angles) < 3:
                self.current_angles = [0.0, 0.0, 0.0]
            self.animation_start_angles = self.current_angles[:]
        if not self.animation_target_angles:
            # Should have been set; fallback to start
            self.animation_target_angles = self.animation_start_angles[:]

        self.anim_duration = 2000  # ms
        self._anim_start_time = None
        from PyQt6.QtCore import QTimer
        self.animation_timer = QTimer()
        self.animation_timer.timeout.connect(self._animation_step)
        self.animation_timer.start(16)  # ~60fps for smoother motion

    def _animation_step(self):
        if self._anim_start_time is None:
            self._anim_start_time = time.perf_counter()

        elapsed = (time.perf_counter() - self._anim_start_time) * 1000  # ms
        t = min(elapsed / self.anim_duration, 1.0)
        t = 0.5 - 0.5 * math.cos(math.pi * t)  # cosine ease-in-out

        start = self.animation_start_angles
        target = self.animation_target_angles
        n = len(start)
        if n == 0:
            return

        # Interpolate all variable joints
        current = [start[i] + (target[i] - start[i]) * t for i in range(n)]

        # In custom DH mode, apply interpolated values to chain joints
        if self.use_custom_chain and self._var_joint_indices:
            for i, joint_idx in enumerate(self._var_joint_indices):
                joint = self.chain.joints[joint_idx]
                if joint.type == 'revolute':
                    joint.theta = current[i]
                elif joint.type == 'prismatic':
                    joint.d = current[i]

        # Emit first three values for UI (standard and custom)
        if n >= 3:
            self.target_angles_updated.emit(current[0], current[1], current[2])
        # else: not enough to emit; should not happen

        if t >= 1.0:
            self.animation_timer.stop()
            self.animating = False
            self.btn_animate.setEnabled(True)
            self.btn_set.setEnabled(True)
            self.btn_stop.setEnabled(False)
            self.lbl_status.setText("Target Set")
            self._var_joint_indices = None  # cleanup

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
        self.update_workspace_ranges()

    def update_workspace_ranges(self):
        """Compute reachable workspace and update slider/spinbox ranges accordingly."""
        # Compute total reach and base height based on current mode
        if self.use_custom_chain and self.chain is not None and len(self.chain.joints) > 0:
            total_reach = sum(joint.a for joint in self.chain.joints)
            # Add prismatic offsets (current d values)
            total_reach += sum(joint.d for joint in self.chain.joints if joint.type == 'prismatic')
            base_h = self.chain.base_height
        else:
            cfg = self.config
            total_reach = cfg.upper_arm_length + cfg.lower_arm_length + cfg.gripper_offset
            base_h = cfg.base_height

        # Add 5% safety margin
        margin = total_reach * 0.05
        max_xy = total_reach + margin
        min_xy = -max_xy
        min_z = 0.0
        max_z = base_h + total_reach + margin

        # Update X and Y controls
        for axis in ['x', 'y']:
            slider = getattr(self, f'slider_{axis}')
            spin = getattr(self, f'spin_{axis}')
            slider.setRange(int(min_xy * 100), int(max_xy * 100))
            spin.setRange(min_xy, max_xy)

        # Update Z controls
        self.slider_z.setRange(int(min_z * 100), int(max_z * 100))
        self.spin_z.setRange(min_z, max_z)

        # Clamp current position if out of new ranges
        x, y, z = self.current_pos
        new_x = max(min_xy, min(max_xy, x))
        new_y = max(min_xy, min(max_xy, y))
        new_z = max(min_z, min(max_z, z))
        if new_x != x or new_y != y or new_z != z:
            self.current_pos = [new_x, new_y, new_z]
            self.spin_x.setValue(new_x)
            self.spin_y.setValue(new_y)
            self.spin_z.setValue(new_z)

        # Update workspace info label
        self.lbl_workspace.setText(f"Max reach: {total_reach:.2f} m")

