#!/usr/bin/env python3
"""
TrajectoryPanel - Interactive XYZ Cartesian control + IK animation.
Kinetic Obsidian theme: no QGroupBox, tight grid, horizontal button bar, status chip.
"""

import time
import math
from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel,
                             QSlider, QPushButton, QDoubleSpinBox, QGridLayout, QFrame)
from PyQt6.QtCore import Qt, pyqtSignal
import numpy as np
from backend.kinematics import inverse_kinematics_3dof, ArmConfig


# ── Style tokens ─────────────────────────────────────────────────────────────
SLIDER_STYLE = """
    QSlider::groove:horizontal {
        background: #202020;
        height: 4px;
        border-radius: 2px;
    }
    QSlider::sub-page:horizontal {
        background: #3498db;
        border-radius: 2px;
    }
    QSlider::handle:horizontal {
        background: #92ccff;
        width: 14px;
        height: 14px;
        margin: -5px 0;
        border-radius: 7px;
    }
    QSlider::handle:horizontal:hover {
        background: #ffffff;
    }
"""

SPIN_STYLE = """
    QDoubleSpinBox {
        background-color: #0e0e0e;
        color: #e5e2e1;
        padding: 4px 6px;
        border: none;
        border-radius: 3px;
        font-size: 11px;
    }
    QDoubleSpinBox:focus {
        border-bottom: 2px solid #3498db;
    }
    QDoubleSpinBox::up-button, QDoubleSpinBox::down-button {
        width: 16px;
        background-color: #202020;
        border: none;
    }
"""

BTN_PRIMARY = """
    QPushButton {
        background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
            stop:0 #3498db, stop:1 #2980b9);
        color: #ffffff; font-weight: 700; font-size: 11px;
        padding: 8px 4px; border: none; border-radius: 5px;
    }
    QPushButton:hover { background-color: #2980b9; }
    QPushButton:disabled { background-color: #353535; color: #89929b; }
"""

BTN_SUCCESS = """
    QPushButton {
        background-color: #27ae60;
        color: #ffffff; font-weight: 700; font-size: 11px;
        padding: 8px 4px; border: none; border-radius: 5px;
    }
    QPushButton:hover { background-color: #2ecc71; }
    QPushButton:disabled { background-color: #353535; color: #89929b; }
"""

BTN_DANGER = """
    QPushButton {
        background-color: #c0392b;
        color: #ffffff; font-weight: 700; font-size: 11px;
        padding: 8px 4px; border: none; border-radius: 5px;
    }
    QPushButton:hover { background-color: #e74c3c; }
    QPushButton:disabled { background-color: #353535; color: #89929b; }
"""

AXIS_LABEL = "color: #92ccff; font-weight: 700; font-size: 12px; font-family: monospace;"
RANGE_LABEL = "color: #3f4850; font-size: 10px;"
SECTION_LABEL = "color: #89929b; font-size: 10px; font-weight: 600; letter-spacing: 0.06em;"


class TrajectoryPanel(QWidget):
    """Panel for interactive Cartesian control."""

    target_angles_updated = pyqtSignal(float, float, float)

    def __init__(self, config: ArmConfig = None, parent=None):
        super().__init__(parent)

        self.config = config if config is not None else ArmConfig()
        self.current_pos = [0.5, 0.0, 0.3]
        self.target_pos = None
        self.chain = None
        self.use_custom_chain = False
        self.animating = False
        self.animation_timer = None
        self.animation_start_angles = []
        self.animation_target_angles = []
        self._var_joint_indices = None

        self._build_ui()
        self.btn_animate.setEnabled(False)
        self.update_workspace_ranges()

    def _build_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(10)

        # Description
        desc = QLabel("Set end-effector position (meters). IK computes joint angles.")
        desc.setWordWrap(True)
        desc.setStyleSheet("color: #89929b; font-size: 11px;")
        layout.addWidget(desc)

        # ── XYZ controls ──────────────────────────────────────────────────
        lbl_target = QLabel("TARGET POSITION")
        lbl_target.setStyleSheet(SECTION_LABEL)
        layout.addWidget(lbl_target)

        grid = QGridLayout()
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(6)

        axes = [('X', -1.2, 1.2, 0.5), ('Y', -1.2, 1.2, 0.0), ('Z', 0.0, 1.44, 0.3)]
        self._range_labels = {}

        for row_idx, (axis, mn, mx, default) in enumerate(axes):
            key = axis.lower()

            # Axis label
            lbl = QLabel(axis)
            lbl.setStyleSheet(AXIS_LABEL)
            lbl.setFixedWidth(20)
            grid.addWidget(lbl, row_idx * 2, 0)

            # Range label
            lbl_range = QLabel(f"[{mn:.2f}, {mx:.2f}]")
            lbl_range.setStyleSheet(RANGE_LABEL)
            grid.addWidget(lbl_range, row_idx * 2, 1, Qt.AlignmentFlag.AlignLeft)
            self._range_labels[key] = lbl_range

            # Spinbox (on same row as label)
            spinbox = QDoubleSpinBox()
            spinbox.setRange(mn, mx)
            spinbox.setValue(default)
            spinbox.setSingleStep(0.01)
            spinbox.setDecimals(3)
            spinbox.setFixedWidth(90)
            spinbox.setStyleSheet(SPIN_STYLE)
            grid.addWidget(spinbox, row_idx * 2, 2)

            # Slider (second sub-row)
            slider = QSlider(Qt.Orientation.Horizontal)
            slider.setRange(int(mn * 1000), int(mx * 1000))
            slider.setValue(int(default * 1000))
            slider.setStyleSheet(SLIDER_STYLE)
            grid.addWidget(slider, row_idx * 2 + 1, 0, 1, 3)

            slider.valueChanged.connect(lambda v, k=key: self._on_slider_changed(k, v / 1000.0))
            spinbox.valueChanged.connect(lambda v, k=key: self._on_spinbox_changed(k, v))

            setattr(self, f"slider_{key}", slider)
            setattr(self, f"spin_{key}", spinbox)

        layout.addLayout(grid)

        # Workspace info
        self.lbl_workspace = QLabel("Max reach: — m")
        self.lbl_workspace.setStyleSheet("color: #89929b; font-size: 11px;")
        layout.addWidget(self.lbl_workspace)

        # ── Separator ─────────────────────────────────────────────────────
        sep = QFrame()
        sep.setFrameShape(QFrame.Shape.HLine)
        sep.setStyleSheet("color: #353535; background: #353535; border: none; max-height: 1px;")
        layout.addWidget(sep)

        # ── Action buttons ────────────────────────────────────────────────
        btn_row = QHBoxLayout()
        btn_row.setSpacing(6)

        self.btn_set = QPushButton("Set Target")
        self.btn_set.setStyleSheet(BTN_PRIMARY)
        self.btn_set.setToolTip("Store current XYZ as animation target")
        self.btn_set.clicked.connect(self._set_target_clicked)
        btn_row.addWidget(self.btn_set)

        self.btn_animate = QPushButton("Animate")
        self.btn_animate.setStyleSheet(BTN_SUCCESS)
        self.btn_animate.setToolTip("Solve IK and animate arm to target")
        self.btn_animate.clicked.connect(self._animate_clicked)
        btn_row.addWidget(self.btn_animate)

        self.btn_stop = QPushButton("Stop")
        self.btn_stop.setStyleSheet(BTN_DANGER)
        self.btn_stop.setToolTip("Halt animation immediately")
        self.btn_stop.clicked.connect(self._stop_clicked)
        self.btn_stop.setEnabled(False)
        btn_row.addWidget(self.btn_stop)

        layout.addLayout(btn_row)

        # ── Status chip ───────────────────────────────────────────────────
        self.lbl_status = QLabel("Idle")
        self.lbl_status.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lbl_status.setStyleSheet("color: #89929b; font-size: 11px; padding: 4px 0;")
        layout.addWidget(self.lbl_status)

    # ── Slot handlers ─────────────────────────────────────────────────────────

    def _on_slider_changed(self, key, value):
        spin = getattr(self, f"spin_{key}")
        spin.blockSignals(True)
        spin.setValue(value)
        spin.blockSignals(False)
        self._update_current_pos()

    def _on_spinbox_changed(self, key, value):
        slider = getattr(self, f"slider_{key}")
        slider.blockSignals(True)
        slider.setValue(int(value * 1000))
        slider.blockSignals(False)
        self._update_current_pos()

    def _update_current_pos(self):
        self.current_pos = [self.spin_x.value(), self.spin_y.value(), self.spin_z.value()]

    def _set_target_clicked(self):
        self.target_pos = self.current_pos[:]
        self.lbl_status.setText("⬤  Target set")
        self.lbl_status.setStyleSheet("color: #2ecc71; font-size: 11px; padding: 4px 0;")
        self.btn_animate.setEnabled(True)

    def _animate_clicked(self):
        if self.target_pos is None:
            self.lbl_status.setText("⬤  No target set")
            self.lbl_status.setStyleSheet("color: #e74c3c; font-size: 11px; padding: 4px 0;")
            return

        x, y, z = self.target_pos

        if self.use_custom_chain and self.chain is not None:
            max_xy = sum(joint.a for joint in self.chain.joints)
            if np.sqrt(x*x + y*y) > max_xy * 1.05:
                self.lbl_status.setText("⬤  Out of reach")
                self.lbl_status.setStyleSheet("color: #e74c3c; font-size: 11px; padding: 4px 0;")
                return
        else:
            max_xy = self.config.upper_arm_length + self.config.lower_arm_length + self.config.gripper_offset
            if np.sqrt(x*x + y*y) > max_xy * 1.05:
                self.lbl_status.setText("⬤  Out of reach")
                self.lbl_status.setStyleSheet("color: #e74c3c; font-size: 11px; padding: 4px 0;")
                return

        if self.use_custom_chain and self.chain is not None:
            initial_angles = []
            for joint in self.chain.joints:
                if joint.type == 'revolute':
                    initial_angles.append(joint.theta)
                elif joint.type == 'prismatic':
                    initial_angles.append(joint.d)
                else:
                    initial_angles.append(0.0)

            var_indices = [i for i, joint in enumerate(self.chain.joints)
                           if joint.type in ('revolute', 'prismatic')]
            if len(var_indices) < 3:
                self.lbl_status.setText("⬤  Need ≥3 variable joints")
                self.lbl_status.setStyleSheet("color: #e74c3c; font-size: 11px; padding: 4px 0;")
                return

            full_solution = self.chain.inverse_kinematics(np.array([x, y, z]), initial_angles=initial_angles)
            if full_solution is None:
                self.lbl_status.setText("⬤  IK failed")
                self.lbl_status.setStyleSheet("color: #e74c3c; font-size: 11px; padding: 4px 0;")
                return

            start_vars = [initial_angles[i] for i in var_indices]
            target_vars = [full_solution[i] for i in var_indices]
            self.current_angles = start_vars[:3]
            self.target_angles_updated.emit(*start_vars[:3])
            self._var_joint_indices = var_indices
            self.animation_start_angles = start_vars
            self.animation_target_angles = target_vars

        else:
            result = inverse_kinematics_3dof(x, y, z, self.config, elbow_down=True)
            if result is None:
                self.lbl_status.setText("⬤  Target unreachable")
                self.lbl_status.setStyleSheet("color: #e74c3c; font-size: 11px; padding: 4px 0;")
                return
            q1, q2, q3 = result
            q1 = (q1 + 180) % 360 - 180
            q2 = (q2 + 180) % 360 - 180
            q3 = (q3 + 180) % 360 - 180
            self.animation_target_angles = [q1, q2, q3]
            self.current_angles = [q1, q2, q3]  # Set current so animation has valid start

        self.animating = True
        self.btn_animate.setEnabled(False)
        self.btn_set.setEnabled(False)
        self.btn_stop.setEnabled(True)
        self.lbl_status.setText("⬤  Animating...")
        self.lbl_status.setStyleSheet("color: #f39c12; font-size: 11px; padding: 4px 0;")
        self._start_animation()

    def _start_animation(self):
        if not self.animation_start_angles:
            if not hasattr(self, 'current_angles') or not self.current_angles or len(self.current_angles) < 3:
                self.current_angles = [0.0, 0.0, 0.0]
            self.animation_start_angles = self.current_angles[:]
        if not self.animation_target_angles:
            self.animation_target_angles = self.animation_start_angles[:]

        self.anim_duration = 2000
        self._anim_start_time = None
        from PyQt6.QtCore import QTimer
        self.animation_timer = QTimer()
        self.animation_timer.timeout.connect(self._animation_step)
        self.animation_timer.start(16)

    def _animation_step(self):
        if self._anim_start_time is None:
            self._anim_start_time = time.perf_counter()

        elapsed = (time.perf_counter() - self._anim_start_time) * 1000
        t = min(elapsed / self.anim_duration, 1.0)
        t = 0.5 - 0.5 * math.cos(math.pi * t)

        start = self.animation_start_angles
        target = self.animation_target_angles
        n = len(start)
        if n == 0:
            return

        current = [start[i] + (target[i] - start[i]) * t for i in range(n)]

        if self.use_custom_chain and self._var_joint_indices:
            for i, joint_idx in enumerate(self._var_joint_indices):
                joint = self.chain.joints[joint_idx]
                if joint.type == 'revolute':
                    joint.theta = current[i]
                elif joint.type == 'prismatic':
                    joint.d = current[i]

        if n >= 3:
            self.target_angles_updated.emit(current[0], current[1], current[2])

        if t >= 1.0:
            self.animation_timer.stop()
            self.animating = False
            self.btn_animate.setEnabled(True)
            self.btn_set.setEnabled(True)
            self.btn_stop.setEnabled(False)
            self.lbl_status.setText("⬤  Target reached")
            self.lbl_status.setStyleSheet("color: #2ecc71; font-size: 11px; padding: 4px 0;")
            self._var_joint_indices = None
            self.animation_start_angles = []

    def _stop_clicked(self):
        if self.animation_timer:
            self.animation_timer.stop()
        self.animating = False
        self.animation_start_angles = []
        self.btn_animate.setEnabled(True)
        self.btn_set.setEnabled(True)
        self.btn_stop.setEnabled(False)
        self.lbl_status.setText("⬤  Stopped")
        self.lbl_status.setStyleSheet("color: #e74c3c; font-size: 11px; padding: 4px 0;")

    def set_current_angles(self, q1, q2, q3):
        self.current_angles = [q1, q2, q3]

    def update_config(self, new_config: ArmConfig):
        self.config = new_config
        self.update_workspace_ranges()

    def update_workspace_ranges(self):
        if self.use_custom_chain and self.chain is not None and len(self.chain.joints) > 0:
            total_reach = sum(joint.a for joint in self.chain.joints)
            total_reach += sum(joint.d for joint in self.chain.joints if joint.type == 'prismatic')
            base_h = self.chain.base_height
        else:
            cfg = self.config
            total_reach = cfg.upper_arm_length + cfg.lower_arm_length + cfg.gripper_offset
            base_h = cfg.base_height

        margin = total_reach * 0.05
        max_xy = total_reach + margin
        min_xy = -max_xy
        min_z = 0.0
        max_z = base_h + total_reach + margin

        for axis in ['x', 'y']:
            slider = getattr(self, f'slider_{axis}')
            spin = getattr(self, f'spin_{axis}')
            slider.setRange(int(min_xy * 1000), int(max_xy * 1000))
            spin.setRange(min_xy, max_xy)
            if axis in self._range_labels:
                self._range_labels[axis].setText(f"[{min_xy:.2f}, {max_xy:.2f}]")

        self.slider_z.setRange(int(min_z * 1000), int(max_z * 1000))
        self.spin_z.setRange(min_z, max_z)
        if 'z' in self._range_labels:
            self._range_labels['z'].setText(f"[{min_z:.2f}, {max_z:.2f}]")

        x, y, z = self.current_pos
        new_x = max(min_xy, min(max_xy, x))
        new_y = max(min_xy, min(max_xy, y))
        new_z = max(min_z, min(max_z, z))
        if new_x != x or new_y != y or new_z != z:
            self.current_pos = [new_x, new_y, new_z]
            self.spin_x.setValue(new_x)
            self.spin_y.setValue(new_y)
            self.spin_z.setValue(new_z)

        self.lbl_workspace.setText(f"Max reach: {total_reach:.2f} m")
