#!/usr/bin/env python3
"""
TrajectoryPanel - Interactive XYZ Cartesian control + IK animation.
Kinetic Obsidian theme: no QGroupBox, tight grid, horizontal button bar, status chip.
"""

import time
import math
from typing import List, Optional
from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel,
                             QSlider, QPushButton, QDoubleSpinBox, QGridLayout, QFrame)
from PyQt6.QtCore import Qt, pyqtSignal
import numpy as np
from backend.kinematics import inverse_kinematics_3dof, ArmConfig, KinematicChain
from backend.logger import get_logger

log = get_logger(__name__)


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

    target_angles_updated = pyqtSignal(object)  # List of all variable joint values

    def __init__(self, config: ArmConfig = None, chain=None, parent=None):
        super().__init__(parent)

        self.config = config if config is not None else ArmConfig()
        self.current_angles = [0.0, 0.0, 0.0]  # Actual current joint angles
        self.current_pos = [0.5, 0.0, 0.3]
        self.target_pos = None
        self.use_custom_chain = chain is not None
        self.chain = chain
        self.animating = False
        self.animation_timer = None
        self.animation_start_angles = []
        self.animation_target_angles = []
        self._var_joint_indices = None

        self._build_ui()
        
        if chain:
            name = getattr(chain, 'name', '') or "Custom Chain"
            self.lbl_active_chain.setText(f"Active: {name}")
        else:
            self.lbl_active_chain.setText("Active: Standard 3-DOF")
        
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

        self.lbl_active_chain = QLabel("Active: Standard 3-DOF")
        self.lbl_active_chain.setStyleSheet("color: #3498db; font-size: 10px; font-weight: bold; margin-bottom: 5px;")
        layout.addWidget(self.lbl_active_chain)

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

        self.btn_load_csv = QPushButton("📂 Load CSV")
        self.btn_load_csv.setStyleSheet("background-color: #9b59b6; color: white; border-radius: 4px; padding: 4px 8px; font-weight: bold;")
        self.btn_load_csv.setToolTip("Load a CSV of targets to simulate automatically")
        self.btn_load_csv.clicked.connect(self._load_csv_clicked)
        btn_row.addWidget(self.btn_load_csv)

        layout.addLayout(btn_row)

        self.target_queue = []
        from PyQt6.QtCore import QTimer
        self.pause_timer = QTimer()
        self.pause_timer.setSingleShot(True)
        self.pause_timer.timeout.connect(self._process_next_target)

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

    def _load_csv_clicked(self):
        from PyQt6.QtWidgets import QFileDialog, QMessageBox
        import csv

        path, _ = QFileDialog.getOpenFileName(
            self, "Load Target CSV",
            "",
            "CSV files (*.csv);;All files (*)"
        )
        if not path:
            return

        try:
            with open(path, 'r', encoding='utf-8') as f:
                reader = csv.DictReader(f)
                targets = []
                for row in reader:
                    x = float(row.get('x', row.get('X', 0.0)))
                    y = float(row.get('y', row.get('Y', 0.0)))
                    z = float(row.get('z', row.get('Z', 0.0)))
                    targets.append([x, y, z])

            if not targets:
                raise ValueError("CSV is empty or missing 'x', 'y', 'z' columns.")

            self.target_queue = targets
            log.info("Loaded CSV with %d targets", len(targets))
            self.lbl_status.setText(f"⬤  Loaded {len(targets)} targets")
            self.lbl_status.setStyleSheet("color: #9b59b6; font-size: 11px; padding: 4px 0;")
            
            # Start the queue
            self._process_next_target()

        except Exception as exc:
            log.error("Failed to load CSV: %s", exc)
            QMessageBox.critical(self, "Load Error", f"Failed to load CSV:\n{exc}")

    def _process_next_target(self):
        if not self.target_queue:
            self.lbl_status.setText("⬤  Simulation Complete")
            self.lbl_status.setStyleSheet("color: #2ecc71; font-size: 11px; padding: 4px 0;")
            self.btn_load_csv.setEnabled(True)
            return

        self.btn_load_csv.setEnabled(False)
        target = self.target_queue.pop(0)
        
        # Update spinboxes
        self.spin_x.setValue(target[0])
        self.spin_y.setValue(target[1])
        self.spin_z.setValue(target[2])
        self._update_current_pos()
        
        self.target_pos = self.current_pos[:]
        log.info("Processing next queue target: (%.4f, %.4f, %.4f)", *self.target_pos)
        self.lbl_status.setText(f"⬤  Animating to queue target ({len(self.target_queue)} remaining)")
        self.lbl_status.setStyleSheet("color: #f39c12; font-size: 11px; padding: 4px 0;")
        
        self._animate_clicked()

    def _set_target_clicked(self):
        self.target_pos = self.current_pos[:]
        log.info("Target set: (%.4f, %.4f, %.4f)", *self.target_pos)
        self.lbl_status.setText("⬤  Target set")
        self.lbl_status.setStyleSheet("color: #2ecc71; font-size: 11px; padding: 4px 0;")
        self.btn_animate.setEnabled(True)

    def _animate_clicked(self):
        if self.target_pos is None:
            log.warning("Animate clicked but no target set")
            self.lbl_status.setText("⬤  No target set")
            self.lbl_status.setStyleSheet("color: #e74c3c; font-size: 11px; padding: 4px 0;")
            return

        x, y, z = self.target_pos
        log.info(
            "Animate clicked | mode=%s | target=(%.4f, %.4f, %.4f)",
            "custom_chain" if self.use_custom_chain else "standard",
            x, y, z,
        )

        if self.use_custom_chain and self.chain is not None:
            max_reach = self.chain.get_max_reach()
            dist = np.linalg.norm(np.array(self.target_pos) - np.array([0.0, 0.0, self.chain.base_height]))
            if dist > max_reach * 1.05:
                log.warning(
                    "Target out of reach | dist=%.4f (target=%s, base_h=%.2f) | max_reach_limit=%.4f (max_reach=%.4f)",
                    dist, self.target_pos, self.chain.base_height, max_reach * 1.05, max_reach
                )
                self.lbl_status.setText("⬤  Out of reach")
                self.lbl_status.setStyleSheet("color: #e74c3c; font-size: 11px; padding: 4px 0;")
                return
        else:
            max_xy = self.config.upper_arm_length + self.config.lower_arm_length + self.config.gripper_offset
            xy_dist = np.sqrt(x*x + y*y)
            log.debug("Standard reach check | max_xy=%.4f | xy_dist=%.4f", max_xy, xy_dist)
            if xy_dist > max_xy * 1.05:
                log.warning("Target out of XY reach (%.4f > %.4f)", xy_dist, max_xy * 1.05)
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

            log.debug(
                "Custom IK | chain joints=%d | initial_angles=%s",
                len(self.chain.joints),
                [round(a, 3) for a in initial_angles],
            )

            var_indices = [i for i, joint in enumerate(self.chain.joints)
                           if joint.type in ('revolute', 'prismatic')]
            if len(var_indices) < 1:
                log.warning("IK aborted — no variable joints in chain")
                self.lbl_status.setText("⬤  No variable joints")
                self.lbl_status.setStyleSheet("color: #e74c3c; font-size: 11px; padding: 4px 0;")
                return

            full_solution = self.chain.inverse_kinematics(np.array([x, y, z]), initial_angles=initial_angles)
            if full_solution is None:
                log.warning(
                    "IK FAILED for custom chain | target=(%.4f, %.4f, %.4f)",
                    x, y, z,
                )
                self.lbl_status.setText("⬤  IK failed")
                self.lbl_status.setStyleSheet("color: #e74c3c; font-size: 11px; padding: 4px 0;")
                return

            start_vars = [initial_angles[i] for i in var_indices]
            target_vars = [full_solution[i] for i in var_indices]
            log.info(
                "IK solved | var_indices=%s | start=%s | target=%s",
                var_indices,
                [round(v, 3) for v in start_vars],
                [round(v, 3) for v in target_vars],
            )
            self.current_angles = start_vars[:3]
            self.target_angles_updated.emit(start_vars)
            self._var_joint_indices = var_indices
            self.animation_start_angles = start_vars
            self.animation_target_angles = target_vars

        else:
            result = inverse_kinematics_3dof(x, y, z, self.config, elbow_down=True)
            if result is None:
                log.warning("Standard IK failed | target=(%.4f, %.4f, %.4f)", x, y, z)
                self.lbl_status.setText("⬤  Target unreachable")
                self.lbl_status.setStyleSheet("color: #e74c3c; font-size: 11px; padding: 4px 0;")
                return
            q1, q2, q3 = result
            q1 = (q1 + 180) % 360 - 180
            q2 = (q2 + 180) % 360 - 180
            q3 = (q3 + 180) % 360 - 180
            log.info("Standard IK solved | q1=%.3f q2=%.3f q3=%.3f", q1, q2, q3)
            self.animation_target_angles = [q1, q2, q3]

        self.animating = True
        self.btn_animate.setEnabled(False)
        self.btn_set.setEnabled(False)
        self.btn_stop.setEnabled(True)
        log.info("Animation started | start=%s | target=%s",
                 [round(v, 3) for v in self.animation_start_angles] if self.animation_start_angles else "TBD",
                 [round(v, 3) for v in self.animation_target_angles])
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

        log.debug(
            "Animation timer starting | start=%s | target=%s | duration=2000ms",
            [round(v, 3) for v in self.animation_start_angles],
            [round(v, 3) for v in self.animation_target_angles],
        )
        self.anim_duration = 2000
        self._anim_start_time = None
        from PyQt6.QtCore import QTimer
        self.animation_timer = QTimer()
        self.animation_timer.timeout.connect(self._animation_step)
        self.animation_timer.start(16)

    def _animation_step(self):
        if self._anim_start_time is None:
            self._anim_start_time = time.perf_counter()
            log.debug("Animation first tick")

        elapsed = (time.perf_counter() - self._anim_start_time) * 1000
        t = min(elapsed / self.anim_duration, 1.0)
        t = 0.5 - 0.5 * math.cos(math.pi * t)

        start = self.animation_start_angles
        target = self.animation_target_angles
        n = len(start)
        if n == 0:
            log.warning("Animation step called with empty start_angles — aborting")
            return

        current = [start[i] + (target[i] - start[i]) * t for i in range(n)]

        if self.use_custom_chain and self._var_joint_indices:
            for i, joint_idx in enumerate(self._var_joint_indices):
                joint = self.chain.joints[joint_idx]
                if joint.type == 'revolute':
                    joint.theta = current[i]
                elif joint.type == 'prismatic':
                    joint.d = current[i]

        self.target_angles_updated.emit(current)

        if t >= 1.0:
            self.animation_timer.stop()
            self.animating = False
            self.btn_animate.setEnabled(True)
            self.btn_set.setEnabled(True)
            self.btn_stop.setEnabled(False)
            log.info(
                "Animation complete | final_angles=%s",
                [round(v, 3) for v in current],
            )
            self.lbl_status.setText("⬤  Target reached")
            self.lbl_status.setStyleSheet("color: #2ecc71; font-size: 11px; padding: 4px 0;")
            
            if self.target_queue:
                self.pause_timer.start(1000)
            else:
                self.btn_load_csv.setEnabled(True)
                
            self._var_joint_indices = None
            self.animation_start_angles = []

    def _stop_clicked(self):
        if self.animation_timer:
            self.animation_timer.stop()
        if self.pause_timer.isActive():
            self.pause_timer.stop()
        self.target_queue.clear()
        self.animating = False
        self.animation_start_angles = []
        self.btn_animate.setEnabled(True)
        self.btn_set.setEnabled(True)
        self.btn_stop.setEnabled(False)
        self.btn_load_csv.setEnabled(True)
        log.info("Animation stopped by user")
        self.lbl_status.setText("⬤  Stopped")
        self.lbl_status.setStyleSheet("color: #e74c3c; font-size: 11px; padding: 4px 0;")

    def set_current_angles(self, angles: list):
        """Update internal state with current angles from the arm."""
        self.current_angles = list(angles)

    def set_chain(self, chain: KinematicChain):
        self.use_custom_chain = True
        self.chain = chain
        if chain:
            name = getattr(chain, 'name', '') or "Custom Chain"
            self.lbl_active_chain.setText(f"Active: {name}")
        else:
            self.lbl_active_chain.setText("Active: Custom (None)")
        self.update_workspace_ranges()

    def update_config(self, new_config: ArmConfig):
        self.config = new_config
        self.update_workspace_ranges()

    def update_workspace_ranges(self):
        if self.use_custom_chain and self.chain is not None and len(self.chain.joints) > 0:
            total_reach = self.chain.get_max_reach()
            base_h = self.chain.base_height
            self.lbl_workspace.setText(f"Arm Reach: {total_reach:.3f}m | Max Z: {(base_h + total_reach):.3f}m")
            self.lbl_workspace.setStyleSheet("color: #2ecc71; font-weight: 600;")
        else:
            cfg = self.config
            total_reach = cfg.upper_arm_length + cfg.lower_arm_length + cfg.gripper_offset
            base_h = cfg.base_height
            self.lbl_workspace.setText(f"Arm Reach: {total_reach:.2f}m | Max Z: {(base_h + total_reach):.2f}m")
            self.lbl_workspace.setStyleSheet("color: #89929b; font-size: 11px;")

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

        # Finished updating workspace
    def set_target_xyz(self, x: float, y: float, z: Optional[float] = None):
        """External entry point to set target coordinates from Map or other panels."""
        self.spin_x.setValue(x)
        self.spin_y.setValue(y)
        if z is not None:
            self.spin_z.setValue(z)
        self._update_current_pos()
        self._set_target_clicked()  # Automatically "Set" it as the animation target
