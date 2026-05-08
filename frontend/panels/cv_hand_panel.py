#!/usr/bin/env python3
"""
CV Hand Control Panel
=====================
PyQt6 panel that:
 • Starts/stops the CVHandController QThread
 • Shows a live 320×240 webcam thumbnail (JPEG via QLabel/QPixmap)
 • Lets the user pick a joint, adjust sensitivity and dead-zone
 • Applies the computed joint value back to the 3D arm via a signal
 • Records joint-angle sequences and exports them as CSV
"""

from __future__ import annotations

import csv
import time
from pathlib import Path
from typing import List, Optional

from PyQt6.QtCore import Qt, QTimer, pyqtSignal
from PyQt6.QtGui import QImage, QPixmap, QFont, QColor
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QComboBox, QSlider, QFrame, QFileDialog, QMessageBox,
    QSizePolicy, QGridLayout, QGroupBox,
)

from backend.logger import get_logger

_log = get_logger(__name__)

# ── Stylesheet constants ─────────────────────────────────────────────────────
_BTN = """
QPushButton {
    background-color: #202020;
    color: #bfc7d2;
    border: 1px solid #353535;
    border-radius: 4px;
    padding: 5px 10px;
    font-size: 10px;
    font-weight: 600;
    min-height: 26px;
}
QPushButton:hover  { background-color: #2a2a2a; color: #e5e2e1; border-color: #454548; }
QPushButton:pressed { background-color: #131313; }
QPushButton:disabled { color: #3f4850; border-color: #252525; }
"""

_BTN_GREEN = """
QPushButton {
    background: qlineargradient(x1:0,y1:0,x2:1,y2:0,stop:0 #27ae60,stop:1 #2ecc71);
    color: #fff; border: none; border-radius: 4px;
    padding: 5px 12px; font-size: 10px; font-weight: 700; min-height: 28px;
}
QPushButton:hover { background: #27ae60; }
QPushButton:pressed { background: #1e8449; }
"""

_BTN_RED = """
QPushButton {
    background: qlineargradient(x1:0,y1:0,x2:1,y2:0,stop:0 #c0392b,stop:1 #e74c3c);
    color: #fff; border: none; border-radius: 4px;
    padding: 5px 12px; font-size: 10px; font-weight: 700; min-height: 28px;
}
QPushButton:hover { background: #c0392b; }
"""

_BTN_REC = """
QPushButton {
    background-color: #7d1a1a; color: #ff8080;
    border: 1px solid #a93226; border-radius: 4px;
    padding: 5px 10px; font-size: 10px; font-weight: 700; min-height: 26px;
}
QPushButton:hover { background-color: #a93226; color: #fff; }
QPushButton:checked { background-color: #e74c3c; color: #fff; border-color: #ff6b6b; }
"""

_LABEL_MONO = "color: #92ccff; font-family: 'Consolas', monospace; font-size: 12px;"
_LABEL_SMALL = "color: #89929b; font-size: 10px;"

# Gesture → color
GESTURE_COLORS = {
    "ROTATING": "#2ecc71",
    "MOVING"  : "#2ecc71",
    "PAUSED"  : "#f39c12",
    "RESET"   : "#3498db",
    "LOCKED"  : "#e8d44d",
    "ESTOP"   : "#e74c3c",
    "IDLE"    : "#3f4850",
}

SENSITIVITY_PRESETS = {"Low": 0.5, "Medium": 1.0, "High": 2.0}


class CVHandPanel(QWidget):
    """
    CV Hand Gesture Joint Controller panel.

    Signals
    -------
    joint_value_changed(int, float)
        Emitted every frame while CV control is active.
        (joint_index, new_value_deg_or_m)
    """

    joint_delta_changed = pyqtSignal(int, float)   # (joint_index, delta_value)
    estop_triggered     = pyqtSignal()              # two-hands emergency stop

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self._controller = None          # CVHandController QThread
        self._chain      = None          # KinematicChain reference
        self._recording: bool = False
        self._record_buf: List[dict] = []
        self._last_joint_value: float = 0.0
        self._active_joint_idx: int = 0

        self._build_ui()

    # ═══════════════════════════════════════════════════════════════════════
    #  UI construction
    # ═══════════════════════════════════════════════════════════════════════

    def _build_ui(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(8)

        # ── Webcam preview ──────────────────────────────────────────────────
        self._lbl_cam = QLabel()
        self._lbl_cam.setFixedSize(320, 240)
        self._lbl_cam.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._lbl_cam.setStyleSheet(
            "background-color: #0e0e0e; border: 1px solid #353535; border-radius: 4px;"
        )
        self._lbl_cam.setText("[ Camera Off ]")
        self._lbl_cam.setStyleSheet(
            "background-color: #0e0e0e; border: 1px solid #353535; border-radius: 4px;"
            "color: #3f4850; font-size: 11px;"
        )
        cam_row = QHBoxLayout()
        cam_row.addStretch()
        cam_row.addWidget(self._lbl_cam)
        cam_row.addStretch()
        root.addLayout(cam_row)

        # ── Gesture status badge ────────────────────────────────────────────
        self._lbl_gesture = QLabel("IDLE")
        self._lbl_gesture.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._lbl_gesture.setFixedHeight(30)
        self._lbl_gesture.setStyleSheet(
            "background-color: #0e0e0e; color: #3f4850;"
            "font-size: 13px; font-weight: 700; letter-spacing: 0.1em;"
            "border-radius: 4px; border: 1px solid #252525;"
        )
        root.addWidget(self._lbl_gesture)

        # ── Joint angle live readout ────────────────────────────────────────
        self._lbl_value = QLabel("Joint Rate: — ")
        self._lbl_value.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._lbl_value.setStyleSheet(_LABEL_MONO)
        root.addWidget(self._lbl_value)

        # ── Controls grid ───────────────────────────────────────────────────
        grid = QGridLayout()
        grid.setSpacing(6)
        grid.setColumnStretch(1, 1)

        # Joint selector
        grid.addWidget(self._mklbl("Joint:"), 0, 0)
        self._combo_joint = QComboBox()
        self._combo_joint.setStyleSheet(
            "QComboBox { background:#202020; color:#e5e2e1; border:1px solid #353535;"
            "border-radius:4px; padding:3px 8px; font-size:10px; }"
            "QComboBox::drop-down { border:none; }"
            "QComboBox QAbstractItemView { background:#202020; color:#e5e2e1; "
            "selection-background-color:#3498db; }"
        )
        self._combo_joint.currentIndexChanged.connect(self._on_joint_changed)
        grid.addWidget(self._combo_joint, 0, 1)

        # Sensitivity
        grid.addWidget(self._mklbl("Sensitivity:"), 1, 0)
        self._combo_sens = QComboBox()
        self._combo_sens.addItems(list(SENSITIVITY_PRESETS.keys()))
        self._combo_sens.setCurrentIndex(1)  # Medium
        self._combo_sens.setStyleSheet(self._combo_joint.styleSheet())
        self._combo_sens.currentTextChanged.connect(self._on_sensitivity_changed)
        grid.addWidget(self._combo_sens, 1, 1)

        # Dead zone slider
        grid.addWidget(self._mklbl("Dead Zone:"), 2, 0)
        dz_row = QHBoxLayout()
        self._slider_dz = QSlider(Qt.Orientation.Horizontal)
        self._slider_dz.setRange(0, 15)
        self._slider_dz.setValue(3)
        self._slider_dz.setStyleSheet(
            "QSlider::groove:horizontal { background:#252525; height:4px; border-radius:2px; }"
            "QSlider::handle:horizontal { background:#3498db; width:14px; height:14px;"
            "margin:-5px 0; border-radius:7px; }"
            "QSlider::sub-page:horizontal { background:#3498db; border-radius:2px; }"
        )
        self._slider_dz.valueChanged.connect(self._on_dz_changed)
        self._lbl_dz = QLabel("0.03")
        self._lbl_dz.setStyleSheet(_LABEL_SMALL)
        self._lbl_dz.setFixedWidth(32)
        dz_row.addWidget(self._slider_dz)
        dz_row.addWidget(self._lbl_dz)
        grid.addLayout(dz_row, 2, 1)

        # Camera index
        grid.addWidget(self._mklbl("Camera:"), 3, 0)
        self._combo_cam = QComboBox()
        self._combo_cam.addItems(["0 (Default)", "1", "2", "3"])
        self._combo_cam.setStyleSheet(self._combo_joint.styleSheet())
        grid.addWidget(self._combo_cam, 3, 1)

        root.addLayout(grid)

        # ── Start / Stop ────────────────────────────────────────────────────
        btn_row = QHBoxLayout()
        self._btn_start = QPushButton("▶  Start CV Control")
        self._btn_start.setStyleSheet(_BTN_GREEN)
        self._btn_start.clicked.connect(self._start_cv)

        self._btn_stop = QPushButton("■  Stop")
        self._btn_stop.setStyleSheet(_BTN_RED)
        self._btn_stop.setEnabled(False)
        self._btn_stop.clicked.connect(self._stop_cv)

        btn_row.addWidget(self._btn_start)
        btn_row.addWidget(self._btn_stop)
        root.addLayout(btn_row)

        # ── Record / Export ─────────────────────────────────────────────────
        rec_row = QHBoxLayout()
        self._btn_record = QPushButton("⏺  Record")
        self._btn_record.setCheckable(True)
        self._btn_record.setStyleSheet(_BTN_REC)
        self._btn_record.clicked.connect(self._toggle_record)
        self._btn_record.setEnabled(False)

        self._btn_export = QPushButton("⬇  Export CSV")
        self._btn_export.setStyleSheet(_BTN)
        self._btn_export.clicked.connect(self._export_csv)
        self._btn_export.setEnabled(False)

        rec_row.addWidget(self._btn_record)
        rec_row.addWidget(self._btn_export)
        root.addLayout(rec_row)

        # ── Help / Instructions ──────────────────────────────────────────────
        help_row = QHBoxLayout()
        help_row.addStretch()
        self._btn_help = QPushButton("ℹ How to use CV Control")
        self._btn_help.setStyleSheet("""
            QPushButton { background: transparent; color: #3498db; border: none; font-size: 11px; text-decoration: underline; }
            QPushButton:hover { color: #5dade2; }
        """)
        self._btn_help.clicked.connect(self._show_help)
        help_row.addWidget(self._btn_help)
        help_row.addStretch()
        root.addLayout(help_row)

        # ── Finger-count hint ───────────────────────────────────────────────
        hint = QLabel("💡  Show N fingers to jump to Joint N")
        hint.setStyleSheet("color: #3f4850; font-size: 9px; font-style: italic;")
        hint.setAlignment(Qt.AlignmentFlag.AlignCenter)
        root.addWidget(hint)

        root.addStretch()

    @staticmethod
    def _mklbl(text: str) -> QLabel:
        lbl = QLabel(text)
        lbl.setStyleSheet("color: #89929b; font-size: 10px; font-weight: 600;")
        return lbl

    # ═══════════════════════════════════════════════════════════════════════
    #  Public API
    # ═══════════════════════════════════════════════════════════════════════

    def set_chain(self, chain):
        """Update the joint list from a KinematicChain."""
        self._chain = chain
        self._combo_joint.blockSignals(True)
        self._combo_joint.clear()
        if chain:
            for i, j in enumerate(chain.joints):
                label = f"J{i+1}: {j.name} ({j.type[:3].upper()})"
                self._combo_joint.addItem(label, userData=i)
        self._combo_joint.blockSignals(False)
        self._on_joint_changed(0)

    def set_standard_joints(self, config):
        """Populate joint list for Standard 3-DOF mode from ArmConfig."""
        from backend.kinematics import ArmConfig, KinematicChain
        chain = KinematicChain.create_3dof_arm(config)
        self.set_chain(chain)

    # ═══════════════════════════════════════════════════════════════════════
    #  Slots — UI controls
    # ═══════════════════════════════════════════════════════════════════════

    def _on_joint_changed(self, index: int):
        if self._controller is None:
            return
        self._active_joint_idx = index
        joint = self._get_joint(index)
        self._controller.set_joint(joint, index)

    def _on_sensitivity_changed(self, text: str):
        val = SENSITIVITY_PRESETS.get(text, 1.0)
        if self._controller:
            self._controller.set_sensitivity(val)

    def _on_dz_changed(self, raw: int):
        dz = raw / 100.0
        self._lbl_dz.setText(f"{dz:.2f}")
        if self._controller:
            self._controller.set_dead_zone(dz)

    def _get_joint(self, index: int):
        if self._chain is None:
            return None
        if 0 <= index < len(self._chain.joints):
            return self._chain.joints[index]
        return None

    # ═══════════════════════════════════════════════════════════════════════
    #  CV start / stop
    # ═══════════════════════════════════════════════════════════════════════

    def _start_cv(self):
        from backend.cv_hand_controller import CVHandController
        cam_idx = int(self._combo_cam.currentText().split()[0])
        self._controller = CVHandController(camera_index=cam_idx)

        # Wire initial params
        sens = SENSITIVITY_PRESETS.get(self._combo_sens.currentText(), 1.0)
        dz   = self._slider_dz.value() / 100.0
        self._controller.set_sensitivity(sens)
        self._controller.set_dead_zone(dz)

        joint_idx = self._combo_joint.currentIndex()
        self._active_joint_idx = joint_idx
        self._controller.set_joint(self._get_joint(joint_idx), joint_idx)

        self._controller.hand_frame.connect(self._on_hand_frame,
                                            Qt.ConnectionType.QueuedConnection)
        self._controller.error_occurred.connect(self._on_cv_error,
                                                 Qt.ConnectionType.QueuedConnection)
        self._controller.start()

        self._btn_start.setEnabled(False)
        self._btn_stop.setEnabled(True)
        self._btn_record.setEnabled(True)

    def _stop_cv(self):
        if self._controller:
            self._controller.stop()
            self._controller.wait(3000)
            self._controller = None

        self._btn_start.setEnabled(True)
        self._btn_stop.setEnabled(False)
        self._btn_record.setEnabled(False)
        if self._btn_record.isChecked():
            self._btn_record.setChecked(False)
            self._recording = False
            self._btn_export.setEnabled(bool(self._record_buf))

        # Clear preview
        self._lbl_cam.setText("[ Camera Off ]")
        self._set_gesture_label("IDLE")
        self._lbl_value.setText("Joint Rate: — ")

    # ═══════════════════════════════════════════════════════════════════════
    #  Hand frame handler
    # ═══════════════════════════════════════════════════════════════════════

    def _on_hand_frame(self, frame):
        """Slot called from the CVHandController signal (queued — safe for GUI)."""
        from backend.cv_hand_controller import GESTURE_ESTOP

        # ── Update webcam preview ──────────────────────────────────────────
        if frame.frame_jpeg:
            qimg = QImage.fromData(frame.frame_jpeg, "JPEG")
            if not qimg.isNull():
                pix = QPixmap.fromImage(qimg)
                self._lbl_cam.setPixmap(pix)

        # ── Gesture badge ──────────────────────────────────────────────────
        self._set_gesture_label(frame.gesture)

        # ── Finger-count → auto joint switch ──────────────────────────────
        n = frame.finger_count
        if 1 <= n <= self._combo_joint.count():
            jidx = n - 1
            if jidx != self._combo_joint.currentIndex():
                self._combo_joint.setCurrentIndex(jidx)

        # ── Joint value readout ────────────────────────────────────────────
        joint = self._get_joint(self._active_joint_idx)
        unit = "°/s" if (joint and joint.type == "revolute") else " m/s"
        rate = frame.delta_value * 30.0
        self._lbl_value.setText(f"Joint Rate: {rate:+.2f}{unit}")
        self._last_joint_value += frame.delta_value  # Track approx accumulated if needed

        # ── Emit to main window ───────────────────────────────────────────
        self.joint_delta_changed.emit(self._active_joint_idx, frame.delta_value)

        # ── Emergency stop ────────────────────────────────────────────────
        if frame.gesture == GESTURE_ESTOP:
            self.estop_triggered.emit()

        # ── Recording ────────────────────────────────────────────────────
        if self._recording:
            self._record_buf.append({
                "t": frame.t,
                "joint_index": self._active_joint_idx,
                "delta_value": frame.delta_value,
                "gesture": frame.gesture,
                "tilt_deg": frame.tilt_deg,
                "palm_y": frame.palm_y_norm,
                "finger_count": frame.finger_count,
            })

    def _on_cv_error(self, msg: str):
        QMessageBox.critical(self, "CV Hand Control Error", msg)
        self._stop_cv()

    def _set_gesture_label(self, gesture: str):
        color = GESTURE_COLORS.get(gesture, "#3f4850")
        self._lbl_gesture.setText(gesture)
        self._lbl_gesture.setStyleSheet(
            f"background-color: #0e0e0e; color: {color};"
            "font-size: 13px; font-weight: 700; letter-spacing: 0.1em;"
            "border-radius: 4px; border: 1px solid #252525;"
        )

    def _show_help(self):
        msg = QMessageBox(self)
        msg.setWindowTitle("CV Hand Control Instructions")
        msg.setIcon(QMessageBox.Icon.Information)
        msg.setText("<b>How to use Computer Vision (CV) Hand Control</b>")
        msg.setInformativeText(
            "<b>1. Switching Joints</b>\n"
            "Hold up 1 to 6 fingers to automatically switch control to Joint 1 through 6. "
            "For example, hold up 2 fingers to control Joint 2.\n\n"
            "<b>2. Moving the Robot (Joystick Mode)</b>\n"
            "• <b>Revolute Joints:</b> Tilt your hand left or right like a joystick. The robot will move continuously in that direction. Level your hand to stop.\n"
            "• <b>Prismatic Joints:</b> Move your palm up or down. Return your hand to the center of the camera to stop.\n\n"
            "<b>3. Gestures</b>\n"
            "• <b>Open Palm (5 fingers):</b> ⏸ PAUSE. Freezes the robot completely. Useful for resting your hand.\n"
            "• <b>Pinch (Thumb & Index touching):</b> 🔒 LOCKED. Pauses movement temporarily.\n"
            "• <b>Closed Fist (0 fingers):</b> 🔄 RESET. Stops movement and resets internal smoothing filters.\n"
            "• <b>Two Hands in frame:</b> 🛑 E-STOP. Triggers an emergency stop and completely freezes the robot.\n\n"
            "<b>Tips:</b>\n"
            "• Adjust the <i>Dead Zone</i> slider if the robot jitters when your hand is level.\n"
            "• Adjust <i>Sensitivity</i> if the robot moves too slow or too fast."
        )
        msg.setStyleSheet("QLabel { font-size: 12px; color: #e5e2e1; min-width: 400px; }")
        msg.exec()

    # ═══════════════════════════════════════════════════════════════════════
    #  Recording
    # ═══════════════════════════════════════════════════════════════════════

    def _toggle_record(self, checked: bool):
        self._recording = checked
        if checked:
            self._record_buf.clear()
            self._btn_export.setEnabled(False)
        else:
            self._btn_export.setEnabled(bool(self._record_buf))

    def _export_csv(self):
        if not self._record_buf:
            QMessageBox.information(self, "Export", "No recorded data to export.")
            return
        path, _ = QFileDialog.getSaveFileName(
            self, "Export CV Recording", "cv_recording.csv",
            "CSV Files (*.csv)"
        )
        if not path:
            return
        try:
            with open(path, "w", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=list(self._record_buf[0].keys()))
                writer.writeheader()
                writer.writerows(self._record_buf)
            QMessageBox.information(
                self, "Export Complete",
                f"Saved {len(self._record_buf)} frames to:\n{path}"
            )
        except OSError as e:
            QMessageBox.critical(self, "Export Error", str(e))

    # ═══════════════════════════════════════════════════════════════════════
    #  Cleanup
    # ═══════════════════════════════════════════════════════════════════════

    def closeEvent(self, event):
        self._stop_cv()
        super().closeEvent(event)
