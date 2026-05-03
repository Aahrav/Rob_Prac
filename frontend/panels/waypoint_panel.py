#!/usr/bin/env python3
"""
WaypointPanel - Manage a list of waypoints for trajectory planning.
Each waypoint is an (x, y, z) position with optional wrist angles.
Kinetic Precision theme.
"""

from PyQt6.QtWidgets import QWidget, QVBoxLayout, QListWidget, QPushButton, QHBoxLayout, QDoubleSpinBox, QLabel, QMessageBox, QFrame
from PyQt6.QtCore import pyqtSignal
import numpy as np


# ── Style tokens ─────────────────────────────────────────────────────────────
_WP_BTN_PRIMARY = """
    QPushButton {
        background: qlineargradient(x1:0,y1:0,x2:1,y2:0, stop:0 #3498db, stop:1 #2980b9);
        color: #ffffff;
        font-family: 'Space Grotesk', 'Inter', sans-serif;
        font-weight: 700; font-size: 11px;
        padding: 8px 14px;
        border: 1px solid rgba(146, 204, 255, 0.1);
        border-radius: 6px;
        letter-spacing: 0.02em;
    }
    QPushButton:hover { background: qlineargradient(x1:0,y1:0,x2:1,y2:0, stop:0 #2980b9, stop:1 #1f6aa5); border-color: rgba(146, 204, 255, 0.2); }
"""

_WP_BTN_SUCCESS = """
    QPushButton {
        background: qlineargradient(x1:0,y1:0,x2:1,y2:0, stop:0 #27ae60, stop:1 #219a52);
        color: #ffffff;
        font-family: 'Space Grotesk', 'Inter', sans-serif;
        font-weight: 700; font-size: 11px;
        padding: 8px 12px;
        border: 1px solid rgba(46, 204, 113, 0.1);
        border-radius: 6px;
        letter-spacing: 0.02em;
    }
    QPushButton:hover { background: qlineargradient(x1:0,y1:0,x2:1,y2:0, stop:0 #2ecc71, stop:1 #27ae60); }
"""

_WP_BTN_DANGER = """
    QPushButton {
        background-color: rgba(192, 57, 43, 0.8);
        color: #ffffff;
        font-family: 'Inter', sans-serif;
        font-weight: 600; font-size: 10px;
        padding: 6px 10px;
        border: 1px solid rgba(231, 76, 60, 0.15);
        border-radius: 6px;
    }
    QPushButton:hover { background-color: #e74c3c; }
"""

_WP_BTN_GHOST = """
    QPushButton {
        background-color: transparent;
        color: #89929b;
        border: 1px solid rgba(255, 255, 255, 0.06);
        border-radius: 6px;
        padding: 6px 10px;
        font-family: 'Inter', sans-serif;
        font-size: 10px;
        letter-spacing: 0.02em;
    }
    QPushButton:hover { background-color: rgba(255,255,255,0.03); color: #dbe3ed; border-color: rgba(255,255,255,0.12); }
"""

_WP_BTN_MUTED = """
    QPushButton {
        background-color: rgba(255, 255, 255, 0.04);
        color: #89929b;
        border: 1px solid rgba(255, 255, 255, 0.06);
        border-radius: 6px;
        padding: 6px 10px;
        font-family: 'Inter', sans-serif;
        font-size: 10px;
    }
    QPushButton:hover { background-color: rgba(255,255,255,0.06); color: #bfc7d2; }
"""

_WP_SPIN = """
    QDoubleSpinBox {
        background-color: #0e0e0e;
        color: #e5e2e1;
        padding: 4px 6px;
        border: 1px solid rgba(255, 255, 255, 0.04);
        border-radius: 4px;
        font-family: 'Space Grotesk', 'Consolas', monospace;
        font-size: 11px;
    }
    QDoubleSpinBox:focus { border-color: rgba(52, 152, 219, 0.3); }
    QDoubleSpinBox::up-button, QDoubleSpinBox::down-button { width: 14px; background: #1a1e24; border: none; border-radius: 2px; }
"""

_WP_LIST = """
    QListWidget {
        background-color: #0e0e0e;
        color: #bfc7d2;
        border: 1px solid rgba(255, 255, 255, 0.04);
        border-radius: 6px;
        font-family: 'Space Grotesk', 'Consolas', monospace;
        font-size: 11px;
        padding: 4px;
    }
    QListWidget::item {
        padding: 6px 8px;
        border-bottom: 1px solid rgba(255, 255, 255, 0.03);
    }
    QListWidget::item:selected {
        background-color: rgba(52, 152, 219, 0.15);
        color: #92ccff;
        border-radius: 4px;
    }
    QListWidget::item:hover {
        background-color: rgba(255, 255, 255, 0.03);
    }
"""

_WP_SECTION = "color: #89929b; font-family: 'Space Grotesk', 'Inter', sans-serif; font-size: 10px; font-weight: 600; letter-spacing: 0.08em;"


class WaypointPanel(QWidget):
    """Panel for managing trajectory waypoints."""

    waypoint_selected = pyqtSignal(int)  # index
    waypoints_changed = pyqtSignal(list)  # list of waypoint dicts

    def __init__(self, parent=None):
        super().__init__(parent)

        self.waypoints = []  # each: {'pos': [x,y,z], 'wrist': [q4,q5,q6]}
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(10)

        # Description
        desc = QLabel("Build a trajectory by adding waypoints. Use current arm position or enter custom XYZ.")
        desc.setWordWrap(True)
        desc.setStyleSheet("color: #89929b; font-family: 'Inter', sans-serif; font-size: 11px;")
        layout.addWidget(desc)

        # "Add Current Position" button
        self.btn_add_current = QPushButton("⊕  Add Current Position")
        self.btn_add_current.setStyleSheet(_WP_BTN_PRIMARY)
        self.btn_add_current.clicked.connect(self._add_current_position)
        layout.addWidget(self.btn_add_current)

        # Custom XYZ input row
        lbl_custom = QLabel("CUSTOM XYZ")
        lbl_custom.setStyleSheet(_WP_SECTION)
        layout.addWidget(lbl_custom)

        hbox_custom = QHBoxLayout()
        hbox_custom.setSpacing(6)
        for axis, default, mn, mx in [('X', 0.0, -0.5, 0.5), ('Y', 0.0, -0.5, 0.5), ('Z', 0.4, 0.0, 1.0)]:
            spin = QDoubleSpinBox()
            spin.setRange(mn, mx)
            spin.setValue(default)
            spin.setSingleStep(0.05)
            spin.setFixedWidth(70)
            spin.setStyleSheet(_WP_SPIN)
            hbox_custom.addWidget(spin)
            setattr(self, f"spin_{axis.lower()}", spin)

        self.btn_add_custom = QPushButton("+ Add")
        self.btn_add_custom.setStyleSheet(_WP_BTN_SUCCESS)
        self.btn_add_custom.clicked.connect(self._add_custom_position)
        hbox_custom.addWidget(self.btn_add_custom)
        layout.addLayout(hbox_custom)

        # Separator
        sep = QFrame()
        sep.setFrameShape(QFrame.Shape.HLine)
        sep.setStyleSheet("color: rgba(255,255,255,0.04); background: rgba(255,255,255,0.04); border: none; max-height: 1px;")
        layout.addWidget(sep)

        # Waypoint list
        lbl_list = QLabel("WAYPOINTS")
        lbl_list.setStyleSheet(_WP_SECTION)
        layout.addWidget(lbl_list)

        self.list_wp = QListWidget()
        self.list_wp.setStyleSheet(_WP_LIST)
        self.list_wp.currentRowChanged.connect(self._on_wp_selected)
        layout.addWidget(self.list_wp)

        # Buttons: Delete, Up, Down, Clear
        hbox_ops = QHBoxLayout()
        hbox_ops.setSpacing(6)

        self.btn_delete = QPushButton("✕ Delete")
        self.btn_delete.setStyleSheet(_WP_BTN_DANGER)
        self.btn_delete.clicked.connect(self._delete_selected)
        hbox_ops.addWidget(self.btn_delete)

        self.btn_up = QPushButton("↑ Up")
        self.btn_up.setStyleSheet(_WP_BTN_GHOST)
        self.btn_up.clicked.connect(self._move_up)
        hbox_ops.addWidget(self.btn_up)

        self.btn_down = QPushButton("↓ Down")
        self.btn_down.setStyleSheet(_WP_BTN_GHOST)
        self.btn_down.clicked.connect(self._move_down)
        hbox_ops.addWidget(self.btn_down)

        self.btn_clear = QPushButton("Clear All")
        self.btn_clear.setStyleSheet(_WP_BTN_MUTED)
        self.btn_clear.clicked.connect(self._clear_all)
        hbox_ops.addWidget(self.btn_clear)
        layout.addLayout(hbox_ops)

    def _add_current_position(self):
        """Add the arm's current end-effector position to waypoints."""
        # We need to get this from somewhere. We'll emit signal to request it,
        # or better: MainWindow will call add_waypoint with actual data.
        # For now, this is a placeholder; MainWindow should call panel.add_waypoint(data).
        pass

    def _add_custom_position(self):
        x = self.spin_x.value()
        y = self.spin_y.value()
        z = self.spin_z.value()
        wp = {'pos': [x, y, z], 'wrist': [0, 0, 0]}
        self._add_waypoint(wp)

    def _add_waypoint(self, wp):
        self.waypoints.append(wp)
        self._refresh_list()
        self.waypoints_changed.emit(self.waypoints)

    def _refresh_list(self):
        self.list_wp.clear()
        for i, wp in enumerate(self.waypoints):
            x, y, z = wp['pos']
            q4, q5, q6 = wp['wrist']
            self.list_wp.addItem(f"{i+1}: ({x:.2f}, {y:.2f}, {z:.2f}) | W: {q4:.0f},{q5:.0f},{q6:.0f}")

    def _on_wp_selected(self, index):
        self.waypoint_selected.emit(index)

    def _delete_selected(self):
        idx = self.list_wp.currentRow()
        if idx >= 0:
            del self.waypoints[idx]
            self._refresh_list()
            self.waypoints_changed.emit(self.waypoints)

    def _move_up(self):
        idx = self.list_wp.currentRow()
        if idx > 0:
            self.waypoints[idx], self.waypoints[idx-1] = self.waypoints[idx-1], self.waypoints[idx]
            self._refresh_list()
            self.list_wp.setCurrentRow(idx-1)
            self.waypoints_changed.emit(self.waypoints)

    def _move_down(self):
        idx = self.list_wp.currentRow()
        if idx >= 0 and idx < len(self.waypoints)-1:
            self.waypoints[idx], self.waypoints[idx+1] = self.waypoints[idx+1], self.waypoints[idx]
            self._refresh_list()
            self.list_wp.setCurrentRow(idx+1)
            self.waypoints_changed.emit(self.waypoints)

    def _clear_all(self):
        self.waypoints.clear()
        self._refresh_list()
        self.waypoints_changed.emit(self.waypoints)

    def get_waypoints(self):
        return self.waypoints.copy()

    def set_current_position_source(self, get_state_func):
        """Store a callback to fetch current (pos, wrist)."""
        self._get_state = get_state_func
        self.btn_add_current.clicked.disconnect()
        self.btn_add_current.clicked.connect(self._add_current_from_source)

    def _add_current_from_source(self):
        if hasattr(self, '_get_state'):
            state = self._get_state()
            if state is not None:
                if isinstance(state, tuple) and len(state) == 2:
                    pos, wrist = state
                else:
                    pos = state
                    wrist = [0,0,0]
                wp = {'pos': pos, 'wrist': wrist}
                self._add_waypoint(wp)
