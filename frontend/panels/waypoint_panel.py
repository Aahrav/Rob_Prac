#!/usr/bin/env python3
"""
WaypointPanel - Manage a list of waypoints for trajectory planning.
Each waypoint is an (x, y, z) position with optional wrist angles.
"""

from PyQt6.QtWidgets import QGroupBox, QVBoxLayout, QListWidget, QPushButton, QHBoxLayout, QDoubleSpinBox, QLabel, QMessageBox
from PyQt6.QtCore import pyqtSignal
import numpy as np


class WaypointPanel(QGroupBox):
    """Panel for managing trajectory waypoints."""

    waypoint_selected = pyqtSignal(int)  # index
    waypoints_changed = pyqtSignal(list)  # list of waypoint dicts

    def __init__(self, parent=None):
        super().__init__("Trajectory Waypoints", parent)

        self.waypoints = []  # each: {'pos': [x,y,z], 'wrist': [q4,q5,q6]}
        self.layout = QVBoxLayout(self)

        # Description
        desc = QLabel("Build a trajectory by adding waypoints. Use current arm position or enter custom XYZ.")
        desc.setWordWrap(True)
        desc.setStyleSheet("color: #aaa; font-size: 11px; padding: 5px;")
        self.layout.addWidget(desc)

        # "Add Current Position" button
        self.btn_add_current = QPushButton("Add Current Position")
        self.btn_add_current.setStyleSheet("background-color: #3498db; color: white; padding: 8px;")
        self.btn_add_current.clicked.connect(self._add_current_position)
        self.layout.addWidget(self.btn_add_current)

        # Custom XYZ input row
        hbox_custom = QHBoxLayout()
        hbox_custom.addWidget(QLabel("Custom XYZ:"))
        self.spin_x = QDoubleSpinBox()
        self.spin_x.setRange(-0.5, 0.5)
        self.spin_x.setValue(0.0)
        self.spin_x.setSingleStep(0.05)
        self.spin_x.setFixedWidth(70)
        hbox_custom.addWidget(self.spin_x)
        self.spin_y = QDoubleSpinBox()
        self.spin_y.setRange(-0.5, 0.5)
        self.spin_y.setValue(0.0)
        self.spin_y.setSingleStep(0.05)
        self.spin_y.setFixedWidth(70)
        hbox_custom.addWidget(self.spin_y)
        self.spin_z = QDoubleSpinBox()
        self.spin_z.setRange(0.0, 1.0)
        self.spin_z.setValue(0.4)
        self.spin_z.setSingleStep(0.05)
        self.spin_z.setFixedWidth(70)
        hbox_custom.addWidget(self.spin_z)
        self.btn_add_custom = QPushButton("Add")
        self.btn_add_custom.setStyleSheet("background-color: #27ae60; color: white; padding: 8px;")
        self.btn_add_custom.clicked.connect(self._add_custom_position)
        hbox_custom.addWidget(self.btn_add_custom)
        self.layout.addLayout(hbox_custom)

        # Wrist angles for waypoint (optional - will use sliders values at add time)
        # We'll capture current wrist spinner values when adding

        # Waypoint list
        self.list_wp = QListWidget()
        self.list_wp.setStyleSheet("background-color: #333; color: #ddd;")
        self.list_wp.currentRowChanged.connect(self._on_wp_selected)
        self.layout.addWidget(self.list_wp)

        # Buttons: Delete, Up, Down, Clear
        hbox_ops = QHBoxLayout()
        self.btn_delete = QPushButton("Delete")
        self.btn_delete.setStyleSheet("background-color: #c0392b; color: white; padding: 6px;")
        self.btn_delete.clicked.connect(self._delete_selected)
        hbox_ops.addWidget(self.btn_delete)

        self.btn_up = QPushButton("Up")
        self.btn_up.setStyleSheet("padding: 6px;")
        self.btn_up.clicked.connect(self._move_up)
        hbox_ops.addWidget(self.btn_up)

        self.btn_down = QPushButton("Down")
        self.btn_down.setStyleSheet("padding: 6px;")
        self.btn_down.clicked.connect(self._move_down)
        hbox_ops.addWidget(self.btn_down)

        self.btn_clear = QPushButton("Clear All")
        self.btn_clear.setStyleSheet("background-color: #7f8c8d; color: white; padding: 6px;")
        self.btn_clear.clicked.connect(self._clear_all)
        hbox_ops.addWidget(self.btn_clear)
        self.layout.addLayout(hbox_ops)

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
