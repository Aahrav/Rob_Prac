#!/usr/bin/env python3
"""
JointCard - Collapsible card for a single joint's DH parameters.
Shows summary when collapsed, full controls when expanded.
"""

from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel,
                              QComboBox, QDoubleSpinBox, QFrame, QSizePolicy, QMessageBox)
from PyQt6.QtCore import pyqtSignal, Qt
from PyQt6.QtGui import QFont

class JointCard(QWidget):
    """Collapsible card for editing a joint's DH parameters."""

    value_changed = pyqtSignal(int)  # joint index
    delete_requested = pyqtSignal(int)
    move_requested = pyqtSignal(int, int)  # from_index, to_index

    def __init__(self, index: int, joint_data: dict, parent=None):
        super().__init__(parent)
        self.index = index
        self.joint_data = joint_data.copy()
        self._expanded = True
        self._init_ui()
        self._update_summary()
        self._load_values_to_inputs()

    def _init_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 6)
        layout.setSpacing(4)

        # Header (summary + expand button)
        header = QFrame()
        header.setStyleSheet("background-color: #2d2d2e; border-radius: 6px; padding: 8px;")
        header_layout = QHBoxLayout(header)
        header_layout.setContentsMargins(8, 4, 8, 4)

        self.summary_label = QLabel()
        self.summary_label.setStyleSheet("color: #ddd; font-size: 12px;")
        header_layout.addWidget(self.summary_label)

        header_layout.addStretch()

        self.btn_expand = QPushButton("−")
        self.btn_expand.setFixedSize(24, 24)
        self.btn_expand.setStyleSheet("""
            QPushButton { background-color: #555; color: #fff; border: none; border-radius: 12px; font-weight: bold; }
            QPushButton:hover { background-color: #666; }
        """)
        self.btn_expand.clicked.connect(self._toggle)
        header_layout.addWidget(self.btn_expand)

        layout.addWidget(header)
        self.header_frame = header

        # Content area (inputs)
        self.content = QWidget()
        content_layout = QVBoxLayout(self.content)
        content_layout.setContentsMargins(0, 0, 0, 0)
        content_layout.setSpacing(8)

        # Type selector (full width)
        row_type = QHBoxLayout()
        lbl_type = QLabel("Type:")
        lbl_type.setFixedWidth(80)
        self.combo_type = QComboBox()
        self.combo_type.addItems(["revolute", "prismatic"])
        self.combo_type.setCurrentText(self.joint_data['type'])
        self.combo_type.currentTextChanged.connect(self._on_type_changed)
        row_type.addWidget(lbl_type)
        row_type.addWidget(self.combo_type)
        content_layout.addLayout(row_type)

        # DH parameters grid: label | spinbox | tooltip hint
        self.spin_theta = self._add_spin(content_layout, "θ (deg):", -180, 180, 0.0, 1.0, "Joint angle (variable for Revolute)")
        self.spin_d = self._add_spin(content_layout, "d (m):", 0, 2, 0.1, 0.01, "Offset along z (variable for Prismatic)")
        self.spin_a = self._add_spin(content_layout, "a (m):", 0, 2, 0.2, 0.01, "Link length along x")
        self.spin_alpha = self._add_spin(content_layout, "α (deg):", -180, 180, 0.0, 1.0, "Twist about x")

        # Buttons row
        row_btns = QHBoxLayout()
        row_btns.setSpacing(6)

        self.btn_up = QPushButton("↑")
        self.btn_up.setFixedSize(28, 28)
        self.btn_up.setToolTip("Move joint up")
        self.btn_up.clicked.connect(lambda: self.move_requested.emit(self.index, self.index-1))
        row_btns.addWidget(self.btn_up)

        self.btn_down = QPushButton("↓")
        self.btn_down.setFixedSize(28, 28)
        self.btn_down.setToolTip("Move joint down")
        self.btn_down.clicked.connect(lambda: self.move_requested.emit(self.index, self.index+1))
        row_btns.addWidget(self.btn_down)

        row_btns.addStretch()

        self.btn_delete = QPushButton("Delete")
        self.btn_delete.setStyleSheet("background-color: #c0392b; color: white; padding: 6px 12px; border-radius: 4px;")
        self.btn_delete.clicked.connect(lambda: self.delete_requested.emit(self.index))
        row_btns.addWidget(self.btn_delete)

        content_layout.addLayout(row_btns)

        layout.addWidget(self.content)

    def _add_spin(self, parent_layout, label: str, min_val, max_val, default, step, tooltip: str):
        row = QHBoxLayout()
        lbl = QLabel(label)
        lbl.setFixedWidth(80)
        spin = QDoubleSpinBox()
        spin.setRange(min_val, max_val)
        spin.setSingleStep(step)
        spin.setDecimals(3)
        spin.setValue(default)
        spin.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        spin.valueChanged.connect(self._on_value_changed)
        spin.setToolTip(tooltip)
        lbl.setToolTip(tooltip)
        row.addWidget(lbl)
        row.addWidget(spin)
        parent_layout.addLayout(row)
        return spin

    def _on_type_changed(self, new_type: str):
        self.joint_data['type'] = new_type
        self._update_summary()
        self.value_changed.emit(self.index)

    def _on_value_changed(self):
        self._read_inputs_to_data()
        self._update_summary()
        self.value_changed.emit(self.index)

    def _read_inputs_to_data(self):
        self.joint_data['type'] = self.combo_type.currentText()
        self.joint_data['theta'] = self.spin_theta.value()
        self.joint_data['d'] = self.spin_d.value()
        self.joint_data['a'] = self.spin_a.value()
        self.joint_data['alpha'] = self.spin_alpha.value()

    def _load_values_to_inputs(self):
        self.combo_type.setCurrentText(self.joint_data['type'])
        self.spin_theta.setValue(self.joint_data.get('theta', 0.0))
        self.spin_d.setValue(self.joint_data.get('d', 0.1))
        self.spin_a.setValue(self.joint_data.get('a', 0.2))
        self.spin_alpha.setValue(self.joint_data.get('alpha', 0.0))

    def _update_summary(self):
        self._read_inputs_to_data()
        jt = self.joint_data['type'][:3]
        theta = self.joint_data['theta']
        a = self.joint_data['a']
        self.summary_label.setText(
            f"<b>Joint {self.index+1}</b> — {jt:3} | θ={theta:6.1f}°, a={a:.3f}m"
        )

    def _toggle(self):
        self._expanded = not self._expanded
        self.content.setVisible(self._expanded)
        self.btn_expand.setText("−" if self._expanded else "+")

    def expand(self):
        if not self._expanded:
            self._toggle()

    def collapse(self):
        if self._expanded:
            self._toggle()

    def set_index(self, idx: int):
        self.index = idx
        self._update_summary()
        # Update move button tooltips? They use self.index directly

    def get_joint_data(self) -> dict:
        self._read_inputs_to_data()
        return self.joint_data.copy()
