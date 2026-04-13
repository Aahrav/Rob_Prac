#!/usr/bin/env python3
"""
JointCard - Collapsible card for a single joint's DH parameters.
Kinetic Obsidian theme: milled input fields, tonal card layers, minimal borders.
"""

from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QPushButton,
                              QLabel, QDoubleSpinBox, QFrame, QSizePolicy)
from PyQt6.QtCore import pyqtSignal, Qt
from frontend.panels.custom_combo_box import CustomComboBox


# ── Style tokens ─────────────────────────────────────────────────────────────
CARD_BG = "#202020"
HEADER_BG = "#2a2a2a"

SPIN_STYLE = """
    QDoubleSpinBox {
        background-color: #0e0e0e;
        color: #e5e2e1;
        padding: 4px 6px;
        border: none;
        border-radius: 2px;
        font-size: 11px;
        min-width: 70px;
    }
    QDoubleSpinBox:focus {
        border-bottom: 2px solid #3498db;
    }
    QDoubleSpinBox::up-button, QDoubleSpinBox::down-button {
        width: 16px;
        background-color: #1b1b1c;
        border: none;
    }
"""

COMBO_STYLE = """
    QComboBox {
        background-color: #0e0e0e;
        color: #e5e2e1;
        padding: 4px 8px;
        border: none;
        border-radius: 2px;
        font-size: 11px;
    }
    QComboBox::drop-down { border: none; width: 20px; }
    QComboBox::down-arrow { image: none; }
    QComboBox QAbstractItemView {
        background-color: #202020;
        color: #e5e2e1;
        selection-background-color: #3498db;
        border: 1px solid #353535;
    }
"""

BTN_ICON_STYLE = """
    QPushButton {
        background-color: #353535;
        color: #bfc7d2;
        border: none;
        border-radius: 3px;
        font-size: 13px;
        font-weight: bold;
        min-width: 28px;
        max-width: 28px;
        min-height: 24px;
        max-height: 24px;
        padding: 0;
    }
    QPushButton:hover { background-color: #454548; color: #e5e2e1; }
    QPushButton:pressed { background-color: #252527; }
"""

BTN_DELETE_STYLE = """
    QPushButton {
        background-color: transparent;
        color: #e74c3c;
        border: 1px solid #93000a;
        border-radius: 3px;
        font-size: 10px;
        font-weight: 600;
        padding: 3px 10px;
    }
    QPushButton:hover { background-color: #93000a; color: #ffdad6; }
"""

BTN_EXPAND_STYLE = """
    QPushButton {
        background-color: transparent;
        color: #89929b;
        border: none;
        font-size: 14px;
        font-weight: bold;
        min-width: 22px;
        max-width: 22px;
        padding: 0;
    }
    QPushButton:hover { color: #e5e2e1; }
"""

KEY_LABEL = "color: #89929b; font-size: 10px; font-weight: 600; letter-spacing: 0.04em;"
TOOLTIP_STYLE = "color: #3f4850; font-size: 10px;"


class JointCard(QWidget):
    """Collapsible card for editing a joint's DH parameters."""

    value_changed = pyqtSignal(int)
    delete_requested = pyqtSignal(int)
    move_requested = pyqtSignal(int, int)

    def __init__(self, index: int, joint_data: dict, parent=None):
        super().__init__(parent)
        self.index = index
        self.joint_data = joint_data.copy()
        self._expanded = True
        self._init_ui()
        self._update_summary()
        self._load_values_to_inputs()

    def _init_ui(self):
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 4)
        outer.setSpacing(0)

        # ── Card container ───────────────────────────────────────────────
        self.card = QFrame()
        self.card.setStyleSheet(
            f"QFrame {{ background-color: {CARD_BG}; border-radius: 5px; }}"
        )
        card_layout = QVBoxLayout(self.card)
        card_layout.setContentsMargins(0, 0, 0, 0)
        card_layout.setSpacing(0)

        # ── Header ──────────────────────────────────────────────────────
        header = QFrame()
        header.setStyleSheet(
            f"QFrame {{ background-color: {HEADER_BG}; border-radius: 5px 5px 0 0; }}"
        )
        header_layout = QHBoxLayout(header)
        header_layout.setContentsMargins(10, 6, 8, 6)
        header_layout.setSpacing(6)

        self.summary_label = QLabel()
        self.summary_label.setStyleSheet("color: #e5e2e1; font-size: 11px; font-weight: 600;")
        header_layout.addWidget(self.summary_label)
        header_layout.addStretch()

        self.btn_expand = QPushButton("−")
        self.btn_expand.setStyleSheet(BTN_EXPAND_STYLE)
        self.btn_expand.clicked.connect(self._toggle)
        header_layout.addWidget(self.btn_expand)

        card_layout.addWidget(header)
        self.header_frame = header

        # ── Content ──────────────────────────────────────────────────────
        self.content = QFrame()
        self.content.setStyleSheet(f"QFrame {{ background-color: {CARD_BG}; }}")
        content_layout = QVBoxLayout(self.content)
        content_layout.setContentsMargins(10, 8, 10, 8)
        content_layout.setSpacing(6)

        # Type row
        type_row = QHBoxLayout()
        type_lbl = QLabel("TYPE")
        type_lbl.setStyleSheet(KEY_LABEL)
        type_lbl.setFixedWidth(60)
        type_row.addWidget(type_lbl)
        self.combo_type = CustomComboBox()
        self.combo_type.addItems(["revolute", "prismatic", "fixed"])
        self.combo_type.setCurrentText(self.joint_data.get('type', 'revolute'))
        self.combo_type.setStyleSheet(COMBO_STYLE)
        self.combo_type.currentTextChanged.connect(self._on_type_changed)
        type_row.addWidget(self.combo_type)
        content_layout.addLayout(type_row)

        # DH parameters
        params = [
            ("θ", "deg", -180, 180, 0.0,  1.0,  "spin_theta", "Joint angle (variable for Revolute)"),
            ("d", "m",   0,   2.0, 0.1,  0.01, "spin_d",     "Link offset along Z (variable for Prismatic)"),
            ("a", "m",   0,   2.0, 0.2,  0.01, "spin_a",     "Link length along X"),
            ("α", "deg", -180, 180, 0.0,  1.0,  "spin_alpha", "Twist about X"),
        ]
        for key_text, unit, mn, mx, default, step, attr, tooltip in params:
            row_layout = QHBoxLayout()
            row_layout.setSpacing(6)

            lbl = QLabel(key_text)
            lbl.setStyleSheet(KEY_LABEL)
            lbl.setFixedWidth(24)
            lbl.setToolTip(tooltip)
            row_layout.addWidget(lbl)

            lbl_unit = QLabel(unit)
            lbl_unit.setStyleSheet(TOOLTIP_STYLE)
            lbl_unit.setFixedWidth(24)
            row_layout.addWidget(lbl_unit)

            spin = QDoubleSpinBox()
            spin.setRange(mn, mx)
            spin.setSingleStep(step)
            spin.setDecimals(3)
            spin.setValue(default)
            spin.setStyleSheet(SPIN_STYLE)
            spin.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
            spin.valueChanged.connect(self._on_value_changed)
            spin.setToolTip(tooltip)
            row_layout.addWidget(spin)
            setattr(self, attr, spin)

            content_layout.addLayout(row_layout)

        # ── Action buttons row ────────────────────────────────────────────
        sep = QFrame()
        sep.setFrameShape(QFrame.Shape.HLine)
        sep.setStyleSheet("background: #353535; border: none; max-height: 1px;")
        content_layout.addWidget(sep)

        btn_row = QHBoxLayout()
        btn_row.setSpacing(4)

        self.btn_up = QPushButton("↑")
        self.btn_up.setStyleSheet(BTN_ICON_STYLE)
        self.btn_up.setToolTip("Move joint up")
        self.btn_up.clicked.connect(lambda: self.move_requested.emit(self.index, self.index - 1))
        btn_row.addWidget(self.btn_up)

        self.btn_down = QPushButton("↓")
        self.btn_down.setStyleSheet(BTN_ICON_STYLE)
        self.btn_down.setToolTip("Move joint down")
        self.btn_down.clicked.connect(lambda: self.move_requested.emit(self.index, self.index + 1))
        btn_row.addWidget(self.btn_down)

        btn_row.addStretch()

        self.btn_delete = QPushButton("✕ Delete")
        self.btn_delete.setStyleSheet(BTN_DELETE_STYLE)
        self.btn_delete.clicked.connect(lambda: self.delete_requested.emit(self.index))
        btn_row.addWidget(self.btn_delete)

        content_layout.addLayout(btn_row)
        card_layout.addWidget(self.content)
        outer.addWidget(self.card)

    # ── Helpers ──────────────────────────────────────────────────────────────

    def _on_type_changed(self, new_type):
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
        self.combo_type.setCurrentText(self.joint_data.get('type', 'revolute'))
        self.spin_theta.setValue(self.joint_data.get('theta', 0.0))
        self.spin_d.setValue(self.joint_data.get('d', 0.1))
        self.spin_a.setValue(self.joint_data.get('a', 0.2))
        self.spin_alpha.setValue(self.joint_data.get('alpha', 0.0))

    def _update_summary(self):
        self._read_inputs_to_data()
        jt = self.joint_data['type'][:3].upper()
        theta = self.joint_data['theta']
        a = self.joint_data['a']
        type_colors = {'rev': '#3498db', 'pri': '#ffba4b', 'fix': '#89929b'}
        color = type_colors.get(jt[:3].lower(), '#bfc7d2')
        self.summary_label.setText(
            f"<span style='color:{color}; font-weight:700;'>J{self.index + 1} {jt}</span>"
            f"  <span style='color:#89929b;'>θ={theta:+.1f}°  a={a:.3f}m</span>"
        )

    def _toggle(self):
        self._expanded = not self._expanded
        self.content.setVisible(self._expanded)
        self.btn_expand.setText("−" if self._expanded else "+")
        self.btn_expand.clearFocus()
        self.btn_expand.repaint()

    def expand(self):
        if not self._expanded:
            self._toggle()

    def collapse(self):
        if self._expanded:
            self._toggle()

    def set_index(self, idx: int):
        self.index = idx
        self._update_summary()

    def get_joint_data(self) -> dict:
        self._read_inputs_to_data()
        return self.joint_data.copy()
