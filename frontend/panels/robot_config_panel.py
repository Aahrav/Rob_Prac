#!/usr/bin/env python3
"""
RobotConfigPanel — Adjustable robotic arm parameters.
Kinetic Obsidian theme: flat inputs, no QGroupBox borders, 2-col grid layout.
"""

from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
                              QDoubleSpinBox, QLabel, QPushButton, QFrame)
from PyQt6.QtCore import pyqtSignal
from backend.kinematics import ArmConfig
from frontend.panels.custom_combo_box import CustomComboBox


# ── Style tokens ─────────────────────────────────────────────────────────────
SPIN_STYLE = """
    QDoubleSpinBox {
        background-color: #0e0e0e;
        color: #e5e2e1;
        padding: 5px 8px;
        border: none;
        border-radius: 3px;
        font-size: 11px;
        selection-background-color: #3498db;
    }
    QDoubleSpinBox:focus {
        border-bottom: 2px solid #3498db;
    }
    QDoubleSpinBox::up-button, QDoubleSpinBox::down-button {
        width: 18px;
        background-color: #202020;
        border: none;
    }
    QDoubleSpinBox::up-button:hover, QDoubleSpinBox::down-button:hover {
        background-color: #2a2a2a;
    }
"""

COMBO_STYLE = """
    QComboBox {
        background-color: #0e0e0e;
        color: #e5e2e1;
        padding: 6px 10px;
        border: none;
        border-radius: 4px;
        font-size: 11px;
    }
    QComboBox::drop-down { border: none; width: 24px; }
    QComboBox::down-arrow { image: none; border-left: 1px solid #353535; width: 20px; }
    QComboBox QAbstractItemView {
        background-color: #202020;
        color: #e5e2e1;
        selection-background-color: #3498db;
        border: 1px solid #353535;
        padding: 4px;
    }
"""

LABEL_STYLE = "color: #bfc7d2; font-size: 11px;"
SECTION_LABEL = "color: #89929b; font-size: 10px; font-weight: 600; letter-spacing: 0.06em;"

BTN_GHOST_STYLE = """
    QPushButton {
        background-color: transparent;
        color: #89929b;
        border: 1px solid #353535;
        border-radius: 4px;
        padding: 6px 12px;
        font-size: 11px;
    }
    QPushButton:hover {
        background-color: #353535;
        color: #e5e2e1;
    }
    QPushButton:pressed {
        background-color: #2a2a2a;
    }
"""


class RobotConfigPanel(QWidget):
    """Panel for editing robot geometry parameters."""

    config_changed = pyqtSignal(object)  # ArmConfig

    PRESETS = {
        'Default': ArmConfig(base_height=0.10, upper_arm_length=0.30, lower_arm_length=0.25, gripper_offset=0.08),
        'Desktop': ArmConfig(base_height=0.06, upper_arm_length=0.20, lower_arm_length=0.18, gripper_offset=0.05),
        'Industrial': ArmConfig(base_height=0.15, upper_arm_length=0.50, lower_arm_length=0.45, gripper_offset=0.12),
        'Long Reach': ArmConfig(base_height=0.12, upper_arm_length=0.70, lower_arm_length=0.65, gripper_offset=0.15),
    }

    def __init__(self, config: ArmConfig = None, parent=None):
        super().__init__(parent)
        self.config = config if config is not None else ArmConfig()
        self._setup_ui()
        self._apply_config_to_spinboxes()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(10)

        # ── Preset row ────────────────────────────────────────────────────
        lbl = QLabel("PRESET")
        lbl.setStyleSheet(SECTION_LABEL)
        layout.addWidget(lbl)

        self.preset_combo = CustomComboBox()
        self.preset_combo.addItems(list(self.PRESETS.keys()))
        self.preset_combo.setStyleSheet(COMBO_STYLE)
        self.preset_combo.currentTextChanged.connect(self._load_preset)
        layout.addWidget(self.preset_combo)

        # ── Separator ─────────────────────────────────────────────────────
        sep = QFrame()
        sep.setFrameShape(QFrame.Shape.HLine)
        sep.setStyleSheet("color: #353535; background: #353535; border: none; max-height: 1px;")
        layout.addWidget(sep)

        # ── Parameters grid ───────────────────────────────────────────────
        lbl2 = QLabel("DIMENSIONS")
        lbl2.setStyleSheet(SECTION_LABEL)
        layout.addWidget(lbl2)

        grid = QGridLayout()
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(6)

        params = [
            ("Base height", "m", 0.05, 0.5, 0.01, "sb_base"),
            ("Upper arm",   "m", 0.10, 1.0, 0.01, "sb_upper"),
            ("Lower arm",   "m", 0.10, 1.0, 0.01, "sb_lower"),
            ("Gripper",     "m", 0.00, 0.3, 0.01, "sb_gripper"),
        ]

        for row, (name, unit, mn, mx, step, attr) in enumerate(params):
            # Label
            lbl = QLabel(f"{name}")
            lbl.setStyleSheet(LABEL_STYLE)
            grid.addWidget(lbl, row, 0)

            # Spinbox
            sb = QDoubleSpinBox()
            sb.setRange(mn, mx)
            sb.setSingleStep(step)
            sb.setDecimals(3)
            sb.setSuffix(f" {unit}")
            sb.setStyleSheet(SPIN_STYLE)
            sb.valueChanged.connect(self._on_param_changed)
            grid.addWidget(sb, row, 1)
            setattr(self, attr, sb)

        layout.addLayout(grid)

        # ── Button row ────────────────────────────────────────────────────
        btn_row = QHBoxLayout()
        btn_row.setSpacing(6)

        btn_reset = QPushButton("↺  Reset")
        btn_reset.setStyleSheet(BTN_GHOST_STYLE)
        btn_reset.setToolTip("Reset to selected preset")
        btn_reset.clicked.connect(self._reset_to_preset)
        btn_row.addWidget(btn_reset)

        btn_export = QPushButton("↓  Export DH")
        btn_export.setStyleSheet(BTN_GHOST_STYLE)
        btn_export.setToolTip("Export DH parameters to file")
        btn_export.clicked.connect(self._export_dh)
        btn_row.addWidget(btn_export)

        layout.addLayout(btn_row)

    # ── Helpers ──────────────────────────────────────────────────────────────

    def _apply_config_to_spinboxes(self):
        self.sb_base.setValue(self.config.base_height)
        self.sb_upper.setValue(self.config.upper_arm_length)
        self.sb_lower.setValue(self.config.lower_arm_length)
        self.sb_gripper.setValue(self.config.gripper_offset)
        for name, preset in self.PRESETS.items():
            if (preset.base_height == self.config.base_height and
                    preset.upper_arm_length == self.config.upper_arm_length and
                    preset.lower_arm_length == self.config.lower_arm_length and
                    preset.gripper_offset == self.config.gripper_offset):
                self.preset_combo.setCurrentText(name)
                return

    def _on_param_changed(self):
        self.config.base_height = self.sb_base.value()
        self.config.upper_arm_length = self.sb_upper.value()
        self.config.lower_arm_length = self.sb_lower.value()
        self.config.gripper_offset = self.sb_gripper.value()
        self.config_changed.emit(self.config)

    def _load_preset(self, name):
        if name in self.PRESETS:
            preset = self.PRESETS[name]
            self.config.base_height = preset.base_height
            self.config.upper_arm_length = preset.upper_arm_length
            self.config.lower_arm_length = preset.lower_arm_length
            self.config.gripper_offset = preset.gripper_offset
            self._apply_config_to_spinboxes()
            self.config_changed.emit(self.config)

    def _reset_to_preset(self):
        self._load_preset(self.preset_combo.currentText())

    def _export_dh(self):
        from PyQt6.QtWidgets import QFileDialog
        path, _ = QFileDialog.getSaveFileName(self, "Export DH Parameters", "", "CSV Files (*.csv);;Text Files (*.txt)")
        if path:
            with open(path, 'w') as f:
                f.write("# DH Parameters\n")
                f.write("# a, alpha, d, theta\n")
                f.write(f"# Base height: {self.config.base_height}\n")
                f.write(f"{self.config.upper_arm_length}, 0, 0, 0\n")
                f.write(f"{self.config.lower_arm_length}, 0, 0, 0\n")
                f.write(f"{self.config.gripper_offset}, 0, 0, 0\n")

    def set_config(self, config: ArmConfig):
        self.config = config
        self._apply_config_to_spinboxes()
