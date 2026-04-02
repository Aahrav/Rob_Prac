#!/usr/bin/env python3
"""RobotConfigPanel - Adjustable robotic arm parameters (lengths, offsets)."""

from PyQt6.QtWidgets import QWidget, QVBoxLayout, QGroupBox, QDoubleSpinBox, QLabel, QComboBox, QPushButton, QHBoxLayout
from PyQt6.QtCore import pyqtSignal
from backend.kinematics import ArmConfig


class RobotConfigPanel(QWidget):
    """Panel for editing robot geometry parameters."""

    # Emitted when config changes
    config_changed = pyqtSignal(object)  # ArmConfig

    # Preset configurations (name, ArmConfig)
    PRESETS = {
        'Default': ArmConfig(base_height=0.1, upper_arm_length=0.3, lower_arm_length=0.25, gripper_offset=0.05),
        'Small': ArmConfig(base_height=0.08, upper_arm_length=0.2, lower_arm_length=0.15, gripper_offset=0.03),
        'Medium': ArmConfig(base_height=0.1, upper_arm_length=0.4, lower_arm_length=0.35, gripper_offset=0.07),
        'Large': ArmConfig(base_height=0.15, upper_arm_length=0.6, lower_arm_length=0.5, gripper_offset=0.1),
    }

    def __init__(self, config: ArmConfig = None, parent=None):
        super().__init__(parent)
        self.config = config if config is not None else ArmConfig()
        self._setup_ui()
        self._apply_config_to_spinboxes()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 12, 8, 12)
        layout.setSpacing(10)

        # Title / preset selector
        title = QLabel("Robot Parameters")
        title.setStyleSheet("font-weight: bold; color: #eee; font-size: 12px;")
        layout.addWidget(title)

        # Preset combo
        self.preset_combo = QComboBox()
        self.preset_combo.addItems(list(self.PRESETS.keys()))
        self.preset_combo.setStyleSheet("""
            QComboBox { background: #444; color: #eee; padding: 6px; border: 1px solid #555; border-radius: 4px; }
            QComboBox::drop-down { border: none; }
            QComboBox::down-arrow { image: none; border-left: 1px solid #555; width: 20px; }
        """)
        self.preset_combo.currentTextChanged.connect(self._load_preset)
        layout.addWidget(self.preset_combo)

        # Parameter group
        group = QGroupBox("Dimensions")
        group.setStyleSheet("""
            QGroupBox { color: #eee; font-size: 11px; border: 1px solid #555; margin-top: 12px; padding-top: 12px; }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 6px; }
        """)
        group_layout = QVBoxLayout(group)
        group_layout.setSpacing(8)
        group_layout.setContentsMargins(8, 12, 8, 12)

        # Spinbox style
        sb_style = """
            QDoubleSpinBox { background: #333; color: #eee; padding: 6px; border: 1px solid #555; border-radius: 4px; }
            QDoubleSpinBox::up-button, QDoubleSpinBox::down-button { width: 20px; }
        """

        # Base height
        group_layout.addWidget(QLabel("Base height (m):"))
        self.sb_base = QDoubleSpinBox()
        self.sb_base.setRange(0.05, 0.5)
        self.sb_base.setSingleStep(0.01)
        self.sb_base.setDecimals(2)
        self.sb_base.setStyleSheet(sb_style)
        self.sb_base.valueChanged.connect(self._on_param_changed)
        group_layout.addWidget(self.sb_base)

        # Upper arm length
        group_layout.addWidget(QLabel("Upper arm (m):"))
        self.sb_upper = QDoubleSpinBox()
        self.sb_upper.setRange(0.1, 1.0)
        self.sb_upper.setSingleStep(0.01)
        self.sb_upper.setDecimals(2)
        self.sb_upper.setStyleSheet(sb_style)
        self.sb_upper.valueChanged.connect(self._on_param_changed)
        group_layout.addWidget(self.sb_upper)

        # Lower arm length
        group_layout.addWidget(QLabel("Lower arm (m):"))
        self.sb_lower = QDoubleSpinBox()
        self.sb_lower.setRange(0.1, 1.0)
        self.sb_lower.setSingleStep(0.01)
        self.sb_lower.setDecimals(2)
        self.sb_lower.setStyleSheet(sb_style)
        self.sb_lower.valueChanged.connect(self._on_param_changed)
        group_layout.addWidget(self.sb_lower)

        # Gripper offset
        group_layout.addWidget(QLabel("Gripper offset (m):"))
        self.sb_gripper = QDoubleSpinBox()
        self.sb_gripper.setRange(0.0, 0.3)
        self.sb_gripper.setSingleStep(0.01)
        self.sb_gripper.setDecimals(2)
        self.sb_gripper.setStyleSheet(sb_style)
        self.sb_gripper.valueChanged.connect(self._on_param_changed)
        group_layout.addWidget(self.sb_gripper)

        layout.addWidget(group)

        # Reset button
        btn_reset = QPushButton("Reset to Preset")
        btn_reset.setStyleSheet("""
            QPushButton { background: #555; color: #eee; padding: 8px; border-radius: 4px; font-weight: bold; }
            QPushButton:hover { background: #666; }
        """)
        btn_reset.clicked.connect(self._reset_to_preset)
        layout.addWidget(btn_reset)

        layout.addStretch()

    def _apply_config_to_spinboxes(self):
        """Populate spinboxes from current config."""
        self.sb_base.setValue(self.config.base_height)
        self.sb_upper.setValue(self.config.upper_arm_length)
        self.sb_lower.setValue(self.config.lower_arm_length)
        self.sb_gripper.setValue(self.config.gripper_offset)

        # Find matching preset or set to 'Default'
        for name, preset in self.PRESETS.items():
            if (preset.base_height == self.config.base_height and
                preset.upper_arm_length == self.config.upper_arm_length and
                preset.lower_arm_length == self.config.lower_arm_length and
                preset.gripper_offset == self.config.gripper_offset):
                self.preset_combo.setCurrentText(name)
                return
        self.preset_combo.setCurrentText('')  # custom

    def _on_param_changed(self):
        """Update config from spinboxes and emit signal."""
        self.config.base_height = self.sb_base.value()
        self.config.upper_arm_length = self.sb_upper.value()
        self.config.lower_arm_length = self.sb_lower.value()
        self.config.gripper_offset = self.sb_gripper.value()
        self.config_changed.emit(self.config)

    def _load_preset(self, name):
        """Load a preset configuration by mutating the existing config."""
        if name in self.PRESETS:
            preset = self.PRESETS[name]
            # Mutate the existing config object (preserve reference)
            self.config.base_height = preset.base_height
            self.config.upper_arm_length = preset.upper_arm_length
            self.config.lower_arm_length = preset.lower_arm_length
            self.config.gripper_offset = preset.gripper_offset
            self._apply_config_to_spinboxes()
            self.config_changed.emit(self.config)

    def _reset_to_preset(self):
        """Reset to the selected preset."""
        self._load_preset(self.preset_combo.currentText())

    def set_config(self, config: ArmConfig):
        """Replace the entire config object (used for external updates)."""
        self.config = config
        self._apply_config_to_spinboxes()
