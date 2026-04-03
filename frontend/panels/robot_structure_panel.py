#!/usr/bin/env python3
"""
RobotStructurePanel - Dynamic kinematic chain builder with DH parameters.
Allows users to add joints (revolute/prismatic), configure DH params, and see end-effector pose.
"""

from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel,
                              QComboBox, QDoubleSpinBox, QMessageBox, QScrollArea,
                              QGroupBox, QCheckBox, QFrame)
from PyQt6.QtCore import pyqtSignal, Qt
from backend.dh_kinematics import (forward_kinematics_dh, joint_angles_to_dh_params,
                                   compute_joint_positions)

class JointCard(QFrame):
    """A card representing one joint with inputs for DH parameters."""

    value_changed = pyqtSignal(int)  # signal when any parameter changes (emit joint index)

    def __init__(self, index: int, joint_type: str = "revolute", parent=None):
        super().__init__(parent)
        self.index = index
        self.setFrameStyle(QFrame.Shape.StyledPanel | QFrame.Shadow.Raised)
        self.setStyleSheet("background-color: #2d2d2e; border-radius: 6px; padding: 8px; margin: 4px;")

        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)

        # Header with type and delete button
        header = QHBoxLayout()
        self.type_combo = QComboBox()
        self.type_combo.addItems(["revolute", "prismatic"])
        self.type_combo.setCurrentText(joint_type)
        self.type_combo.currentTextChanged.connect(self._on_type_changed)

        self.btn_delete = QPushButton("×")
        self.btn_delete.setFixedSize(24, 24)
        self.btn_delete.setStyleSheet("background-color: #c0392b; color: white; border: none; font-weight: bold; border-radius: 12px;")
        self.btn_delete.clicked.connect(self._on_delete)

        header.addWidget(QLabel(f"Joint {index+1}"))
        header.addStretch()
        header.addWidget(QLabel("Type:"))
        header.addWidget(self.type_combo)
        header.addWidget(self.btn_delete)
        layout.addLayout(header)

        # DH parameter inputs (θ or d, a, α) — label adapts to type
        grid = QHBoxLayout()
        grid.setSpacing(8)

        # Variable parameter (θ for revolute, d for prismatic)
        self.var_label = QLabel("θ (rad):")
        self.var_spin = QDoubleSpinBox()
        self.var_spin.setRange(-6.283185, 6.283185)
        self.var_spin.setSingleStep(0.1)
        self.var_spin.setValue(0.0)
        self.var_spin.valueChanged.connect(lambda: self._emit_change())

        # d (offset along z)
        self.d_spin = self._create_spin(0.0, -10, 10, 0.01, "d (m):")
        self.d_spin.valueChanged.connect(lambda: self._emit_change())

        # a (link length along x)
        self.a_spin = self._create_spin(0.1, 0, 5, 0.01, "a (m):")
        self.a_spin.valueChanged.connect(lambda: self._emit_change())

        # α (twist about x)
        self.alpha_spin = self._create_spin(0.0, -3.141593, 3.141593, 0.01, "α (rad):")
        self.alpha_spin.valueChanged.connect(lambda: self._emit_change())

        grid.addWidget(self.var_label)
        grid.addWidget(self.var_spin)
        grid.addWidget(QLabel("d:"))
        grid.addWidget(self.d_spin)
        grid.addWidget(QLabel("a:"))
        grid.addWidget(self.a_spin)
        grid.addWidget(QLabel("α:"))
        grid.addWidget(self.alpha_spin)
        layout.addLayout(grid)

    def _create_spin(self, value: float, min_val: float, max_val: float, step: float, label: str):
        spin = QDoubleSpinBox()
        spin.setRange(min_val, max_val)
        spin.setSingleStep(step)
        spin.setValue(value)
        spin.setDecimals(3)
        return spin

    def _on_type_changed(self, new_type: str):
        if new_type == "revolute":
            self.var_label.setText("θ (rad):")
            self.var_spin.setRange(-6.283185, 6.283185)
            self.var_spin.setValue(0.0)
        elif new_type == "prismatic":
            self.var_label.setText("d (m):")
            self.var_spin.setRange(0, 1.0)
            self.var_spin.setValue(0.0)
        self._emit_change()

    def _on_delete(self):
        self.deleteLater()

    def _emit_change(self):
        self.value_changed.emit(self.index)

    def get_joint_data(self) -> Dict:
        """Return joint dict with DH parameters."""
        jt = self.type_combo.currentText()
        return {
            'type': jt,
            'theta': self.var_spin.value() if jt == 'revolute' else 0.0,
            'd': self.var_spin.value() if jt == 'prismatic' else self.d_spin.value(),
            'a': self.a_spin.value(),
            'alpha': self.alpha_spin.value()
        }

    def set_joint_data(self, data: Dict):
        """Set values from a dict."""
        self.type_combo.setCurrentText(data['type'])
        if data['type'] == 'revolute':
            self.var_spin.setValue(data['theta'])
        else:
            self.var_spin.setValue(data['d'])
        self.d_spin.setValue(data['d'])
        self.a_spin.setValue(data['a'])
        self.alpha_spin.setValue(data['alpha'])


class RobotStructurePanel(QWidget):
    """Panel for building and editing a serial robot chain."""

    kinematics_updated = pyqtSignal(list, np.ndarray)  # emits (joints, end_effector_transform)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.joints: List[Dict] = []  # list of joint dicts
        self.joint_cards: List[JointCard] = []
        self._dh_mode = False  # False = auto (sliders), True = manual DH mode

        self._init_ui()

    def _init_ui(self):
        main = QVBoxLayout(self)
        main.setContentsMargins(8, 8, 8, 8)
        main.setSpacing(8)

        # Title
        title = QLabel("Robot Structure")
        title.setStyleSheet("font-weight: bold; font-size: 14px; color: #ddd;")
        main.addWidget(title)

        # Mode toggle
        mode_layout = QHBoxLayout()
        self.chk_dh_mode = QCheckBox("Manual DH Mode")
        self.chk_dh_mode.setToolTip("Check to directly edit DH parameters; uncheck for sliders controlling joint angles.")
        self.chk_dh_mode.toggled.connect(self._on_mode_toggled)
        mode_layout.addWidget(self.chk_dh_mode)
        mode_layout.addStretch()
        main.addLayout(mode_layout)

        # Add Joint button
        self.btn_add_joint = QPushButton("+ Add Joint")
        self.btn_add_joint.setStyleSheet("background-color: #27ae60; color: white; padding: 8px; font-weight: bold; border-radius: 4px;")
        self.btn_add_joint.clicked.connect(self._add_joint)
        main.addWidget(self.btn_add_joint)

        # Scroll area for joint cards
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setStyleSheet("QScrollArea { border: none; background-color: transparent; }")
        self.scroll_content = QWidget()
        self.scroll_layout = QVBoxLayout(self.scroll_content)
        self.scroll_layout.setAlignment(Qt.AlignmentFlag.AlignTop)
        scroll.setWidget(self.scroll_content)
        main.addWidget(scroll, stretch=1)

        # End-effector position display
        self.lbl_ee = QLabel("End-effector: —")
        self.lbl_ee.setStyleSheet("color: #aaa; font-family: monospace; font-size: 11px;")
        main.addWidget(self.lbl_ee)

        # Add a default revolute joint to start
        self._add_joint()

    def _add_joint(self, joint_data: Optional[Dict] = None):
        """Add a new joint card."""
        idx = len(self.joint_cards)
        if joint_data is None:
            joint_data = {'type': 'revolute', 'theta': 0.0, 'd': 0.0, 'a': 0.1, 'alpha': 0.0}

        card = JointCard(idx, joint_data['type'])
        card.set_joint_data(joint_data)
        card.value_changed.connect(self._on_joint_value_changed)
        card.delete_requested = lambda i=idx: self._delete_joint(i)  # alt: connect signal

        self.joint_cards.append(card)
        self.scroll_layout.addWidget(card)
        self.joints.append(joint_data.copy())

        # Renumber cards after index
        self._renumber_cards()
        self._compute_kinematics()

    def _delete_joint(self, index: int):
        """Remove joint at index."""
        if index < 0 or index >= len(self.joint_cards):
            return
        card = self.joint_cards.pop(index)
        card.deleteLater()
        self.joints.pop(index)
        self._renumber_cards()
        self._compute_kinematics()

    def _renumber_cards(self):
        """Update indices on all cards."""
        for i, card in enumerate(self.joint_cards):
            card.index = i
            # Update header text
            layout = card.layout()
            if layout:
                header_item = layout.itemAt(0)
                if header_item:
                    header_layout = header_item.layout()
                    if header_layout and header_layout.count() > 0:
                        label = header_layout.itemAt(0).widget()
                        if isinstance(label, QLabel):
                            label.setText(f"Joint {i+1}")

    def _on_joint_value_changed(self, index: int):
        """Update joint data from card and recompute kinematics."""
        card = self.joint_cards[index]
        data = card.get_joint_data()
        self.joints[index] = data
        self._compute_kinematics()

    def _on_mode_toggled(self, checked: bool):
        """Switch between Auto (sliders) and Manual DH mode."""
        self._dh_mode = checked
        # When switching to auto mode, we could set variable parameters to 0 or preserve them
        self._compute_kinematics()
        # Could also emit a signal to tell main window to show/hide sliders?

    def _compute_kinematics(self):
        """Compute forward kinematics and emit update."""
        if not self.joints:
            self.lbl_ee.setText("End-effector: —")
            self.kinematics_updated.emit([], np.eye(4))
            return

        # If not in DH manual mode, we might need to convert from sliders, but for now we use direct DH
        transforms, T_ee = forward_kinematics_dh(self.joints)
        pos = T_ee[:3, 3]
        self.lbl_ee.setText(f"End-effector: X={pos[0]:.3f}, Y={pos[1]:.3f}, Z={pos[2]:.3f}")
        self.kinematics_updated.emit(self.joints.copy(), T_ee)

    def get_robot_definition(self) -> List[Dict]:
        """Return the current joint chain."""
        return self.joints.copy()

    def load_robot(self, joints: List[Dict]):
        """Load a robot definition (replace all joints)."""
        # Clear existing
        for card in self.joint_cards:
            card.deleteLater()
        self.joint_cards = []
        self.joints = []
        # Add new ones
        for jd in joints:
            self._add_joint(jd.copy())

    def get_auto_mode(self) -> bool:
        """Return True if using manual DH mode (False = auto sliders)."""
        return self._dh_mode
