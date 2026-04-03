#!/usr/bin/env python3
"""
KinematicChainPanel - Build a robot using Denavit-Hartenberg parameters.
Add joints, specify type and DH parameters, and visualize the resulting chain.
"""

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox, QLabel, QDoubleSpinBox,
    QComboBox, QPushButton, QScrollArea, QMessageBox, QGridLayout, QCheckBox, QLineEdit, QSlider
)
from PyQt6.QtCore import pyqtSignal, Qt
from backend.kinematics import KinematicChain, DHJoint
import numpy as np


class JointCard(QGroupBox):
    """A collapsible card representing one joint with editable DH parameters."""

    changed = pyqtSignal(int)  # index of joint that changed
    delete_requested = pyqtSignal(int)  # index to delete
    move_up_requested = pyqtSignal(int)
    move_down_requested = pyqtSignal(int)

    def __init__(self, index: int, joint: DHJoint, parent=None):
        super().__init__(parent)
        self.index = index
        self.joint = joint
        self.setTitle(f"{index+1}. {self.joint.name}")
        self.setCheckable(True)
        self.setChecked(True)  # expanded by default
        self._setup_ui()
        self._apply_to_fields()

    def _setup_ui(self):
        layout = QGridLayout(self)
        layout.setContentsMargins(8, 20, 8, 8)
        layout.setSpacing(6)

        # Joint type
        layout.addWidget(QLabel("Type:"), 0, 0)
        self.combo_type = QComboBox()
        self.combo_type.addItems(["revolute", "prismatic", "fixed"])
        self.combo_type.setCurrentText(self.joint.type)
        self.combo_type.currentTextChanged.connect(self._on_type_changed)
        layout.addWidget(self.combo_type, 0, 1)

        # Theta (angle in degrees)
        layout.addWidget(QLabel("θ (deg):"), 1, 0)
        self.spin_theta = QDoubleSpinBox()
        self.spin_theta.setRange(-180.0, 180.0)
        self.spin_theta.setSingleStep(1.0)
        self.spin_theta.valueChanged.connect(self._on_param_changed)
        layout.addWidget(self.spin_theta, 1, 1)

        # d (offset in meters)
        layout.addWidget(QLabel("d (m):"), 2, 0)
        self.spin_d = QDoubleSpinBox()
        self.spin_d.setRange(0.0, 1.0)
        self.spin_d.setSingleStep(0.01)
        self.spin_d.setDecimals(3)
        self.spin_d.valueChanged.connect(self._on_param_changed)
        layout.addWidget(self.spin_d, 2, 1)

        # a (link length in meters)
        layout.addWidget(QLabel("a (m):"), 3, 0)
        self.spin_a = QDoubleSpinBox()
        self.spin_a.setRange(0.0, 1.0)
        self.spin_a.setSingleStep(0.01)
        self.spin_a.setDecimals(3)
        self.spin_a.valueChanged.connect(self._on_param_changed)
        layout.addWidget(self.spin_a, 3, 1)

        # alpha (twist in degrees)
        layout.addWidget(QLabel("α (deg):"), 4, 0)
        self.spin_alpha = QDoubleSpinBox()
        self.spin_alpha.setRange(-180.0, 180.0)
        self.spin_alpha.setSingleStep(1.0)
        self.spin_alpha.valueChanged.connect(self._on_param_changed)
        layout.addWidget(self.spin_alpha, 4, 1)

        # Name
        layout.addWidget(QLabel("Name:"), 5, 0)
        self.edit_name = QDoubleSpinBox()  # Placeholder: we could use QLineEdit but consistency with spin?
        # Let's use QLineEdit for name
        from PyQt6.QtWidgets import QLineEdit
        self.edit_name = QLineEdit()
        self.edit_name.setText(self.joint.name)
        self.edit_name.textChanged.connect(self._on_param_changed)
        layout.addWidget(self.edit_name, 5, 1)

        # Buttons: move up, move down, delete
        btn_up = QPushButton("↑")
        btn_up.setFixedWidth(30)
        btn_up.clicked.connect(lambda: self.move_up_requested.emit(self.index))
        btn_down = QPushButton("↓")
        btn_down.setFixedWidth(30)
        btn_down.clicked.connect(lambda: self.move_down_requested.emit(self.index))
        btn_del = QPushButton("×")
        btn_del.setFixedWidth(30)
        btn_del.setStyleSheet("QPushButton { background: #c0392b; color: white; font-weight: bold; }")
        btn_del.clicked.connect(lambda: self.delete_requested.emit(self.index))

        hbox = QHBoxLayout()
        hbox.addWidget(btn_up)
        hbox.addWidget(btn_down)
        hbox.addStretch()
        hbox.addWidget(btn_del)
        layout.addLayout(hbox, 6, 0, 1, 2)

        # Stretch to fill
        layout.setRowStretch(7, 1)

    def _apply_to_fields(self):
        self.combo_type.blockSignals(True)
        self.combo_type.setCurrentText(self.joint.type)
        self.combo_type.blockSignals(False)
        self.spin_theta.setValue(self.joint.theta)
        self.spin_d.setValue(self.joint.d)
        self.spin_a.setValue(self.joint.a)
        self.spin_alpha.setValue(self.joint.alpha)
        self.edit_name.setText(self.joint.name)

    def _on_type_changed(self, new_type: str):
        self.joint.type = new_type
        self.changed.emit(self.index)

    def _on_param_changed(self, _=None):
        self.joint.theta = self.spin_theta.value()
        self.joint.d = self.spin_d.value()
        self.joint.a = self.spin_a.value()
        self.joint.alpha = self.spin_alpha.value()
        self.joint.name = self.edit_name.text()
        self.setTitle(f"{self.index+1}. {self.joint.name}")
        self.changed.emit(self.index)


class JointSlider(QWidget):
    """A slider + spinbox for controlling a variable joint (revolute or prismatic) in degrees/meters."""
    value_changed = pyqtSignal(int, float)  # index, new value

    def __init__(self, index: int, joint: DHJoint, parent=None):
        super().__init__(parent)
        self.index = index
        self.joint = joint
        self.setup_ui()

    def setup_ui(self):
        hbox = QHBoxLayout(self)
        hbox.setContentsMargins(0, 0, 0, 0)
        lbl = QLabel(f"{self.index+1}. {self.joint.name}:")
        lbl.setFixedWidth(100)
        hbox.addWidget(lbl)

        # Slider range: use joint limits if provided, else generous defaults
        if self.joint.type == 'revolute':
            vmin = self.joint.q_min if self.joint.q_min is not None else -180.0
            vmax = self.joint.q_max if self.joint.q_max is not None else 180.0
            default = self.joint.theta
            step = 1.0
        elif self.joint.type == 'prismatic':
            vmin = self.joint.q_min if self.joint.q_min is not None else 0.0
            vmax = self.joint.q_max if self.joint.q_max is not None else 1.0
            default = self.joint.d
            step = 0.001
        else:
            return  # don't create slider for fixed joints

        self.vmin = vmin
        self.vmax = vmax
        self.step = step

        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setRange(int(round(vmin/step)), int(round(vmax/step)))
        self.slider.setValue(int(round(default/step)))
        self.slider.valueChanged.connect(self._on_slider_moved)

        self.spin = QDoubleSpinBox()
        self.spin.setRange(vmin, vmax)
        self.spin.setSingleStep(step)
        self.spin.setDecimals(3 if step < 0.1 else 1)
        self.spin.setValue(default)
        self.spin.valueChanged.connect(self._on_spin_changed)

        hbox.addWidget(self.slider, 1)
        hbox.addWidget(self.spin)

    def _on_slider_moved(self, int_val):
        val = int_val * self.step
        self.spin.blockSignals(True)
        self.spin.setValue(val)
        self.spin.blockSignals(False)
        self.value_changed.emit(self.index, val)

    def _on_spin_changed(self, val):
        int_val = int(round(val / self.step))
        self.slider.blockSignals(True)
        self.slider.setValue(int_val)
        self.slider.blockSignals(False)
        self.value_changed.emit(self.index, val)

    def update_from_joint(self):
        if self.joint.type == 'revolute':
            val = self.joint.theta
        elif self.joint.type == 'prismatic':
            val = self.joint.d
        else:
            return
        self.spin.setValue(val)
        self.slider.setValue(int(round(val / self.step)))


class KinematicChainPanel(QGroupBox):
    """Panel for building a custom kinematic chain with DH parameters."""

    chain_updated = pyqtSignal(KinematicChain)  # emitted when chain changes (structure or values)
    end_effector_updated = pyqtSignal(np.ndarray)  # emitted with (x,y,z) of tip

    def __init__(self, parent=None):
        super().__init__("Robot Structure (DH)", parent)
        self.setStyleSheet("""
            QGroupBox { border: 1px solid #555; margin-top: 8px; padding-top: 12px; }
        """)
        self.chain = KinematicChain(base_height=0.1)
        self._variable_joint_sliders = []  # list of JointSlider for current variable joints

        self._setup_ui()
        # Add a default joint to get started
        self._on_add_joint()
        self._update_sliders()
        # Connect internal signals: any chain update triggers FK and label update
        self.chain_updated.connect(self._compute_and_emit_fk)
        self.end_effector_updated.connect(self._update_ee_label)

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 12, 8, 12)
        layout.setSpacing(10)

        # Header: Add joint button
        h_top = QHBoxLayout()
        h_top.addWidget(QLabel("Add new joint:"))
        self.combo_new_type = QComboBox()
        self.combo_new_type.addItems(["revolute", "prismatic", "fixed"])
        h_top.addWidget(self.combo_new_type)
        btn_add = QPushButton("+ Add Joint")
        btn_add.setStyleSheet("QPushButton { background: #27ae60; color: white; font-weight: bold; padding: 6px; }")
        btn_add.clicked.connect(self._on_add_joint)
        h_top.addWidget(btn_add)
        h_top.addStretch()
        layout.addLayout(h_top)

        # Scroll area for joint cards
        self.scroll = QScrollArea()
        self.scroll.setWidgetResizable(True)
        self.scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.scroll.setStyleSheet("QScrollArea { border: none; }")
        self.cards_container = QWidget()
        self.cards_layout = QVBoxLayout(self.cards_container)
        self.cards_layout.setSpacing(8)
        self.scroll.setWidget(self.cards_container)
        layout.addWidget(self.scroll, 1)

        # Separator
        from PyQt6.QtWidgets import QFrame
        sep = QFrame()
        sep.setFrameShape(QFrame.Shape.HLine)
        sep.setStyleSheet("background-color: #444;")
        layout.addWidget(sep)

        # Joint sliders section (for real-time control of variable joints)
        self.sliders_group = QGroupBox("Joint Control")
        self.sliders_group.setStyleSheet("QGroupBox { border: 1px solid #555; margin-top: 8px; padding-top: 12px; }")
        self.sliders_layout = QVBoxLayout(self.sliders_group)
        layout.addWidget(self.sliders_group)

        # End effector position readout
        h_ee = QHBoxLayout()
        self.lbl_ee = QLabel("End-Effector: X=0.000 Y=0.000 Z=0.000")
        self.lbl_ee.setStyleSheet("font-family: monospace; color: #eee;")
        h_ee.addWidget(self.lbl_ee)
        layout.addLayout(h_ee)

    def _on_add_joint(self):
        joint_type = self.combo_new_type.currentText()
        if joint_type in ('revolute', 'prismatic'):
            joint = DHJoint(
                joint_type=joint_type,
                theta=0.0 if joint_type == 'revolute' else 0.0,
                d=0.0 if joint_type == 'prismatic' else 0.0,
                a=0.1,
                alpha=0.0,
                name=f"{joint_type.capitalize()} {len(self.chain.joints)+1}"
            )
        else:  # fixed
            joint = DHJoint(
                joint_type='fixed',
                theta=0.0,
                d=0.0,
                a=0.05,
                alpha=0.0,
                name=f"Link {len(self.chain.joints)+1}"
            )
        self.chain.add_joint(joint)
        self._rebuild_cards()
        self._update_sliders()
        self.chain_updated.emit(self.chain)
        # Also compute FK to update end-effector
        self._compute_and_emit_fk()

    def _rebuild_cards(self):
        # Clear existing cards
        while self.cards_layout.count():
            w = self.cards_layout.takeAt(0).widget()
            if w:
                w.deleteLater()
        self.joint_cards = []
        for i, joint in enumerate(self.chain.joints):
            card = JointCard(i, joint, self.cards_container)
            card.changed.connect(self._on_joint_card_changed)
            card.delete_requested.connect(self._on_delete_joint)
            card.move_up_requested.connect(self._on_move_joint_up)
            card.move_down_requested.connect(self._on_move_joint_down)
            self.cards_layout.addWidget(card)
            self.joint_cards.append(card)

    def _on_joint_card_changed(self, index: int):
        # Update chain's joint from card
        card = self.joint_cards[index]
        self.chain.joints[index] = card.joint
        # If type or variable status changed, rebuild sliders
        if self._needs_slider_rebuild():
            self._update_sliders()
        else:
            # Update values and emit FK
            self._compute_and_emit_fk()
        self.chain_updated.emit(self.chain)

    def _needs_slider_rebuild(self) -> bool:
        # Check if any joint's type or variable status changed relative to existing sliders count
        expected_variable = sum(1 for j in self.chain.joints if j.type in ('revolute', 'prismatic'))
        return len(self._variable_joint_sliders) != expected_variable

    def _on_delete_joint(self, index: int):
        if len(self.chain.joints) <= 1:
            QMessageBox.warning(self, "Cannot delete", "At least one joint must remain.")
            return
        self.chain.remove_joint(index)
        self._rebuild_cards()
        self._update_sliders()
        self.chain_updated.emit(self.chain)
        self._compute_and_emit_fk()

    def _on_move_joint_up(self, index: int):
        if index > 0:
            self.chain.move_joint(index, index-1)
            self._rebuild_cards()
            self._update_sliders()
            self.chain_updated.emit(self.chain)
            self._compute_and_emit_fk()

    def _on_move_joint_down(self, index: int):
        if index < len(self.chain.joints) - 1:
            self.chain.move_joint(index, index+1)
            self._rebuild_cards()
            self._update_sliders()
            self.chain_updated.emit(self.chain)
            self._compute_and_emit_fk()

    def _update_sliders(self):
        # Clear existing sliders
        while self.sliders_layout.count():
            w = self.sliders_layout.takeAt(0).widget()
            if w:
                w.deleteLater()
        self._variable_joint_sliders.clear()

        # Create sliders for each variable (revolute/prismatic) joint
        for i, joint in enumerate(self.chain.joints):
            if joint.type in ('revolute', 'prismatic'):
                slider = JointSlider(i, joint, self.sliders_group)
                slider.value_changed.connect(self._on_slider_value_changed)
                self.sliders_layout.addWidget(slider)
                self._variable_joint_sliders.append(slider)

        self.sliders_layout.addStretch()

    def _on_slider_value_changed(self, index: int, value: float):
        # Update the joint in chain
        joint = self.chain.joints[index]
        if joint.type == 'revolute':
            joint.theta = value
        elif joint.type == 'prismatic':
            joint.d = value
        # Also update the corresponding card's field to reflect change
        if index < len(self.joint_cards):
            card = self.joint_cards[index]
            card.joint = joint
            card._apply_to_fields()
        self.chain_updated.emit(self.chain)
        self._compute_and_emit_fk()

    def _compute_and_emit_fk(self):
        # Compute forward kinematics using current joint values (theta/d) directly
        try:
            positions = self.chain.joint_positions()  # (N+1,3), uses each joint's own theta/d
            tip_pos = positions[-1]
            self.end_effector_updated.emit(tip_pos)
        except Exception as e:
            print(f"FK error: {e}")

    def _update_ee_label(self, pos):
        """Update end-effector position label."""
        self.lbl_ee.setText(f"End-Effector: X={pos[0]:.3f} Y={pos[1]:.3f} Z={pos[2]:.3f}")

    def set_chain(self, chain: KinematicChain):
        """Replace the current chain (used for loading presets)."""
        self.chain = chain
        self._rebuild_cards()
        self._update_sliders()
        self.chain_updated.emit(self.chain)
        self._compute_and_emit_fk()

    def get_chain(self) -> KinematicChain:
        return self.chain
