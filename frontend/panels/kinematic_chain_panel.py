#!/usr/bin/env python3
"""
KinematicChainPanel - Build a robot using Denavit-Hartenberg parameters.
Add joints, specify type and DH parameters, and visualize the resulting chain.
"""

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox, QLabel, QDoubleSpinBox,
    QComboBox, QPushButton, QScrollArea, QMessageBox, QGridLayout, QCheckBox, QLineEdit, QSlider
)
from frontend.panels.custom_combo_box import CustomComboBox

from PyQt6.QtCore import pyqtSignal, Qt
from backend.kinematics import KinematicChain, DHJoint
from backend.logger import get_logger
import numpy as np

log = get_logger(__name__)


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
        _SPIN = _KCP_SPIN
        _COMBO = _KCP_COMBO
        _KEY = "color: #89929b; font-size: 10px; font-weight: 600;"
        _BTN_ICON = """
            QPushButton { background: #353535; color: #bfc7d2; border: none; border-radius: 3px;
                font-size: 12px; font-weight: bold; min-width:26px; max-width:26px; min-height:22px; padding:0; }
            QPushButton:hover { background: #454548; color: #e5e2e1; }
        """
        _BTN_DEL = """
            QPushButton { background: transparent; color: #e74c3c; border: 1px solid #93000a;
                border-radius: 3px; font-size: 10px; font-weight: 600; padding: 3px 8px; }
            QPushButton:hover { background: #93000a; color: #ffdad6; }
        """
        from PyQt6.QtWidgets import QFrame
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        # ── Card frame ────────────────────────────────────────────────────
        card = QFrame()
        card.setStyleSheet("QFrame { background: #202020; border-radius: 5px; }")
        card_v = QVBoxLayout(card)
        card_v.setContentsMargins(0, 0, 0, 0)
        card_v.setSpacing(0)

        # Header
        header = QFrame()
        header.setStyleSheet("QFrame { background: #2a2a2a; border-radius: 5px 5px 0 0; }")
        h_row = QHBoxLayout(header)
        h_row.setContentsMargins(10, 6, 8, 6)
        title_lbl = QLabel(f"<b style='color:#e5e2e1;'>J{self.index+1}</b> <span style='color:#89929b;'>{self.joint.name}</span>")
        title_lbl.setStyleSheet("font-size: 11px;")
        h_row.addWidget(title_lbl)
        self._title_lbl = title_lbl
        h_row.addStretch()
        card_v.addWidget(header)

        # Body
        body = QFrame()
        body.setStyleSheet("QFrame { background: #202020; border-radius: 0 0 5px 5px; }")
        grid = QGridLayout(body)
        grid.setContentsMargins(10, 8, 10, 8)
        grid.setSpacing(5)

        def _lbl(text):
            l = QLabel(text)
            l.setStyleSheet(_KEY)
            l.setFixedWidth(58)
            return l

        # Type
        grid.addWidget(_lbl("Type"), 0, 0)
        self.combo_type = CustomComboBox()
        self.combo_type.addItems(["revolute", "prismatic", "fixed"])
        self.combo_type.setCurrentText(self.joint.type)
        self.combo_type.setStyleSheet(_COMBO)
        self.combo_type.currentTextChanged.connect(self._on_type_changed)
        grid.addWidget(self.combo_type, 0, 1)

        # θ
        grid.addWidget(_lbl("θ  deg"), 1, 0)
        self.spin_theta = QDoubleSpinBox()
        self.spin_theta.setRange(-180.0, 180.0)
        self.spin_theta.setSingleStep(1.0)
        self.spin_theta.setStyleSheet(_SPIN)
        self.spin_theta.valueChanged.connect(self._on_param_changed)
        grid.addWidget(self.spin_theta, 1, 1)

        # d
        grid.addWidget(_lbl("d  m"), 2, 0)
        self.spin_d = QDoubleSpinBox()
        self.spin_d.setRange(0.0, 2.0)
        self.spin_d.setSingleStep(0.01)
        self.spin_d.setDecimals(3)
        self.spin_d.setStyleSheet(_SPIN)
        self.spin_d.valueChanged.connect(self._on_param_changed)
        grid.addWidget(self.spin_d, 2, 1)

        # a
        grid.addWidget(_lbl("a  m"), 3, 0)
        self.spin_a = QDoubleSpinBox()
        self.spin_a.setRange(0.0, 2.0)
        self.spin_a.setSingleStep(0.01)
        self.spin_a.setDecimals(3)
        self.spin_a.setStyleSheet(_SPIN)
        self.spin_a.valueChanged.connect(self._on_param_changed)
        grid.addWidget(self.spin_a, 3, 1)

        # α
        grid.addWidget(_lbl("α  deg"), 4, 0)
        self.spin_alpha = QDoubleSpinBox()
        self.spin_alpha.setRange(-180.0, 180.0)
        self.spin_alpha.setSingleStep(1.0)
        self.spin_alpha.setStyleSheet(_SPIN)
        self.spin_alpha.valueChanged.connect(self._on_param_changed)
        grid.addWidget(self.spin_alpha, 4, 1)

        # Name (QLineEdit)
        from PyQt6.QtWidgets import QLineEdit
        grid.addWidget(_lbl("Name"), 5, 0)
        self.edit_name = QLineEdit()
        self.edit_name.setText(self.joint.name)
        self.edit_name.setStyleSheet(
            "QLineEdit { background: #0e0e0e; color: #e5e2e1; border: none; border-radius: 2px; "
            "padding: 4px 6px; font-size: 11px; }"
            "QLineEdit:focus { border-bottom: 2px solid #3498db; }"
        )
        self.edit_name.textChanged.connect(self._on_param_changed)
        grid.addWidget(self.edit_name, 5, 1)

        # Separator + button row
        sep = QFrame()
        sep.setFrameShape(QFrame.Shape.HLine)
        sep.setStyleSheet("background: #353535; border: none; max-height: 1px;")
        grid.addWidget(sep, 6, 0, 1, 2)

        hbox = QHBoxLayout()
        hbox.setSpacing(4)
        btn_up = QPushButton("↑")
        btn_up.setStyleSheet(_BTN_ICON)
        btn_up.clicked.connect(lambda: self.move_up_requested.emit(self.index))
        hbox.addWidget(btn_up)
        btn_down = QPushButton("↓")
        btn_down.setStyleSheet(_BTN_ICON)
        btn_down.clicked.connect(lambda: self.move_down_requested.emit(self.index))
        hbox.addWidget(btn_down)
        hbox.addStretch()
        btn_del = QPushButton("✕ Delete")
        btn_del.setStyleSheet(_BTN_DEL)
        btn_del.clicked.connect(lambda: self.delete_requested.emit(self.index))
        hbox.addWidget(btn_del)
        grid.addLayout(hbox, 7, 0, 1, 2)

        card_v.addWidget(body)
        layout.addWidget(card)

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
        if hasattr(self, '_title_lbl'):
            self._title_lbl.setText(
                f"<b style='color:#e5e2e1;'>J{self.index+1}</b> <span style='color:#89929b;'>{self.joint.name}</span>"
            )
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
        hbox.setSpacing(8)
        lbl = QLabel(f"J{self.index+1}")
        lbl.setFixedWidth(28)
        lbl.setStyleSheet("color: #92ccff; font-weight: 700; font-size: 11px; font-family: monospace;")
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
        self.slider.setStyleSheet(_KCP_SLIDER)
        self.slider.valueChanged.connect(self._on_slider_moved)

        self.spin = QDoubleSpinBox()
        self.spin.setRange(vmin, vmax)
        self.spin.setSingleStep(step)
        self.spin.setDecimals(3 if step < 0.1 else 1)
        self.spin.setValue(default)
        self.spin.setStyleSheet(_KCP_SPIN)
        self.spin.setFixedWidth(80)
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


# ── Kinematic chain panel style tokens ───────────────────────────────────────
_KCP_SLIDER = """
    QSlider::groove:horizontal { background: #202020; height: 4px; border-radius: 2px; }
    QSlider::sub-page:horizontal { background: #3498db; border-radius: 2px; }
    QSlider::handle:horizontal { background: #92ccff; width: 14px; height: 14px; margin: -5px 0; border-radius: 7px; }
"""
_KCP_SPIN = """
    QDoubleSpinBox { background: #0e0e0e; color: #e5e2e1; border: none; border-radius: 2px; padding: 3px 6px; font-size: 11px; }
    QDoubleSpinBox:focus { border-bottom: 2px solid #3498db; }
    QDoubleSpinBox::up-button, QDoubleSpinBox::down-button { width: 14px; background: #1b1b1c; border: none; }
"""
_KCP_COMBO = """
    QComboBox { background: #0e0e0e; color: #e5e2e1; border: none; border-radius: 3px; padding: 5px 8px; font-size: 11px; }
    QComboBox::drop-down { border: none; }
    QComboBox QAbstractItemView { background: #202020; color: #e5e2e1; selection-background-color: #3498db; border: 1px solid #353535; }
"""
_KCP_BTN_ADD = """
    QPushButton { background-color: #27ae60; color: #ffffff; font-weight: 700; font-size: 11px; padding: 6px 14px; border: none; border-radius: 4px; }
    QPushButton:hover { background-color: #2ecc71; }
"""
_KCP_LABEL = "color: #89929b; font-size: 10px; font-weight: 600; letter-spacing: 0.06em;"
_KCP_MONO = "color: #92ccff; font-family: 'Consolas', monospace; font-size: 12px;"


class KinematicChainPanel(QWidget):
    """Panel for building a custom kinematic chain with DH parameters."""

    chain_updated = pyqtSignal(KinematicChain)
    end_effector_updated = pyqtSignal(np.ndarray)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setStyleSheet("background-color: transparent;")
        self.chain = KinematicChain(base_height=0.1)
        self._variable_joint_sliders = []

        self._setup_ui()
        # Add a default joint to get started
        self._on_add_joint()
        self._update_sliders()
        # Connect internal signals: any chain update triggers FK and label update
        self.chain_updated.connect(self._compute_and_emit_fk)
        self.end_effector_updated.connect(self._update_ee_label)
        # Compute initial FK to populate end-effector label
        self._compute_and_emit_fk()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(10)

        # ── Add joint row ──────────────────────────────────────────────────
        lbl_add = QLabel("ADD JOINT")
        lbl_add.setStyleSheet(_KCP_LABEL)
        layout.addWidget(lbl_add)

        h_top = QHBoxLayout()
        h_top.setSpacing(6)
        self.combo_new_type = CustomComboBox()
        self.combo_new_type.addItems(["revolute", "prismatic", "fixed"])
        self.combo_new_type.setStyleSheet(_KCP_COMBO)
        h_top.addWidget(self.combo_new_type)
        btn_add = QPushButton("+ Add Joint")
        btn_add.setStyleSheet(_KCP_BTN_ADD)
        btn_add.clicked.connect(self._on_add_joint)
        h_top.addWidget(btn_add)
        layout.addLayout(h_top)

        # ── Joint cards scroll area ───────────────────────────────────────
        lbl_chain = QLabel("CHAIN")
        lbl_chain.setStyleSheet(_KCP_LABEL)
        layout.addWidget(lbl_chain)

        self.scroll = QScrollArea()
        self.scroll.setWidgetResizable(True)
        self.scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.scroll.setStyleSheet("QScrollArea { border: none; background: transparent; } QWidget { background: transparent; }")
        self.scroll.setMaximumHeight(320)
        self.cards_container = QWidget()
        self.cards_container.setStyleSheet("background: transparent;")
        self.cards_layout = QVBoxLayout(self.cards_container)
        self.cards_layout.setSpacing(4)
        self.cards_layout.setContentsMargins(0, 0, 0, 0)
        self.scroll.setWidget(self.cards_container)
        layout.addWidget(self.scroll, 1)

        # ── Separator ────────────────────────────────────────────────────
        from PyQt6.QtWidgets import QFrame
        sep = QFrame()
        sep.setFrameShape(QFrame.Shape.HLine)
        sep.setStyleSheet("background: #353535; border: none; max-height: 1px;")
        layout.addWidget(sep)

        # ── Joint sliders section ────────────────────────────────────────
        lbl_ctrl = QLabel("JOINT CONTROL")
        lbl_ctrl.setStyleSheet(_KCP_LABEL)
        layout.addWidget(lbl_ctrl)

        self.sliders_container = QWidget()
        self.sliders_container.setStyleSheet("background: transparent;")
        self.sliders_layout = QVBoxLayout(self.sliders_container)
        self.sliders_layout.setContentsMargins(0, 0, 0, 0)
        self.sliders_layout.setSpacing(6)
        layout.addWidget(self.sliders_container)
        # keep group reference for compatibility
        self.sliders_group = self.sliders_container

        # ── End-effector readout ─────────────────────────────────────────
        self.lbl_ee = QLabel("EE: X=0.000  Y=0.000  Z=0.000")
        self.lbl_ee.setStyleSheet(_KCP_MONO)
        layout.addWidget(self.lbl_ee)

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
        log.info("Joint added | index=%d | %r", len(self.chain.joints) - 1, joint)
        self._rebuild_cards()
        self._update_sliders()
        self.chain_updated.emit(self.chain)
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
        log.info("Joint deleted | index=%d | %r", index, self.chain.joints[index])
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
        log.debug("Slider J%d changed | value=%.3f", index + 1, value)
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
            log.debug("FK computed | EE=(%.4f, %.4f, %.4f)", tip_pos[0], tip_pos[1], tip_pos[2])
            self.end_effector_updated.emit(tip_pos)
        except Exception as e:
            log.error("FK error: %s", e, exc_info=True)

    def _update_ee_label(self, pos):
        """Update end-effector position label."""
        self.lbl_ee.setText(f"EE: X={pos[0]:+.3f}  Y={pos[1]:+.3f}  Z={pos[2]:+.3f}")

    def set_chain(self, chain: KinematicChain):
        """Replace the current chain (used for loading presets)."""
        self.chain = chain
        self._rebuild_cards()
        self._update_sliders()
        self.chain_updated.emit(self.chain)
        self._compute_and_emit_fk()

    def get_chain(self) -> KinematicChain:
        return self.chain
