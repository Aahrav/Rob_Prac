#!/usr/bin/env python3
"""
MainWindow — Robotic Arm Simulation.
Kinetic Obsidian dark engineering theme (Stitch "Kinetic Monolith" design system).
"""

import sys
from PyQt6.QtWidgets import (QMainWindow, QWidget, QSplitter, QVBoxLayout,
                              QHBoxLayout, QLabel, QPushButton, QCheckBox,
                              QButtonGroup, QRadioButton, QScrollArea,
                              QStatusBar, QFrame, QSizePolicy, QMenu, QMessageBox)
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QFont, QFontDatabase, QColor, QAction

from backend.kinematics import compute_arm_positions, ArmConfig, inverse_kinematics_3dof, KinematicChain
from frontend.panels.robot_config_panel import RobotConfigPanel
from frontend.panels.kinematic_chain_panel import KinematicChainPanel
from frontend.panels.accordion_section import AccordionSection


# ═══════════════════════════════════════════════════════════════════════════════
#  Global dark engineering stylesheet  (Kinetic Obsidian)
# ═══════════════════════════════════════════════════════════════════════════════
GLOBAL_QSS = """
/* ── Base ─────────────────────────────────────────────────────────────── */
QMainWindow, QWidget {
    background-color: #131313;
    color: #e5e2e1;
    font-family: "Inter", "Segoe UI", Arial, sans-serif;
    font-size: 11px;
}

/* ── Scrollbars ───────────────────────────────────────────────────────── */
QScrollBar:vertical {
    background: #131313;
    width: 8px;
    margin: 0;
    border: none;
}
QScrollBar::handle:vertical {
    background: #353535;
    border-radius: 4px;
    min-height: 30px;
}
QScrollBar::handle:vertical:hover { background: #454548; }
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }
QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical { background: none; }

QScrollBar:horizontal {
    background: #131313;
    height: 6px;
    border: none;
}
QScrollBar::handle:horizontal { background: #353535; border-radius: 3px; }

/* ── Splitter ─────────────────────────────────────────────────────────── */
QSplitter::handle { background-color: #0e0e0e; width: 2px; }
QSplitter::handle:horizontal { width: 2px; }

/* ── Tooltips ─────────────────────────────────────────────────────────── */
QToolTip {
    background-color: #202020;
    color: #e5e2e1;
    border: 1px solid #353535;
    border-radius: 4px;
    padding: 4px 8px;
    font-size: 11px;
}

/* ── Status bar ───────────────────────────────────────────────────────── */
QStatusBar {
    background-color: #0e0e0e;
    color: #89929b;
    border-top: 1px solid #1a1a1a;
    font-size: 10px;
}
QStatusBar::item { border: none; }

/* ── Menu bar ─────────────────────────────────────────────────────────── */
QMenuBar {
    background-color: #0e0e0e;
    color: #bfc7d2;
    border-bottom: 1px solid #1a1a1a;
    padding: 2px 0;
}
QMenuBar::item:selected { background-color: #202020; color: #e5e2e1; }
QMenu {
    background-color: #202020;
    color: #e5e2e1;
    border: 1px solid #353535;
    border-radius: 4px;
}
QMenu::item:selected { background-color: #3498db; color: #ffffff; }
QMenu::separator { background-color: #353535; height: 1px; margin: 4px 8px; }

/* ── Checkboxes ───────────────────────────────────────────────────────── */
QCheckBox { color: #bfc7d2; spacing: 6px; }
QCheckBox::indicator {
    width: 14px; height: 14px;
    border: 1px solid #353535;
    border-radius: 3px;
    background: #0e0e0e;
}
QCheckBox::indicator:checked { background: #3498db; border-color: #3498db; }

/* ── Radio Buttons ────────────────────────────────────────────────────── */
QRadioButton { color: #bfc7d2; spacing: 6px; }
QRadioButton::indicator {
    width: 14px; height: 14px;
    border: 1px solid #353535;
    border-radius: 7px;
    background: #0e0e0e;
}
QRadioButton::indicator:checked { background: #3498db; border-color: #92ccff; }

/* ── Message boxes ────────────────────────────────────────────────────── */
QMessageBox { background-color: #202020; }
QMessageBox QPushButton {
    background-color: #353535; color: #e5e2e1;
    border: none; border-radius: 4px;
    padding: 6px 16px; min-width: 80px;
}
QMessageBox QPushButton:hover { background-color: #454548; }
"""

# ── Toolbar button style ───────────────────────────────────────────────────
TOOLBAR_BTN = """
    QPushButton {
        background-color: #202020;
        color: #89929b;
        border: none;
        border-radius: 4px;
        padding: 4px 12px;
        font-size: 10px;
        font-weight: 500;
        min-height: 26px;
    }
    QPushButton:hover { background-color: #2a2a2a; color: #e5e2e1; }
    QPushButton:pressed { background-color: #131313; }
    QPushButton:checked { background-color: #3498db; color: #ffffff; }
"""

TOOLBAR_BTN_RESET = """
    QPushButton {
        background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
            stop:0 #3498db, stop:1 #2980b9);
        color: #ffffff;
        border: none;
        border-radius: 4px;
        padding: 4px 14px;
        font-size: 10px;
        font-weight: 600;
        min-height: 26px;
    }
    QPushButton:hover { background-color: #2980b9; }
"""

MODE_PILL_ACTIVE = """
    QPushButton {
        background-color: #353535;
        color: #92ccff;
        border: 1px solid #3498db;
        border-radius: 4px;
        padding: 4px 12px;
        font-size: 11px;
        font-weight: 600;
        min-height: 26px;
    }
"""

MODE_PILL_INACTIVE = """
    QPushButton {
        background-color: #131313;
        color: #89929b;
        border: 1px solid #353535;
        border-radius: 4px;
        padding: 4px 12px;
        font-size: 11px;
        font-weight: 500;
        min-height: 26px;
    }
    QPushButton:hover { background-color: #202020; color: #bfc7d2; border-color: #454548; }
"""


class MainWindow(QMainWindow):
    """Main application window — Kinetic Obsidian split-panel layout."""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("RoboSim — Robotic Arm Simulation")
        self.resize(1400, 900)
        self.setMinimumSize(1100, 650)

        # Apply global stylesheet
        self.setStyleSheet(GLOBAL_QSS)

        # ── Shared state ──────────────────────────────────────────────────
        self.kinematics_config = ArmConfig()
        self.current_angles = [0.0, 0.0, 0.0]
        self.mode = 'standard'
        self.simulator = None
        self.interactive_controller = None

        # P3-T2: Calibration offsets — subtracted from raw r/p/y before filter.
        # Set by _on_calibrate(); reset to [0,0,0] on new connect.
        # Stores RAW angles at click-time (before any filter, since filter is
        # not yet integrated; Part 2 P2-T5 subtracts these in _on_sample_received).
        self.calib_offset: list[float] = [0.0, 0.0, 0.0]

        # ── Central widget ────────────────────────────────────────────────
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        # ── Menu bar (P3-T7) ──────────────────────────────────────────────
        self._setup_menu_bar()

        # ── Title bar ─────────────────────────────────────────────────────
        title_bar = self._make_title_bar()
        root.addWidget(title_bar)

        # ── Splitter ──────────────────────────────────────────────────────
        self.splitter = QSplitter(Qt.Orientation.Horizontal)
        self.splitter.setHandleWidth(2)

        # LEFT sidebar
        self.left_scroll = QScrollArea()
        self.left_scroll.setWidgetResizable(True)
        self.left_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.left_scroll.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        self.left_scroll.setStyleSheet("QScrollArea { border: none; background-color: #131313; }")

        self.left_content = QWidget()
        self.left_content.setStyleSheet("background-color: #131313;")
        self.left_content.setMinimumWidth(280)
        self.left_layout = QVBoxLayout(self.left_content)
        self.left_layout.setContentsMargins(0, 0, 0, 0)
        self.left_layout.setSpacing(0)
        self.left_scroll.setWidget(self.left_content)

        # RIGHT panel
        self.right_panel = QWidget()
        self.right_panel.setStyleSheet("background-color: #131313;")
        self.right_layout = QVBoxLayout(self.right_panel)
        self.right_layout.setContentsMargins(0, 0, 0, 0)
        self.right_layout.setSpacing(0)

        self.splitter.addWidget(self.left_scroll)
        self.splitter.addWidget(self.right_panel)
        self.splitter.setStretchFactor(0, 0)
        self.splitter.setStretchFactor(1, 1)
        self.splitter.setSizes([340, 1060])

        root.addWidget(self.splitter, 1)

        # ── Status bar ────────────────────────────────────────────────────
        self._setup_status_bar()

        # ── Build panels ──────────────────────────────────────────────────
        self._build_left_panel()
        self._setup_right_panel()

    # ═══════════════════════════════════════════════════════════════════════
    #  UI Construction
    # ═══════════════════════════════════════════════════════════════════════

    def _make_title_bar(self):
        bar = QFrame()
        bar.setFixedHeight(46)
        bar.setStyleSheet("""
            QFrame {
                background-color: #0e0e0e;
                border-bottom: 1px solid #1a1a1a;
            }
        """)
        layout = QHBoxLayout(bar)
        layout.setContentsMargins(16, 0, 16, 0)
        layout.setSpacing(12)

        # App icon placeholder
        icon_lbl = QLabel("⚙")
        icon_lbl.setStyleSheet("color: #3498db; font-size: 18px; font-weight: 700;")
        layout.addWidget(icon_lbl)

        app_name = QLabel("RoboSim")
        app_name.setStyleSheet("color: #e5e2e1; font-size: 14px; font-weight: 700; letter-spacing: 0.02em;")
        layout.addWidget(app_name)

        subtitle = QLabel("Robotic Arm Simulation")
        subtitle.setStyleSheet("color: #3f4850; font-size: 11px;")
        layout.addWidget(subtitle)

        layout.addStretch()

        # Mode toggle pills (Standard 3-DOF | Custom DH)
        mode_lbl = QLabel("MODE:")
        mode_lbl.setStyleSheet("color: #3f4850; font-size: 10px; font-weight: 600; letter-spacing: 0.05em;")
        layout.addWidget(mode_lbl)

        self.btn_standard = QPushButton("Standard 3-DOF")
        self.btn_standard.setStyleSheet(MODE_PILL_ACTIVE)
        self.btn_standard.clicked.connect(lambda: self._set_robot_mode('standard'))
        layout.addWidget(self.btn_standard)

        self.btn_custom = QPushButton("Custom DH")
        self.btn_custom.setStyleSheet(MODE_PILL_INACTIVE)
        self.btn_custom.clicked.connect(lambda: self._set_robot_mode('custom'))
        layout.addWidget(self.btn_custom)

        return bar

    def _setup_status_bar(self):
        """3-zone status bar."""
        sb = self.statusBar()
        sb.setSizeGripEnabled(False)

        # Zone 1: mode indicator
        self.sb_mode = QLabel("●  Standard 3-DOF  |  Disconnected")
        self.sb_mode.setStyleSheet("color: #e74c3c; padding: 0 12px; font-size: 10px;")
        sb.addWidget(self.sb_mode)

        # Zone 2: message (permanent, center)
        self.sb_message = QLabel("Ready")
        self.sb_message.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.sb_message.setStyleSheet("color: #89929b; font-size: 10px;")
        sb.addWidget(self.sb_message, 1)

        # Zone 3: EE position
        self.sb_ee = QLabel("EE: (—, —, —)")
        self.sb_ee.setStyleSheet("color: #3f4850; padding: 0 12px; font-size: 10px;")
        sb.addPermanentWidget(self.sb_ee)

    def _build_left_panel(self):
        """Build all accordion sections in the left sidebar."""

        # ── SECTION 1: Mode & Connection ──────────────────────────────────
        self.section_mode = AccordionSection("Mode & Connection")

        from frontend.panels.connection_panel import ConnectionPanel
        self.connection_panel = ConnectionPanel()
        self.connection_panel.connect_requested.connect(self._on_connect_requested)
        self.connection_panel.disconnect_requested.connect(self._on_disconnect_requested)
        self.connection_panel.mode_changed.connect(self._on_mode_changed)

        mode_layout = QVBoxLayout()
        mode_layout.setContentsMargins(0, 0, 0, 0)
        mode_layout.setSpacing(8)
        mode_layout.addWidget(self.connection_panel)
        self.section_mode.setContentLayout(mode_layout)
        self.left_layout.addWidget(self.section_mode)

        # ── SECTION 2: Robot Parameters ───────────────────────────────────
        self.section_robot_params = AccordionSection("Robot Parameters")
        self.robot_config_panel = RobotConfigPanel(self.kinematics_config)
        self.robot_config_panel.config_changed.connect(self._on_robot_config_changed)

        from frontend.panels.data_panel import DataPanel
        self.data_panel = DataPanel()

        params_layout = QVBoxLayout()
        params_layout.setContentsMargins(0, 0, 0, 0)
        params_layout.setSpacing(10)
        params_layout.addWidget(self.robot_config_panel)

        # ── Calibration row (P3-T2) ───────────────────────────────────────
        calib_row = QHBoxLayout()
        calib_row.setSpacing(8)

        self.btn_calibrate = QPushButton("⊕  Calibrate")
        self.btn_calibrate.setStyleSheet("""
            QPushButton {
                background-color: #202020;
                color: #f39c12;
                border: 1px solid #353535;
                border-radius: 4px;
                padding: 5px 12px;
                font-size: 10px;
                font-weight: 600;
                min-height: 26px;
            }
            QPushButton:hover { background-color: #2a2a2a; border-color: #f39c12; }
            QPushButton:pressed { background-color: #131313; }
        """)
        self.btn_calibrate.setToolTip(
            "Capture current angles as zero-reference.\n"
            "Future samples will subtract these offsets."
        )
        self.btn_calibrate.clicked.connect(self._on_calibrate)
        calib_row.addWidget(self.btn_calibrate)

        self.lbl_calib_status = QLabel("offsets: 0.0 / 0.0 / 0.0")
        self.lbl_calib_status.setStyleSheet(
            "color: #3f4850; font-size: 9px; font-family: 'Consolas', monospace;"
        )
        calib_row.addWidget(self.lbl_calib_status, 1)
        params_layout.addLayout(calib_row)

        # Joint angles sub-label
        lbl_joints = QLabel("JOINT ANGLES")
        lbl_joints.setStyleSheet("color: #89929b; font-size: 10px; font-weight: 600; letter-spacing: 0.06em;")
        params_layout.addWidget(lbl_joints)
        params_layout.addWidget(self.data_panel)

        self.section_robot_params.setContentLayout(params_layout)
        self.left_layout.addWidget(self.section_robot_params)

        # ── SECTION 3: Robot Structure (Custom DH) ────────────────────────
        self.section_chain = AccordionSection("Kinematic Chain")
        self.chain_panel = KinematicChainPanel()
        self.chain_panel.chain_updated.connect(self._on_chain_updated)
        self.chain_panel.end_effector_updated.connect(self._on_ee_updated)

        chain_layout = QVBoxLayout()
        chain_layout.setContentsMargins(0, 0, 0, 0)
        chain_layout.addWidget(self.chain_panel)
        self.section_chain.setContentLayout(chain_layout)
        self.section_chain.setVisible(False)
        self.left_layout.addWidget(self.section_chain)

        # ── SECTION 4: Trajectory Control ─────────────────────────────────
        self.section_joint_control = AccordionSection("Trajectory Control")

        from frontend.panels.trajectory_panel import TrajectoryPanel
        self.trajectory_panel = TrajectoryPanel(config=self.kinematics_config)
        self.trajectory_panel.target_angles_updated.connect(self._on_target_angles)

        joint_layout = QVBoxLayout()
        joint_layout.setContentsMargins(0, 0, 0, 0)
        joint_layout.addWidget(self.trajectory_panel)
        self.section_joint_control.setContentLayout(joint_layout)
        self.left_layout.addWidget(self.section_joint_control)

        # ── SECTION 5: End-Effector ───────────────────────────────────────
        self.section_ee = AccordionSection("End Effector")

        # EE info card
        ee_card = QFrame()
        ee_card.setStyleSheet("""
            QFrame {
                background-color: #0e0e0e;
                border-radius: 4px;
                border-left: 2px solid #3498db;
            }
        """)
        ee_card_layout = QVBoxLayout(ee_card)
        ee_card_layout.setContentsMargins(10, 8, 10, 8)
        ee_card_layout.setSpacing(4)

        self.lbl_ee_status = QLabel("— awaiting data —")
        self.lbl_ee_status.setStyleSheet(
            "color: #89929b; font-family: 'Consolas', monospace; font-size: 12px;"
        )
        self.lbl_ee_status.setWordWrap(True)
        ee_card_layout.addWidget(self.lbl_ee_status)

        ee_layout = QVBoxLayout()
        ee_layout.setContentsMargins(0, 0, 0, 0)
        ee_layout.addWidget(ee_card)
        self.section_ee.setContentLayout(ee_layout)
        self.left_layout.addWidget(self.section_ee)

        self.left_layout.addStretch()

    def _setup_right_panel(self):
        """Camera toolbar + 3D canvas."""
        from frontend.panels.arm_canvas import ArmCanvas

        # ── Viewport toolbar ───────────────────────────────────────────────
        toolbar = QFrame()
        toolbar.setFixedHeight(42)
        toolbar.setStyleSheet("""
            QFrame {
                background-color: #0e0e0e;
                border-bottom: 1px solid #1a1a1a;
            }
        """)
        tb_layout = QHBoxLayout(toolbar)
        tb_layout.setContentsMargins(12, 0, 12, 0)
        tb_layout.setSpacing(6)

        # Camera label
        cam_lbl = QLabel("CAMERA:")
        cam_lbl.setStyleSheet("color: #3f4850; font-size: 10px; font-weight: 600; letter-spacing: 0.05em;")
        tb_layout.addWidget(cam_lbl)

        # View preset buttons
        for name, label in [('iso', 'Iso'), ('front', 'Front'), ('side', 'Side'), ('top', 'Top'), ('back', 'Back')]:
            btn = QPushButton(label)
            btn.setStyleSheet(TOOLBAR_BTN)
            btn.clicked.connect(lambda checked, n=name: self._set_view_preset(n))
            tb_layout.addWidget(btn)

        # Separator
        sep = QFrame()
        sep.setFrameShape(QFrame.Shape.VLine)
        sep.setStyleSheet("color: #353535; background: #353535; border: none; max-width: 1px;")
        tb_layout.addWidget(sep)

        # Ground toggle
        self.chk_ground = QCheckBox("Ground")
        self.chk_ground.setChecked(True)
        self.chk_ground.setStyleSheet("color: #bfc7d2; font-size: 10px; spacing: 5px;")
        self.chk_ground.toggled.connect(self._toggle_ground)
        tb_layout.addWidget(self.chk_ground)

        tb_layout.addSeparator = lambda: None  # dummy for spacing
        tb_layout.addStretch()

        # Reset view button
        self.btn_reset_view = QPushButton("⟳  Reset View")
        self.btn_reset_view.setStyleSheet(TOOLBAR_BTN_RESET)
        self.btn_reset_view.clicked.connect(self._reset_view)
        tb_layout.addWidget(self.btn_reset_view)

        self.right_layout.addWidget(toolbar)

        # ── 3D Canvas ─────────────────────────────────────────────────────
        self.arm_canvas = ArmCanvas()
        self.arm_canvas.config = self.kinematics_config
        self.arm_canvas.setMinimumSize(600, 500)
        self.right_layout.addWidget(self.arm_canvas, stretch=1)

        # ── Wire up remaining connections ──────────────────────────────────
        self.connection_panel.connect_requested.connect(self._on_connect_requested)
        self.connection_panel.disconnect_requested.connect(self._on_disconnect_requested)
        self.connection_panel.mode_changed.connect(self._on_mode_changed)
        self.trajectory_panel.target_angles_updated.connect(self._on_target_angles)
        self._on_robot_config_changed(self.kinematics_config)
        self._update_panel_visibility()

    # ═══════════════════════════════════════════════════════════════════════
    #  Mode management
    # ═══════════════════════════════════════════════════════════════════════

    def _set_robot_mode(self, mode: str):
        """Switch between 'standard' and 'custom' robot modes."""
        if mode == self.mode:
            return
        self.mode = mode

        if mode == 'standard':
            self.btn_standard.setStyleSheet(MODE_PILL_ACTIVE)
            self.btn_custom.setStyleSheet(MODE_PILL_INACTIVE)
        else:
            self.btn_standard.setStyleSheet(MODE_PILL_INACTIVE)
            self.btn_custom.setStyleSheet(MODE_PILL_ACTIVE)

        self._update_panel_visibility()
        self._configure_trajectory_panel_mode()
        self._update_view_for_robot()
        self._refresh_arm_display()
        self._update_status_bar_mode()
        self.centralWidget().repaint()

    def _on_mode_changed(self, mode: str):
        """Connection mode changed (simulation/interactive)."""
        self._update_panel_visibility()
        self._configure_trajectory_panel_mode()
        try:
            self.connection_panel.mode_combo.clearFocus()
        except Exception:
            pass
        self._update_status_bar_mode()
        self.centralWidget().repaint()

    def _update_panel_visibility(self):
        if self.mode == 'standard':
            self.section_robot_params.setVisible(True)
            is_interactive = (self.connection_panel.mode_combo.currentText() == "Interactive")
            self.section_joint_control.setVisible(is_interactive)
            self.section_chain.setVisible(False)
        else:
            self.section_robot_params.setVisible(False)
            is_interactive = (self.connection_panel.mode_combo.currentText() == "Interactive")
            self.section_joint_control.setVisible(is_interactive)
            self.section_chain.setVisible(True)

    def _update_status_bar_mode(self):
        robot_mode = "Standard 3-DOF" if self.mode == 'standard' else "Custom DH"
        conn_mode = self.connection_panel._current_mode
        is_connected = self.connection_panel.is_connected
        color = "#2ecc71" if is_connected else "#e74c3c"
        dot = "●"
        self.sb_mode.setText(f"{dot}  {robot_mode}  |  {conn_mode}")
        self.sb_mode.setStyleSheet(f"color: {color}; padding: 0 12px; font-size: 10px;")

    # ═══════════════════════════════════════════════════════════════════════
    #  Connection logic (preserved from original)
    # ═══════════════════════════════════════════════════════════════════════

    def _on_connect_requested(self, port: str, baud: int):
        self._stop_current_mode()
        if port == "INTERACTIVE":
            self._start_interactive()
        elif port == "SIMULATED (no hardware)":
            self._start_simulation()
        else:
            self.connection_panel.set_status(f"Real serial not yet implemented: {port}")
            self.connection_panel.set_connected(False)
        self._update_status_bar_mode()

    def _on_disconnect_requested(self):
        self._stop_current_mode()
        self.connection_panel.set_connected(False)
        self.connection_panel.set_status("Disconnected")
        self.sb_message.setText("Disconnected")
        self._update_status_bar_mode()

    def _start_simulation(self):
        import numpy as np
        from PyQt6.QtCore import QObject, pyqtSignal as Signal

        class Simulator(QObject):
            data_updated = Signal(float, float, float)

            def __init__(self, parent=None):
                super().__init__(parent)
                self.t = 0.0
                self.timer = QTimer(parent)
                self.timer.timeout.connect(self._update)

            def start(self):
                self.timer.start(50)

            def stop(self):
                self.timer.stop()

            def _update(self):
                self.t += 0.05
                self.data_updated.emit(
                    20 * np.sin(self.t * 0.5),
                    15 * np.sin(self.t * 0.7 + 1),
                    30 * np.sin(self.t * 0.3 + 2),
                )

        self.simulator = Simulator(self)
        self.simulator.data_updated.connect(self._on_data_received)
        self.simulator.start()
        self.connection_panel.set_connected(True)
        self.connection_panel.set_status("Simulation running")
        self.sb_message.setText("Simulation mode active")

    def _start_interactive(self):
        self.interactive_controller = type('IC', (), {'active': True, 'target': [0.0, 0.0, 0.0]})()
        self.connection_panel.set_connected(True)
        self.connection_panel.set_status("Interactive — use sliders")
        self.sb_message.setText("Interactive mode active")

    def _stop_current_mode(self):
        if self.simulator:
            self.simulator.stop()
            self.simulator = None
        if self.interactive_controller:
            self.interactive_controller.active = False
            self.interactive_controller = None
        if hasattr(self, 'trajectory_panel') and getattr(self.trajectory_panel, 'animating', False):
            self.trajectory_panel._stop_clicked()
        self.arm_canvas._init_empty_plot()

    # ═══════════════════════════════════════════════════════════════════════
    #  Data / kinematics handlers (preserved)
    # ═══════════════════════════════════════════════════════════════════════

    def _on_data_received(self, roll: float, pitch: float, yaw: float):
        # TODO: connect to Part 2 _on_sample_received — this fallback handles
        #       SimWorker data until the unified pipeline slot lands.
        self.data_panel.record_sample()   # P3-T3: increment PKT/S counter

        # P3-T2: Apply calibration offsets (raw angles, before future filter).
        # TODO: Part 2 P2-T5 should subtract calib_offset inside _on_sample_received
        #       before calling filter.step() — move this block there when it lands.
        roll  -= self.calib_offset[0]
        pitch -= self.calib_offset[1]
        yaw   -= self.calib_offset[2]

        self.data_panel.update_values(roll, pitch, yaw)
        self.trajectory_panel.set_current_angles(roll, pitch, yaw)
        self.current_angles = [roll, pitch, yaw]
        positions = compute_arm_positions(roll, pitch, yaw, config=self.kinematics_config)
        self.arm_canvas.draw_arm(positions)

    def _on_target_angles(self, q1: float, q2: float, q3: float):
        if self.interactive_controller and self.interactive_controller.active:
            self.data_panel.update_values(q1, q2, q3)
            if self.mode == 'standard':
                self._apply_target_angles(q1, q2, q3)
            else:
                chain = self.chain_panel.chain
                count = 0
                for joint in chain.joints:
                    if joint.type == 'revolute':
                        if count == 0: joint.theta = q1
                        elif count == 1: joint.theta = q2
                        elif count == 2: joint.theta = q3
                        count += 1
                    if count >= 3:
                        break
                self.trajectory_panel.set_current_angles(q1, q2, q3)
                self.chain_panel._compute_and_emit_fk()
                self._refresh_arm_display()

    def _apply_target_angles(self, q1: float, q2: float, q3: float):
        self.data_panel.update_values(q1, q2, q3)
        self.current_angles = [q1, q2, q3]
        positions = compute_arm_positions(q1, q2, q3, config=self.kinematics_config)
        self.arm_canvas.draw_arm(positions)
        tip = positions[-1]
        self._update_ee_display(tip[0], tip[1], tip[2])
        self.trajectory_panel.set_current_angles(q1, q2, q3)

    def _update_ee_display(self, x, y, z):
        self.lbl_ee_status.setText(
            f"<span style='color:#92ccff;'>X</span> {x:+.4f} m\n"
            f"<span style='color:#2ecc71;'>Y</span> {y:+.4f} m\n"
            f"<span style='color:#ffba4b;'>Z</span> {z:+.4f} m"
        )
        self.sb_ee.setText(f"EE: ({x:.3f}, {y:.3f}, {z:.3f})")

    def _on_robot_config_changed(self, config):
        self.kinematics_config = config
        max_reach = config.upper_arm_length + config.lower_arm_length + config.gripper_offset
        self.arm_canvas.update_workspace_boundary(max_reach)
        self.trajectory_panel.update_config(config)
        x, y, z = self.trajectory_panel.current_pos
        result = inverse_kinematics_3dof(x, y, z, config=config, elbow_down=True)
        if result:
            q1, q2, q3 = result
            self._apply_target_angles(q1, q2, q3)
        else:
            self.sb_message.setText("Current target unreachable with new dimensions")
        self._update_view_for_robot()

    def _configure_trajectory_panel_mode(self):
        if self.mode == 'custom':
            self.trajectory_panel.use_custom_chain = True
            self.trajectory_panel.chain = self.chain_panel.chain
            angles = []
            for joint in self.chain_panel.chain.joints:
                if joint.type == 'revolute':
                    angles.append(joint.theta)
                elif joint.type == 'prismatic':
                    angles.append(joint.d)
                if len(angles) >= 3:
                    break
            if len(angles) == 3:
                self.trajectory_panel.set_current_angles(angles[0], angles[1], angles[2])
        else:
            self.trajectory_panel.use_custom_chain = False
            self.trajectory_panel.chain = None
        self.trajectory_panel.update_workspace_ranges()

    def _update_view_for_robot(self):
        if self.mode == 'standard':
            cfg = self.kinematics_config
            max_xy = cfg.upper_arm_length + cfg.lower_arm_length + cfg.gripper_offset
            max_z = cfg.base_height + cfg.upper_arm_length + cfg.lower_arm_length + cfg.gripper_offset
            base_h = cfg.base_height
        else:
            chain = self.chain_panel.chain
            max_xy = sum(joint.a for joint in chain.joints)
            max_z = chain.base_height + max_xy
            base_h = chain.base_height
        self.arm_canvas.frame_to_fit_robot(max_xy, max_z, base_h)

    def _on_chain_updated(self, chain):
        if self.mode == 'custom':
            self._refresh_arm_display()
            angles = []
            for joint in chain.joints:
                if joint.type == 'revolute':
                    angles.append(joint.theta)
                elif joint.type == 'prismatic':
                    angles.append(joint.d)
                if len(angles) >= 3:
                    break
            if len(angles) == 3:
                self.trajectory_panel.set_current_angles(angles[0], angles[1], angles[2])
            self._update_view_for_robot()
            self.trajectory_panel.update_workspace_ranges()

    def _refresh_arm_display(self):
        if self.mode == 'standard':
            q1, q2, q3 = self.current_angles
            positions = compute_arm_positions(q1, q2, q3, config=self.kinematics_config)
            self.arm_canvas.draw_arm(positions)
            tip = positions[-1]
            self._update_ee_display(tip[0], tip[1], tip[2])
        else:
            positions = self.chain_panel.chain.joint_positions()
            self.arm_canvas.draw_chain(positions, base_height=self.chain_panel.chain.base_height)
            tip = positions[-1]
            self._update_ee_display(tip[0], tip[1], tip[2])

    def _on_ee_updated(self, pos):
        x, y, z = pos
        self._update_ee_display(x, y, z)

    # ═══════════════════════════════════════════════════════════════════════
    #  Viewport controls
    # ═══════════════════════════════════════════════════════════════════════

    def _reset_view(self):
        self._update_view_for_robot()
        self.arm_canvas.set_view(name='iso')
        self.sb_message.setText("View reset — Isometric")

    def _set_view_preset(self, name):
        if hasattr(self.arm_canvas, 'set_view'):
            self.arm_canvas.set_view(name=name)
            names = {'front': 'Front', 'side': 'Side', 'top': 'Top', 'iso': 'Isometric', 'back': 'Back'}
            self.sb_message.setText(f"View: {names.get(name, name)}")

    def _toggle_ground(self, checked):
        if hasattr(self.arm_canvas, 'toggle_ground'):
            self.arm_canvas.toggle_ground(checked)
            self.sb_message.setText("Ground " + ("shown" if checked else "hidden"))

    # ═══════════════════════════════════════════════════════════════════════
    #  P3-T2 — Calibration
    # ═══════════════════════════════════════════════════════════════════════

    def _on_calibrate(self):
        """Capture current raw angles as zero-reference offsets (P3-T2).

        After calibration, _on_data_received subtracts these values so the
        arm rests at its neutral pose.  Works for both Simulate and Replay.
        Offsets are reset to [0,0,0] when a new connection is started.
        """
        self.calib_offset = list(self.current_angles)   # snapshot raw r/p/y
        r, p, y = self.calib_offset
        self.lbl_calib_status.setText(f"offsets: {r:+.1f} / {p:+.1f} / {y:+.1f}")
        self.lbl_calib_status.setStyleSheet(
            "color: #f39c12; font-size: 9px; font-family: 'Consolas', monospace;"
        )
        self.sb_message.setText(f"Calibrated — offsets R={r:+.1f}° P={p:+.1f}° Y={y:+.1f}°")

    # ═══════════════════════════════════════════════════════════════════════
    #  P3-T5 — Error / status UX
    # ═══════════════════════════════════════════════════════════════════════

    def show_producer_error(self, message: str, fatal: bool = False) -> None:
        """Surface a producer error in the UI (P3-T5).

        Args:
            message: Human-readable error string from producer_error signal.
            fatal:   True  → QMessageBox.critical (blocking); also disconnects.
                     False → status bar only (non-blocking); used for skipped
                             malformed lines or parse warnings.

        TODO: connect to Part 2 producer_error signal once workers exist:
            worker.producer_error.connect(
                lambda msg: self.show_producer_error(msg, fatal=True)
            )
        """
        if fatal:
            self._stop_current_mode()
            self.connection_panel.set_connected(False)
            self.connection_panel.set_status("Error — connection stopped")
            self._update_status_bar_mode()
            QMessageBox.critical(
                self,
                "Replay / Producer Error",
                f"⚠  {message}\n\nThe data source has been stopped."
            )
        else:
            # Non-fatal: show transiently in status bar (non-blocking)
            self.sb_message.setText(f"⚠ {message}")
            self.sb_message.setStyleSheet("color: #f39c12; font-size: 10px;")
            # Reset style after 5 s so bar doesn’t stay amber forever
            QTimer.singleShot(
                5000,
                lambda: self.sb_message.setStyleSheet("color: #89929b; font-size: 10px;")
            )

    # ═══════════════════════════════════════════════════════════════════════
    #  P3-T7 — Help menu (About + Keyboard Shortcuts)
    # ═══════════════════════════════════════════════════════════════════════

    def _setup_menu_bar(self) -> None:
        """Create the application menu bar with a Help menu (P3-T7)."""
        menu_bar = self.menuBar()

        help_menu = menu_bar.addMenu("&Help")

        act_about = QAction("&About RoboSim", self)
        act_about.setShortcut("F1")
        act_about.triggered.connect(self._show_about)
        help_menu.addAction(act_about)

        act_shortcuts = QAction("&Keyboard Shortcuts", self)
        act_shortcuts.setShortcut("Ctrl+/")
        act_shortcuts.triggered.connect(self._show_shortcuts)
        help_menu.addAction(act_shortcuts)

    def _show_about(self) -> None:
        QMessageBox.about(
            self,
            "About RoboSim",
            "<b>RoboSim</b> — Robotic Arm Simulation Platform<br>"
            "<b>Version:</b> 0.1 (MVP — Software Phase)<br><br>"
            "Real-time simulation of a 3-DOF robotic arm driven by an<br>"
            "ESP32 + MPU-6050 sensor glove (hardware phase pending).<br><br>"
            "<b>Stack:</b> Python 3 · PyQt6 · Matplotlib · NumPy<br>"
            "<b>Theme:</b> Kinetic Obsidian (dark engineering)<br><br>"
            "Parallel build: Part 1 (pipeline) · Part 2 (workers) · "
            "Part 3 (UI/UX)"
        )

    def _show_shortcuts(self) -> None:
        QMessageBox.information(
            self,
            "Keyboard Shortcuts",
            "<b>Keyboard Shortcuts</b><br><br>"
            "<table cellspacing='4'>"
            "<tr><td><b>F1</b></td><td>About RoboSim</td></tr>"
            "<tr><td><b>Ctrl+/</b></td><td>This dialog</td></tr>"
            "<tr><td colspan='2'><hr></td></tr>"
            "<tr><td><b>I</b></td><td>Isometric view <i>(stub)</i></td></tr>"
            "<tr><td><b>F</b></td><td>Front view <i>(stub)</i></td></tr>"
            "<tr><td><b>S</b></td><td>Side view <i>(stub)</i></td></tr>"
            "<tr><td><b>T</b></td><td>Top view <i>(stub)</i></td></tr>"
            "<tr><td><b>C</b></td><td>Calibrate <i>(stub)</i></td></tr>"
            "<tr><td><b>Space</b></td><td>Connect / Disconnect <i>(stub)</i></td></tr>"
            "</table><br>"
            "<i>Stub shortcuts will be wired in the hardware-phase release.</i>"
        )
