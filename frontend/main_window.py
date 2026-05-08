#!/usr/bin/env python3
"""
MainWindow — Robotic Arm Simulation.
Kinetic Obsidian dark engineering theme (Stitch "Kinetic Monolith" design system).

Part 2 additions (P2-T5):
  - Unified _on_sample_received() slot — single entry point for all producers.
  - Calibration offsets stored on MainWindow; applied before filter.
  - ComplementaryFilter from ``backend.filter`` (honours ``--no-filter``).
  - SimWorker / ReplayWorker replace the inline Simulator class.
  - --record CSV writer opened at startup; raw dict written before filter.
  - --replay auto-starts via QTimer.singleShot after window.show().
  - Public API for Part 3: set_calibration_offsets(), get_packet_count(),
    set_idle_overlay_visible().
"""

from pathlib import Path
from PyQt6.QtWidgets import (QMainWindow, QWidget, QSplitter, QVBoxLayout,
                              QHBoxLayout, QLabel, QPushButton, QCheckBox,
                              QButtonGroup, QRadioButton, QScrollArea,
                              QStatusBar, QFrame, QSizePolicy, QMenu, QMessageBox)
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QFont, QFontDatabase, QColor, QAction, QShortcut, QKeySequence
import numpy as np

from backend.kinematics import compute_arm_positions, ArmConfig, inverse_kinematics_3dof, KinematicChain
from backend.logger import get_logger
from frontend.panels.robot_config_panel import RobotConfigPanel
from frontend.panels.kinematic_chain_panel import KinematicChainPanel
from frontend.panels.accordion_section import AccordionSection


_log = get_logger(__name__)


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
QSplitter::handle:vertical { height: 2px; }

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

    def __init__(self, replay_path: str | None = None,
                 record_path: str | None = None,
                 use_filter: bool = True):
        """
        Parameters
        ----------
        replay_path:
            Path to a CSV file to auto-replay after the window is shown.
            Passed from ``--replay`` CLI flag.
        record_path:
            Path to write recorded sensor dicts (raw, pre-filter).
            Passed from ``--record`` CLI flag.
        use_filter:
            When ``False`` the ComplementaryFilter is bypassed; raw r,p,y
            are forwarded directly. Corresponds to ``--no-filter`` flag.
        """
        super().__init__()
        self.setWindowTitle("RoboSim — Robotic Arm Simulation")
        self.resize(1400, 900)
        self.setMinimumSize(1100, 650)

        # Apply global stylesheet
        self.setStyleSheet(GLOBAL_QSS)

        # ── Shared kinematics state ───────────────────────────────────────
        self.kinematics_config = ArmConfig()
        self.current_angles = [0.0, 0.0, 0.0]
        self.mode = 'standard'
        # Legacy attribute preserved so any existing code referencing
        # self.simulator still works during transition to SimWorker.
        self.simulator = None

        # Raw r,p,y from the latest producer sample (before calibration subtract).
        self._last_raw_rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)

        # ── Part 2: sensor pipeline state (P2-T5) ────────────────────────
        # Active worker (SimWorker | ReplayWorker | None)
        self._active_worker = None

        # Calibration: subtract these raw-sensor values once before the filter.
        self._cal_r: float = 0.0
        self._cal_p: float = 0.0
        self._cal_y: float = 0.0

        # Filter — lazy import from Part 1; identity fallback if not yet available
        self._use_filter: bool = use_filter
        self._filter = None
        if use_filter:
            self._init_filter()

        # Packet counter — incremented per valid sample; read by Part 3 (P3-T3)
        self._packet_count: int = 0

        # CSV recording (--record)
        self._record_file = None
        self._record_writer = None
        if record_path:
            self._open_record_csv(record_path)

        # Auto-replay path (--replay) — started after window.show()
        self._replay_path: str | None = replay_path

        # ── Telemetry Logging (for Analysis Dashboard) ────────────────────
        self._telemetry = {
            't': [], 'joints': [], 'ee_pos': [],
            'vel': [], 'acc': [], 'torques': []
        }
        self._last_telemetry_t = 0.0
        self._last_q = None
        self._last_dq = None
        self.analysis_dashboard = None

        # ── Animation Replay Module ───────────────────────────────────────
        from backend.replay_buffer import ReplayBuffer
        from backend.replay_controller import ReplayController
        self._replay_buffer = ReplayBuffer(name="live_session")
        self._replay_controller = ReplayController(self)
        self._replay_controller.frame_changed.connect(self._on_replay_frame)

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
        self.section_mode = AccordionSection("Mode & Connection TEST")

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

        # ── SECTION 1.5: Robot Models Library (Presets) ───────────────────
        self.section_presets = AccordionSection("Robot Library")
        from frontend.panels.robot_presets_panel import RobotPresetsPanel
        self.presets_panel = RobotPresetsPanel()
        self.presets_panel.load_requested.connect(self._on_preset_loaded)

        preset_layout = QVBoxLayout()
        preset_layout.setContentsMargins(0, 0, 0, 0)
        preset_layout.addWidget(self.presets_panel)
        self.section_presets.setContentLayout(preset_layout)
        self.left_layout.addWidget(self.section_presets)

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
            "Capture latest raw R/P/Y from the producer as zero.\n"
            "Offsets are subtracted once before the complementary filter.\n"
            "Shortcut: Ctrl+K"
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

        # ── SECTION 6: CV Hand Gesture Control ──────────────────────────
        self.section_cv = AccordionSection("CV Hand Control")

        from frontend.panels.cv_hand_panel import CVHandPanel
        self.cv_hand_panel = CVHandPanel()
        self.cv_hand_panel.joint_delta_changed.connect(self._on_cv_joint_value)
        self.cv_hand_panel.estop_triggered.connect(self._on_cv_estop)
        # Pre-populate joints for current mode
        self.cv_hand_panel.set_standard_joints(self.kinematics_config)

        cv_layout = QVBoxLayout()
        cv_layout.setContentsMargins(0, 0, 0, 0)
        cv_layout.addWidget(self.cv_hand_panel)
        self.section_cv.setContentLayout(cv_layout)
        self.left_layout.addWidget(self.section_cv)

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

        # Space separator
        spacer = QWidget()
        spacer.setFixedWidth(8)
        tb_layout.addWidget(spacer)

        # EE Path Trace toggle
        self.chk_trace = QCheckBox("Trace Path")
        self.chk_trace.setChecked(False)
        self.chk_trace.setStyleSheet("color: #e74c3c; font-size: 10px; spacing: 5px; font-weight: bold;")
        self.chk_trace.toggled.connect(self._toggle_trace)
        tb_layout.addWidget(self.chk_trace)

        # Clear Trace button
        self.btn_clear_trace = QPushButton("Clear Trace")
        self.btn_clear_trace.setStyleSheet("""
            QPushButton {
                background-color: transparent;
                color: #89929b;
                border: 1px solid #353535;
                border-radius: 4px;
                padding: 2px 8px;
                font-size: 9px;
            }
            QPushButton:hover { background-color: #353535; color: #e5e2e1; }
        """)
        self.btn_clear_trace.clicked.connect(self._clear_trace)
        tb_layout.addWidget(self.btn_clear_trace)


        tb_layout.addSeparator = lambda: None  # dummy for spacing
        tb_layout.addStretch()

        # Reset view button
        self.btn_reset_view = QPushButton("⟳  Reset View")
        self.btn_reset_view.setStyleSheet(TOOLBAR_BTN_RESET)
        self.btn_reset_view.clicked.connect(self._reset_view)
        tb_layout.addWidget(self.btn_reset_view)

        # Analysis Dashboard Button
        self.btn_analysis = QPushButton("📈  Analysis")
        self.btn_analysis.setStyleSheet("""
            QPushButton {
                background-color: #2c3e50;
                color: #ecf0f1;
                border: none;
                border-radius: 4px;
                padding: 4px 14px;
                font-size: 10px;
                font-weight: 600;
                min-height: 26px;
            }
            QPushButton:hover { background-color: #34495e; }
        """)
        self.btn_analysis.clicked.connect(self._on_show_analysis)
        tb_layout.addWidget(self.btn_analysis)

        self.right_layout.addWidget(toolbar)

        # ── Viewport Splitter (Horizontal: 3D canvas left, EE map right) ──
        self.right_splitter = QSplitter(Qt.Orientation.Horizontal)
        self.right_splitter.setHandleWidth(3)
        self.right_splitter.setStyleSheet("""
            QSplitter::handle:horizontal {
                background-color: #1a1a1a;
                width: 3px;
            }
            QSplitter::handle:horizontal:hover {
                background-color: #3498db;
            }
        """)

        # ── 3D Canvas ─────────────────────────────────────────────────────
        from frontend.panels.arm_canvas import ArmCanvas
        from frontend.panels.ee_map_panel import EEMapPanel

        self.arm_canvas = ArmCanvas()
        self.arm_canvas.config = self.kinematics_config
        self.arm_canvas.setMinimumSize(400, 300)  # Relaxed minimums
        self.right_splitter.addWidget(self.arm_canvas)

        # ── 2D Map (vertical side panel on the right) ─────────────────────
        self.ee_map = EEMapPanel()
        self.ee_map.setMinimumWidth(250)  # Vertical side panel width
        self.right_splitter.addWidget(self.ee_map)

        # Set initial proportions (70% canvas, 30% map)
        self.right_splitter.setStretchFactor(0, 7)
        self.right_splitter.setStretchFactor(1, 3)
        self.right_splitter.setSizes([740, 320])

        self.right_layout.addWidget(self.right_splitter, stretch=1)

        # ── Replay Control Bar (at the bottom, below the splitter) ────────
        from frontend.panels.replay_bar import ReplayControlBar
        self.replay_bar = ReplayControlBar(self._replay_controller)
        self.replay_bar.record_toggled.connect(self._on_record_toggled)
        self.replay_bar.export_requested.connect(self._on_export_requested)
        self.right_layout.addWidget(self.replay_bar)
        
        # Connect map click to trajectory panel
        self.ee_map.target_selected.connect(
            lambda x, y: self.trajectory_panel.set_target_xyz(x, y)
        )

        # ── Wire up remaining connections ──────────────────────────────────
        self.connection_panel.connect_requested.connect(self._on_connect_requested)
        self.connection_panel.disconnect_requested.connect(self._on_disconnect_requested)
        self.connection_panel.mode_changed.connect(self._on_mode_changed)
        self.trajectory_panel.target_angles_updated.connect(self._on_target_angles)
        self._on_robot_config_changed(self.kinematics_config)
        self._update_panel_visibility()

        # Ctrl+H → toggle CV Hand Control accordion
        cv_shortcut = QShortcut(QKeySequence("Ctrl+H"), self)
        cv_shortcut.setContext(Qt.ShortcutContext.WidgetWithChildrenShortcut)
        cv_shortcut.activated.connect(self.section_cv._toggle)

        cal_shortcut = QShortcut(QKeySequence("Ctrl+K"), self)
        cal_shortcut.setContext(Qt.ShortcutContext.WidgetWithChildrenShortcut)
        cal_shortcut.activated.connect(self._on_calibrate)

        # ── Replay keyboard shortcuts ─────────────────────────────────────
        QShortcut(QKeySequence("Space"), self).activated.connect(
            self._replay_controller.toggle_play_pause
        )
        QShortcut(QKeySequence("Left"), self).activated.connect(
            lambda: self._replay_controller.step(-1)
        )
        QShortcut(QKeySequence("Right"), self).activated.connect(
            lambda: self._replay_controller.step(+1)
        )
        QShortcut(QKeySequence("R"), self).activated.connect(
            self._toggle_record_shortcut
        )
        QShortcut(QKeySequence("Ctrl+E"), self).activated.connect(
            self._on_export_requested
        )

        # ── Auto-replay (--replay CLI flag) ───────────────────────────────
        # Deferred via singleShot(0) so the window is fully rendered first.
        if self._replay_path:
            QTimer.singleShot(0, self._schedule_auto_replay)

    # ═══════════════════════════════════════════════════════════════════════
    #  Mode management
    # ═══════════════════════════════════════════════════════════════════════

    def _on_preset_loaded(self, preset_name: str):
        """Called when a preset is loaded from the Robot Library."""
        from backend.robot_presets import get_preset
        preset_chain = get_preset(preset_name)
        if not preset_chain:
            return
            
        # Switch to Custom DH mode if not already
        if self.mode != 'custom':
            self._set_robot_mode('custom')
            
        # Pass the chain to the kinematic panel
        self.chain_panel.set_chain(preset_chain)

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
        """Data source pill changed (Simulate / Replay / Serial)."""
        self._update_panel_visibility()
        self._configure_trajectory_panel_mode()
        try:
            self.connection_panel.mode_combo.clearFocus()
        except Exception:
            pass
        self._update_status_bar_mode()
        self.centralWidget().repaint()

    def _update_panel_visibility(self):
        """Trajectory panel: manual IK only while no producer streams angles.

        Simulate + connected → SimWorker owns the arm (hide trajectory).
        Replay + connected → file owns the arm (hide trajectory).
        Disconnected → user can drag IK sliders.
        """
        replay_live = (
            self.connection_panel._current_mode == "Replay"
            and self.connection_panel.is_connected
        )
        sim_live = (
            self.connection_panel._current_mode == "Simulate"
            and self.connection_panel.is_connected
        )
        show_trajectory = not replay_live and not sim_live
        if self.mode == 'standard':
            self.section_robot_params.setVisible(True)
            self.section_presets.setVisible(True)
            self.section_joint_control.setVisible(show_trajectory)
            self.section_chain.setVisible(False)
        else:
            self.section_robot_params.setVisible(False)
            self.section_presets.setVisible(True)
            self.section_joint_control.setVisible(show_trajectory)
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
        if port == "SIMULATED (no hardware)":
            self._start_simulation()
        elif port.startswith("REPLAY:"):
            replay_file = port[len("REPLAY:"):]
            self._start_replay(replay_file)
        else:
            # Real serial port
            self._start_serial(port, baud)
        self._update_status_bar_mode()
        self._update_panel_visibility()

    def _start_serial(self, port: str, baud: int):
        """Start the SerialWorker (P2-T6) for a real USB-serial device."""
        from backend.serial_worker import SerialWorker
        worker = SerialWorker(port, baud)
        qc = Qt.ConnectionType.QueuedConnection
        worker.sample_received.connect(self._on_sample_received, qc)
        worker.producer_status.connect(self._on_producer_status, qc)
        worker.producer_error.connect(self._on_producer_error, qc)
        self._active_worker = worker
        worker.start()
        self.connection_panel.set_connected(True)
        self.connection_panel.set_status(f"Connected: {port}")
        self.sb_message.setText(f"Serial active on {port}")
        self._update_panel_visibility()

    def _on_disconnect_requested(self):
        self._stop_current_mode()
        self.connection_panel.set_connected(False)
        self.connection_panel.set_status("Disconnected")
        self.sb_message.setText("Disconnected")
        self._update_status_bar_mode()
        self._update_panel_visibility()

    def _start_simulation(self):
        """Start the SimWorker (P2-T3) — replaces inline Simulator."""
        from backend.sim_worker import SimWorker
        worker = SimWorker()
        qc = Qt.ConnectionType.QueuedConnection
        worker.sample_received.connect(self._on_sample_received, qc)
        worker.producer_status.connect(self._on_producer_status, qc)
        worker.producer_error.connect(self._on_producer_error, qc)
        self._active_worker = worker
        worker.start()
        # Keep legacy self.simulator alias so any remaining references don't crash
        self.simulator = worker
        self.connection_panel.set_connected(True)
        self.connection_panel.set_status("Simulation running")
        self.sb_message.setText("Simulation mode active")
        self._update_panel_visibility()

    def _start_replay(self, file_path: str):
        """Start the ReplayWorker (P2-T2) for a given CSV file path."""
        from backend.replay_worker import ReplayWorker
        worker = ReplayWorker(file_path)
        qc = Qt.ConnectionType.QueuedConnection
        worker.sample_received.connect(self._on_sample_received, qc)
        worker.producer_error.connect(self._on_producer_error, qc)
        worker.producer_status.connect(self._on_producer_status, qc)
        self._active_worker = worker
        worker.start()
        self.connection_panel.set_connected(True)
        self.connection_panel.set_status(f"Replay: {Path(file_path).name}")
        self.sb_message.setText(f"Replaying {Path(file_path).name}")
        self._update_panel_visibility()

    def _stop_current_mode(self):
        """Stop and join any active producer worker or legacy Simulator."""
        # Stop QThread workers (SimWorker / ReplayWorker)
        if self._active_worker is not None:
            self._active_worker.stop()
            if not self._active_worker.wait(2000):  # 2 s timeout
                self._active_worker.terminate()
            self._active_worker = None
            self.simulator = None  # keep legacy alias in sync

        # Fallback: stop legacy QTimer-based simulator if somehow still alive
        if self.simulator and hasattr(self.simulator, 'stop'):
            self.simulator.stop()
            self.simulator = None

        if hasattr(self, 'trajectory_panel') and getattr(self.trajectory_panel, 'animating', False):
            self.trajectory_panel._stop_clicked()
        self.arm_canvas._init_empty_plot()
        self._packet_count = 0
        self.set_calibration_offsets(0.0, 0.0, 0.0)
        self._last_raw_rpy = (0.0, 0.0, 0.0)
        if self._filter is not None:
            self._filter.reset()
        if hasattr(self, "lbl_calib_status"):
            self.lbl_calib_status.setText("offsets: 0.0 / 0.0 / 0.0")
            self.lbl_calib_status.setStyleSheet(
                "color: #3f4850; font-size: 9px; font-family: 'Consolas', monospace;"
            )

    # ═══════════════════════════════════════════════════════════════════════
    #  Unified pipeline slot (P2-T5)
    # ═══════════════════════════════════════════════════════════════════════

    def _on_sample_received(self, sample: dict) -> None:
        """Single entry point for ALL producers (Sim, Replay, future Serial).

        Pipeline:
          1. Error check  — forward _error dicts to status UI.
          2. Record raw   — write to CSV before any transformation (--record).
          3. Calibrate    — subtract offsets set by Part 3 Calibrate button.
          4. Filter       — ComplementaryFilter.step() (skipped if --no-filter).
          5. Display      — call _on_data_received() with filtered r,p,y.
        """
        try:
            if sample.get("_error"):
                self._on_producer_error(str(sample["_error"]))
                return

            raw_r = float(sample.get("r", 0.0))
            raw_p = float(sample.get("p", 0.0))
            raw_y = float(sample.get("y", 0.0))
            self._last_raw_rpy = (raw_r, raw_p, raw_y)

            if self._record_writer is not None:
                self._append_record(sample)

            r = raw_r - self._cal_r
            p = raw_p - self._cal_p
            y = raw_y - self._cal_y

            if self._use_filter and self._filter is not None:
                sample_adj = {**sample, "r": r, "p": p, "y": y}
                r, p, y = self._filter.step(sample_adj)

            self._on_data_received(r, p, y)
        except Exception:
            _log.exception("Sensor pipeline error | sample=%r", sample)
            self.sb_message.setText("Pipeline error — see logs/robosim.log")

    # ── Producer status / error helpers ────────────────────────────────────

    def _on_producer_status(self, message: str) -> None:
        """Handle status messages from any producer worker."""
        self.sb_message.setText(message)
        # When replay completes, update connection panel
        if "complete" in message.lower() or "stopped" in message.lower():
            self.connection_panel.set_connected(False)
            self.connection_panel.set_status(message)
            self._update_panel_visibility()
            if hasattr(self, 'ee_map'):
                self.ee_map.clear_history()

    def _on_producer_error(self, error: str) -> None:
        """Handle error signals from any producer worker."""
        self.connection_panel.set_status(f"Error: {error}")
        self.connection_panel.set_connected(False)
        self.sb_message.setText(f"Producer error: {error}")
        # Non-blocking error notification (modal-free for replay errors)
        QMessageBox.warning(
            self,
            "Producer Error",
            error,
            QMessageBox.StandardButton.Ok,
        )

    # ── Recording helpers (--record) ────────────────────────────────────────

    def _open_record_csv(self, path: str) -> None:
        """Open the CSV file for recording and write the header row."""
        try:
            from backend.recording import open_record_csv

            self._record_file, self._record_writer = open_record_csv(path)
        except OSError as exc:
            self._record_file = None
            self._record_writer = None
            _log.warning("Failed to open record CSV %s: %s", path, exc)

    def _append_record(self, sample: dict) -> None:
        """Write one row to the record CSV."""
        from backend.recording import row_from_sample

        self._record_writer.writerow(row_from_sample(sample))

    def _close_record_csv(self) -> None:
        """Flush and close the recording file handle."""
        if self._record_file is not None:
            try:
                self._record_file.flush()
                self._record_file.close()
            except OSError:
                pass
            self._record_file = None
            self._record_writer = None

    # ── Filter init helper ──────────────────────────────────────────────────

    def _init_filter(self) -> None:
        """Initialise ComplementaryFilter unless ``--no-filter`` was passed."""
        try:
            from backend.filter import ComplementaryFilter

            self._filter = ComplementaryFilter(alpha=0.98)
        except ImportError:
            self._filter = None
            _log.warning("backend.filter import failed — running without ComplementaryFilter")

    # ═══════════════════════════════════════════════════════════════════════
    #  Part 3 public API — called by panels, not by internal logic
    # ═══════════════════════════════════════════════════════════════════════

    def set_calibration_offsets(self, r: float, p: float, y: float) -> None:
        """Store raw-sensor offsets subtracted once before the complementary filter."""
        self._cal_r = float(r)
        self._cal_p = float(p)
        self._cal_y = float(y)

    def get_packet_count(self) -> int:
        """Return the total number of valid samples received since last connect.

        Part 3 reads this on a 1-second QTimer to compute packets/sec (P3-T3).
        Reset to zero on each new _stop_current_mode() call.
        """
        return self._packet_count

    def set_idle_overlay_visible(self, visible: bool) -> None:
        """Show or hide the arm canvas idle overlay (no streaming data)."""
        if hasattr(self, "arm_canvas") and hasattr(self.arm_canvas, "set_idle_message"):
            self.arm_canvas.set_idle_message(bool(visible))

    # ═══════════════════════════════════════════════════════════════════════
    #  Data / kinematics handlers (preserved)
    # ═══════════════════════════════════════════════════════════════════════

    def _on_data_received(self, roll: float, pitch: float, yaw: float):
        """Display path — calibration + filtering already applied upstream."""
        self.data_panel.record_sample()
        self._packet_count += 1

        self.data_panel.update_values(roll, pitch, yaw)
        self.trajectory_panel.set_current_angles([roll, pitch, yaw])
        self.current_angles = [roll, pitch, yaw]
        positions = compute_arm_positions(roll, pitch, yaw, config=self.kinematics_config)
        self.arm_canvas.draw_arm(positions)
        
        # Log telemetry for analysis
        self._log_telemetry([roll, pitch, yaw], positions[-1])

    def _on_target_angles(self, angles: list):
        """IK sliders — active only while disconnected (no streaming producer)."""
        if self.connection_panel.is_connected:
            return
            
        if not angles:
            return

        # Map first 3 to data panel for visualization
        q1 = angles[0] if len(angles) > 0 else 0.0
        q2 = angles[1] if len(angles) > 1 else 0.0
        q3 = angles[2] if len(angles) > 2 else 0.0
        self.data_panel.update_values(q1, q2, q3)
        
        if self.mode == 'standard':
            self._apply_target_angles(q1, q2, q3)
        else:
            chain = self.chain_panel.chain
            var_idx = 0
            for joint in chain.joints:
                if joint.type in ('revolute', 'prismatic'):
                    if var_idx < len(angles):
                        if joint.type == 'revolute':
                            joint.theta = angles[var_idx]
                        else:
                            joint.d = angles[var_idx]
                    var_idx += 1
            
            self.trajectory_panel.set_current_angles(angles)
            self.chain_panel._compute_and_emit_fk()
            self._refresh_arm_display()

    def _apply_target_angles(self, q1: float, q2: float, q3: float):
        self.data_panel.update_values(q1, q2, q3)
        self.current_angles = [q1, q2, q3]
        positions = compute_arm_positions(q1, q2, q3, config=self.kinematics_config)
        self.arm_canvas.draw_arm(positions)
        tip = positions[-1]
        self._update_ee_display(tip[0], tip[1], tip[2])
        self.trajectory_panel.set_current_angles([q1, q2, q3])

    def _update_ee_display(self, x, y, z):
        self.lbl_ee_status.setText(
            f"<span style='color:#92ccff;'>X</span> {x:+.4f} m\n"
            f"<span style='color:#2ecc71;'>Y</span> {y:+.4f} m\n"
            f"<span style='color:#ffba4b;'>Z</span> {z:+.4f} m"
        )
        self.sb_ee.setText(f"EE: ({x:.3f}, {y:.3f}, {z:.3f})")
        if hasattr(self, 'ee_map'):
            self.ee_map.set_position(x, y)

    def _on_robot_config_changed(self, config):
        self.kinematics_config = config
        max_reach = config.upper_arm_length + config.lower_arm_length + config.gripper_offset
        self.arm_canvas.update_workspace_boundary(max_reach)
        if hasattr(self, 'ee_map'):
            self.ee_map.update_workspace(max_reach)
        if hasattr(self, 'cv_hand_panel'):
            self.cv_hand_panel.set_standard_joints(config)
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
            self.trajectory_panel.set_chain(self.chain_panel.chain)
            angles = []
            for joint in self.chain_panel.chain.joints:
                if joint.type == 'revolute':
                    angles.append(joint.theta)
                elif joint.type == 'prismatic':
                    angles.append(joint.d)
                if len(angles) >= 3:
                    break
            if len(angles) == 3:
                self.trajectory_panel.set_current_angles(angles[:3])
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
            # Sum both 'a' and 'd' for a safe maximum reach bound in custom DH
            max_xy = sum(abs(joint.a) + abs(joint.d) for joint in chain.joints)
            max_z = chain.base_height + max_xy
            base_h = chain.base_height
            
        # Ensure max_xy is at least 0.1 to avoid map scaling errors
        max_xy = max(max_xy, 0.1)

        self.arm_canvas.frame_to_fit_robot(max_xy, max_z, base_h)
        self.arm_canvas.update_workspace_boundary(max_xy)
        if hasattr(self, 'ee_map'):
            self.ee_map.update_workspace(max_xy)

    def _on_chain_updated(self, chain):
        if self.mode == 'custom':
            self.trajectory_panel.set_chain(chain)
            if hasattr(self, 'cv_hand_panel'):
                self.cv_hand_panel.set_chain(chain)
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
                self.trajectory_panel.set_current_angles(angles[:3])
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
            
            # Log telemetry for analysis (Custom DH mode)
            angles = [j.theta if j.type == 'revolute' else j.d for j in self.chain_panel.chain.joints]
            torques = self.chain_panel.chain.compute_gravity_torques()
            self._log_telemetry(angles, tip, torques=torques)

    def _log_telemetry(self, angles, ee_pos, torques=None):
        """Record a slice of robot state into the telemetry buffer."""
        import time
        now = time.monotonic()
        if not hasattr(self, '_start_time'):
            self._start_time = now
        
        t_rel = now - self._start_time
        dt = t_rel - self._last_telemetry_t
        
        if dt < 0.001: return # Avoid div by zero
        
        q = np.array(angles)
        dq = np.zeros_like(q)
        ddq = np.zeros_like(q)
        
        if self._last_q is not None and len(self._last_q) == len(q):
            dq = (q - self._last_q) / dt
            if self._last_dq is not None:
                ddq = (dq - self._last_dq) / dt
        
        self._last_telemetry_t = t_rel
        self._last_q = q
        self._last_dq = dq
        
        # Update buffer
        self._telemetry['t'].append(t_rel)
        self._telemetry['joints'].append(angles)
        self._telemetry['ee_pos'].append(list(ee_pos))
        self._telemetry['vel'].append(dq.tolist())
        self._telemetry['acc'].append(ddq.tolist())
        
        if torques is None:
            # Estimate torques if not provided (Standard 3-DOF)
            if self.mode == 'standard':
                chain = KinematicChain.create_3dof_arm(self.kinematics_config)
                torques = chain.compute_gravity_torques(angles)
            else:
                torques = [0.0] * len(angles)
        self._telemetry['torques'].append(torques)
        
        # Keep buffer size manageable (5000 points ~ 80 seconds at 60Hz)
        max_buf = 5000
        if len(self._telemetry['t']) > max_buf:
            for key in self._telemetry:
                self._telemetry[key].pop(0)

        # ── Live recording into ReplayBuffer ─────────────────────────────
        if self._replay_controller.is_recording:
            from backend.replay_buffer import Frame
            idx = len(self._replay_buffer)
            frame = Frame(
                index=idx,
                t=t_rel,
                joints=list(angles),
                ee_pos=list(ee_pos),
                torques=list(torques) if torques else [],
                metadata={'vel': dq.tolist(), 'acc': ddq.tolist()},
            )
            self._replay_buffer.record(frame)

    def _toggle_trace(self, checked: bool):
        """Enable or disable EE path tracing in the viewport."""
        self.arm_canvas.set_trace_enabled(checked)

    def _clear_trace(self):
        """Clear the current EE path trace history."""
        self.arm_canvas.clear_trace()

    def _on_show_analysis(self):
        """Launch the Robot Analysis Dashboard."""
        from frontend.panels.analysis_dashboard import AnalysisDashboard
        if self.analysis_dashboard is None or not self.analysis_dashboard.isVisible():
            self.analysis_dashboard = AnalysisDashboard(
                get_telemetry=lambda: self._telemetry,
                seek_callback=self._replay_controller.seek_time,
                parent=self,
            )
            self.analysis_dashboard.show()
        else:
            self.analysis_dashboard.raise_()
            self.analysis_dashboard.activateWindow()

    # ═══════════════════════════════════════════════════════════════════════
    #  Animation Replay handlers
    # ═══════════════════════════════════════════════════════════════════════

    def _on_replay_frame(self, idx: int) -> None:
        """Render the frame at index idx from the replay buffer."""
        buf = self._replay_buffer
        if not buf or idx >= len(buf):
            return
        
        # Auto-reset trace if playing from the start
        if idx == 0:
            self.arm_canvas.clear_trace()

        frame = buf.get_frame(idx)
        self._render_delegate(frame)

    def _render_delegate(self, frame) -> None:
        """
        RenderDelegate — the ONLY place that knows robot mode.
        Translates a generic Frame into a concrete canvas draw call.
        """
        from backend.kinematics import compute_arm_positions
        joints = frame.joints
        ee = frame.ee_pos

        if self.mode == 'standard':
            q1 = joints[0] if len(joints) > 0 else 0.0
            q2 = joints[1] if len(joints) > 1 else 0.0
            q3 = joints[2] if len(joints) > 2 else 0.0
            positions = compute_arm_positions(q1, q2, q3, config=self.kinematics_config)
            self.arm_canvas.draw_arm(positions)
        else:
            chain = self.chain_panel.chain
            var_joints = [j for j in chain.joints if j.type in ('revolute', 'prismatic')]
            for i, joint in enumerate(var_joints):
                if i < len(joints):
                    if joint.type == 'revolute':
                        joint.theta = joints[i]
                    else:
                        joint.d = joints[i]
            positions = chain.joint_positions()
            self.arm_canvas.draw_chain(positions, base_height=chain.base_height)

        if len(ee) >= 3:
            self._update_ee_display(ee[0], ee[1], ee[2])

        # Sync Analysis Dashboard cursor if open
        if self.analysis_dashboard and self.analysis_dashboard.isVisible():
            if hasattr(self.analysis_dashboard, 'update_cursor'):
                self.analysis_dashboard.update_cursor(frame.t)

    def _on_record_toggled(self, active: bool) -> None:
        """Called when the ReplayControlBar record button is toggled."""
        if active:
            self._replay_buffer.clear()
            self._replay_controller.start_recording(self._replay_buffer)
            self.sb_message.setText("⏺ Recording…")
        else:
            self._replay_controller.stop_recording()
            n = len(self._replay_buffer)
            self.sb_message.setText(
                f"Recording stopped — {n} frames ({self._replay_buffer.duration:.1f}s) ready for replay."
            )
            # Attach buffer to controller so playback can start immediately
            self._replay_controller.set_buffer(self._replay_buffer)

    def _toggle_record_shortcut(self) -> None:
        """Keyboard shortcut R — toggle recording via the bar button."""
        if hasattr(self, 'replay_bar'):
            self.replay_bar.btn_record.toggle()

    def _on_export_requested(self) -> None:
        """Launch the export dialog with a robot-agnostic render function."""
        from frontend.panels.replay_export_dialog import ReplayExportDialog
        buf = self._replay_buffer
        if not buf:
            self.sb_message.setText("Nothing to export — record an animation first.")
            return

        # Build a render_fn that captures the current canvas state
        def render_fn(frame):
            self._render_delegate(frame)
            return self.arm_canvas.fig  # matplotlib Figure

        dlg = ReplayExportDialog(buf, render_fn=render_fn, parent=self)
        dlg.exec()

    def _on_ee_updated(self, pos):
        x, y, z = pos
        self._update_ee_display(x, y, z)

    # ═══════════════════════════════════════════════════════════════════════
    #  CV Hand Gesture Control handlers
    # ═══════════════════════════════════════════════════════════════════════

    def _on_cv_joint_value(self, joint_index: int, delta: float) -> None:
        """
        Called every hand-tracking frame.
        Updates the selected joint angle in whichever mode is active using relative joystick-style movement.
        """
        if self.mode == 'standard':
            # Map first 3 variable joints onto q1/q2/q3
            angles = list(self.current_angles)
            if joint_index < len(angles):
                angles[joint_index] += delta
            q1 = angles[0] if len(angles) > 0 else 0.0
            q2 = angles[1] if len(angles) > 1 else 0.0
            q3 = angles[2] if len(angles) > 2 else 0.0
            # _apply_target_angles automatically updates spinboxes which handles clamping
            self._apply_target_angles(q1, q2, q3)
            self._log_telemetry([q1, q2, q3], self.arm_canvas._last_positions[-1]
                                 if self.arm_canvas._last_positions is not None else [0, 0, 0])
        else:
            # Custom DH mode
            chain = self.chain_panel.chain
            var_joints = [j for j in chain.joints if j.type in ('revolute', 'prismatic')]
            if joint_index < len(var_joints):
                j = var_joints[joint_index]
                if j.type == 'revolute':
                    j.theta += delta
                    if j.q_min is not None:
                        j.theta = max(j.q_min, min(j.theta, j.q_max))
                else:
                    j.d += delta
                    if j.q_min is not None:
                        j.d = max(j.q_min, min(j.d, j.q_max))
            self.chain_panel._compute_and_emit_fk()
            self._refresh_arm_display()

    def _on_cv_estop(self) -> None:
        """Emergency stop — two hands detected. Freeze the arm at current pose."""
        self.sb_message.setText(
            "⚠ CV ESTOP — Two hands detected. Arm frozen. Remove one hand to resume."
        )
        self.sb_message.setStyleSheet("color: #e74c3c; font-size: 10px; font-weight: 700;")
        QTimer.singleShot(
            4000,
            lambda: self.sb_message.setStyleSheet("color: #89929b; font-size: 10px;")
        )

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
        """Capture latest raw r/p/y from the producer as the zero reference."""
        if self._active_worker is None:
            self.sb_message.setText("Connect Simulate or Replay before calibrating.")
            return
        r, p, y = self._last_raw_rpy
        self.set_calibration_offsets(r, p, y)
        if self._filter is not None:
            self._filter.reset()
        self.lbl_calib_status.setText(f"offsets: {r:+.1f} / {p:+.1f} / {y:+.1f}")
        self.lbl_calib_status.setStyleSheet(
            "color: #f39c12; font-size: 9px; font-family: 'Consolas', monospace;"
        )
        self.sb_message.setText(f"Calibrated — raw zero R={r:+.1f}° P={p:+.1f}° Y={y:+.1f}°")

    # ═══════════════════════════════════════════════════════════════════════
    #  P3-T5 — Error / status UX
    # ═══════════════════════════════════════════════════════════════════════

    def show_producer_error(self, message: str, fatal: bool = False) -> None:
        """Surface a producer error — optional fatal modal vs status-only."""
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

        help_menu.addSeparator()

        act_analysis = QAction("Launch &Analysis Dashboard", self)
        act_analysis.setShortcut("Ctrl+G")
        act_analysis.triggered.connect(self._on_show_analysis)
        help_menu.addAction(act_analysis)

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
            "<tr><td><b>Ctrl+K</b></td><td>Calibrate (raw zero capture)</td></tr>"
            "</table>"
        )

    # ═══════════════════════════════════════════════════════════════════════
    #  Window lifecycle
    # ═══════════════════════════════════════════════════════════════════════

    def _schedule_auto_replay(self) -> None:
        """Auto-start replay after the event loop begins (P2-T4).

        Called via QTimer.singleShot(0, …) from _setup_right_panel so the
        window is fully shown before replay starts.
        """
        if self._replay_path:
            self._stop_current_mode()
            self._start_replay(self._replay_path)
            self._update_status_bar_mode()

    def closeEvent(self, event) -> None:  # type: ignore[override]
        """Clean up worker threads and open file handles on window close."""
        self._stop_current_mode()
        self._close_record_csv()
        # Stop CV hand controller if running
        if hasattr(self, 'cv_hand_panel'):
            self.cv_hand_panel._stop_cv()
        super().closeEvent(event)
