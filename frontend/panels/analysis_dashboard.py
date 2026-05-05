#!/usr/bin/env python3
"""
AnalysisDashboard — Comprehensive multi-tab analysis suite for robotic motion.
Inspired by RoboAnalyzer. Displays Kinematics, Dynamics, and Motion Profiles.

Replay integration (Task 6):
  - update_cursor(t) moves a vertical red line across all graphs to track playback.
  - Clicking any graph emits seek_requested(t) so the replay controller can seek.
  - Both features are optional — dashboard still works standalone without a controller.
"""

from __future__ import annotations

from typing import Callable, Dict, List, Optional
import numpy as np

from PyQt6.QtCore import QTimer, Qt, pyqtSignal
from PyQt6.QtWidgets import (
    QVBoxLayout, QHBoxLayout, QWidget, QDialog,
    QTabWidget, QLabel, QFrame,
)
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from matplotlib.lines import Line2D


# ──────────────────────────────────────────────────────────────────────────────
#  Shared palette
# ──────────────────────────────────────────────────────────────────────────────

_JOINT_COLORS = ["#3498db", "#e67e22", "#2ecc71", "#9b59b6", "#f1c40f", "#e74c3c"]
_CURSOR_COLOR = "#ff4757"
_CURSOR_ALPHA = 0.8


# ──────────────────────────────────────────────────────────────────────────────
#  AnalysisTab — base class with cursor support
# ──────────────────────────────────────────────────────────────────────────────

class AnalysisTab(QWidget):
    """
    Base class for dashboard tabs.

    Cursor support
    --------------
    Each tab maintains one vertical Line2D cursor per axis.
    Call update_cursor(t) to move all cursors to time t efficiently
    (uses set_xdata — no full redraw).

    Click-to-seek
    -------------
    Clicking anywhere on the canvas emits seek_requested(t) via the
    mpl_connect 'button_press_event'.  The parent dialog forwards this
    signal upward to MainWindow → ReplayController.
    """

    seek_requested = pyqtSignal(float)   # time in seconds

    def __init__(self, parent=None):
        super().__init__(parent)
        self._layout = QVBoxLayout(self)
        self._layout.setContentsMargins(5, 5, 5, 5)

        self.fig = Figure(figsize=(8, 5), dpi=100)
        self.fig.patch.set_facecolor("#131313")
        self.canvas = FigureCanvas(self.fig)
        self._layout.addWidget(self.canvas)

        self.axes: List = []
        self._cursors: Dict[int, Line2D] = {}   # ax id → cursor line

        # Wire click-to-seek
        self.canvas.mpl_connect("button_press_event", self._on_canvas_click)

    # ── Subplot factory ───────────────────────────────────────────────────────

    def add_subplot(self, rows, cols, index, title="", ylabel="", xlabel="Time (s)"):
        ax = self.fig.add_subplot(rows, cols, index)
        ax.set_facecolor("#0e0e0e")
        ax.set_title(title, color="#e5e2e1", fontsize=10, pad=10)
        ax.set_xlabel(xlabel, color="#89929b", fontsize=9)
        ax.set_ylabel(ylabel, color="#89929b", fontsize=9)
        ax.tick_params(colors="#89929b", labelsize=8)
        ax.grid(True, color="#222", linewidth=0.5, alpha=0.5)
        for spine in ax.spines.values():
            spine.set_color("#333")
        self.axes.append(ax)

        # Pre-create cursor line (invisible until first update_cursor call)
        cursor = ax.axvline(x=0, color=_CURSOR_COLOR, lw=1.2,
                            alpha=_CURSOR_ALPHA, ls="--", visible=False)
        self._cursors[id(ax)] = cursor

        return ax

    # ── Cursor API ────────────────────────────────────────────────────────────

    def update_cursor(self, t: float) -> None:
        """Move all cursor lines to time t without a full redraw."""
        for cursor in self._cursors.values():
            cursor.set_xdata([t, t])
            cursor.set_visible(True)
        self.canvas.draw_idle()

    def hide_cursor(self) -> None:
        for cursor in self._cursors.values():
            cursor.set_visible(False)
        self.canvas.draw_idle()

    # ── Click handler ─────────────────────────────────────────────────────────

    def _on_canvas_click(self, event) -> None:
        """Emit seek_requested(t) when user clicks on the graph."""
        if event.inaxes is not None and event.xdata is not None:
            self.seek_requested.emit(float(event.xdata))


# ──────────────────────────────────────────────────────────────────────────────
#  KinematicsTab
# ──────────────────────────────────────────────────────────────────────────────

class KinematicsTab(AnalysisTab):
    """EE position (X/Y/Z) and all joint angles vs. time."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.ax_cart  = self.add_subplot(2, 1, 1, "End-Effector Position", "Position (m)")
        self.ax_joint = self.add_subplot(2, 1, 2, "Joint Positions", "Angle (deg) / Pos (m)")

        self.lx, = self.ax_cart.plot([], [], color="#92ccff", label="X", lw=1.2)
        self.ly, = self.ax_cart.plot([], [], color="#2ecc71", label="Y", lw=1.2)
        self.lz, = self.ax_cart.plot([], [], color="#ffba4b", label="Z", lw=1.2)
        self.ax_cart.legend(fontsize=8, facecolor="#1a1a1a", edgecolor="#333", labelcolor="#eee")

        self.joint_lines: List = []
        self.fig.tight_layout()

    def update_data(self, t, cart_data, joint_data):
        if len(t) < 2:
            return
        self.lx.set_data(t, cart_data[:, 0])
        self.ly.set_data(t, cart_data[:, 1])
        self.lz.set_data(t, cart_data[:, 2])

        num_joints = joint_data.shape[1]
        while len(self.joint_lines) < num_joints:
            idx = len(self.joint_lines)
            line, = self.ax_joint.plot([], [], color=_JOINT_COLORS[idx % len(_JOINT_COLORS)],
                                       label=f"J{idx+1}", lw=1.2)
            self.joint_lines.append(line)
            self.ax_joint.legend(fontsize=8, ncol=3, facecolor="#1a1a1a",
                                  edgecolor="#333", labelcolor="#eee")

        for i in range(num_joints):
            self.joint_lines[i].set_data(t, joint_data[:, i])

        for ax in self.axes:
            ax.relim()
            ax.autoscale_view()
        self.canvas.draw_idle()


# ──────────────────────────────────────────────────────────────────────────────
#  DynamicsTab
# ──────────────────────────────────────────────────────────────────────────────

class DynamicsTab(AnalysisTab):
    """Gravity-compensation joint torques vs. time."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.ax_torque = self.add_subplot(
            1, 1, 1, "Estimated Joint Torques (Gravity Comp)", "Torque (Nm) / Force (N)"
        )
        self.torque_lines: List = []

    def update_data(self, t, torque_data):
        if len(t) < 2:
            return
        num_joints = torque_data.shape[1]
        while len(self.torque_lines) < num_joints:
            idx = len(self.torque_lines)
            line, = self.ax_torque.plot([], [], color=_JOINT_COLORS[idx % len(_JOINT_COLORS)],
                                        label=f"J{idx+1}", lw=1.4)
            self.torque_lines.append(line)
            self.ax_torque.legend(fontsize=8, ncol=3, facecolor="#1a1a1a",
                                   edgecolor="#333", labelcolor="#eee")

        for i in range(num_joints):
            self.torque_lines[i].set_data(t, torque_data[:, i])

        self.ax_torque.relim()
        self.ax_torque.autoscale_view()
        self.canvas.draw_idle()


# ──────────────────────────────────────────────────────────────────────────────
#  ProfilesTab
# ──────────────────────────────────────────────────────────────────────────────

class ProfilesTab(AnalysisTab):
    """Joint velocity and acceleration profiles vs. time."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.ax_vel = self.add_subplot(2, 1, 1, "Joint Velocities",     "deg/s or m/s")
        self.ax_acc = self.add_subplot(2, 1, 2, "Joint Accelerations",  "deg/s² or m/s²")
        self.vel_lines: List = []
        self.acc_lines: List = []
        self.fig.tight_layout()

    def update_data(self, t, vel_data, acc_data):
        if len(t) < 3:
            return
        num_joints = vel_data.shape[1]

        while len(self.vel_lines) < num_joints:
            idx = len(self.vel_lines)
            c = _JOINT_COLORS[idx % len(_JOINT_COLORS)]
            v_line, = self.ax_vel.plot([], [], color=c, label=f"J{idx+1}", lw=1.2)
            a_line, = self.ax_acc.plot([], [], color=c, label=f"J{idx+1}", lw=1.2)
            self.vel_lines.append(v_line)
            self.acc_lines.append(a_line)
            self.ax_vel.legend(fontsize=8, ncol=3, facecolor="#1a1a1a",
                               edgecolor="#333", labelcolor="#eee")
            self.ax_acc.legend(fontsize=8, ncol=3, facecolor="#1a1a1a",
                               edgecolor="#333", labelcolor="#eee")

        for i in range(num_joints):
            self.vel_lines[i].set_data(t, vel_data[:, i])
            self.acc_lines[i].set_data(t, acc_data[:, i])

        for ax in self.axes:
            ax.relim()
            ax.autoscale_view()
        self.canvas.draw_idle()


# ──────────────────────────────────────────────────────────────────────────────
#  AnalysisDashboard — main dialog
# ──────────────────────────────────────────────────────────────────────────────

class AnalysisDashboard(QDialog):
    """
    Multi-tab analysis window.

    Parameters
    ----------
    get_telemetry : Callable[[], dict]
        Returns the MainWindow._telemetry dict on each call.
    seek_callback : Callable[[float], None], optional
        Called when the user clicks on a graph (time in seconds).
        Typically: lambda t: replay_controller.seek_time(t)
    """

    def __init__(
        self,
        get_telemetry: Callable[[], Dict],
        seek_callback: Optional[Callable[[float], None]] = None,
        parent=None,
    ):
        super().__init__(parent)
        self.setWindowTitle("Robot Analysis Dashboard — RoboSim")
        self.resize(1000, 700)
        self._get_telemetry = get_telemetry
        self._seek_callback = seek_callback

        self.setStyleSheet("""
            QDialog { background-color: #131313; }
            QTabWidget::pane { border: 1px solid #353535; background: #131313; top: -1px; }
            QTabBar::tab {
                background: #202020; color: #89929b;
                padding: 8px 20px; border: 1px solid #353535;
                border-bottom: none;
                border-top-left-radius: 4px; border-top-right-radius: 4px;
                font-size: 11px; font-weight: 600;
            }
            QTabBar::tab:selected { background: #3498db; color: white; border-color: #3498db; }
            QTabBar::tab:hover:!selected { background: #2a2a2a; color: #e5e2e1; }
        """)

        # ── Build tabs ────────────────────────────────────────────────────────
        layout = QVBoxLayout(self)
        self.tabs = QTabWidget()

        self.kin_tab  = KinematicsTab()
        self.prof_tab = ProfilesTab()
        self.dyn_tab  = DynamicsTab()

        self.tabs.addTab(self.kin_tab,  "📐 Kinematics")
        self.tabs.addTab(self.prof_tab, "📈 Motion Profiles")
        self.tabs.addTab(self.dyn_tab,  "⚙ Dynamics")

        layout.addWidget(self.tabs)

        # ── Hint label when seek is available ─────────────────────────────────
        if seek_callback:
            hint = QLabel("💡 Click on any graph to jump the replay to that time.")
            hint.setStyleSheet(
                "color: #3498db; font-size: 10px; padding: 2px 8px;"
            )
            layout.addWidget(hint)

        # ── Wire click-to-seek for all tabs ───────────────────────────────────
        if seek_callback:
            for tab in (self.kin_tab, self.prof_tab, self.dyn_tab):
                tab.seek_requested.connect(seek_callback)

        # ── Polling timer (telemetry refresh) ─────────────────────────────────
        self._last_data_len = 0
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._refresh)
        self._timer.start(100)   # 10 Hz — plenty for graphs

    # ── Public replay API ─────────────────────────────────────────────────────

    def update_cursor(self, t: float) -> None:
        """
        Move the playback cursor line to time t on all visible graphs.
        Called by MainWindow._render_delegate during replay.
        Efficient: uses set_xdata, no full figure redraw.
        """
        current_tab = self.tabs.currentIndex()
        tab = [self.kin_tab, self.prof_tab, self.dyn_tab][current_tab]
        tab.update_cursor(t)

    def hide_cursors(self) -> None:
        """Hide all cursor lines (e.g., when replay stops)."""
        for tab in (self.kin_tab, self.prof_tab, self.dyn_tab):
            tab.hide_cursor()

    # ── Internal refresh ──────────────────────────────────────────────────────

    def _refresh(self) -> None:
        """Poll the telemetry dict and refresh only the active tab."""
        data = self._get_telemetry()
        if not data:
            return

        n = len(data.get("t", []))
        if n == self._last_data_len or n < 2:
            return
        self._last_data_len = n

        t = np.array(data["t"])

        current_tab = self.tabs.currentIndex()
        try:
            if current_tab == 0:
                ee  = np.array(data["ee_pos"])
                jts = np.array(data["joints"])
                if ee.ndim == 2 and jts.ndim == 2:
                    self.kin_tab.update_data(t, ee, jts)

            elif current_tab == 1:
                vel = np.array(data["vel"])
                acc = np.array(data["acc"])
                if vel.ndim == 2 and acc.ndim == 2:
                    self.prof_tab.update_data(t, vel, acc)

            elif current_tab == 2:
                torq = np.array(data["torques"])
                if torq.ndim == 2:
                    self.dyn_tab.update_data(t, torq)
        except (ValueError, IndexError):
            pass   # shape mismatch during robot switching — skip this tick
