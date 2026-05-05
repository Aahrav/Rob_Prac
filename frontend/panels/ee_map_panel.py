#!/usr/bin/env python3
"""
EEMapPanel — 2D Top-Down (X-Y) Workspace mapping and interaction.

Redesigned as a vertical right-side panel with a styled header bar,
tight matplotlib layout, and consistent Kinetic Obsidian dark theme.
"""

from __future__ import annotations
from typing import Optional, List

import numpy as np
from PyQt6.QtWidgets import (QVBoxLayout, QWidget, QLabel, QFrame,
                              QSizePolicy, QHBoxLayout)
from PyQt6.QtCore import pyqtSignal, Qt

from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure


_BG        = "#0e0e0e"
_PANEL_BG  = "#131313"
_BORDER    = "#1e1e1e"
_ACCENT    = "#3498db"
_GRID      = "#1e2228"
_TICK_CLR  = "#454548"
_LABEL_CLR = "#89929b"
_HEADER_BG = "#0e0e0e"


class EEMapPanel(QWidget):
    """
    A 2D top-down (XY) map of the robot's workspace.
    Designed as a narrow vertical panel on the right side of the viewport.
    Shows the current EE position, trajectory history, and reachable boundary.
    Allows clicking to set a target position.
    """

    target_selected = pyqtSignal(float, float)   # Emits (x, y) when clicked

    def __init__(self, parent=None):
        super().__init__(parent)

        self.max_reach   = 0.6
        self.history_len = 200
        self.history_x: list[float] = []
        self.history_y: list[float] = []

        # Fixed width for the side panel — splitter can still resize it
        self.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Expanding)
        self.setStyleSheet(f"background-color: {_PANEL_BG};")

        self._setup_ui()
        self._setup_plot()

    # ── UI construction ───────────────────────────────────────────────────

    def _setup_ui(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        # ── Header bar ───────────────────────────────────────────────────
        header_bar = QFrame()
        header_bar.setFixedHeight(32)
        header_bar.setStyleSheet(f"""
            QFrame {{
                background-color: {_HEADER_BG};
                border-bottom: 1px solid {_BORDER};
                border-left: 1px solid {_BORDER};
            }}
        """)
        h_layout = QHBoxLayout(header_bar)
        h_layout.setContentsMargins(10, 0, 10, 0)
        h_layout.setSpacing(6)

        dot = QLabel("●")
        dot.setStyleSheet(f"color: {_ACCENT}; font-size: 8px;")
        h_layout.addWidget(dot)

        title = QLabel("WORKSPACE MAP")
        title.setStyleSheet(
            "color: #bfc7d2; font-size: 10px; font-weight: 700; "
            "letter-spacing: 0.06em;"
        )
        h_layout.addWidget(title)

        h_layout.addStretch()

        sub = QLabel("X-Y · TOP VIEW")
        sub.setStyleSheet("color: #3f4850; font-size: 9px; font-weight: 500;")
        h_layout.addWidget(sub)

        root.addWidget(header_bar)

        # ── Canvas wrapper (adds left border for visual separation) ───────
        canvas_wrap = QFrame()
        canvas_wrap.setStyleSheet(f"""
            QFrame {{
                background-color: {_PANEL_BG};
                border-left: 1px solid {_BORDER};
            }}
        """)
        wrap_layout = QVBoxLayout(canvas_wrap)
        wrap_layout.setContentsMargins(4, 4, 4, 4)
        wrap_layout.setSpacing(0)

        # Matplotlib figure — square canvas that fills the panel vertically
        self.fig = Figure(facecolor=_PANEL_BG)
        self.fig.subplots_adjust(left=0.18, right=0.97, top=0.97, bottom=0.12)

        self.canvas = FigureCanvas(self.fig)
        self.canvas.setStyleSheet("background-color: transparent;")
        self.canvas.setSizePolicy(
            QSizePolicy.Policy.Expanding,
            QSizePolicy.Policy.Expanding,
        )

        wrap_layout.addWidget(self.canvas)

        # ── Coord readout bar ─────────────────────────────────────────────
        coord_bar = QFrame()
        coord_bar.setFixedHeight(22)
        coord_bar.setStyleSheet(f"background-color: {_BG}; border-top: 1px solid {_BORDER};")
        coord_layout = QHBoxLayout(coord_bar)
        coord_layout.setContentsMargins(8, 0, 8, 0)

        self.lbl_coords = QLabel("X: —    Y: —")
        self.lbl_coords.setStyleSheet(
            "color: #3f4850; font-size: 9px; font-family: 'Consolas', monospace;"
        )
        coord_layout.addWidget(self.lbl_coords)
        coord_layout.addStretch()

        self.lbl_hint = QLabel("click to set target")
        self.lbl_hint.setStyleSheet("color: #2a3038; font-size: 9px;")
        coord_layout.addWidget(self.lbl_hint)

        wrap_layout.addWidget(coord_bar)

        root.addWidget(canvas_wrap, 1)

        # Connect click event
        self.canvas.mpl_connect('button_press_event', self._on_click)
        self.canvas.mpl_connect('motion_notify_event', self._on_mouse_move)

    def _setup_plot(self):
        self.ax = self.fig.add_subplot(111)
        self.ax.set_facecolor(_PANEL_BG)

        # Spines
        for spine in self.ax.spines.values():
            spine.set_color(_BORDER)
            spine.set_linewidth(0.8)

        # Ticks
        self.ax.tick_params(colors=_TICK_CLR, labelsize=7, length=3, width=0.6)
        for lbl in self.ax.get_xticklabels() + self.ax.get_yticklabels():
            lbl.set_color(_LABEL_CLR)

        # Grid
        self.ax.grid(True, color=_GRID, linestyle='--', linewidth=0.5, alpha=0.8)
        self.ax.set_axisbelow(True)

        # Axes labels
        self.ax.set_xlabel("X (m)", color=_LABEL_CLR, fontsize=8, labelpad=3)
        self.ax.set_ylabel("Y (m)", color=_LABEL_CLR, fontsize=8, labelpad=3)

        # Origin cross-hair
        self.ax.axhline(0, color='#252930', linewidth=0.6, zorder=0)
        self.ax.axvline(0, color='#252930', linewidth=0.6, zorder=0)

        # Reachable boundary circle
        self.boundary_line, = self.ax.plot(
            [], [], color=_ACCENT, linestyle='--', linewidth=1.0, alpha=0.45, zorder=1
        )

        # History trace
        self.trace_line, = self.ax.plot(
            [], [], color=_ACCENT, linewidth=1.0, alpha=0.35, zorder=2
        )

        # Current EE position marker
        self.ee_marker, = self.ax.plot(
            [], [], 'o', color='#e74c3c', markersize=5, zorder=4,
            markeredgecolor='#ff6b6b', markeredgewidth=0.8
        )

        # Target marker
        self.target_marker, = self.ax.plot(
            [], [], '+', color=_ACCENT, markersize=10,
            markeredgewidth=1.5, zorder=3
        )

        self.update_workspace(self.max_reach)

    # ── Public API (unchanged — backward compatible) ──────────────────────

    def update_workspace(self, radius: float):
        """Update the reachable workspace boundary circle."""
        self.max_reach = radius
        theta = np.linspace(0, 2 * np.pi, 120)
        self.boundary_line.set_data(radius * np.cos(theta), radius * np.sin(theta))

        margin = radius * 0.12
        limit  = radius + margin
        self.ax.set_xlim(-limit, limit)
        self.ax.set_ylim(-limit, limit)
        self.ax.set_aspect('equal', adjustable='box')
        self.canvas.draw_idle()

    def set_position(self, x: float, y: float):
        """Update current EE position and append to history trace."""
        self.ee_marker.set_data([x], [y])

        self.history_x.append(x)
        self.history_y.append(y)
        if len(self.history_x) > self.history_len:
            self.history_x.pop(0)
            self.history_y.pop(0)

        self.trace_line.set_data(self.history_x, self.history_y)
        self.lbl_coords.setText(f"X: {x:+.3f}    Y: {y:+.3f}")
        self.canvas.draw_idle()

    def set_target(self, x: float, y: float):
        """Update target marker position."""
        self.target_marker.set_data([x], [y])
        self.canvas.draw_idle()

    def clear_history(self):
        """Clear the EE trajectory trace."""
        self.history_x.clear()
        self.history_y.clear()
        self.trace_line.set_data([], [])
        self.canvas.draw_idle()

    # ── Mouse events ──────────────────────────────────────────────────────

    def _on_mouse_move(self, event) -> None:
        if event.inaxes != self.ax or event.xdata is None:
            return
        self.lbl_coords.setText(f"X: {event.xdata:+.3f}    Y: {event.ydata:+.3f}")

    def _on_click(self, event) -> None:
        """Handle mouse clicks — emit target_selected if within workspace."""
        if event.inaxes != self.ax or event.xdata is None:
            return

        dist = np.sqrt(event.xdata ** 2 + event.ydata ** 2)
        if dist > self.max_reach * 1.1:   # small margin for usability
            return

        x, y = event.xdata, event.ydata
        self.set_target(x, y)
        self.target_selected.emit(x, y)
