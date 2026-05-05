#!/usr/bin/env python3
"""
EEPlotWindow — simple live plot of end-effector XYZ vs time.
"""

from __future__ import annotations

from typing import Callable, Sequence

from PyQt6.QtWidgets import QVBoxLayout, QWidget, QDialog
from PyQt6.QtCore import QTimer

from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure


class EEPlotWindow(QDialog):
    """A small dialog that plots X/Y/Z vs time (seconds)."""

    def __init__(self, get_trace: Callable[[], Sequence[tuple[float, float, float, float]]], parent=None):
        super().__init__(parent)
        self.setWindowTitle("End-effector plot — X/Y/Z vs time")
        self.resize(860, 420)

        self._get_trace = get_trace

        root = QVBoxLayout(self)
        root.setContentsMargins(10, 10, 10, 10)

        fig = Figure(figsize=(9, 4), dpi=100)
        fig.patch.set_facecolor("#131313")
        self.canvas = FigureCanvas(fig)
        root.addWidget(self.canvas)

        self.ax = fig.add_subplot(111)
        self.ax.set_facecolor("#0e0e0e")
        self.ax.grid(True, color="#222", linewidth=0.6, alpha=0.6)

        self.ax.set_xlabel("t (s)", color="#89929b")
        self.ax.set_ylabel("position (m)", color="#89929b")
        self.ax.tick_params(colors="#89929b")

        self.lx, = self.ax.plot([], [], color="#92ccff", linewidth=1.4, label="X")
        self.ly, = self.ax.plot([], [], color="#2ecc71", linewidth=1.4, label="Y")
        self.lz, = self.ax.plot([], [], color="#ffba4b", linewidth=1.4, label="Z")

        leg = self.ax.legend(loc="upper right", frameon=True)
        leg.get_frame().set_facecolor("#202020")
        leg.get_frame().set_edgecolor("#353535")
        for text in leg.get_texts():
            text.set_color("#e5e2e1")

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._tick)
        self._timer.start(200)  # 5 Hz UI refresh

    def _tick(self):
        trace = list(self._get_trace() or [])
        if len(trace) < 2:
            return
        t = [p[0] for p in trace]
        x = [p[1] for p in trace]
        y = [p[2] for p in trace]
        z = [p[3] for p in trace]

        self.lx.set_data(t, x)
        self.ly.set_data(t, y)
        self.lz.set_data(t, z)

        self.ax.relim()
        self.ax.autoscale_view()
        self.canvas.draw_idle()

