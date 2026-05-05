#!/usr/bin/env python3
"""
EEMapPanel — 2D Top-Down (X-Y) Workspace mapping and interaction.
"""

from __future__ import annotations
from typing import Optional, List

import numpy as np
from PyQt6.QtWidgets import QVBoxLayout, QWidget, QLabel
from PyQt6.QtCore import pyqtSignal, Qt

from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.pyplot as plt

class EEMapPanel(QWidget):
    """
    A 2D top-down (XY) map of the robot's workspace.
    Shows the current EE position, trajectory history, and reachable boundary.
    Allows clicking to set a target position.
    """
    
    target_selected = pyqtSignal(float, float)  # Emits (x, y) when clicked

    def __init__(self, parent=None):
        super().__init__(parent)
        
        self.max_reach = 0.6  # Default, updated by config
        self.history_len = 200
        self.history_x = []
        self.history_y = []
        
        self._setup_ui()
        self._setup_plot()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(5)
        
        # Header
        header = QLabel("WORKSPACE MAP (X-Y TOP VIEW)")
        header.setStyleSheet("color: #89929b; font-size: 10px; font-weight: 600; letter-spacing: 0.05em;")
        layout.addWidget(header)
        
        # Matplotlib Figure
        # Using same dark theme colors as ArmCanvas
        self.fig = Figure(figsize=(4, 4), facecolor='#131313')
        self.canvas = FigureCanvas(self.fig)
        self.canvas.setStyleSheet("background-color: transparent;")
        layout.addWidget(self.canvas)
        
        # Connect click event
        self.canvas.mpl_connect('button_press_event', self._on_click)

    def _setup_plot(self):
        self.ax = self.fig.add_subplot(111)
        self.ax.set_facecolor('#131313')
        
        # Remove spines and ticks for a cleaner map look
        for spine in self.ax.spines.values():
            spine.set_color('#333')
        self.ax.tick_params(colors='#555', labelsize=8)
        self.ax.grid(True, color='#252525', linestyle='--', linewidth=0.5)
        
        self.ax.set_xlabel("X (m)", color='#555', fontsize=8)
        self.ax.set_ylabel("Y (m)", color='#555', fontsize=8)
        
        # Reachable boundary
        theta = np.linspace(0, 2*np.pi, 100)
        self.boundary_line, = self.ax.plot([], [], color='#3498db', linestyle='--', linewidth=1, alpha=0.5)
        
        # History trace
        self.trace_line, = self.ax.plot([], [], color='#3498db', linewidth=1, alpha=0.3)
        
        # Current position marker
        self.ee_marker, = self.ax.plot([], [], 'ro', markersize=6, label='EE')
        
        # Target marker
        self.target_marker, = self.ax.plot([], [], 'bx', markersize=8, markeredgewidth=2, label='Target')
        
        self.update_workspace(self.max_reach)
        self.fig.tight_layout()

    def update_workspace(self, radius: float):
        """Update the reachable workspace boundary."""
        self.max_reach = radius
        theta = np.linspace(0, 2*np.pi, 100)
        bx = radius * np.cos(theta)
        by = radius * np.sin(theta)
        self.boundary_line.set_data(bx, by)
        
        # Set axis limits with some margin
        margin = radius * 0.1
        limit = radius + margin
        self.ax.set_xlim(-limit, limit)
        self.ax.set_ylim(-limit, limit)
        self.ax.set_aspect('equal')
        self.canvas.draw_idle()

    def set_position(self, x: float, y: float):
        """Update current EE position and add to history."""
        self.ee_marker.set_data([x], [y])
        
        self.history_x.append(x)
        self.history_y.append(y)
        if len(self.history_x) > self.history_len:
            self.history_x.pop(0)
            self.history_y.pop(0)
            
        self.trace_line.set_data(self.history_x, self.history_y)
        self.canvas.draw_idle()

    def set_target(self, x: float, y: float):
        """Update target marker position."""
        self.target_marker.set_data([x], [y])
        self.canvas.draw_idle()

    def clear_history(self):
        """Clear the trajectory trace."""
        self.history_x.clear()
        self.history_y.clear()
        self.trace_line.set_data([], [])
        self.canvas.draw_idle()

    def _on_click(self, event):
        """Handle mouse clicks on the canvas."""
        if event.inaxes != self.ax:
            return
        
        # Check if click is within reach
        dist = np.sqrt(event.xdata**2 + event.ydata**2)
        if dist > self.max_reach * 1.1: # Allow a small margin
            return
            
        x, y = event.xdata, event.ydata
        self.set_target(x, y)
        self.target_selected.emit(x, y)
