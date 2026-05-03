#!/usr/bin/env python3
"""
AccordionSection - Collapsible container (Kinetic Precision theme).
Uses tonal layering — no border lines, background contrast defines sections.
Active sections get a left-border accent indicator per the Kinetic Precision spec.
"""

from PyQt6.QtWidgets import QWidget, QVBoxLayout, QPushButton, QScrollArea, QSizePolicy
from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtGui import QFont


ACCORDION_HEADER_EXPANDED = """
    QPushButton {{
        background-color: #222832;
        color: #dbe3ed;
        border: none;
        border-top: 1px solid rgba(255, 255, 255, 0.03);
        border-left: 3px solid #3498db;
        padding: 10px 14px;
        font-family: 'Space Grotesk', 'Inter', sans-serif;
        font-weight: 600;
        font-size: 11px;
        text-align: left;
        text-transform: uppercase;
        letter-spacing: 0.1em;
        border-radius: 0px;
    }}
    QPushButton:hover {{
        background-color: #2a3040;
        color: #e5e2e1;
    }}
    QPushButton:pressed {{
        background-color: #1a1e24;
    }}
"""

ACCORDION_HEADER_COLLAPSED = """
    QPushButton {{
        background-color: #222832;
        color: #89929b;
        border: none;
        border-top: 1px solid rgba(255, 255, 255, 0.03);
        border-left: 3px solid transparent;
        padding: 10px 14px;
        font-family: 'Space Grotesk', 'Inter', sans-serif;
        font-weight: 600;
        font-size: 11px;
        text-align: left;
        text-transform: uppercase;
        letter-spacing: 0.1em;
        border-radius: 0px;
    }}
    QPushButton:hover {{
        background-color: #2a3040;
        color: #bfc7d2;
    }}
    QPushButton:pressed {{
        background-color: #1a1e24;
    }}
"""


class AccordionSection(QWidget):
    """A collapsible section with a header button and content area."""

    def __init__(self, title: str, parent=None):
        super().__init__(parent)
        self.title = title
        self._expanded = True
        self._init_ui()

    def _init_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        # Header button
        self.header_btn = QPushButton(f"▼  {self.title}")
        self.header_btn.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self.header_btn.setFixedHeight(40)
        self.header_btn.setStyleSheet(ACCORDION_HEADER_EXPANDED)
        self.header_btn.clicked.connect(self._toggle)
        layout.addWidget(self.header_btn)

        # Content area with glassmorphism top-line highlight
        self.content = QWidget()
        self.content.setStyleSheet("""
            background-color: #141820;
            border-top: 1px solid rgba(255, 255, 255, 0.025);
        """)
        self.content_layout = QVBoxLayout(self.content)
        self.content_layout.setContentsMargins(12, 10, 12, 12)
        self.content_layout.setSpacing(8)
        self.content.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)
        layout.addWidget(self.content)

    def _toggle(self):
        self._expanded = not self._expanded
        self.content.setVisible(self._expanded)
        arrow = "▼" if self._expanded else "▶"
        self.header_btn.setText(f"{arrow}  {self.title}")
        if self._expanded:
            self.header_btn.setStyleSheet(ACCORDION_HEADER_EXPANDED)
        else:
            self.header_btn.setStyleSheet(ACCORDION_HEADER_COLLAPSED)
        self.header_btn.setAutoDefault(False)
        self.header_btn.clearFocus()
        self.header_btn.repaint()
        self.repaint()

    def expand(self):
        if not self._expanded:
            self._toggle()

    def collapse(self):
        if self._expanded:
            self._toggle()

    def addWidget(self, widget):
        self.content_layout.addWidget(widget)

    def addLayout(self, layout):
        self.content_layout.addLayout(layout)

    def addStretch(self):
        self.content_layout.addStretch()

    def setContentLayout(self, layout):
        # Clear existing
        while self.content_layout.count():
            item = self.content_layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()
        self.content_layout.addLayout(layout)
