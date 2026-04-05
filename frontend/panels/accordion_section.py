#!/usr/bin/env python3
"""
AccordionSection - Collapsible container with header and content.
Used for grouping controls in the left panel.
"""

from PyQt6.QtWidgets import QWidget, QVBoxLayout, QPushButton, QFrame, QScrollArea, QSizePolicy
from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtGui import QFont

class AccordionSection(QWidget):
    """A collapsible section with a header button and scrollable content."""

    def __init__(self, title: str, parent=None):
        super().__init__(parent)
        self.title = title
        self._expanded = True
        self._init_ui()

    def _init_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)

        # Header button (always visible)
        self.header_btn = QPushButton(self.title)
        self.header_btn.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self.header_btn.setStyleSheet("""
            QPushButton {
                background-color: #3a3a3c;
                color: #ffffff;
                border: none;
                padding: 8px 12px;
                font-weight: bold;
                font-size: 12px;
                text-align: left;
                border-radius: 4px;
            }
            QPushButton:hover {
                background-color: #454548;
            }
        """)
        self.header_btn.clicked.connect(self._toggle)
        layout.addWidget(self.header_btn)

        # Content area (scrollable)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        scroll.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.MinimumExpanding)
        scroll.setStyleSheet("QScrollArea { border: none; background-color: transparent; }")

        self.content = QWidget()
        self.content_layout = QVBoxLayout(self.content)
        self.content_layout.setContentsMargins(12, 8, 12, 12)
        self.content_layout.setSpacing(8)
        self.content.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

        scroll.setWidget(self.content)
        layout.addWidget(scroll)

        self.scroll_area = scroll

    def _toggle(self):
        self._expanded = not self._expanded
        self.scroll_area.setVisible(self._expanded)
        # Update chevron indicator
        arrow = "▼" if self._expanded else "▶"
        self.header_btn.setText(f"{arrow}  {self.title}")
        # Ensure proper visual state change: clear focus, release pressed state
        self.header_btn.setAutoDefault(False)
        self.header_btn.clearFocus()
        self.header_btn.repaint()
        # Also force the section to repaint so the layout update is visible
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
