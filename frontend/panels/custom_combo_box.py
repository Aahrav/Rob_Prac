#!/usr/bin/env python3
"""
CustomComboBox - QComboBox with improved focus management and visual behavior.
Prevents dropdowns from getting visually stuck.
"""

from PyQt6.QtWidgets import QComboBox


class CustomComboBox(QComboBox):
    """QComboBox that clears focus and repaints after selection."""

    def __init__(self, parent=None):
        super().__init__(parent)
        # Connect activated signal (fires on selection) to cleanup
        self.activated.connect(self._on_selection)

    def _on_selection(self):
        """Called when user selects an option. Clear focus and force repaint."""
        self.clearFocus()
        self.repaint()
