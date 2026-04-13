#!/usr/bin/env python3
"""
DataPanel - Displays live Roll, Pitch, Yaw values.
Kinetic Obsidian theme: compact monospace readout card.
"""

from PyQt6.QtWidgets import QWidget, QGridLayout, QLabel, QFrame, QVBoxLayout
from PyQt6.QtCore import Qt


CARD_STYLE = """
    QWidget#data_card {
        background-color: #0e0e0e;
        border-radius: 4px;
    }
"""

VAL_LABEL_STYLE = "color: #92ccff; font-family: 'Consolas', monospace; font-size: 13px; font-weight: 700;"
KEY_LABEL_STYLE = "color: #89929b; font-size: 10px; font-weight: 600; letter-spacing: 0.05em;"
UNIT_LABEL_STYLE = "color: #3f4850; font-size: 10px;"


class DataPanel(QWidget):
    """Live joint angle / data readout panel."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._init_ui()

    def _init_ui(self):
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)

        # Milled card container
        card = QFrame()
        card.setObjectName("data_card")
        card.setStyleSheet("QFrame#data_card { background-color: #0e0e0e; border-radius: 4px; }")
        grid = QGridLayout(card)
        grid.setContentsMargins(10, 8, 10, 8)
        grid.setHorizontalSpacing(12)
        grid.setVerticalSpacing(4)

        def row(r, key, unit_text="°", attr_prefix=""):
            lbl_key = QLabel(key)
            lbl_key.setStyleSheet(KEY_LABEL_STYLE)
            grid.addWidget(lbl_key, r, 0)

            lbl_val = QLabel("  0.0")
            lbl_val.setStyleSheet(VAL_LABEL_STYLE)
            lbl_val.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            grid.addWidget(lbl_val, r, 1)

            lbl_unit = QLabel(unit_text)
            lbl_unit.setStyleSheet(UNIT_LABEL_STYLE)
            grid.addWidget(lbl_unit, r, 2)
            return lbl_val

        self.lbl_roll  = row(0, "ROLL",  "°")
        self.lbl_pitch = row(1, "PITCH", "°")
        self.lbl_yaw   = row(2, "YAW",   "°")

        outer.addWidget(card)

    def update_values(self, roll: float, pitch: float, yaw: float, rate: float = None):
        self.lbl_roll.setText(f"{roll:+7.1f}")
        self.lbl_pitch.setText(f"{pitch:+7.1f}")
        self.lbl_yaw.setText(f"{yaw:+7.1f}")
        # Color-code by magnitude
        for val, lbl in [(abs(roll), self.lbl_roll), (abs(pitch), self.lbl_pitch), (abs(yaw), self.lbl_yaw)]:
            if val > 60:
                color = "#ffba4b"
            elif val > 30:
                color = "#92ccff"
            else:
                color = "#abcae8"
            lbl.setStyleSheet(VAL_LABEL_STYLE.replace("#92ccff", color))
