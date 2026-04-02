#!/usr/bin/env python3
"""
DataPanel - Displays live Roll, Pitch, Yaw values and packet rate.
"""

from PyQt6.QtWidgets import QGroupBox, QVBoxLayout, QLabel


class DataPanel(QGroupBox):
    """Panel for real-time sensor data readout."""

    def __init__(self, parent=None):
        super().__init__("Data Readout", parent)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 12, 8, 12)
        layout.setSpacing(8)

        self.lbl_roll = QLabel("Roll: 0.0°")
        self.lbl_pitch = QLabel("Pitch: 0.0°")
        self.lbl_yaw = QLabel("Yaw: 0.0°")
        self.lbl_rate = QLabel("Rate: 0 pps")

        # Style
        for lbl in [self.lbl_roll, self.lbl_pitch, self.lbl_yaw]:
            lbl.setStyleSheet("font-size: 14px; font-weight: bold; color: #eee; padding: 4px;")
            layout.addWidget(lbl)

        layout.addSpacing(6)
        self.lbl_rate.setStyleSheet("font-size: 12px; color: #aaa; padding: 4px;")
        layout.addWidget(self.lbl_rate)

        self.setStyleSheet("""
            QGroupBox {
                border: 1px solid #444;
                border-radius: 5px;
                margin-top: 10px;
                font-weight: bold;
                padding: 10px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
            }
        """)

    def update_values(self, roll: float, pitch: float, yaw: float, rate: float = None):
        """Update displayed values."""
        self.lbl_roll.setText(f"Roll: {roll:6.1f}°")
        self.lbl_pitch.setText(f"Pitch: {pitch:6.1f}°")
        self.lbl_yaw.setText(f"Yaw: {yaw:6.1f}°")
        if rate is not None:
            self.lbl_rate.setText(f"Rate: {rate:.0f} pps")
