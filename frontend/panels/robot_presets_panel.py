from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QScrollArea, QFrame, QButtonGroup, QSizePolicy
)
from PyQt6.QtCore import pyqtSignal, Qt, pyqtSlot
from PyQt6.QtGui import QCursor

from backend.robot_presets import PRESETS, CATEGORIES


class PresetCard(QFrame):
    """A card representing a single robot preset."""
    selected_signal = pyqtSignal(str)

    def __init__(self, name: str, meta: dict, parent=None):
        super().__init__(parent)
        self.preset_name = name
        self.meta = meta
        self.is_selected = False

        self.setCursor(QCursor(Qt.CursorShape.PointingHandCursor))
        self.setFixedHeight(70)

        # Base style
        self.default_style = """
            PresetCard {
                background-color: #0e0e0e;
                border: 1px solid #1a1a1a;
                border-radius: 4px;
            }
            PresetCard:hover {
                background-color: #161616;
                border: 1px solid #2a2a2a;
            }
        """
        self.selected_style = """
            PresetCard {
                background-color: #121820;
                border: 1px solid #3498db;
                border-left: 4px solid #3498db;
                border-radius: 4px;
            }
        """
        self.setStyleSheet(self.default_style)

        layout = QHBoxLayout(self)
        layout.setContentsMargins(10, 8, 10, 8)
        layout.setSpacing(10)

        # Icon
        lbl_icon = QLabel(meta.get("icon", "🤖"))
        lbl_icon.setStyleSheet("font-size: 24px; background: transparent; border: none;")
        lbl_icon.setFixedSize(30, 30)
        lbl_icon.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(lbl_icon)

        # Text column
        text_layout = QVBoxLayout()
        text_layout.setSpacing(2)

        header_layout = QHBoxLayout()
        header_layout.setSpacing(5)

        lbl_name = QLabel(name)
        lbl_name.setStyleSheet("color: #e3e2e2; font-size: 13px; font-weight: 600; font-family: Inter, sans-serif; background: transparent; border: none;")
        header_layout.addWidget(lbl_name)

        header_layout.addStretch()

        # Badge
        dof = meta.get("dof", 3)
        lbl_badge = QLabel(f"{dof}-DOF")
        
        # Color based on DOF
        if dof <= 3:
            badge_color = "#2ecc71" # green
        elif dof == 4:
            badge_color = "#f39c12" # orange
        else:
            badge_color = "#3498db" # blue
            
        lbl_badge.setStyleSheet(f"""
            color: {badge_color};
            border: 1px solid {badge_color};
            border-radius: 3px;
            padding: 1px 4px;
            font-size: 9px;
            font-weight: 700;
            background: rgba(0, 0, 0, 0.3);
        """)
        header_layout.addWidget(lbl_badge)
        text_layout.addLayout(header_layout)

        lbl_desc = QLabel(meta.get("description", ""))
        lbl_desc.setStyleSheet("color: #89929b; font-size: 11px; font-family: Inter, sans-serif; background: transparent; border: none;")
        lbl_desc.setWordWrap(True)
        text_layout.addWidget(lbl_desc)

        layout.addLayout(text_layout, 1)

    def set_selected(self, selected: bool):
        self.is_selected = selected
        self.setStyleSheet(self.selected_style if selected else self.default_style)

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self.selected_signal.emit(self.preset_name)
        super().mousePressEvent(event)


class RobotPresetsPanel(QWidget):
    """Panel to select and load pre-configured DH robot models."""
    
    # Emits the preset name when "Load Robot" is clicked
    load_requested = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.selected_preset = None
        self.cards: list[PresetCard] = []

        self._setup_ui()
        # Default select the first one
        if self.cards:
            self._on_card_selected(self.cards[0].preset_name)

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(12)
        
        # We assume background is provided by AccordionSection/MainWindow (#131313)
        
        # 1. Category Filter Row
        filter_layout = QHBoxLayout()
        filter_layout.setSpacing(4)
        
        self.btn_group = QButtonGroup(self)
        self.btn_group.setExclusive(True)
        
        categories = ["All"] + CATEGORIES
        for i, cat in enumerate(categories):
            btn = QPushButton(cat)
            btn.setCheckable(True)
            if i == 0:
                btn.setChecked(True)
                
            # Kinetic Obsidian pill styling
            btn.setStyleSheet("""
                QPushButton {
                    background-color: #202020;
                    color: #89929b;
                    border: 1px solid #1a1a1a;
                    border-radius: 12px;
                    padding: 4px 10px;
                    font-size: 11px;
                    font-weight: 600;
                }
                QPushButton:hover {
                    background-color: #2a2a2a;
                    color: #e3e2e2;
                }
                QPushButton:checked {
                    background-color: #3498db;
                    color: #ffffff;
                    border: 1px solid #3498db;
                }
            """)
            btn.toggled.connect(self._on_filter_changed)
            self.btn_group.addButton(btn, i)
            filter_layout.addWidget(btn)
            
        filter_layout.addStretch()
        layout.addLayout(filter_layout)

        # 2. Scrollable List of Robots
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        scroll.setStyleSheet("""
            QScrollArea { background: transparent; }
            QScrollBar:vertical {
                background: #131313; width: 8px; margin: 0px;
            }
            QScrollBar::handle:vertical {
                background: #353535; border-radius: 4px; min-height: 20px;
            }
            QScrollBar::handle:vertical:hover { background: #454548; }
            QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical { background: none; }
        """)

        self.list_container = QWidget()
        self.list_container.setStyleSheet("background: transparent;")
        self.list_layout = QVBoxLayout(self.list_container)
        self.list_layout.setContentsMargins(0, 0, 4, 0)
        self.list_layout.setSpacing(8)

        # Populate cards
        for name, meta in PRESETS.items():
            card = PresetCard(name, meta)
            card.selected_signal.connect(self._on_card_selected)
            self.cards.append(card)
            self.list_layout.addWidget(card)

        self.list_layout.addStretch()
        scroll.setWidget(self.list_container)
        
        # Fix panel height to ensure it fits well in accordion
        scroll.setMinimumHeight(250)
        layout.addWidget(scroll)

        # 3. Footer
        footer_layout = QVBoxLayout()
        footer_layout.setSpacing(4)

        self.btn_load = QPushButton("Load Robot")
        self.btn_load.setCursor(QCursor(Qt.CursorShape.PointingHandCursor))
        self.btn_load.setStyleSheet("""
            QPushButton {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #2980b9, stop:1 #3498db);
                color: white;
                border: none;
                border-radius: 4px;
                padding: 10px;
                font-size: 14px;
                font-weight: bold;
            }
            QPushButton:hover {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #3498db, stop:1 #4aa3df);
            }
            QPushButton:pressed {
                background: #2573a7;
            }
            QPushButton:disabled {
                background: #202020;
                color: #555555;
            }
        """)
        self.btn_load.clicked.connect(self._on_load_clicked)
        footer_layout.addWidget(self.btn_load)

        lbl_hint = QLabel("Loads into Custom DH mode")
        lbl_hint.setAlignment(Qt.AlignmentFlag.AlignCenter)
        lbl_hint.setStyleSheet("color: #89929b; font-size: 10px; font-weight: 500;")
        footer_layout.addWidget(lbl_hint)

        layout.addLayout(footer_layout)

    @pyqtSlot(bool)
    def _on_filter_changed(self, checked: bool):
        if not checked:
            return
        
        btn = self.sender()
        if not isinstance(btn, QPushButton): return
        category = btn.text()

        # Filter visibility
        for card in self.cards:
            if category == "All" or card.meta.get("category") == category:
                card.setVisible(True)
            else:
                card.setVisible(False)
                
    @pyqtSlot(str)
    def _on_card_selected(self, preset_name: str):
        self.selected_preset = preset_name
        for card in self.cards:
            card.set_selected(card.preset_name == preset_name)
        self.btn_load.setEnabled(True)

    @pyqtSlot()
    def _on_load_clicked(self):
        if self.selected_preset:
            self.load_requested.emit(self.selected_preset)
