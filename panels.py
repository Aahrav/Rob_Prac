import sys
from PyQt6.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QHBoxLayout,
    QFrame
)

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Aarav App")
        self.setGeometry(100, 100, 900, 500)

        # Central widget (MANDATORY in QMainWindow)
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        # Layout
        layout = QHBoxLayout()
        central_widget.setLayout(layout)

        # Left panel
        self.left_panel = QFrame()
        self.left_panel.setStyleSheet("""background-color: #2c2c2c;
                                      border: 1px solid #444;""")

        # Right panel
        self.right_panel = QFrame()
        self.right_panel.setStyleSheet("""background-color: #1e1e1e;
                                       border: 1px solid #444;""")

        # Add panels to layout
        layout.addWidget(self.left_panel)
        layout.addWidget(self.right_panel)

app = QApplication(sys.argv)
window = MainWindow()
window.show()
app.exec()