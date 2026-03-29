import sys
from PyQt6.QtWidgets import QApplication, QLabel, QMainWindow
class Helloworld(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Hello World")
        self.setGeometry(100, 100, 400, 300)
        label = QLabel("Hello, World!", self)
        label.move(150, 130)
app = QApplication(sys.argv)
window = Helloworld()
window.show()
app.exec()