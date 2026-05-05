import sys
import os
sys.path.append(os.getcwd())
from PyQt6.QtWidgets import QApplication

app = QApplication(sys.argv)
try:
    from frontend.panels.connection_panel import ConnectionPanel
    print("Import ConnectionPanel OK")
    p = ConnectionPanel()
    print("Instantiate ConnectionPanel OK")
except Exception as e:
    import traceback
    traceback.print_exc()
