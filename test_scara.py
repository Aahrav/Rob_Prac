import sys
import threading
import traceback
from PyQt6.QtWidgets import QApplication
from frontend.app import MainWindow
from backend.robot_presets import preset_scara
import os

def trace_calls(frame, event, arg):
    if event == 'call':
        func_name = frame.f_code.co_name
        filename = frame.f_code.co_filename
        if 'rob_project' in filename:
            with open('trace_output.txt', 'a', encoding='utf-8') as f:
                f.write(f"{filename}:{frame.f_lineno} {func_name}\n")
    return trace_calls

with open('trace_output.txt', 'w', encoding='utf-8') as f:
    f.write("Starting trace\n")

sys.settrace(trace_calls)

app = QApplication(sys.argv)
window = MainWindow(replay_path=None, record_path=None, use_filter=False)
window.show()

# Set a timer to trigger the SCARA click
def click_scara():
    print("Clicking SCARA...")
    chain = preset_scara()
    try:
        window._on_preset_loaded(chain)
        print("SCARA click finished.")
    except Exception as e:
        print(f"Exception: {e}")
        traceback.print_exc()
    sys.exit(0)

import threading
threading.Timer(1.0, click_scara).start()

app.exec()
