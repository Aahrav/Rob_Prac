import sys
import os
sys.path.append(os.getcwd())
from frontend.main_window import MainWindow
import inspect

print(f"File: {inspect.getfile(MainWindow)}")
source, starting_line = inspect.getsourcelines(MainWindow._update_panel_visibility)
print(f"Starting line: {starting_line}")
for i, line in enumerate(source):
    print(f"{starting_line + i}: {line.strip()}")
