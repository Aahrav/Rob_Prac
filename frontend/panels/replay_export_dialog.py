#!/usr/bin/env python3
"""
replay_export_dialog.py — Generic export dialog for replay sessions.

Exports:
  - CSV     : joint angles + EE positions + torques per frame
  - PNG seq : one image per frame via a RenderDelegate
  - GIF     : animated GIF via Pillow (always available)
  - MP4     : video via FFMpegWriter (shown only if ffmpeg is on PATH)

Design:
  - Knows ReplayBuffer (data) and a render_fn callable (robot-agnostic).
  - render_fn(frame) → matplotlib.figure.Figure  (or None to skip)
  - No direct robot model or canvas imports.
"""

from __future__ import annotations

import shutil
from pathlib import Path
from typing import Callable, Optional

from PyQt6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QTabWidget, QWidget,
    QLabel, QPushButton, QLineEdit, QSpinBox, QDoubleSpinBox,
    QFileDialog, QProgressBar, QMessageBox, QFormLayout, QComboBox,
    QCheckBox,
)
from PyQt6.QtCore import Qt, QThread, pyqtSignal

from backend.replay_buffer import ReplayBuffer, Frame


# ──────────────────────────────────────────────────────────────────────────────
#  Helpers
# ──────────────────────────────────────────────────────────────────────────────

_FFMPEG_AVAILABLE = shutil.which("ffmpeg") is not None

_DIALOG_QSS = """
QDialog { background: #131313; color: #e5e2e1; }
QTabWidget::pane { border: 1px solid #2a2a2a; background: #131313; }
QTabBar::tab {
    background: #1a1a1a; color: #89929b;
    padding: 6px 16px; border: 1px solid #2a2a2a;
    border-bottom: none; border-radius: 3px 3px 0 0;
}
QTabBar::tab:selected { background: #3498db; color: #fff; border-color: #3498db; }
QLabel { color: #c8d0da; }
QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox {
    background: #1a1a1a; color: #e5e2e1;
    border: 1px solid #2a2a2a; border-radius: 4px; padding: 4px 8px;
}
QPushButton {
    background: #2a2a2a; color: #e5e2e1;
    border: 1px solid #353535; border-radius: 4px;
    padding: 5px 14px; font-size: 11px;
}
QPushButton:hover { background: #353535; }
QPushButton#btn_export_action {
    background: #2980b9; border-color: #3498db; font-weight: 600;
}
QPushButton#btn_export_action:hover { background: #3498db; }
QProgressBar {
    background: #1a1a1a; border: 1px solid #2a2a2a; border-radius: 4px;
    text-align: center; color: #e5e2e1; font-size: 10px;
}
QProgressBar::chunk { background: #3498db; border-radius: 3px; }
"""


def _path_row(label: str, placeholder: str, ext_filter: str, save: bool = True):
    """Returns (QLineEdit, QPushButton) for a path selector row."""
    edit = QLineEdit()
    edit.setPlaceholderText(placeholder)
    btn = QPushButton("Browse…")

    def _browse():
        if save:
            p, _ = QFileDialog.getSaveFileName(None, f"Save {label}", "", ext_filter)
        else:
            p = QFileDialog.getExistingDirectory(None, f"Select folder for {label}")
        if p:
            edit.setText(p)

    btn.clicked.connect(_browse)
    return edit, btn


# ──────────────────────────────────────────────────────────────────────────────
#  Export worker thread (keeps UI responsive)
# ──────────────────────────────────────────────────────────────────────────────

class _ExportWorker(QThread):
    progress = pyqtSignal(int)   # 0–100
    finished = pyqtSignal(bool, str)  # success, message

    def __init__(self, task: Callable[[], None]):
        super().__init__()
        self._task = task

    def run(self):
        try:
            self._task()
            self.finished.emit(True, "Export completed successfully.")
        except Exception as exc:
            self.finished.emit(False, str(exc))


# ──────────────────────────────────────────────────────────────────────────────
#  Tab widgets
# ──────────────────────────────────────────────────────────────────────────────

class _CSVTab(QWidget):
    def __init__(self, buffer: ReplayBuffer):
        super().__init__()
        self._buf = buffer
        lay = QVBoxLayout(self)
        lay.setSpacing(10)

        form = QFormLayout()
        self._edit_path, btn = _path_row("CSV", "output.csv", "CSV files (*.csv)")
        row = QHBoxLayout()
        row.addWidget(self._edit_path)
        row.addWidget(btn)
        form.addRow("Output file:", row)
        lay.addLayout(form)

        info = QLabel(
            f"Will export {len(buffer)} frames × "
            f"({len(buffer.get_frame(0).joints) if buffer else 0} joints + EE + torques) columns."
            if buffer else "No frames recorded yet."
        )
        info.setStyleSheet("color: #89929b; font-size: 10px;")
        lay.addWidget(info)

        lay.addStretch()
        self.btn_go = QPushButton("Export CSV")
        self.btn_go.setObjectName("btn_export_action")
        lay.addWidget(self.btn_go)
        self._progress = QProgressBar()
        self._progress.setVisible(False)
        lay.addWidget(self._progress)

    def run_export(self) -> None:
        path = self._edit_path.text().strip()
        if not path:
            QMessageBox.warning(self, "No path", "Please choose an output file.")
            return
        self._progress.setVisible(True)
        self._progress.setValue(0)

        def task():
            self._buf.export_csv(path)
            self._progress.setValue(100)

        self._worker = _ExportWorker(task)
        self._worker.finished.connect(self._done)
        self._worker.start()

    def _done(self, ok: bool, msg: str) -> None:
        self._progress.setVisible(False)
        if ok:
            QMessageBox.information(self, "Done", msg)
        else:
            QMessageBox.critical(self, "Export failed", msg)


class _PNGTab(QWidget):
    def __init__(self, buffer: ReplayBuffer, render_fn: Optional[Callable]):
        super().__init__()
        self._buf = buffer
        self._render_fn = render_fn
        lay = QVBoxLayout(self)
        lay.setSpacing(10)

        form = QFormLayout()
        self._edit_dir, btn = _path_row("PNG folder", "/output/frames", "", save=False)
        row = QHBoxLayout()
        row.addWidget(self._edit_dir)
        row.addWidget(btn)
        form.addRow("Output folder:", row)
        self._spin_dpi = QSpinBox()
        form.addRow("DPI:", self._spin_dpi)
        self._spin_dpi.setRange(72, 300)
        self._spin_dpi.setValue(100)
        lay.addLayout(form)

        if not render_fn:
            warn = QLabel("⚠ No render function provided — PNG export unavailable.")
            warn.setStyleSheet("color: #f39c12;")
            lay.addWidget(warn)

        lay.addStretch()
        self.btn_go = QPushButton("Export PNG Sequence")
        self.btn_go.setObjectName("btn_export_action")
        self.btn_go.setEnabled(bool(render_fn and buffer))
        lay.addWidget(self.btn_go)
        self._progress = QProgressBar()
        self._progress.setVisible(False)
        lay.addWidget(self._progress)

    def run_export(self) -> None:
        folder = self._edit_dir.text().strip()
        if not folder:
            QMessageBox.warning(self, "No folder", "Please choose an output folder.")
            return
        out = Path(folder)
        out.mkdir(parents=True, exist_ok=True)
        dpi = self._spin_dpi.value()
        n = len(self._buf)
        self._progress.setVisible(True)
        self._progress.setMaximum(n)

        render_fn = self._render_fn
        buf = self._buf

        def task():
            for i, frame in enumerate(buf):
                fig = render_fn(frame)
                if fig is not None:
                    fig.savefig(out / f"frame_{i:05d}.png", dpi=dpi, bbox_inches="tight")
                self._progress.setValue(i + 1)

        self._worker = _ExportWorker(task)
        self._worker.finished.connect(self._done)
        self._worker.start()

    def _done(self, ok, msg):
        self._progress.setVisible(False)
        if ok:
            QMessageBox.information(self, "Done", msg)
        else:
            QMessageBox.critical(self, "Export failed", msg)


class _AnimationTab(QWidget):
    """Shared base for GIF and MP4 tabs."""

    def __init__(self, buffer: ReplayBuffer, render_fn: Optional[Callable],
                 fmt: str, ext_filter: str, writer_name: str, available: bool):
        super().__init__()
        self._buf = buffer
        self._render_fn = render_fn
        self._writer_name = writer_name
        self._fmt = fmt

        lay = QVBoxLayout(self)
        lay.setSpacing(10)

        if not available:
            unavail = QLabel(f"⚠ {fmt} export requires ffmpeg on PATH.\nInstall ffmpeg and restart.")
            unavail.setStyleSheet("color: #f39c12;")
            lay.addWidget(unavail)
            lay.addStretch()
            return

        form = QFormLayout()
        self._edit_path, btn = _path_row(fmt, f"output.{fmt.lower()}", ext_filter)
        row = QHBoxLayout()
        row.addWidget(self._edit_path)
        row.addWidget(btn)
        form.addRow("Output file:", row)

        self._spin_fps = QSpinBox()
        self._spin_fps.setRange(5, 120)
        self._spin_fps.setValue(30)
        form.addRow("FPS:", self._spin_fps)

        self._spin_dpi = QSpinBox()
        self._spin_dpi.setRange(72, 300)
        self._spin_dpi.setValue(100)
        form.addRow("DPI:", self._spin_dpi)

        lay.addLayout(form)
        lay.addStretch()

        self.btn_go = QPushButton(f"Export {fmt}")
        self.btn_go.setObjectName("btn_export_action")
        self.btn_go.setEnabled(bool(render_fn and buffer))
        lay.addWidget(self.btn_go)
        self._progress = QProgressBar()
        self._progress.setVisible(False)
        lay.addWidget(self._progress)

    def run_export(self) -> None:
        if not hasattr(self, '_edit_path'):
            return
        path = self._edit_path.text().strip()
        if not path:
            QMessageBox.warning(self, "No path", "Please choose an output file.")
            return

        import matplotlib.animation as anim
        from matplotlib.figure import Figure

        fps = self._spin_fps.value()
        dpi = self._spin_dpi.value()
        buf = self._buf
        render_fn = self._render_fn
        writer_name = self._writer_name
        n = len(buf)

        self._progress.setVisible(True)
        self._progress.setMaximum(n)

        def task():
            fig = render_fn(buf.get_frame(0))
            if fig is None:
                raise RuntimeError("render_fn returned None — cannot export.")

            writer_cls = (
                anim.FFMpegWriter if writer_name == "ffmpeg"
                else anim.PillowWriter
            )
            writer = writer_cls(fps=fps)
            with writer.saving(fig, path, dpi):
                for i, frame in enumerate(buf):
                    render_fn(frame)        # updates fig in-place
                    writer.grab_frame()
                    self._progress.setValue(i + 1)

        self._worker = _ExportWorker(task)
        self._worker.finished.connect(self._done)
        self._worker.start()

    def _done(self, ok, msg):
        self._progress.setVisible(False)
        if ok:
            QMessageBox.information(self, "Done", msg)
        else:
            QMessageBox.critical(self, "Export failed", msg)


# ──────────────────────────────────────────────────────────────────────────────
#  ReplayExportDialog
# ──────────────────────────────────────────────────────────────────────────────

class ReplayExportDialog(QDialog):
    """
    Multi-format export dialog for replay sessions.

    Parameters
    ----------
    buffer      : ReplayBuffer — the frames to export
    render_fn   : Callable[[Frame], Figure | None] — robot-agnostic renderer
                  (set to None to disable PNG/GIF/MP4 tabs)
    """

    def __init__(
        self,
        buffer: ReplayBuffer,
        render_fn: Optional[Callable] = None,
        parent=None,
    ):
        super().__init__(parent)
        self.setWindowTitle("Export Replay")
        self.setMinimumSize(480, 340)
        self.setStyleSheet(_DIALOG_QSS)

        root = QVBoxLayout(self)

        # Header
        hdr = QLabel(
            f"<b>Export Replay Session</b> — "
            f"{len(buffer)} frames, {buffer.duration:.1f}s duration"
        )
        hdr.setStyleSheet("font-size: 12px; padding: 6px 0;")
        root.addWidget(hdr)

        tabs = QTabWidget()
        root.addWidget(tabs)

        # CSV tab (always available)
        self._csv_tab = _CSVTab(buffer)
        self._csv_tab.btn_go.clicked.connect(self._csv_tab.run_export)
        tabs.addTab(self._csv_tab, "📊 CSV")

        # PNG tab
        self._png_tab = _PNGTab(buffer, render_fn)
        self._png_tab.btn_go.clicked.connect(self._png_tab.run_export)
        tabs.addTab(self._png_tab, "🖼 PNG Sequence")

        # GIF tab (Pillow)
        try:
            from PIL import Image  # noqa: F401
            gif_ok = True
        except ImportError:
            gif_ok = False
        self._gif_tab = _AnimationTab(
            buffer, render_fn, "GIF", "GIF (*.gif)", "pillow", gif_ok
        )
        self._gif_tab.btn_go.clicked.connect(self._gif_tab.run_export)
        tabs.addTab(self._gif_tab, "🎞 GIF")

        # MP4 tab (ffmpeg)
        self._mp4_tab = _AnimationTab(
            buffer, render_fn, "MP4", "Video (*.mp4)", "ffmpeg", _FFMPEG_AVAILABLE
        )
        self._mp4_tab.btn_go.clicked.connect(self._mp4_tab.run_export)
        tabs.addTab(self._mp4_tab, "🎬 MP4" + ("" if _FFMPEG_AVAILABLE else " ⚠"))

        # Close button
        btn_close = QPushButton("Close")
        btn_close.clicked.connect(self.accept)
        root.addWidget(btn_close, alignment=Qt.AlignmentFlag.AlignRight)
