import serial
import serial.tools.list_ports
from PyQt6.QtCore import QThread, pyqtSignal
from backend.parser import parse_sensor_line
from backend.logger import get_logger

_log = get_logger(__name__)


class SerialWorker(QThread):
    """Hardware USB-serial producer.

    Reads UTF-8 lines from a serial port, parses them as JSON sensor samples,
    and emits signals for the main window pipeline.
    """

    # ── Unified producer signal interface (P2-T1) ───────────────────────────
    sample_received = pyqtSignal(dict)
    producer_error  = pyqtSignal(str)
    producer_status = pyqtSignal(str)

    def __init__(self, port: str, baud: int = 115200, parent=None):
        super().__init__(parent)
        self._port = port
        self._baud = baud
        self._stop_flag = False

    def stop(self) -> None:
        """Signal the run loop to exit."""
        self._stop_flag = True

    def run(self) -> None:
        """Entry point — opens port and enters read loop."""
        _log.info("SerialWorker starting on %s at %d baud", self._port, self._baud)
        
        try:
            # timeout=1.0 allows the loop to check self._stop_flag periodically
            ser = serial.Serial(self._port, self._baud, timeout=1.0)
            self.producer_status.emit(f"Connected to {self._port}")
        except Exception as e:
            _log.error("Failed to open serial port %s: %s", self._port, e)
            self.producer_error.emit(f"Port Error: {e}")
            return

        try:
            # Clear input buffer to avoid stale data
            ser.reset_input_buffer()
            
            while not self._stop_flag:
                if ser.in_waiting > 0:
                    try:
                        line = ser.readline().decode('utf-8', errors='replace').strip()
                        if not line:
                            continue
                            
                        sample = parse_sensor_line(line)
                        if sample:
                            self.sample_received.emit(sample)
                    except Exception as e:
                        _log.warning("Serial read/parse error: %s", e)
                        # We don't emit producer_error for every bad line to avoid UI spam
                else:
                    # Small sleep if no data to prevent high CPU usage in the thread
                    self.msleep(5)
        except Exception as e:
            _log.error("Serial worker loop crash: %s", e)
            self.producer_error.emit(f"Serial Error: {e}")
        finally:
            ser.close()
            _log.info("Serial port closed")
            self.producer_status.emit("Disconnected")
