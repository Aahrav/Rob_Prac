import serial
from PyQt6.QtCore import QThread, pyqtSignal
from backend.parser import parse_sensor_line
from backend.logger import get_logger

_log = get_logger(__name__)


class SerialWorker(QThread):
    """Hardware USB-serial producer.
    
    Reads UTF-8 lines from a serial port, parses them as JSON sensor samples,
    and emits signals for the unified pipeline.
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
        """Entry point — opens serial port and enters read loop."""
        ser = None
        try:
            _log.info("Opening serial port %s at %d baud", self._port, self._baud)
            ser = serial.Serial(self._port, self._baud, timeout=0.1)
            self.producer_status.emit(f"Connected to {self._port}")
            
            while not self._stop_flag:
                if ser.in_waiting > 0:
                    try:
                        line = ser.readline().decode('utf-8', errors='ignore').strip()
                        if not line:
                            continue
                        
                        sample = parse_sensor_line(line)
                        if sample:
                            self.sample_received.emit(sample)
                    except Exception as e:
                        _log.warning("Serial read error: %s", e)
                else:
                    # Small sleep to prevent 100% CPU usage
                    self.msleep(5)
                    
        except serial.SerialException as e:
            _log.error("Failed to open serial port %s: %s", self._port, e)
            self.producer_error.emit(f"Serial Error: {e}")
        except Exception as e:
            _log.exception("Unexpected error in SerialWorker")
            self.producer_error.emit(f"Unexpected Error: {e}")
        finally:
            if ser and ser.is_open:
                ser.close()
                _log.info("Closed serial port %s", self._port)
            self.producer_status.emit("Disconnected")
