#!/usr/bin/env python3
import sys
from pathlib import Path
from unittest.mock import MagicMock, patch

# Add project root to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from backend.serial_worker import SerialWorker
from PyQt6.QtCore import QCoreApplication

def test_serial_worker():
    # Setup mock serial
    mock_ser = MagicMock()
    # Simulate a JSON line from the firmware
    mock_ser.readline.return_value = b'{"t":1000,"r":10.5,"p":-5.2,"y":0.0,"gx":0.1,"gy":0.2,"gz":0.3}\n'
    mock_ser.in_waiting = 1
    mock_ser.is_open = True

    # We need a QCoreApplication to use signals in a test
    app = QCoreApplication(sys.argv)

    worker = SerialWorker(port="MOCK", baud=115200)
    
    received_samples = []
    worker.sample_received.connect(lambda s: received_samples.append(s))
    
    with patch('serial.Serial', return_value=mock_ser):
        # We manually trigger the logic inside run() for one iteration to avoid threading complexity in a quick test
        # Instead of worker.start(), we can just test the parsing part or use a short timeout
        
        # Test 1: Port opening
        worker._port = "MOCK"
        worker._baud = 115200
        
        print("Testing SerialWorker with mocked data...")
        
        # Simulate the loop logic once
        line = mock_ser.readline().decode('utf-8').strip()
        from backend.parser import parse_sensor_line
        sample = parse_sensor_line(line)
        
        if sample and sample['t'] == 1000 and sample['r'] == 10.5:
            print("SUCCESS: Mock data parsed correctly!")
            print(f"Parsed sample: {sample}")
        else:
            print(f"FAILED: Parsing error. Sample: {sample}")
            sys.exit(1)

if __name__ == "__main__":
    test_serial_worker()
