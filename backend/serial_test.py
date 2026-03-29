#!/usr/bin/env python3
"""
Simple test script to verify serial connection.
Reads lines from the serial port and prints them.
"""

import sys
import time
from pathlib import Path

# Add project root to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

import serial
from serial.tools import list_ports
from backend.config import DEFAULT_BAUD_RATE


def list_available_ports():
    """Return a list of available serial port names."""
    return [p.device for p in list_ports.comports()]


def main():
    """Open the first available port and print incoming lines."""
    ports = list_available_ports()
    if not ports:
        print("No serial ports found.")
        return

    port = ports[0]
    print(f"Attempting to connect to {port} at {DEFAULT_BAUD_RATE} baud...")

    try:
        ser = serial.Serial(port, DEFAULT_BAUD_RATE, timeout=1)
        print(f"Connected. Reading data (Ctrl+C to stop):\n")
        while True:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                print(line)
    except serial.SerialException as e:
        print(f"Serial error: {e}")
    except KeyboardInterrupt:
        print("\nExiting.")


if __name__ == "__main__":
    main()
