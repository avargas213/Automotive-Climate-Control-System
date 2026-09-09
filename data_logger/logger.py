"""
Minimal serial logger for Predictive Automotive Climate Control System V0.1.

Unlike the original V6.0 experiment logger, this writes one simple session CSV.
It listens only for firmware lines beginning with STATE,.
"""

import csv
import os
import time
from datetime import datetime

import serial

PORT = "COM6"
BAUD = 115200
DATA_FOLDER = "Saved Data"

os.makedirs(DATA_FOLDER, exist_ok=True)

filename = os.path.join(
    DATA_FOLDER,
    datetime.now().strftime("climate_session_%Y%m%d_%H%M%S.csv"),
)

headers = [
    "ESP32 Time ms",
    "Cabin Temperature F",
    "Target Temperature F",
    "Predicted Remaining Seconds",
    "Controller Mode",
    "HVAC Setpoint F",
    "Fan Level",
    "AC On",
    "Recirculation On",
    "Auto Enabled",
]


def connect():
    while True:
        try:
            print(f"Connecting to {PORT}...")
            ser = serial.Serial(PORT, BAUD, timeout=1)
            time.sleep(2)
            ser.reset_input_buffer()
            print("ESP32 connected.")
            return ser
        except serial.SerialException:
            time.sleep(1)


def main():
    ser = connect()

    with open(filename, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(headers)
        f.flush()

        print(f"Logging to: {filename}")

        while True:
            try:
                line = ser.readline().decode("utf-8", errors="ignore").strip()
            except serial.SerialException:
                ser.close()
                ser = connect()
                continue

            if not line.startswith("STATE,"):
                continue

            values = line.split(",")[1:]

            if len(values) != len(headers):
                print("Skipped malformed state line:", line)
                continue

            writer.writerow(values)
            f.flush()
            print(line)


if __name__ == "__main__":
    main()
