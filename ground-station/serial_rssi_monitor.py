#!/usr/bin/env python3
"""
Real-time Si4463 RSSI monitor.

Parses PIC24 serial output from the Si4463 RX polling loop and displays
timestamped RSSI readings with change detection. Highlights sudden RSSI
jumps (signal appearing/disappearing) in color.

All non-RSSI serial lines are passed through so you don't lose visibility
into firmware messages (CTS timeouts, packet events, etc.).

Usage:
    python serial_rssi_monitor.py
    python serial_rssi_monitor.py --threshold 10 --log rssi_data.csv

Requirements:
    pip install pyserial
"""

import argparse
import csv
import re
import sys
import time
from collections import deque
from datetime import datetime

import serial

DEFAULT_PORT = "/dev/tty.usbserial-BG03U9VQ"
DEFAULT_BAUD = 9600

# Matches firmware output: "task_si4463: \tRX: RSSI=85(~-91dBm) st=0x00 afc=0"
RSSI_PATTERN = re.compile(
    r"RSSI=(\d+)\(~(-?\d+)dBm\)\s+st=0x([0-9A-Fa-f]+)\s+afc=(-?\d+)"
)

# Matches "RSSI JUMP: 85->120"
JUMP_PATTERN = re.compile(r"RSSI JUMP:\s*(\d+)->(\d+)")

# Matches RF switch test: "Peak RSSI=85 (~-91 dBm)"
PEAK_PATTERN = re.compile(r"Peak RSSI=(\d+)")

# ANSI colors
RED = "\033[91m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
CYAN = "\033[96m"
BOLD = "\033[1m"
RESET = "\033[0m"


def main():
    parser = argparse.ArgumentParser(description="Si4463 RSSI monitor")
    parser.add_argument("--port", default=DEFAULT_PORT, help="Serial port")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="Baud rate")
    parser.add_argument("--threshold", type=int, default=5,
                        help="RSSI change threshold for highlighting (default: 5)")
    parser.add_argument("--log", metavar="FILE",
                        help="Log RSSI data to CSV file")
    args = parser.parse_args()

    csv_file = None
    csv_writer = None
    if args.log:
        csv_file = open(args.log, "w", newline="")
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(["timestamp", "rssi_raw", "rssi_dbm", "modem_status", "afc_offset"])
        print(f"Logging to {args.log}")

    history = deque(maxlen=10)
    stats_rssi = []
    stats_time = time.time()
    STATS_INTERVAL = 30.0

    print(f"Connecting to {args.port} at {args.baud} baud...")
    print(f"Change threshold: {args.threshold} RSSI units")
    print(f"Press Ctrl+C to stop.\n")

    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}", file=sys.stderr)
        sys.exit(1)

    try:
        while True:
            raw = ser.readline()
            if not raw:
                continue

            try:
                line = raw.decode("ascii", errors="replace").rstrip()
            except Exception:
                continue

            if not line:
                continue

            now = datetime.now()
            ts = now.strftime("%H:%M:%S.") + f"{now.microsecond // 1000:03d}"

            # Check for RSSI status line
            m = RSSI_PATTERN.search(line)
            if m:
                rssi_raw = int(m.group(1))
                rssi_dbm = int(m.group(2))
                status = m.group(3)
                afc = int(m.group(4))

                history.append(rssi_raw)
                stats_rssi.append(rssi_raw)

                # Compute baseline
                if len(history) >= 3:
                    baseline = sum(history) / len(history)
                    delta = abs(rssi_raw - baseline)
                else:
                    baseline = rssi_raw
                    delta = 0

                # Format with color
                if delta > args.threshold:
                    if rssi_raw > baseline:
                        color = GREEN  # Signal appearing
                        marker = " ^^^"
                    else:
                        color = RED  # Signal disappearing
                        marker = " vvv"
                else:
                    color = ""
                    marker = ""

                prefix = f"[{ts}]"
                rssi_str = f"RSSI={rssi_raw:3d} ({rssi_dbm:4d} dBm)"
                detail = f"st=0x{status} afc={afc}"

                if color:
                    print(f"{prefix} {color}{BOLD}{rssi_str}{RESET} {detail}{color}{marker}{RESET}")
                else:
                    print(f"{prefix} {rssi_str} {detail}")

                if csv_writer:
                    csv_writer.writerow([now.isoformat(), rssi_raw, rssi_dbm, status, afc])
                    csv_file.flush()

                # Periodic stats
                elapsed = time.time() - stats_time
                if elapsed >= STATS_INTERVAL and stats_rssi:
                    mean = sum(stats_rssi) / len(stats_rssi)
                    mn = min(stats_rssi)
                    mx = max(stats_rssi)
                    n = len(stats_rssi)
                    if n > 1:
                        var = sum((x - mean) ** 2 for x in stats_rssi) / (n - 1)
                        std = var ** 0.5
                    else:
                        std = 0
                    print(f"{CYAN}--- {n} samples in {elapsed:.0f}s: "
                          f"min={mn} max={mx} mean={mean:.1f} std={std:.1f} ---{RESET}")
                    stats_rssi.clear()
                    stats_time = time.time()

                continue

            # Check for RSSI JUMP line
            m = JUMP_PATTERN.search(line)
            if m:
                print(f"[{ts}] {YELLOW}{BOLD}{line}{RESET}")
                continue

            # Check for Peak RSSI line
            m = PEAK_PATTERN.search(line)
            if m:
                print(f"[{ts}] {CYAN}{line}{RESET}")
                continue

            # Pass through all other lines
            print(f"[{ts}] {line}")

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        ser.close()
        if csv_file:
            csv_file.close()


if __name__ == "__main__":
    main()
