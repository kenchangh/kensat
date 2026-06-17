#!/usr/bin/env python3
"""
Pluto SDR frequency sweep with Si4463 RSSI logging.

Sweeps the Pluto TX frequency in configurable steps around 437.08 MHz
while reading Si4463 RSSI from the PIC24 serial port. Finds the peak
RSSI frequency, which reveals the actual frequency offset between
Pluto and Si4463 crystals.

Usage:
    python pluto_freq_sweep.py
    python pluto_freq_sweep.py --sweep-range 100000 --step-size 5000  # coarse
    python pluto_freq_sweep.py --sweep-range 20000 --step-size 500    # fine

Requirements:
    pip install pyadi-iio numpy pyserial
"""

import argparse
import re
import sys
import threading
import time

import adi
import numpy as np
import serial

PLUTO_URI = "ip:192.168.3.1"
CENTER_FREQ = 437_080_000
PLUTO_XO_CORRECTION = 26_000
SAMPLE_RATE = 960_000
NUM_SAMPLES = 2**14

DEFAULT_PORT = "/dev/tty.usbserial-BG03U9VQ"
DEFAULT_BAUD = 9600

RSSI_PATTERN = re.compile(r"RSSI=(\d+)")


class RSSICollector:
    """Background thread that reads serial and collects RSSI values."""

    def __init__(self, port, baud):
        self.ser = serial.Serial(port, baud, timeout=0.5)
        self.lock = threading.Lock()
        self.values = []
        self.running = True
        self.thread = threading.Thread(target=self._reader, daemon=True)
        self.thread.start()

    def _reader(self):
        while self.running:
            try:
                raw = self.ser.readline()
                if not raw:
                    continue
                line = raw.decode("ascii", errors="replace").rstrip()
                m = RSSI_PATTERN.search(line)
                if m:
                    with self.lock:
                        self.values.append(int(m.group(1)))
            except Exception:
                continue

    def collect(self):
        """Return and clear collected RSSI values."""
        with self.lock:
            vals = list(self.values)
            self.values.clear()
        return vals

    def stop(self):
        self.running = False
        self.thread.join(timeout=2)
        self.ser.close()


def main():
    parser = argparse.ArgumentParser(description="Pluto frequency sweep + Si4463 RSSI")
    parser.add_argument("--center", type=int, default=CENTER_FREQ,
                        help=f"Center frequency in Hz (default: {CENTER_FREQ})")
    parser.add_argument("--sweep-range", type=int, default=50_000,
                        help="Sweep +/- range in Hz (default: 50000)")
    parser.add_argument("--step-size", type=int, default=1_000,
                        help="Step size in Hz (default: 1000)")
    parser.add_argument("--dwell-time", type=float, default=3.0,
                        help="Dwell time per step in seconds (default: 3.0)")
    parser.add_argument("--tx-gain", type=int, default=-10,
                        help="TX gain in dB (default: -10)")
    parser.add_argument("--xo-correction", type=int, default=PLUTO_XO_CORRECTION,
                        help=f"Pluto XO correction in Hz (default: {PLUTO_XO_CORRECTION})")
    parser.add_argument("--port", default=DEFAULT_PORT, help="Serial port")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="Baud rate")
    parser.add_argument("--uri", default=PLUTO_URI, help="Pluto URI")
    args = parser.parse_args()

    freqs = list(range(
        args.center - args.sweep_range,
        args.center + args.sweep_range + 1,
        args.step_size
    ))
    num_steps = len(freqs)
    est_time = num_steps * args.dwell_time

    print(f"Frequency Sweep: Si4463 RSSI vs Pluto TX Frequency")
    print(f"  Range:      {freqs[0]/1e6:.3f} - {freqs[-1]/1e6:.3f} MHz")
    print(f"  Steps:      {num_steps} x {args.step_size} Hz")
    print(f"  Dwell:      {args.dwell_time}s per step")
    print(f"  Est. time:  {est_time:.0f}s ({est_time/60:.1f} min)")
    print(f"  XO corr:    {args.xo_correction} Hz")
    print(f"  TX gain:    {args.tx_gain} dB")
    print()

    # Connect to serial
    print(f"Opening serial port {args.port}...")
    collector = RSSICollector(args.port, args.baud)

    # Connect to Pluto
    print(f"Connecting to Pluto at {args.uri}...")
    sdr = adi.Pluto(args.uri)
    sdr.sample_rate = SAMPLE_RATE
    sdr.tx_rf_bandwidth = SAMPLE_RATE
    sdr.tx_hardwaregain_chan0 = args.tx_gain
    sdr.tx_cyclic_buffer = True

    # CW IQ data
    iq = np.ones(NUM_SAMPLES, dtype=np.complex64) * (2**14)

    results = []
    peak_rssi = 0
    peak_freq = freqs[0]

    print(f"\n{'Freq (MHz)':>14}  {'Offset':>8}  {'RSSI_max':>8}  {'RSSI_mean':>9}  {'dBm_max':>7}  {'N':>3}")
    print("-" * 60)

    try:
        for i, freq in enumerate(freqs):
            tx_lo = freq + args.xo_correction
            offset = freq - args.center

            # Update Pluto frequency
            sdr.tx_destroy_buffer()
            sdr.tx_lo = tx_lo
            sdr.tx(iq)

            # Clear any stale RSSI readings
            collector.collect()

            # Dwell and collect
            time.sleep(args.dwell_time)
            vals = collector.collect()

            if vals:
                rssi_max = max(vals)
                rssi_mean = sum(vals) / len(vals)
                dbm_max = rssi_max // 2 - 134
            else:
                rssi_max = 0
                rssi_mean = 0
                dbm_max = -134

            results.append((freq, offset, rssi_max, rssi_mean, len(vals)))

            marker = ""
            if rssi_max > peak_rssi:
                peak_rssi = rssi_max
                peak_freq = freq
                marker = " <-- PEAK"

            print(f"  {freq/1e6:12.3f}  {offset:+7d}  {rssi_max:8d}  {rssi_mean:9.1f}  {dbm_max:7d}  {len(vals):3d}{marker}")

    except KeyboardInterrupt:
        print("\n\nSweep interrupted.")
    finally:
        print("\nStopping TX...")
        sdr.tx_destroy_buffer()
        collector.stop()

    # Summary
    if results:
        noise_floor = min(r[2] for r in results)
        print(f"\n{'='*60}")
        print(f"RESULTS:")
        print(f"  Peak RSSI:     {peak_rssi} (~{peak_rssi//2 - 134} dBm)")
        print(f"  Peak freq:     {peak_freq/1e6:.3f} MHz (offset {peak_freq - args.center:+d} Hz)")
        print(f"  Noise floor:   {noise_floor} (~{noise_floor//2 - 134} dBm)")
        print(f"  SNR:           {peak_rssi - noise_floor} RSSI units (~{(peak_rssi - noise_floor)//2} dB)")

        if peak_rssi > noise_floor + 10:
            new_xo = args.xo_correction + (peak_freq - args.center)
            print(f"\n  Signal detected! Suggested PLUTO_XO_CORRECTION = {new_xo}")
        else:
            print(f"\n  No signal detected above noise floor. Check RF path.")


if __name__ == "__main__":
    main()
