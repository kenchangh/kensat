#!/usr/bin/env python3
"""
Pluto SDR CW carrier generator for Si4463 signal path testing.

Transmits an unmodulated carrier (or a single tone at a configurable
offset) at 437.08 MHz via ADALM-Pluto. Use this to verify the RF path
between Pluto TX and Si4463 RX by watching RSSI on the PIC24 serial
console.

Usage:
    python pluto_cw.py                       # CW at 437.08 MHz
    python pluto_cw.py --freq-offset 5000    # Tone at +5 kHz offset
    python pluto_cw.py --tx-gain -5          # Higher power

Requirements:
    pip install pyadi-iio numpy
"""

import argparse
import signal
import sys

import adi
import numpy as np

PLUTO_URI = "ip:192.168.3.1"
CENTER_FREQ = 437_080_000
PLUTO_XO_CORRECTION = 26_000
SAMPLE_RATE = 960_000
NUM_SAMPLES = 2**14  # 16384 samples in cyclic buffer


def main():
    parser = argparse.ArgumentParser(description="Pluto CW carrier for RF path testing")
    parser.add_argument("--tx-gain", type=int, default=-10,
                        help="TX attenuation in dB (default: -10)")
    parser.add_argument("--freq-offset", type=float, default=0,
                        help="Tone offset from center in Hz (default: 0 = pure carrier)")
    parser.add_argument("--xo-correction", type=int, default=PLUTO_XO_CORRECTION,
                        help=f"Pluto XO frequency correction in Hz (default: {PLUTO_XO_CORRECTION})")
    parser.add_argument("--uri", default=PLUTO_URI, help="Pluto URI")
    args = parser.parse_args()

    tx_lo = CENTER_FREQ + args.xo_correction
    print(f"Pluto CW Carrier Generator")
    print(f"  Center freq:   {CENTER_FREQ / 1e6:.3f} MHz")
    print(f"  XO correction: {args.xo_correction} Hz")
    print(f"  TX LO:         {tx_lo} Hz ({tx_lo / 1e6:.6f} MHz)")
    print(f"  Freq offset:   {args.freq_offset} Hz")
    print(f"  TX gain:       {args.tx_gain} dB")
    print(f"  Sample rate:   {SAMPLE_RATE} Hz")
    print()

    # Connect to Pluto
    print(f"Connecting to Pluto at {args.uri}...")
    sdr = adi.Pluto(args.uri)

    sdr.sample_rate = SAMPLE_RATE
    sdr.tx_rf_bandwidth = SAMPLE_RATE
    sdr.tx_lo = tx_lo
    sdr.tx_hardwaregain_chan0 = args.tx_gain
    sdr.tx_cyclic_buffer = True

    # Generate IQ samples
    if args.freq_offset == 0:
        # Pure carrier at LO: constant I=1, Q=0
        iq = np.ones(NUM_SAMPLES, dtype=np.complex64) * (2**14)
    else:
        # Single tone at LO + offset
        t = np.arange(NUM_SAMPLES) / SAMPLE_RATE
        iq = np.exp(2j * np.pi * args.freq_offset * t).astype(np.complex64) * (2**14)

    # Start transmitting
    print("Starting CW transmission (Ctrl+C to stop)...")
    sdr.tx(iq)
    print("TX active. Watch Si4463 RSSI on serial console.")
    print()

    # Handle cleanup
    def cleanup(*_):
        print("\nStopping TX...")
        sdr.tx_destroy_buffer()
        print("Done.")
        sys.exit(0)

    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)

    # Block until interrupted
    try:
        signal.pause()
    except AttributeError:
        # Windows fallback
        import time
        while True:
            time.sleep(1)


if __name__ == "__main__":
    main()
