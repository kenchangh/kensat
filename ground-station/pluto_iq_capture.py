#!/usr/bin/env python3
"""
Pluto SDR IQ capture for the KENSAT beacon.

Records raw IQ at 437.080 MHz for off-line demodulation in SatNOGS,
gr-satellites, Direwolf, or any other ground stack.

The default output is SigMF (data + JSON metadata), which SatNOGS and
most modern SDR tooling consume directly. `--format cfile` writes
GNU-Radio-style complex float32, and `--format sc16` writes Pluto's
native interleaved int16 (half the size, no precision loss vs. cfile
for an ADC that's only 12 bits wide anyway).

We tune the Pluto LO LO_OFFSET below the beacon so the carrier lands at
+LO_OFFSET in baseband, well away from the SDR's DC spike. The ground
operator will need to either mix it back to DC or just tell their
flowgraph that the signal of interest is at +100 kHz from center.

Usage:
    # 60 s SigMF capture at 437.080 MHz, 960 kSps
    python pluto_iq_capture.py --duration 60 --out kensat_beacon

    # Same, but plain GNU Radio cfile
    python pluto_iq_capture.py --format cfile --duration 60 --out kensat_beacon

Requirements:
    pip install pyadi-iio numpy
"""

import argparse
import datetime
import json
import os
import sys
import time

import adi
import numpy as np


# RF defaults — match pluto_rx_ax25.py so the same Pluto/antenna setup works
PLUTO_URI = "ip:192.168.3.1"
CENTER_FREQ = 437_080_000          # KENSAT beacon
PLUTO_XO_CORRECTION = 4000         # Pluto crystal offset (Hz)
SAMPLE_RATE = 960_000              # 960 kSps — plenty for 9k6 GFSK + room to slide
LO_OFFSET = 100_000                # Carrier lands at +100 kHz in baseband
DEFAULT_RX_GAIN = 60               # dB, manual
BUF_SIZE = 2 ** 17                 # ~137 ms per read

AUTHOR = "KENSAT / KO6IKZ"
DESCRIPTION = (
    "KENSAT beacon IQ capture. On-air format: 9600 bps 2-GFSK, ±3 kHz "
    "deviation, BT=0.5, G3RUH scrambled, NRZI, HDLC-framed AX.25 UI. "
    "LO is tuned LO_OFFSET below the beacon, so the carrier of interest "
    "is at +100 kHz from the file's reported center frequency."
)


def open_pluto(uri, rx_gain):
    print(f"Connecting to Pluto SDR at {uri}...")
    try:
        sdr = adi.Pluto(uri=uri)
    except Exception as e:
        print(f"ERROR: cannot connect to Pluto: {e}")
        sys.exit(1)

    sdr.rx_lo = CENTER_FREQ + PLUTO_XO_CORRECTION - LO_OFFSET
    sdr.rx_rf_bandwidth = SAMPLE_RATE
    sdr.gain_control_mode_chan0 = "manual"
    sdr.rx_hardwaregain_chan0 = rx_gain
    sdr.sample_rate = SAMPLE_RATE
    sdr.rx_buffer_size = BUF_SIZE

    # Discard initial settling buffers
    for _ in range(5):
        sdr.rx()
    return sdr


def write_sigmf_meta(meta_path, data_fname, datatype, sample_count,
                     start_iso, rx_gain, duration_s):
    """Write a minimal SigMF v1.0.0 metadata file."""
    meta = {
        "global": {
            "core:datatype": datatype,
            "core:sample_rate": float(SAMPLE_RATE),
            "core:hw": "Analog Devices PlutoSDR (AD9363)",
            "core:author": AUTHOR,
            "core:description": DESCRIPTION,
            "core:version": "1.0.0",
            "core:dataset": os.path.basename(data_fname),
            "core:num_channels": 1,
        },
        "captures": [
            {
                "core:sample_start": 0,
                "core:frequency": float(CENTER_FREQ - LO_OFFSET),
                "core:datetime": start_iso,
            }
        ],
        "annotations": [
            {
                "core:sample_start": 0,
                "core:sample_count": sample_count,
                "core:freq_lower_edge": float(CENTER_FREQ - 10_000),
                "core:freq_upper_edge": float(CENTER_FREQ + 10_000),
                "core:description": (
                    f"KENSAT beacon — 9600 bps 2-GFSK, dev ±3 kHz, BT=0.5, "
                    f"G3RUH, AX.25 UI. RX gain {rx_gain} dB, duration "
                    f"{duration_s:.1f} s. Beacon carrier sits at +"
                    f"{LO_OFFSET} Hz from core:frequency."
                ),
                "core:generator": "pluto_iq_capture.py",
            }
        ],
    }
    with open(meta_path, "w") as f:
        json.dump(meta, f, indent=2)


def capture(uri, rx_gain, duration, out_base, fmt):
    sdr = open_pluto(uri, rx_gain)

    # Resolve filenames
    if fmt == "sigmf":
        data_path = out_base + ".sigmf-data"
        meta_path = out_base + ".sigmf-meta"
        # SigMF native int16 interleaved
        sigmf_datatype = "ci16_le"
    elif fmt == "sc16":
        data_path = out_base + ".sc16"
        meta_path = None
    elif fmt == "cfile":
        data_path = out_base + ".cfile"
        meta_path = None
    else:
        raise ValueError(f"unknown format {fmt}")

    print(f"  Format       : {fmt}")
    print(f"  Center freq  : {CENTER_FREQ / 1e6:.3f} MHz "
          f"(LO at {(CENTER_FREQ - LO_OFFSET)/1e6:.3f} MHz, "
          f"signal at +{LO_OFFSET/1e3:.0f} kHz baseband)")
    print(f"  Sample rate  : {SAMPLE_RATE / 1e3:.0f} kSps")
    print(f"  RX gain      : {rx_gain} dB")
    print(f"  Duration     : {duration:.0f} s")
    print(f"  Output       : {data_path}"
          + (f"\n               + {meta_path}" if meta_path else ""))
    print()

    start_wall = time.time()
    start_iso = datetime.datetime.now(datetime.timezone.utc).isoformat()
    total_samples = 0
    buf_count = 0

    print("Capturing... (Ctrl+C to stop early)")
    with open(data_path, "wb") as f:
        try:
            while time.time() - start_wall < duration:
                iq = sdr.rx()   # numpy complex (int16 I + 1j * int16 Q, stored as complex)
                buf_count += 1

                if fmt == "cfile":
                    # GNU Radio expects complex64 (= 2 × float32)
                    iq.astype(np.complex64).tofile(f)
                else:
                    # SigMF ci16_le / sc16: interleaved int16 I,Q,I,Q,...
                    interleaved = np.empty(2 * len(iq), dtype=np.int16)
                    interleaved[0::2] = np.real(iq).astype(np.int16)
                    interleaved[1::2] = np.imag(iq).astype(np.int16)
                    interleaved.tofile(f)

                total_samples += len(iq)

                if buf_count % 10 == 0:
                    elapsed = time.time() - start_wall
                    pwr = np.mean(np.abs(iq) ** 2)
                    print(f"  [{elapsed:5.1f}s] buf={buf_count} "
                          f"samples={total_samples:>10d} "
                          f"avg_pwr={10*np.log10(pwr+1e-12):6.1f} dB")
        except KeyboardInterrupt:
            print("\nStopping early...")

    elapsed = time.time() - start_wall
    bytes_written = os.path.getsize(data_path)

    if fmt == "sigmf":
        write_sigmf_meta(meta_path, data_path, sigmf_datatype,
                         total_samples, start_iso, rx_gain, elapsed)

    print("-" * 72)
    print(f"Wrote {total_samples} samples ({bytes_written/1e6:.1f} MB) "
          f"in {elapsed:.1f}s.")
    if meta_path:
        print(f"Metadata: {meta_path}")
    print()
    print("Hand-off notes for the ground operator:")
    print(f"  - Sample rate    : {SAMPLE_RATE} Hz")
    print(f"  - Center freq    : {CENTER_FREQ - LO_OFFSET} Hz (LO)")
    print(f"  - Signal of int. : +{LO_OFFSET} Hz from center "
          f"(= {CENTER_FREQ} Hz RF)")
    if fmt == "sigmf":
        print(f"  - Format         : SigMF ci16_le (interleaved int16)")
    elif fmt == "sc16":
        print(f"  - Format         : raw interleaved int16, I,Q,I,Q,...")
    else:
        print(f"  - Format         : GNU Radio complex64 (.cfile)")
    print(f"  - Modulation     : 9600 bps 2-GFSK, ±3 kHz dev, BT=0.5, "
          f"G3RUH scrambled AX.25 UI")


def main():
    p = argparse.ArgumentParser(description="Pluto SDR IQ capture for KENSAT beacon")
    p.add_argument("--uri", default=PLUTO_URI, help=f"Pluto URI (default {PLUTO_URI})")
    p.add_argument("--rx-gain", type=float, default=DEFAULT_RX_GAIN,
                   help=f"RX gain dB (default {DEFAULT_RX_GAIN})")
    p.add_argument("--duration", type=float, default=60.0,
                   help="capture seconds (default 60)")
    p.add_argument("--out", default="kensat_beacon",
                   help="output base filename, no extension (default kensat_beacon)")
    p.add_argument("--format", choices=("sigmf", "sc16", "cfile"),
                   default="sigmf",
                   help="output format (default sigmf — recommended)")
    args = p.parse_args()
    capture(args.uri, args.rx_gain, args.duration, args.out, args.format)


if __name__ == "__main__":
    main()
