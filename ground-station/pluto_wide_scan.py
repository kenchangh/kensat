#!/usr/bin/env python3
"""
Pluto SDR stepped-LO wideband scan.

Steps the Pluto RX LO across a configurable range (default 100-1500 MHz)
and stitches the per-step averaged spectra into one wide RF spectrum.
Used to look for Si4463 PA harmonics, broadband emissions, or conducted
noise that a narrow-span waterfall around 437.08 MHz cannot reveal.

Usage:
    # Baseline (Si4463 cable disconnected from Pluto RX)
    python scripts/pluto_wide_scan.py --baseline baseline.npz

    # Live (Si4463 keyed CW, cable connected through 30 dB pad)
    python scripts/pluto_wide_scan.py --subtract baseline.npz

    # Narrower range, finer step
    python scripts/pluto_wide_scan.py --start 400e6 --stop 900e6
"""

import argparse
import sys
import time

import adi
import numpy as np
import matplotlib.pyplot as plt

PLUTO_URI = "ip:192.168.3.1"
PLUTO_XO_CORRECTION = 26_000

DEFAULT_START = 100e6
DEFAULT_STOP = 1500e6
DEFAULT_SAMPLE_RATE = 20e6
DEFAULT_RX_GAIN = 30
DEFAULT_AVG = 4               # buffers averaged per step
DEFAULT_FFT_SIZE = 4096
EDGE_KEEP = 0.8               # keep middle 80% of each slice
FLUSH_BUFFERS = 2
BUF_SIZE = 2**15


def setup_pluto(uri, sample_rate, rx_gain):
    print(f"Connecting to Pluto SDR at {uri}...")
    sdr = adi.Pluto(uri=uri)
    sdr.sample_rate = int(sample_rate)
    sdr.rx_rf_bandwidth = int(sample_rate)
    sdr.gain_control_mode_chan0 = "manual"
    sdr.rx_hardwaregain_chan0 = float(rx_gain)
    sdr.rx_buffer_size = BUF_SIZE
    print(f"  Sample rate : {sample_rate/1e6:.2f} MHz")
    print(f"  RX gain     : {rx_gain} dB")
    print(f"  Buffer      : {BUF_SIZE} samples ({BUF_SIZE/sample_rate*1000:.2f} ms)")
    return sdr


def averaged_spectrum(sdr, fft_size, n_avg, window):
    """Capture n_avg buffers, return averaged linear power spectrum (fftshifted)."""
    accum = np.zeros(fft_size, dtype=np.float64)
    for _ in range(n_avg):
        iq = sdr.rx().astype(np.complex128)
        n_seg = len(iq) // fft_size
        if n_seg == 0:
            continue
        seg = iq[: n_seg * fft_size].reshape(n_seg, fft_size) * window[np.newaxis, :]
        ffts = np.fft.fftshift(np.fft.fft(seg, axis=1), axes=1)
        accum += np.mean(np.abs(ffts) ** 2, axis=0)
    return accum / max(1, n_avg)


def null_dc(power, fft_size):
    """Interpolate the few bins around DC to suppress LO leakage."""
    c = fft_size // 2
    w = max(1, fft_size // 256)
    lo, hi = max(0, c - w), min(fft_size, c + w + 1)
    if lo > 0 and hi < fft_size:
        power[lo:hi] = np.linspace(power[lo - 1], power[hi], hi - lo)
    return power


def scan(sdr, start_hz, stop_hz, sample_rate, fft_size, n_avg):
    keep_bw = sample_rate * EDGE_KEEP
    step = keep_bw                                # non-overlapping kept slices
    n_steps = int(np.ceil((stop_hz - start_hz) / step))
    window = np.hanning(fft_size)
    bin_hz = sample_rate / fft_size
    keep_bins = int((keep_bw / sample_rate) * fft_size)
    if keep_bins % 2:
        keep_bins -= 1
    half_keep = keep_bins // 2
    center_bin = fft_size // 2

    freqs_all = []
    power_all = []

    print(f"\nScanning {start_hz/1e6:.1f} MHz to {stop_hz/1e6:.1f} MHz "
          f"in {n_steps} steps of {step/1e6:.1f} MHz...")

    for i in range(n_steps):
        lo = start_hz + i * step + sample_rate / 2
        if lo > stop_hz:
            break
        lo_corrected = lo + PLUTO_XO_CORRECTION
        try:
            sdr.rx_lo = int(lo_corrected)
        except OSError:
            sys.stdout.write(f"  step {i+1}/{n_steps}  LO={lo/1e6:.2f} MHz (out of range, skip)\n")
            continue
        for _ in range(FLUSH_BUFFERS):
            sdr.rx()
        spec = averaged_spectrum(sdr, fft_size, n_avg, window)
        spec = null_dc(spec, fft_size)
        slice_power = spec[center_bin - half_keep : center_bin + half_keep]
        slice_freqs = lo + (np.arange(-half_keep, half_keep) * bin_hz)
        freqs_all.append(slice_freqs)
        power_all.append(slice_power)
        sys.stdout.write(f"  step {i+1}/{n_steps}  LO={lo/1e6:.2f} MHz\r")
        sys.stdout.flush()
    print()

    freqs = np.concatenate(freqs_all)
    power = np.concatenate(power_all)
    order = np.argsort(freqs)
    return freqs[order], power[order]


def main():
    ap = argparse.ArgumentParser(description="Pluto SDR stepped-LO wideband scan")
    ap.add_argument("--uri", default=PLUTO_URI)
    ap.add_argument("--start", type=float, default=DEFAULT_START, help="Start Hz (default 100e6)")
    ap.add_argument("--stop", type=float, default=DEFAULT_STOP, help="Stop Hz (default 1500e6)")
    ap.add_argument("--sample-rate", type=float, default=DEFAULT_SAMPLE_RATE,
                    help="Pluto sample rate Hz (default 20e6)")
    ap.add_argument("--rx-gain", type=float, default=DEFAULT_RX_GAIN)
    ap.add_argument("--avg", type=int, default=DEFAULT_AVG, help="Buffers per step (default 4)")
    ap.add_argument("--fft-size", type=int, default=DEFAULT_FFT_SIZE)
    ap.add_argument("--baseline", type=str, default=None,
                    help="Save this run as a baseline .npz (no plot subtraction)")
    ap.add_argument("--subtract", type=str, default=None,
                    help="Subtract a previously saved baseline before plotting")
    ap.add_argument("--save-plot", type=str, default=None, help="Save plot PNG path")
    ap.add_argument("--no-show", action="store_true", help="Don't open the matplotlib window")
    args = ap.parse_args()

    sdr = setup_pluto(args.uri, args.sample_rate, args.rx_gain)

    t0 = time.time()
    freqs, power = scan(sdr, args.start, args.stop, args.sample_rate,
                        args.fft_size, args.avg)
    print(f"Scan took {time.time()-t0:.1f} s, {len(freqs)} bins")

    power_db = 10 * np.log10(power + 1e-30)

    if args.baseline:
        np.savez(args.baseline, freqs=freqs, power_db=power_db)
        print(f"Saved baseline to {args.baseline}")

    sub_db = None
    if args.subtract:
        b = np.load(args.subtract)
        if not np.allclose(b["freqs"], freqs):
            print("WARNING: baseline frequency axis differs; interpolating.")
            sub_db = np.interp(freqs, b["freqs"], b["power_db"])
        else:
            sub_db = b["power_db"]

    fig, ax = plt.subplots(figsize=(14, 5))
    ax.plot(freqs / 1e6, power_db, lw=0.5, label="scan")
    if sub_db is not None:
        diff = power_db - sub_db
        ax.plot(freqs / 1e6, diff, lw=0.5, color="red", alpha=0.7,
                label="scan − baseline")
    ax.set_xlabel("Frequency (MHz)")
    ax.set_ylabel("Power (dB)")
    ax.set_title(
        f"Pluto wideband scan {args.start/1e6:.0f}–{args.stop/1e6:.0f} MHz "
        f"({args.sample_rate/1e6:.0f} MHz/step, gain={args.rx_gain} dB)"
    )
    ax.grid(True, alpha=0.3)
    # Annotate Si4463 carrier and its first few harmonics
    for n in (1, 2, 3):
        f = 437.08 * n
        if args.start / 1e6 <= f <= args.stop / 1e6:
            ax.axvline(f, color="green", ls="--", lw=0.6, alpha=0.5)
            ax.text(f, ax.get_ylim()[1], f" {n}×437.08", color="green",
                    fontsize=7, va="top")
    ax.legend(loc="upper right", fontsize=8)
    fig.tight_layout()

    if args.save_plot:
        fig.savefig(args.save_plot, dpi=120)
        print(f"Saved plot to {args.save_plot}")

    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
