#!/usr/bin/env python3
"""
Extended TX test suite for the kensat_radio RF board using ADALM-Pluto SDR.

Subcommands:
    sweep           Sweep TX gain and check linearity / compression
    harmonics       Measure 2nd and 3rd harmonic levels (dBc)
    insertion-loss  Measure insertion loss through the RF board

Usage:
    python tx_test_suite.py sweep [--tx-start -20] [--tx-stop 0] [--tx-step 1]
    python tx_test_suite.py harmonics [--tx-gain -10]
    python tx_test_suite.py insertion-loss [--tx-gain -10]

Requirements:
    pip install pyadi-iio numpy matplotlib
"""

import argparse
import sys
import time

import adi
import matplotlib.pyplot as plt
import numpy as np

from tx_test_pluto import (
    PLUTO_URI,
    CENTER_FREQ,
    SAMPLE_RATE,
    RX_BUFFER_SIZE,
    create_tx_tone,
    configure_pluto,
    measure_power_dbm,
    compute_spectrum,
)


def connect_pluto():
    """Connect to Pluto SDR or exit on failure."""
    print(f"Connecting to Pluto SDR at {PLUTO_URI}...")
    try:
        return adi.Pluto(uri=PLUTO_URI)
    except Exception as e:
        print(f"ERROR: Cannot connect to Pluto SDR: {e}")
        sys.exit(1)


def flush_rx(sdr, count=5):
    """Discard a few RX buffers to let the signal settle."""
    for _ in range(count):
        sdr.rx()


def measure_mean_power(sdr, num_captures=20):
    """Collect multiple RX captures and return mean power in dBm."""
    powers = []
    for _ in range(num_captures):
        rx_data = sdr.rx() / (2**11)
        powers.append(measure_power_dbm(rx_data))
    return np.mean(powers), np.std(powers)


def measure_tone_power(sdr, sample_rate, tone_freq, num_captures=20, bin_width=5):
    """Measure power at a specific tone frequency using FFT peak detection.

    Instead of broadband RMS power, this finds the tone bin in the FFT and
    sums power in a narrow window around it. This distinguishes real signal
    from noise floor / LO leakage.

    Returns (tone_power_dB, noise_floor_dB, snr_dB) averaged over captures.
    """
    tone_powers = []
    noise_floors = []

    for _ in range(num_captures):
        rx_data = sdr.rx() / (2**11)
        n = len(rx_data)
        window = np.hanning(n)
        fft_data = np.fft.fftshift(np.fft.fft(rx_data * window))
        freqs = np.fft.fftshift(np.fft.fftfreq(n, 1 / sample_rate))
        mag = np.abs(fft_data) ** 2  # power spectrum

        # Find the bin closest to the expected tone
        tone_idx = np.argmin(np.abs(freqs - tone_freq))

        # Sum power in a narrow window around the tone
        lo = max(0, tone_idx - bin_width)
        hi = min(n, tone_idx + bin_width + 1)
        tone_pwr = np.sum(mag[lo:hi])

        # Noise floor: median of bins excluding the tone region
        exclusion_lo = max(0, tone_idx - bin_width * 5)
        exclusion_hi = min(n, tone_idx + bin_width * 5 + 1)
        noise_bins = np.concatenate([mag[:exclusion_lo], mag[exclusion_hi:]])
        noise_pwr = np.median(noise_bins) * (hi - lo)  # scale to same bandwidth

        tone_powers.append(10 * np.log10(tone_pwr + 1e-20))
        noise_floors.append(10 * np.log10(noise_pwr + 1e-20))

    tone_db = np.mean(tone_powers)
    noise_db = np.mean(noise_floors)
    snr_db = tone_db - noise_db
    return tone_db, noise_db, snr_db


# ---------- Test 1: TX Gain Sweep ----------

def run_sweep(args):
    """Sweep TX gain and measure RX power using tone-based FFT measurement.

    Uses SNR to identify noise-floor-limited points and excludes them from
    the linearity / compression analysis.
    """
    sdr = connect_pluto()

    gains = np.arange(args.tx_start, args.tx_stop + args.tx_step, args.tx_step)
    rx_gain = args.rx_gain
    tone_freq = 100e3
    snr_threshold = 10  # dB — points below this are noise-limited

    print(f"  Sweep range : {args.tx_start} to {args.tx_stop} dB (step {args.tx_step})")
    print(f"  RX gain     : {rx_gain} dB")
    print(f"  Frequency   : {CENTER_FREQ / 1e6:.2f} MHz")
    print(f"  SNR cutoff  : {snr_threshold} dB (points below excluded from analysis)")
    print()

    # Initial config at lowest gain
    configure_pluto(sdr, CENTER_FREQ, SAMPLE_RATE, float(gains[0]), rx_gain)
    tx_samples = create_tx_tone(SAMPLE_RATE, tone_freq=tone_freq)
    sdr.tx(tx_samples)
    print("TX active.")

    results = []
    for g in gains:
        sdr.tx_hardwaregain_chan0 = float(g)
        flush_rx(sdr, count=5)
        tone_db, noise_db, snr_db = measure_tone_power(
            sdr, SAMPLE_RATE, tone_freq, num_captures=20
        )
        results.append((g, tone_db, noise_db, snr_db))
        flag = "" if snr_db >= snr_threshold else "  (noise-limited)"
        print(f"  TX gain {g:+6.1f} dB  =>  tone {tone_db:.1f} dB | "
              f"noise {noise_db:.1f} dB | SNR {snr_db:.1f} dB{flag}")

    sdr.tx_destroy_buffer()
    print("TX stopped.\n")

    # Analysis
    gains_arr = np.array([r[0] for r in results])
    tone_arr = np.array([r[1] for r in results])
    noise_arr = np.array([r[2] for r in results])
    snr_arr = np.array([r[3] for r in results])

    # Separate valid (above noise) and noise-limited points
    valid_mask = snr_arr >= snr_threshold
    n_valid = np.sum(valid_mask)
    n_excluded = len(gains_arr) - n_valid

    if n_valid >= 2:
        # Fit only on valid points
        coeffs = np.polyfit(gains_arr[valid_mask], tone_arr[valid_mask], 1)
        slope, intercept = coeffs
        fit_line = np.polyval(coeffs, gains_arr)
        residuals = tone_arr[valid_mask] - fit_line[valid_mask]
        max_deviation = np.max(np.abs(residuals))

        # Point-to-point slope on valid points only
        valid_gains = gains_arr[valid_mask]
        valid_tones = tone_arr[valid_mask]
        diffs = np.diff(valid_tones) / np.diff(valid_gains)

        print("=== Gain Sweep Results ===")
        print(f"  Valid points       : {n_valid} / {len(gains_arr)} "
              f"({n_excluded} excluded, SNR < {snr_threshold} dB)")
        print(f"  Linear fit slope   : {slope:.3f} dB/dB (ideal = 1.0)")
        print(f"  Max deviation      : {max_deviation:.2f} dB")

        if len(diffs) > 0 and np.any(diffs < 0.8):
            comp_idx = np.where(diffs < 0.8)[0][0]
            print(f"  Compression        : detected at TX gain {valid_gains[comp_idx + 1]:+.0f} dB "
                  f"(slope = {diffs[comp_idx]:.2f} dB/dB)")
        else:
            min_slope = np.min(diffs) if len(diffs) > 0 else float("nan")
            print(f"  Compression        : none detected (min slope = {min_slope:.2f} dB/dB)")
    else:
        slope = float("nan")
        fit_line = np.full_like(tone_arr, np.nan)
        diffs = np.array([])
        print("=== Gain Sweep Results ===")
        print(f"  ERROR: Only {n_valid} valid points — cannot compute linearity.")
        print(f"  All points are noise-limited. Increase RX gain or reduce cable loss.")

    # Plot
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
    fig.suptitle("TX Gain Sweep — Linearity & Compression Check")

    # Top: tone power and noise floor
    ax1.plot(gains_arr, tone_arr, "o-", label="Tone power", zorder=3)
    ax1.plot(gains_arr, noise_arr, "x--", color="gray", alpha=0.6, label="Noise floor")
    if n_valid >= 2:
        ax1.plot(gains_arr, fit_line, "r--", alpha=0.7,
                 label=f"Linear fit (slope={slope:.3f}, valid points only)")
    # Shade noise-limited region
    if n_excluded > 0:
        noise_gains = gains_arr[~valid_mask]
        ax1.axvspan(noise_gains[0] - 0.5, noise_gains[-1] + 0.5,
                    alpha=0.1, color="red", label="Noise-limited region")
    ax1.set_xlabel("TX Gain (dB)")
    ax1.set_ylabel("Power (dB)")
    ax1.set_title("Tone Power & Noise Floor vs TX Gain")
    ax1.legend(fontsize=8)
    ax1.grid(True, alpha=0.3)

    # Bottom: incremental slope (valid points only)
    if len(diffs) > 0:
        valid_gains = gains_arr[valid_mask]
        ax2.bar(valid_gains[1:], diffs, width=0.6, color="steelblue")
        ax2.axhline(1.0, color="g", linestyle="--", alpha=0.7, label="Ideal (1.0)")
        ax2.axhline(0.8, color="r", linestyle="--", alpha=0.7, label="Compression threshold")
        ax2.legend()
    ax2.set_xlabel("TX Gain (dB)")
    ax2.set_ylabel("Incremental Slope (dB/dB)")
    ax2.set_title("Point-to-Point Gain Slope (valid points only)")
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    out_path = "tx_gain_sweep.png"
    plt.savefig(out_path, dpi=150)
    print(f"\n  Plot saved to {out_path}")
    plt.show()


# ---------- Test 2: Harmonic Measurement ----------

def run_harmonics(args):
    """Measure harmonic levels at 2f0 and 3f0 relative to fundamental.

    Uses a 100 kHz tone offset so the signal is distinguishable from LO
    leakage, and measures power at the specific tone bin in the FFT.
    """
    sdr = connect_pluto()
    tx_gain = args.tx_gain
    rx_gain = args.rx_gain
    tone_offset = 100e3  # 100 kHz offset to avoid LO leakage

    f0 = CENTER_FREQ
    # At each harmonic, the tone offset also scales: TX at f0+100kHz
    # produces harmonics at 2*(f0+100kHz), 3*(f0+100kHz), etc.
    # When we tune RX LO to N*f0, the tone appears at N*100kHz offset.
    harmonics = {
        "f0 (fundamental)": (f0, tone_offset),
        "2f0 (2nd harmonic)": (2 * f0, 2 * tone_offset),
        "3f0 (3rd harmonic)": (3 * f0, 3 * tone_offset),
    }

    print(f"  TX gain     : {tx_gain} dB")
    print(f"  RX gain     : {rx_gain} dB")
    print(f"  Tone offset : {tone_offset / 1e3:.0f} kHz")
    print(f"  f0          : {f0 / 1e6:.2f} MHz (tone at {(f0 + tone_offset) / 1e6:.2f})")
    print(f"  2f0         : {2 * f0 / 1e6:.2f} MHz (tone at {(2 * f0 + 2 * tone_offset) / 1e6:.2f})")
    print(f"  3f0         : {3 * f0 / 1e6:.2f} MHz (tone at {(3 * f0 + 3 * tone_offset) / 1e6:.2f})")
    print()

    # Start TX at f0 with tone offset
    configure_pluto(sdr, f0, SAMPLE_RATE, tx_gain, rx_gain)
    tx_samples = create_tx_tone(SAMPLE_RATE, tone_freq=tone_offset)
    sdr.tx(tx_samples)
    print("TX active at fundamental.\n")

    measured = {}
    snr_data = {}
    spectra = {}

    for label, (rx_lo, expected_tone) in harmonics.items():
        print(f"  Measuring {label} @ {rx_lo / 1e6:.2f} MHz (tone at +{expected_tone / 1e3:.0f} kHz)...")
        sdr.rx_lo = int(rx_lo)
        flush_rx(sdr, count=10)

        # Measure tone power at the expected offset bin
        tone_db, noise_db, snr_db = measure_tone_power(
            sdr, SAMPLE_RATE, expected_tone, num_captures=30
        )

        # Grab one capture for spectrum plot
        rx_data = sdr.rx() / (2**11)
        freqs_hz, psd = compute_spectrum(rx_data, SAMPLE_RATE)

        measured[label] = tone_db
        snr_data[label] = snr_db
        spectra[label] = (freqs_hz + rx_lo, psd, expected_tone)
        print(f"    Tone power: {tone_db:.1f} dB | Noise floor: {noise_db:.1f} dB | SNR: {snr_db:.1f} dB")

    sdr.tx_destroy_buffer()
    print("\nTX stopped.\n")

    # Calculate dBc
    fund_power = measured["f0 (fundamental)"]
    print("=== Harmonic Measurement Results ===")
    print(f"  {'Harmonic':<25} {'Tone (dB)':>10} {'dBc':>8} {'SNR (dB)':>10}")
    print(f"  {'-' * 55}")
    for label, pwr in measured.items():
        dbc = pwr - fund_power
        snr = snr_data[label]
        note = ""
        if label != "f0 (fundamental)" and snr < 6:
            note = "  (near noise floor — upper bound only)"
        print(f"  {label:<25} {pwr:>8.1f}   {dbc:>6.1f}   {snr:>8.1f}{note}")

    # Plot
    fig = plt.figure(figsize=(12, 10))
    gs = fig.add_gridspec(2, 2, height_ratios=[1, 1])

    # Spectrum subplots
    for idx, (label, (freqs_hz, psd, expected_tone)) in enumerate(spectra.items()):
        row, col = divmod(idx, 2) if idx < 2 else (1, 0)
        ax = fig.add_subplot(gs[row, col])
        ax.plot(freqs_hz / 1e6, psd)
        # Mark expected tone position
        rx_lo = freqs_hz[len(freqs_hz) // 2]
        ax.axvline((rx_lo + expected_tone) / 1e6, color="r", linestyle="--",
                   alpha=0.7, label=f"Expected tone")
        ax.set_xlabel("Frequency (MHz)")
        ax.set_ylabel("Power (dB)")
        ax.set_title(f"{label} (SNR: {snr_data[label]:.1f} dB)")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

    # Bar chart of harmonic levels
    ax_bar = fig.add_subplot(gs[1, 1])
    labels_short = ["f0", "2f0", "3f0"]
    dbc_vals = [measured[l] - fund_power for l in measured]
    colors = ["green" if d == 0 else ("orange" if d > -30 else "steelblue") for d in dbc_vals]
    ax_bar.bar(labels_short, dbc_vals, color=colors)
    ax_bar.set_ylabel("Level (dBc)")
    ax_bar.set_title("Harmonic Levels Relative to Fundamental")
    ax_bar.axhline(-30, color="r", linestyle="--", alpha=0.5, label="-30 dBc ref")
    ax_bar.legend()
    ax_bar.grid(True, alpha=0.3, axis="y")

    fig.suptitle(f"Harmonic Analysis @ {f0 / 1e6:.2f} MHz | TX gain: {tx_gain} dB")
    plt.tight_layout()
    out_path = "tx_harmonics.png"
    plt.savefig(out_path, dpi=150)
    print(f"\n  Plot saved to {out_path}")
    plt.show()


# ---------- Test 3: Insertion Loss ----------

def run_insertion_loss(args):
    """Measure insertion loss by comparing direct connection vs through DUT.

    Uses tone-based FFT measurement for accuracy, and warns about the
    AS179-92LF antenna switch requiring DC bias.
    """
    tx_gain = args.tx_gain
    rx_gain = args.rx_gain
    tone_freq = 100e3

    print("=== Insertion Loss Measurement ===")
    print(f"  TX gain     : {tx_gain} dB")
    print(f"  RX gain     : {rx_gain} dB")
    print(f"  Frequency   : {CENTER_FREQ / 1e6:.2f} MHz")
    print()
    print("  NOTE: The RF board's AS179-92LF antenna switch requires DC bias")
    print("  from the Si4463 to route signals. If the board is unpowered,")
    print("  expect very high insertion loss (no signal path).")
    print("  Ensure the Si4463 is powered and configured for TX before step 2.")
    print()

    results = {}

    for step, label in [(1, "baseline"), (2, "dut")]:
        if label == "baseline":
            print("STEP 1: Connect Pluto TX directly to Pluto RX (cable only).")
        else:
            print("STEP 2: Connect Pluto TX -> RF board (powered) -> Pluto RX.")

        input("  Press Enter when connected...")
        print()

        sdr = connect_pluto()
        configure_pluto(sdr, CENTER_FREQ, SAMPLE_RATE, tx_gain, rx_gain)
        tx_samples = create_tx_tone(SAMPLE_RATE, tone_freq=tone_freq)
        sdr.tx(tx_samples)
        print("  TX active. Measuring...")

        flush_rx(sdr, count=10)
        tone_db, noise_db, snr_db = measure_tone_power(
            sdr, SAMPLE_RATE, tone_freq, num_captures=50
        )

        # Grab spectrum
        rx_data = sdr.rx() / (2**11)
        freqs_hz, psd = compute_spectrum(rx_data, SAMPLE_RATE)

        results[label] = {
            "tone_power": tone_db,
            "noise_floor": noise_db,
            "snr": snr_db,
            "spectrum": (freqs_hz + CENTER_FREQ, psd),
        }

        sdr.tx_destroy_buffer()
        del sdr
        print(f"  Tone: {tone_db:.1f} dB | Noise: {noise_db:.1f} dB | SNR: {snr_db:.1f} dB")
        if snr_db < 6:
            print("  WARNING: SNR < 6 dB — signal may be at noise floor")
        print(f"  TX stopped.\n")

    # Results
    baseline_pwr = results["baseline"]["tone_power"]
    dut_pwr = results["dut"]["tone_power"]
    insertion_loss = baseline_pwr - dut_pwr

    print("=== Insertion Loss Results ===")
    print(f"  Baseline (direct) : {baseline_pwr:.1f} dB (SNR {results['baseline']['snr']:.1f} dB)")
    print(f"  Through DUT       : {dut_pwr:.1f} dB (SNR {results['dut']['snr']:.1f} dB)")
    print(f"  Insertion loss    : {insertion_loss:.1f} dB")

    if results["dut"]["snr"] < 6:
        print(f"  Status            : NO SIGNAL — DUT SNR < 6 dB")
        print(f"                      Check: is the Si4463 powered? Is the switch biased for TX?")
    elif insertion_loss < 3:
        print(f"  Status            : GOOD (< 3 dB)")
    elif insertion_loss < 6:
        print(f"  Status            : ACCEPTABLE (< 6 dB)")
    else:
        print(f"  Status            : HIGH ({insertion_loss:.1f} dB) — check matching network")

    # Plot
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
    fig.suptitle(f"Insertion Loss @ {CENTER_FREQ / 1e6:.2f} MHz | TX gain: {tx_gain} dB")

    # Overlay spectra
    for label, data in results.items():
        freqs_mhz, psd = data["spectrum"]
        display_label = f"Baseline (SNR {results['baseline']['snr']:.0f} dB)" if label == "baseline" \
            else f"Through DUT (SNR {results['dut']['snr']:.0f} dB)"
        ax1.plot(freqs_mhz / 1e6, psd, label=display_label, alpha=0.8)
    ax1.set_xlabel("Frequency (MHz)")
    ax1.set_ylabel("Power (dB)")
    ax1.set_title("Spectrum Comparison")
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    # Bar chart
    bars = ax2.bar(
        ["Baseline\n(direct)", "Through\nRF board"],
        [baseline_pwr, dut_pwr],
        color=["steelblue", "coral"],
        width=0.5,
    )
    ax2.set_ylabel("Tone Power (dB)")
    ax2.set_title(f"Insertion Loss: {insertion_loss:.1f} dB")
    ax2.grid(True, alpha=0.3, axis="y")
    for bar, val in zip(bars, [baseline_pwr, dut_pwr]):
        ax2.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.3,
                 f"{val:.1f}", ha="center", va="bottom", fontweight="bold")

    plt.tight_layout()
    out_path = "tx_insertion_loss.png"
    plt.savefig(out_path, dpi=150)
    print(f"\n  Plot saved to {out_path}")
    plt.show()


# ---------- CLI ----------

def main():
    parser = argparse.ArgumentParser(
        description="Extended TX test suite for kensat_radio via Pluto SDR"
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    # Sweep
    p_sweep = subparsers.add_parser("sweep", help="TX gain sweep for linearity check")
    p_sweep.add_argument("--tx-start", type=float, default=-20, help="Start TX gain (dB)")
    p_sweep.add_argument("--tx-stop", type=float, default=0, help="Stop TX gain (dB)")
    p_sweep.add_argument("--tx-step", type=float, default=1, help="TX gain step (dB)")
    p_sweep.add_argument("--rx-gain", type=float, default=40, help="RX gain (dB)")

    # Harmonics
    p_harm = subparsers.add_parser("harmonics", help="Measure 2nd/3rd harmonic levels")
    p_harm.add_argument("--tx-gain", type=float, default=-10, help="TX gain (dB)")
    p_harm.add_argument("--rx-gain", type=float, default=40, help="RX gain (dB)")

    # Insertion loss
    p_il = subparsers.add_parser("insertion-loss", help="Measure insertion loss through RF board")
    p_il.add_argument("--tx-gain", type=float, default=-10, help="TX gain (dB)")
    p_il.add_argument("--rx-gain", type=float, default=40, help="RX gain (dB)")

    args = parser.parse_args()

    if args.command == "sweep":
        run_sweep(args)
    elif args.command == "harmonics":
        run_harmonics(args)
    elif args.command == "insertion-loss":
        run_insertion_loss(args)


if __name__ == "__main__":
    main()
