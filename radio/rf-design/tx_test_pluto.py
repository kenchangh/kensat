#!/usr/bin/env python3
"""
TX test for the kensat_radio RF board using ADALM-Pluto SDR.

Transmits a CW tone at 437.08 MHz from the Pluto, receives it back,
and measures power level to verify the RF board's TX path.

Usage:
    python tx_test_pluto.py [--freq 437.08e6] [--tx-gain -10] [--duration 5]

Requirements:
    pip install pyadi-iio numpy matplotlib
"""

import argparse
import sys
import time

import adi
import matplotlib.pyplot as plt
import numpy as np


PLUTO_URI = "ip:192.168.2.1"
CENTER_FREQ = 437.08e6  # Hz - kensat_radio design frequency
SAMPLE_RATE = 1e6       # 1 MSPS
RX_BUFFER_SIZE = 2**14


def create_tx_tone(sample_rate, tone_freq=100e3, num_samples=2**14):
    """Generate a complex sinusoid for TX."""
    t = np.arange(num_samples) / sample_rate
    iq = 2**14 * np.exp(2j * np.pi * tone_freq * t)
    return iq.astype(np.complex64)


def configure_pluto(sdr, center_freq, sample_rate, tx_gain_db, rx_gain_db):
    """Configure Pluto TX and RX channels."""
    # TX config
    sdr.tx_lo = int(center_freq)
    sdr.tx_rf_bandwidth = int(sample_rate)
    sdr.tx_hardwaregain_chan0 = tx_gain_db
    sdr.sample_rate = int(sample_rate)
    sdr.tx_cyclic_buffer = True

    # RX config
    sdr.rx_lo = int(center_freq)
    sdr.rx_rf_bandwidth = int(sample_rate)
    sdr.gain_control_mode_chan0 = "manual"
    sdr.rx_hardwaregain_chan0 = rx_gain_db
    sdr.rx_buffer_size = RX_BUFFER_SIZE


def measure_power_dbm(samples, impedance=50.0):
    """Estimate power in dBm from IQ samples."""
    rms = np.sqrt(np.mean(np.abs(samples) ** 2))
    if rms == 0:
        return -np.inf
    power_w = (rms**2) / impedance
    return 10 * np.log10(power_w) + 30


def compute_spectrum(samples, sample_rate):
    """Compute power spectral density."""
    window = np.hanning(len(samples))
    fft_data = np.fft.fftshift(np.fft.fft(samples * window))
    freqs = np.fft.fftshift(np.fft.fftfreq(len(samples), 1 / sample_rate))
    psd = 20 * np.log10(np.abs(fft_data) + 1e-12)
    return freqs, psd


def run_tx_test(center_freq, tx_gain_db, rx_gain_db, duration, tone_freq, plot):
    """Run the TX loopback test."""
    print(f"Connecting to Pluto SDR at {PLUTO_URI}...")
    try:
        sdr = adi.Pluto(uri=PLUTO_URI)
    except Exception as e:
        print(f"ERROR: Cannot connect to Pluto SDR: {e}")
        print("Check that the Pluto is powered on and reachable at 192.168.2.1")
        sys.exit(1)

    print(f"  Center freq : {center_freq / 1e6:.2f} MHz")
    print(f"  Tone offset : {tone_freq / 1e3:.1f} kHz")
    print(f"  Sample rate : {SAMPLE_RATE / 1e6:.1f} MSPS")
    print(f"  TX gain     : {tx_gain_db} dB")
    print(f"  RX gain     : {rx_gain_db} dB")
    print(f"  Duration    : {duration} s")
    print()

    configure_pluto(sdr, center_freq, SAMPLE_RATE, tx_gain_db, rx_gain_db)

    # Generate and start TX
    tx_samples = create_tx_tone(SAMPLE_RATE, tone_freq)
    sdr.tx(tx_samples)
    print("TX active. Transmitting CW tone...")

    # Collect RX captures over the test duration
    num_captures = max(1, int(duration * SAMPLE_RATE / RX_BUFFER_SIZE))
    all_power = []
    last_capture = None

    try:
        for i in range(num_captures):
            rx_data = sdr.rx()
            rx_data = rx_data / (2**11)  # normalize ADC scale
            pwr = measure_power_dbm(rx_data)
            all_power.append(pwr)
            last_capture = rx_data

            if (i + 1) % max(1, num_captures // 5) == 0:
                print(f"  Capture {i + 1}/{num_captures}: {pwr:.1f} dBm")

    except KeyboardInterrupt:
        print("\nTest interrupted by user.")
    finally:
        sdr.tx_destroy_buffer()
        print("TX stopped.")

    # Results
    print()
    print("=== TX Test Results ===")
    if all_power:
        mean_pwr = np.mean(all_power)
        std_pwr = np.std(all_power)
        print(f"  Mean RX power : {mean_pwr:.1f} dBm")
        print(f"  Std deviation : {std_pwr:.2f} dB")
        print(f"  Min / Max     : {min(all_power):.1f} / {max(all_power):.1f} dBm")
        print(f"  Captures      : {len(all_power)}")

        if mean_pwr > -60:
            print("  Status        : PASS - Signal detected")
        else:
            print("  Status        : FAIL - Signal too weak or not detected")
    else:
        print("  No captures collected.")

    # Plot spectrum of last capture
    if plot and last_capture is not None:
        freqs, psd = compute_spectrum(last_capture, SAMPLE_RATE)
        freqs_mhz = (freqs + center_freq) / 1e6

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
        fig.suptitle(
            f"TX Test @ {center_freq / 1e6:.2f} MHz | "
            f"TX gain: {tx_gain_db} dB | RX gain: {rx_gain_db} dB"
        )

        # Spectrum
        ax1.plot(freqs_mhz, psd)
        ax1.set_xlabel("Frequency (MHz)")
        ax1.set_ylabel("Power (dB)")
        ax1.set_title("RX Spectrum")
        ax1.grid(True, alpha=0.3)
        ax1.axvline(center_freq / 1e6, color="r", linestyle="--", alpha=0.5, label="Center")
        ax1.legend()

        # Power over time
        ax2.plot(all_power, marker=".", markersize=3)
        ax2.set_xlabel("Capture #")
        ax2.set_ylabel("Power (dBm)")
        ax2.set_title("RX Power Over Time")
        ax2.grid(True, alpha=0.3)
        ax2.axhline(np.mean(all_power), color="r", linestyle="--", alpha=0.5, label="Mean")
        ax2.legend()

        plt.tight_layout()
        out_path = "tx_test_result.png"
        plt.savefig(out_path, dpi=150)
        print(f"\n  Plot saved to {out_path}")
        plt.show()


def main():
    parser = argparse.ArgumentParser(description="TX test for kensat_radio via Pluto SDR")
    parser.add_argument("--freq", type=float, default=CENTER_FREQ,
                        help=f"Center frequency in Hz (default: {CENTER_FREQ / 1e6} MHz)")
    parser.add_argument("--tone", type=float, default=100e3,
                        help="TX tone offset in Hz (default: 100 kHz)")
    parser.add_argument("--tx-gain", type=float, default=-10,
                        help="TX attenuation in dB (default: -10)")
    parser.add_argument("--rx-gain", type=float, default=40,
                        help="RX gain in dB (default: 40)")
    parser.add_argument("--duration", type=float, default=5,
                        help="Test duration in seconds (default: 5)")
    parser.add_argument("--no-plot", action="store_true",
                        help="Disable spectrum plot")
    args = parser.parse_args()

    run_tx_test(
        center_freq=args.freq,
        tx_gain_db=args.tx_gain,
        rx_gain_db=args.rx_gain,
        duration=args.duration,
        tone_freq=args.tone,
        plot=not args.no_plot,
    )


if __name__ == "__main__":
    main()
