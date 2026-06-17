#!/usr/bin/env python3
"""
Full-spectrum diagnostic: find where the Si4463 signal actually is.
Shows the full 0-480 kHz baseband spectrum during TX vs silence.
"""
import sys
import time
import numpy as np
import adi

PLUTO_URI = "ip:192.168.3.1"
CENTER_FREQ = 437_080_000
PLUTO_XO_CORRECTION = 26000
LO_OFFSET = 100_000
SAMPLE_RATE = 960_000
BUF_SIZE = 65536

sdr = adi.Pluto(PLUTO_URI)
sdr.rx_lo = CENTER_FREQ + PLUTO_XO_CORRECTION - LO_OFFSET
sdr.rx_rf_bandwidth = SAMPLE_RATE
sdr.gain_control_mode_chan0 = 'manual'
sdr.rx_hardwaregain_chan0 = 40
sdr.sample_rate = SAMPLE_RATE
sdr.rx_buffer_size = BUF_SIZE

print("Flushing...")
for _ in range(5):
    sdr.rx()

print(f"LO commanded: {(CENTER_FREQ + PLUTO_XO_CORRECTION - LO_OFFSET)/1e6:.6f} MHz")
print(f"Si4463 expected at: +{LO_OFFSET/1e3:.0f} kHz in baseband")
print()

freqs = np.fft.fftshift(np.fft.fftfreq(BUF_SIZE, 1.0 / SAMPLE_RATE))

# Build noise baseline
print("Noise baseline (5 buffers)...")
noise_sum = None
for _ in range(5):
    iq = sdr.rx().astype(np.complex128)
    fft = np.abs(np.fft.fftshift(np.fft.fft(iq))) ** 2
    noise_sum = fft if noise_sum is None else noise_sum + fft
noise_baseline = noise_sum / 5

# Show full spectrum noise floor
print("Noise floor (10 kHz steps, positive freqs only):")
for f_khz in range(0, 490, 10):
    idx = np.argmin(np.abs(freqs - f_khz * 1000))
    print(f"  {f_khz:4d} kHz: {noise_baseline[idx]:.2e}")
print()

print("Monitoring for signal (fire 'x' now)...")
print("-" * 70)

sig_count = 0
for buf_idx in range(200):
    iq = sdr.rx().astype(np.complex128)
    iq_rms = np.sqrt(np.mean(np.abs(iq) ** 2))
    fft = np.abs(np.fft.fftshift(np.fft.fft(iq))) ** 2
    diff = fft - noise_baseline

    if iq_rms > 50:
        sig_count += 1
        # Find top 5 peaks in difference spectrum (positive freqs)
        pos_mask = freqs > 0
        diff_pos = diff[pos_mask]
        freqs_pos = freqs[pos_mask]

        # Get top 5 peaks
        sorted_idx = np.argsort(diff_pos)[::-1][:10]
        top_freqs = freqs_pos[sorted_idx]
        top_powers = diff_pos[sorted_idx]

        print(f"\n>>> SIGNAL BUF #{buf_idx} (iq_rms={iq_rms:.0f})")
        print(f"    Top 10 peaks above noise (full 0-480 kHz spectrum):")
        for f, p in zip(top_freqs, top_powers):
            if p > 0:
                print(f"      {f/1e3:7.2f} kHz  excess={p:.2e}")

        # Also show 10 kHz resolution sweep
        print(f"    Full spectrum (10 kHz steps):")
        for f_khz in range(0, 490, 10):
            idx = np.argmin(np.abs(freqs - f_khz * 1000))
            pwr = fft[idx]
            noise_here = noise_baseline[idx]
            snr = pwr / noise_here if noise_here > 0 else 0
            if snr > 5 or f_khz % 50 == 0:
                bar = "#" * min(int(snr), 30)
                print(f"      {f_khz:4d} kHz: pwr={pwr:.2e} snr={snr:5.1f}x {bar}")

        if sig_count >= 4:
            print("\nCaptured 4 signal buffers, done.")
            break

    elif buf_idx % 15 == 0:
        print(f"  buf {buf_idx:3d}: quiet iq_rms={iq_rms:.1f}")
    sys.stdout.flush()

print("Done.")
