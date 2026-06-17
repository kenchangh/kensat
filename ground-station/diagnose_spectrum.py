#!/usr/bin/env python3
"""
Spectrum diagnostic: capture IQ from Pluto while Si4463 is transmitting.
Dumps raw FFT power at key frequencies in the 70-130 kHz window.
Shows centroid vs argmax carrier estimate.
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

print("Capturing 40 buffers. Fire 'x' on Si4463 now!")
print(f"LO set to: {(CENTER_FREQ + PLUTO_XO_CORRECTION - LO_OFFSET)/1e6:.6f} MHz")
print(f"Expected carrier in baseband: ~{LO_OFFSET/1e3:.0f} kHz")
print()

# Compute noise baseline first (5 buffers without signal)
print("Building noise baseline (5 buffers)...")
noise_sum = None
for _ in range(5):
    iq = sdr.rx().astype(np.complex128)
    fft = np.abs(np.fft.fftshift(np.fft.fft(iq))) ** 2
    noise_sum = fft if noise_sum is None else noise_sum + fft
noise_baseline = noise_sum / 5

freqs = np.fft.fftshift(np.fft.fftfreq(BUF_SIZE, 1.0 / SAMPLE_RATE))
mask70_130 = (freqs >= 70000) & (freqs <= 130000)

# Print noise baseline key points
print(f"Noise baseline in 70-130 kHz window:")
for target in [80000, 85000, 90000, 95000, 100000, 105000, 110000, 115000, 120000]:
    idx = np.argmin(np.abs(freqs - target))
    print(f"  {target/1e3:.0f} kHz: {noise_baseline[idx]:.2e}")
noise_max_power = noise_baseline[mask70_130].max()
noise_max_freq = freqs[mask70_130][np.argmax(noise_baseline[mask70_130])]
print(f"  Noise max: {noise_max_power:.2e} at {noise_max_freq/1e3:.1f} kHz")
print()

# Monitor for signal
print("Monitoring... (send 'x' to Si4463)")
print("-" * 70)
signal_buf_count = 0
for buf_idx in range(200):
    iq = sdr.rx().astype(np.complex128)
    iq_rms = np.sqrt(np.mean(np.abs(iq) ** 2))

    fft = np.abs(np.fft.fftshift(np.fft.fft(iq))) ** 2
    diff = fft - noise_baseline

    # Centroid in 70-130 kHz window
    power_w = fft[mask70_130]
    total_p = np.sum(power_w)
    centroid = float(np.sum(freqs[mask70_130] * power_w) / total_p) if total_p > 0 else LO_OFFSET

    # Argmax in 70-130 kHz window
    peak_idx = np.argmax(fft[mask70_130])
    argmax_freq = freqs[mask70_130][peak_idx]
    argmax_power = fft[mask70_130][peak_idx]

    # Argmax of DIFFERENCE (signal above noise)
    diff_peak_idx = np.argmax(diff[mask70_130])
    diff_argmax_freq = freqs[mask70_130][diff_peak_idx]
    diff_argmax_power = diff[mask70_130][diff_peak_idx]

    if iq_rms > 50:
        signal_buf_count += 1
        print(f"\n>>> SIGNAL BUF #{buf_idx} (iq_rms={iq_rms:.0f})")
        print(f"    Centroid : {centroid/1e3:.2f} kHz")
        print(f"    ArgMax   : {argmax_freq/1e3:.2f} kHz  power={argmax_power:.2e}")
        print(f"    Diff-Max : {diff_argmax_freq/1e3:.2f} kHz  excess={diff_argmax_power:.2e}  noise={noise_max_power:.2e}")
        print(f"    Spectrum (5 kHz steps, 70-130 kHz):")
        for target in range(70000, 131000, 2500):
            idx = np.argmin(np.abs(freqs - target))
            pwr = fft[idx]
            bar = "#" * int(np.log10(max(pwr, 1)) - 4)
            print(f"      {target/1e3:6.1f} kHz: {pwr:.2e} {bar}")

        if signal_buf_count >= 5:
            print("\nCaptured 5 signal buffers, done.")
            break
    elif buf_idx % 10 == 0:
        print(f"  buf {buf_idx:3d}: quiet iq_rms={iq_rms:.1f} centroid={centroid/1e3:.1f}kHz")
    sys.stdout.flush()

print("Done.")
