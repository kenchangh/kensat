#!/usr/bin/env python3
"""
Diagnose FM demodulation quality during Si4463 TX.
Capture IQ, mix to baseband, LPF, FM discriminator.
Shows FM demod output for first signal buffer.
"""
import sys, numpy as np, adi
from scipy.signal import firwin, lfilter

PLUTO_URI = "ip:192.168.3.1"
CENTER_FREQ = 437_080_000
PLUTO_XO_CORRECTION = 20800
LO_OFFSET = 100_000
SAMPLE_RATE = 960_000
DATA_RATE = 9600
FREQ_DEVIATION = 4800
SPS = SAMPLE_RATE // DATA_RATE  # 100

sdr = adi.Pluto(PLUTO_URI)
sdr.rx_lo = CENTER_FREQ + PLUTO_XO_CORRECTION - LO_OFFSET
sdr.rx_rf_bandwidth = SAMPLE_RATE
sdr.gain_control_mode_chan0 = 'manual'
sdr.rx_hardwaregain_chan0 = 40
sdr.sample_rate = SAMPLE_RATE
sdr.rx_buffer_size = 65536

for _ in range(5): sdr.rx()

print("Waiting for Si4463 TX (fire 'x' now)...")
print("Looking for iq_rms > 100...")

lpf = firwin(501, 15000 / (SAMPLE_RATE / 2))

found = 0
for i in range(200):
    iq = sdr.rx().astype(np.complex128)
    iq_rms = np.sqrt(np.mean(np.abs(iq)**2))

    if iq_rms < 50:
        if i % 20 == 0:
            print(f"  buf {i}: quiet iq_rms={iq_rms:.1f}")
        continue

    # Found signal buffer
    iq_peak = np.max(np.abs(iq))
    iq_min = np.min(iq.real)
    iq_max = np.max(iq.real)

    # Carrier lock - find actual fc
    n = len(iq)
    fft_data = np.fft.fftshift(np.fft.fft(iq))
    freqs = np.fft.fftshift(np.fft.fftfreq(n, 1.0/SAMPLE_RATE))
    mask = (freqs > 70000) & (freqs < 130000)
    power = np.abs(fft_data[mask])**2
    fc = float(np.sum(freqs[mask] * power) / np.sum(power))

    print(f"\n=== SIGNAL BUF #{i}: iq_rms={iq_rms:.0f}, iq_peak={iq_peak:.0f}, fc={fc/1e3:.2f} kHz")
    print(f"    IQ saturated: {'YES - CLIPPING' if iq_peak > 2040 else 'no'}")

    # Mix to baseband at fc
    bb = iq * np.exp(-1j * 2 * np.pi * fc * np.arange(n) / SAMPLE_RATE)
    bb_filt = lfilter(lpf, 1.0, bb)
    freq_raw = np.angle(bb_filt[1:] * np.conj(bb_filt[:-1]))
    freq_norm = freq_raw * SAMPLE_RATE / (2 * np.pi * FREQ_DEVIATION)

    demod_rms = np.sqrt(np.mean(freq_norm**2))
    demod_mean = np.mean(freq_norm)
    demod_std = np.std(freq_norm)
    print(f"    FM demod: rms={demod_rms:.3f}, mean={demod_mean:.3f}, std={demod_std:.3f}")

    # Print first 20 symbols (at symbol centers, skip first 5 for filter settling)
    skip = 500 + SPS//2  # skip LPF settling
    centers = [skip + i*SPS for i in range(30)]
    vals = [freq_norm[c] for c in centers if c < len(freq_norm)]
    bits = [1 if v > 0 else 0 for v in vals]
    print(f"    First 30 symbol samples (should be alternating 0,1 for preamble):")
    print(f"    values: {[f'{v:+.2f}' for v in vals[:15]]}")
    print(f"    bits:   {bits[:15]}")
    print(f"    values: {[f'{v:+.2f}' for v in vals[15:]]}")
    print(f"    bits:   {bits[15:]}")

    # Histogram of demod values
    hist, edges = np.histogram(freq_norm, bins=20, range=(-6, 6))
    print(f"\n    FM demod histogram (should be bimodal near ±1 for GFSK):")
    for hi, (e1, e2) in zip(hist, zip(edges, edges[1:])):
        bar = '#' * (hi // max(hist//30, 1))
        pct = hi/len(freq_norm)*100
        if pct > 0.5:
            print(f"      [{e1:+.1f},{e2:+.1f}]: {bar} ({pct:.1f}%)")

    # Spectrum of FM demod output (should show energy at ~4800 Hz / (9600/2))
    fft_demod = np.abs(np.fft.rfft(freq_norm))**2
    demod_freqs = np.fft.rfftfreq(len(freq_norm), 1.0/SAMPLE_RATE)
    print(f"\n    FM demod spectrum (key frequencies):")
    for target in [0, 2400, 4800, 9600, 19200, 48000, 100000]:
        if target > SAMPLE_RATE//2: continue
        idx = np.argmin(np.abs(demod_freqs - target))
        pwr = fft_demod[idx]
        print(f"      {target:6d} Hz: {pwr:.2e}")

    found += 1
    if found >= 3:
        break

print("\nDone.")
