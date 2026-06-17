#!/usr/bin/env python3
"""Fine spectrum around 85-115 kHz: quiet vs TX, find exact peaks."""
import numpy as np, adi, sys
from scipy.signal import firwin, lfilter

PLUTO_URI = "ip:192.168.3.1"
CENTER_FREQ = 437_080_000
PLUTO_XO_CORRECTION = 20800
LO_OFFSET = 100_000
SAMPLE_RATE = 960_000

sdr = adi.Pluto(PLUTO_URI)
sdr.rx_lo = CENTER_FREQ + PLUTO_XO_CORRECTION - LO_OFFSET
sdr.rx_rf_bandwidth = SAMPLE_RATE
sdr.gain_control_mode_chan0 = 'manual'
sdr.rx_hardwaregain_chan0 = 40
sdr.sample_rate = SAMPLE_RATE
sdr.rx_buffer_size = 65536
for _ in range(5): sdr.rx()

# First: quiet spectrum
print("=== QUIET SPECTRUM (no Si4463 TX) ===")
iq = sdr.rx().astype(np.complex128)
n = len(iq)
freqs = np.fft.fftshift(np.fft.fftfreq(n, 1.0/SAMPLE_RATE))
fft_mag = np.abs(np.fft.fftshift(np.fft.fft(iq)))**2
rms = np.sqrt(np.mean(np.abs(iq)**2))
print(f"iq_rms={rms:.1f}")
print("85-115 kHz fine spectrum (500 Hz steps):")
for f_khz_x2 in range(170, 231, 1):
    f = f_khz_x2 * 500
    idx = np.argmin(np.abs(freqs - f))
    pwr = fft_mag[idx]
    bar = '#' * min(int(pwr/1e7), 20)
    print(f"  {f/1e3:6.1f} kHz: {pwr:.2e} {bar}")

print("\n\nNow fire 'x' and waiting for Si4463 TX...")
for i in range(200):
    iq = sdr.rx().astype(np.complex128)
    rms = np.sqrt(np.mean(np.abs(iq)**2))
    if rms < 50:
        if i % 20 == 0: print(f"  buf {i}: quiet rms={rms:.1f}")
        continue

    freqs_buf = np.fft.fftshift(np.fft.fftfreq(n, 1.0/SAMPLE_RATE))
    fft_tx = np.abs(np.fft.fftshift(np.fft.fft(iq)))**2

    print(f"\n=== TX BUF {i}: rms={rms:.0f} ===")
    print("85-115 kHz fine spectrum (500 Hz steps):")
    for f_khz_x2 in range(170, 231, 1):
        f = f_khz_x2 * 500
        idx = np.argmin(np.abs(freqs - f))
        pwr = fft_tx[idx]
        ratio = pwr / max(fft_mag[idx], 1)
        bar = '#' * min(int(ratio/2), 30)
        print(f"  {f/1e3:6.1f} kHz: {pwr:.2e} ratio={ratio:5.0f}x {bar}")

    # Find the top 5 peaks in 85-115 kHz
    mask = (freqs_buf >= 85000) & (freqs_buf <= 115000)
    top5 = np.argsort(fft_tx[mask])[-5:][::-1]
    top_freqs = freqs_buf[mask][top5]
    top_pwrs = fft_tx[mask][top5]
    print(f"\nTop 5 peaks 85-115 kHz:")
    for f, p in zip(top_freqs, top_pwrs):
        print(f"  {f/1e3:.3f} kHz: {p:.2e}")
    break

print("Done.")
