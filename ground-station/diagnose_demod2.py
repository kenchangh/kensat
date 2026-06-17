#!/usr/bin/env python3
"""
Deep FM demod diagnostic. Show IQ spectrum and FM demod output for Si4463 TX.
"""
import numpy as np, adi, sys
from scipy.signal import firwin, lfilter

PLUTO_URI = "ip:192.168.3.1"
CENTER_FREQ = 437_080_000
PLUTO_XO_CORRECTION = 20800
LO_OFFSET = 100_000
SAMPLE_RATE = 960_000
FREQ_DEVIATION = 4800
SPS = 100

sdr = adi.Pluto(PLUTO_URI)
sdr.rx_lo = CENTER_FREQ + PLUTO_XO_CORRECTION - LO_OFFSET
sdr.rx_rf_bandwidth = SAMPLE_RATE
sdr.gain_control_mode_chan0 = 'manual'
sdr.rx_hardwaregain_chan0 = 40
sdr.sample_rate = SAMPLE_RATE
sdr.rx_buffer_size = 65536
for _ in range(5): sdr.rx()

print("Waiting for signal (fire 'x' now)...")
lpf15 = firwin(501, 15000 / (SAMPLE_RATE / 2))
lpf30 = firwin(501, 30000 / (SAMPLE_RATE / 2))

for i in range(200):
    iq = sdr.rx().astype(np.complex128)
    rms = np.sqrt(np.mean(np.abs(iq)**2))
    if rms < 50:
        if i % 20 == 0: print(f"  buf {i}: quiet rms={rms:.1f}")
        continue

    n = len(iq)
    fft_data = np.fft.fftshift(np.fft.fft(iq))
    freqs = np.fft.fftshift(np.fft.fftfreq(n, 1.0/SAMPLE_RATE))

    # Carrier lock
    mask = (freqs > 70000) & (freqs < 130000)
    power = np.abs(fft_data[mask])**2
    fc = float(np.sum(freqs[mask] * power) / np.sum(power))

    print(f"\n=== BUF {i}: rms={rms:.0f}, fc={fc/1e3:.2f} kHz")

    # IQ spectrum around carrier — show the GFSK sidebands
    print("IQ spectrum 85-115 kHz (should see two peaks at fc±4.8 kHz):")
    for f_khz in range(85, 116, 1):
        idx = np.argmin(np.abs(freqs - f_khz*1000))
        pwr = np.abs(fft_data[idx])**2
        bar = '#' * min(int(pwr/1e8), 40)
        print(f"  {f_khz:3d} kHz: {pwr:.2e} {bar}")

    # Mix and demod with both LPF widths
    t = np.arange(n)
    mix = np.exp(-1j * 2 * np.pi * fc * t / SAMPLE_RATE)
    bb = iq * mix

    for lpf_name, lpf in [('15kHz', lpf15), ('30kHz', lpf30)]:
        bb_f = lfilter(lpf, 1.0, bb)
        freq_raw = np.angle(bb_f[1:] * np.conj(bb_f[:-1]))
        freq_norm = freq_raw * SAMPLE_RATE / (2 * np.pi * FREQ_DEVIATION)
        demod_rms = np.sqrt(np.mean(freq_norm**2))

        # FFT of FM demod output (shows energy at modulation frequency)
        fm_fft = np.abs(np.fft.rfft(freq_norm))**2
        fm_freqs = np.fft.rfftfreq(len(freq_norm), 1.0/SAMPLE_RATE)

        # Find top 3 peaks
        top3 = np.argsort(fm_fft)[-3:][::-1]

        print(f"\n  LPF {lpf_name}: demod_rms={demod_rms:.3f}")
        print(f"  FM demod top energy at: ", end="")
        for k in top3:
            print(f"{fm_freqs[k]/1e3:.1f}kHz ({fm_fft[k]:.2e})", end="  ")
        print()
        print(f"  FM demod spectrum (should have peak ~4800 Hz for preamble):")
        for f_hz in [0, 1000, 2400, 4800, 7200, 9600, 14400, 19200, 28800, 48000]:
            if f_hz > SAMPLE_RATE//2: continue
            idx = np.argmin(np.abs(fm_freqs - f_hz))
            pwr = fm_fft[idx]
            bar = '#' * min(int(pwr/fm_fft.max()*40), 40)
            print(f"    {f_hz:6d} Hz: {pwr:.2e} {bar}")

    # Show autocorrelation of squared FM demod output (to find symbol rate)
    bb_f15 = lfilter(lpf15, 1.0, bb)
    freq_n = np.angle(bb_f15[1:] * np.conj(bb_f15[:-1]))
    freq_sq = freq_n**2
    # Use partial autocorrelation
    center = len(freq_sq) // 2
    chunk = freq_sq[center:center+5000]
    autocorr = np.correlate(chunk - chunk.mean(), chunk - chunk.mean(), mode='full')
    autocorr = autocorr[len(autocorr)//2:][:500]
    print(f"\n  Autocorr of (FM demod)^2 (peaks indicate symbol period):")
    # Show every 10 lags
    top_lags = np.argsort(autocorr[1:])[-5:]+1
    print(f"  Top 5 autocorr lags: {sorted(top_lags)} (SPS=100 expected)")

    break

print("Done.")
