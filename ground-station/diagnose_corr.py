#!/usr/bin/env python3
"""
Correlation diagnostic: shows peak correlation values during signal buffers.
Helps calibrate the threshold and check if sync word is being found.
Tests multiple baud rates (2400, 4800, 9600) in case actual rate differs.
"""
import sys
import time
import numpy as np
import adi
from scipy.signal import firwin, lfilter, oaconvolve, fftconvolve

PLUTO_URI = "ip:192.168.3.1"
CENTER_FREQ = 437_080_000
PLUTO_XO_CORRECTION = 20800
LO_OFFSET = 100_000
SAMPLE_RATE = 960_000
BUF_SIZE = 65536
FREQ_DEVIATION = 7200
SYNC_WORD = 0x2DD4

def gaussian_filter(bt, sps, span=2):
    n_taps = int(2 * span * sps) + 1
    t = np.arange(n_taps) / sps - span
    alpha = np.sqrt(2 * np.log(2)) / bt
    h = np.sqrt(2 * np.pi) / alpha * np.exp(-2 * (np.pi * t / alpha) ** 2)
    h /= np.sum(h)
    return h

def demod(iq, fc, lpf):
    n = len(iq)
    bb = iq * np.exp(-1j * 2 * np.pi * fc * np.arange(n) / SAMPLE_RATE)
    bb_filt = lfilter(lpf, 1.0, bb)
    freq = np.angle(bb_filt[1:] * np.conj(bb_filt[:-1]))
    freq_norm = freq * SAMPLE_RATE / (2 * np.pi * FREQ_DEVIATION)
    # Amplitude gate
    amp = np.abs(bb_filt[1:])
    amp_thresh = 0.05 * np.median(amp)
    freq_norm[amp < amp_thresh] = 0.0
    return np.clip(freq_norm, -3.0, 3.0)

def carrier_lock(iq):
    """Midpoint of the two GFSK spectral peaks (carrier ± deviation).
    Robust to data bit asymmetry unlike power-weighted centroid."""
    n = len(iq)
    fft_data = np.fft.fftshift(np.fft.fft(iq))
    freqs = np.fft.fftshift(np.fft.fftfreq(n, 1.0 / SAMPLE_RATE))
    power = np.abs(fft_data) ** 2
    mask_lo = (freqs > 70000) & (freqs < LO_OFFSET)
    mask_hi = (freqs > LO_OFFSET) & (freqs < 130000)
    if not mask_lo.any() or not mask_hi.any():
        return float(LO_OFFSET)
    f_lo = float(freqs[mask_lo][np.argmax(power[mask_lo])])
    f_hi = float(freqs[mask_hi][np.argmax(power[mask_hi])])
    if abs(f_lo - LO_OFFSET) > 15000 or abs(f_hi - LO_OFFSET) > 15000:
        return float(LO_OFFSET)
    return (f_lo + f_hi) / 2.0

def corr_peak_for_rate(freq_window, data_rate):
    """Return max correlation and its index for a given baud rate.
    Tests both with and without Gaussian matched filter to compare."""
    sps = SAMPLE_RATE // data_rate
    sync_bits = [(SYNC_WORD >> (15 - i)) & 1 for i in range(16)]
    preamble_bits = [1, 0] * 16
    search_bits = preamble_bits + sync_bits

    nrz = np.array([2.0 * b - 1.0 for b in search_bits])
    template = np.repeat(nrz, sps)

    # Option A: no Gaussian filter (raw FM demod × NRZ template)
    corr_raw = fftconvolve(freq_window, template[::-1], mode='valid')

    # Option B: Gaussian-filtered signal × NRZ template (old approach)
    g = gaussian_filter(0.5, sps, span=2)
    freq_matched = oaconvolve(freq_window, g, mode='same')
    corr_gauss = fftconvolve(freq_matched, template[::-1], mode='valid')

    # Option C: Gaussian-filtered signal × Gaussian-filtered template (matched)
    template_gauss = oaconvolve(template, g, mode='same')
    corr_matched = fftconvolve(freq_matched, template_gauss[::-1], mode='valid')

    template_energy = len(template)

    def best_ratio(corr):
        mx = np.max(corr)
        mn = np.min(corr)
        return max(mx, -mn) / template_energy

    return best_ratio(corr_raw), best_ratio(corr_gauss), best_ratio(corr_matched)

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

lpf = firwin(501, 15000 / (SAMPLE_RATE / 2))

print("Monitoring for signal. Fire 'x' now!")
print(f"{'Buf':>5} {'iq_rms':>8} {'fc(kHz)':>9} {'rms_demod':>10}  "
      f"{'raw9600':>10} {'gauss9600':>10} {'matched9600':>12}")
print("-" * 80)

prev_freq = None
sig_bufs_captured = 0

for buf_idx in range(300):
    iq = sdr.rx().astype(np.complex128)
    iq_rms = np.sqrt(np.mean(np.abs(iq) ** 2))

    if iq_rms > 100:
        fc = carrier_lock(iq)
        freq = demod(iq, fc, lpf)
        demod_rms = np.sqrt(np.mean(freq**2))

        # Build 2-buffer window with previous buffer
        if prev_freq is not None:
            window = np.concatenate([prev_freq, freq])
        else:
            window = freq

        raw9600, gauss9600, matched9600 = corr_peak_for_rate(window, 9600)

        print(f"{buf_idx:>5} {iq_rms:>8.0f} {fc/1e3:>9.2f} {demod_rms:>10.3f}  "
              f"raw={raw9600:.4f} gauss={gauss9600:.4f} matched={matched9600:.4f}")
        sys.stdout.flush()

        sig_bufs_captured += 1
        if sig_bufs_captured >= 20:
            print("\nCaptured 20 signal buffers, done.")
            break
    elif buf_idx % 20 == 0:
        print(f"{buf_idx:>5} {iq_rms:>8.1f} {'quiet':>9}")
        sys.stdout.flush()

    prev_freq = demod(iq, carrier_lock(iq), lpf) if iq_rms > 50 else None

print("Done.")
