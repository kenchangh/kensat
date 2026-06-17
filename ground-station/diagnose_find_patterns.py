#!/usr/bin/env python3
"""
Find ALL 16-bit patterns that appear in signal buffers, report frequency.
Helps identify the actual sync word being transmitted.
"""
import sys
import numpy as np
import adi
from scipy.signal import firwin, lfilter, fftconvolve
from collections import Counter

PLUTO_URI = "ip:192.168.3.1"
CENTER_FREQ = 437_080_000
PLUTO_XO_CORRECTION = 20800
LO_OFFSET = 100_000
SAMPLE_RATE = 960_000
BUF_SIZE = 65536
FREQ_DEVIATION = 7200
DATA_RATE = 9600
SPS = SAMPLE_RATE // DATA_RATE  # 100

def gaussian_filter(bt, sps, span=2):
    n_taps = int(2 * span * sps) + 1
    t = np.arange(n_taps) / sps - span
    alpha = np.sqrt(2 * np.log(2)) / bt
    h = np.sqrt(2 * np.pi) / alpha * np.exp(-2 * (np.pi * t / alpha) ** 2)
    h /= np.sum(h)
    return h

def carrier_lock(iq):
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

def demod(iq, fc, lpf):
    n = len(iq)
    bb = iq * np.exp(-1j * 2 * np.pi * fc * np.arange(n) / SAMPLE_RATE)
    bb_filt = lfilter(lpf, 1.0, bb)
    freq = np.angle(bb_filt[1:] * np.conj(bb_filt[:-1]))
    freq_norm = freq * SAMPLE_RATE / (2 * np.pi * FREQ_DEVIATION)
    amp = np.abs(bb_filt[1:])
    amp_thresh = 0.05 * np.median(amp)
    freq_norm[amp < amp_thresh] = 0.0
    return np.clip(freq_norm, -3.0, 3.0)

sdr = adi.Pluto(PLUTO_URI)
sdr.rx_lo = CENTER_FREQ + PLUTO_XO_CORRECTION - LO_OFFSET
sdr.rx_rf_bandwidth = SAMPLE_RATE
sdr.gain_control_mode_chan0 = 'manual'
sdr.rx_hardwaregain_chan0 = 60
sdr.sample_rate = SAMPLE_RATE
sdr.rx_buffer_size = BUF_SIZE

print("Flushing...")
for _ in range(5):
    sdr.rx()

lpf = firwin(501, 15000 / (SAMPLE_RATE / 2))
g = gaussian_filter(0.5, SPS, span=2)

print("Waiting for signal. Fire 'x' now...")

# Collect 16-bit patterns from all signal buffers across both polarities
# Sample at every timing offset t=0..SPS-1
pattern_counter = Counter()
byte_counter_after = Counter()  # counts bytes that follow each pattern appearance

sig_bufs = 0
all_fd = []

for buf_idx in range(500):
    iq = sdr.rx().astype(np.complex128)
    iq_rms = np.sqrt(np.mean(np.abs(iq) ** 2))

    if iq_rms < 30:
        if buf_idx % 30 == 0:
            print(f"  buf {buf_idx}: quiet iq_rms={iq_rms:.1f}")
            sys.stdout.flush()
        if all_fd and iq_rms < 20:
            print(f"Signal ended. Analyzing {sig_bufs} signal buffers...")
            break
        continue

    fc = carrier_lock(iq)
    fd = demod(iq, fc, lpf)
    fd_g = fftconvolve(fd, g, mode='same')

    print(f"  SIGNAL buf={buf_idx} iq_rms={iq_rms:.0f} fc={fc/1e3:.2f}kHz")
    sys.stdout.flush()
    all_fd.append(fd_g)
    sig_bufs += 1

    if sig_bufs >= 10:
        print("Captured 10 signal buffers.")
        break

if not all_fd:
    print("No signal captured.")
    sys.exit(1)

full_fd = np.concatenate(all_fd)
print(f"Total Gaussian-filtered samples: {len(full_fd)} ({len(full_fd)/SAMPLE_RATE*1000:.1f}ms)")

# For the optimal timing offset (brute-force all t=0..SPS-1):
# find all 16-bit patterns and tabulate counts, weighted by timing offset
# Use only t=0 for speed (representative)
print("\nScanning 16-bit patterns at t=0 (Gaussian-filtered, both polarities)...")
for polarity in [1.0, -1.0]:
    centers = np.arange(SPS // 2, len(full_fd), SPS)
    bits = (full_fd[centers] * polarity > 0).astype(np.uint8)
    N = len(bits)
    for i in range(N - 16):
        word = int(''.join(str(b) for b in bits[i:i+16]), 2)
        pattern_counter[word] += 1

print("\nTop 30 most frequent 16-bit patterns (both polarities, t=0):")
for word, count in pattern_counter.most_common(30):
    # Hamming distance to 0x2DD4
    xor = word ^ 0x2DD4
    ham = bin(xor).count('1')
    flag = ' ← SYNC 0x2DD4' if ham == 0 else (f' (ham={ham} to 0x2DD4)' if ham <= 4 else '')
    print(f"  0x{word:04X} = {word:016b}  count={count:4d}{flag}")

# Also scan at optimal timing
print("\nNow scanning ALL timing offsets for patterns with count >5...")
best_word_counts = Counter()
for t in range(SPS):
    centers = np.arange(t + SPS // 2, len(full_fd), SPS)
    bits = (full_fd[centers] > 0).astype(np.uint8)  # polarity +1 only
    N = len(bits)
    for i in range(N - 16):
        word = int(''.join(str(b) for b in bits[i:i+16]), 2)
        best_word_counts[word] += 1

print("Top 30 patterns across all timing offsets (pol=+1):")
for word, count in best_word_counts.most_common(30):
    xor = word ^ 0x2DD4
    ham = bin(xor).count('1')
    flag = ' ← 0x2DD4' if ham == 0 else (f' ham={ham} to 0x2DD4' if ham <= 4 else '')
    xor2 = word ^ (~0x2DD4 & 0xFFFF)  # inverted
    ham2 = bin(xor2).count('1')
    flag2 = f' (ham={ham2} to ~0x2DD4=0x{~0x2DD4&0xFFFF:04X})' if ham2 <= 4 else ''
    print(f"  0x{word:04X}  count={count:6d}{flag}{flag2}")

print("\nDone.")
