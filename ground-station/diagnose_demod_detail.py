#!/usr/bin/env python3
"""
Detailed demod inspection: show FM demod sampled at every half-symbol (50 samples)
to see if the preamble is visible as clean alternating ±1.
Also show the raw correlation trace around its peak.
"""
import sys
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
SYNC_WORD = 0xB42B
DATA_RATE = 9600
SPS = SAMPLE_RATE // DATA_RATE  # 100

def carrier_lock(iq):
    n = len(iq)
    fft_data = np.fft.fftshift(np.fft.fft(iq))
    freqs = np.fft.fftshift(np.fft.fftfreq(n, 1.0 / SAMPLE_RATE))
    mask = (freqs > 70000) & (freqs < 130000)
    if not mask.any():
        return float(LO_OFFSET)
    power = np.abs(fft_data[mask]) ** 2
    total = np.sum(power)
    if total == 0:
        return float(LO_OFFSET)
    return float(np.sum(freqs[mask] * power) / total)

def demod_full(iq, fc, lpf):
    n = len(iq)
    bb = iq * np.exp(-1j * 2 * np.pi * fc * np.arange(n) / SAMPLE_RATE)
    bb_filt = lfilter(lpf, 1.0, bb)
    freq = np.angle(bb_filt[1:] * np.conj(bb_filt[:-1]))
    freq_norm = freq * SAMPLE_RATE / (2 * np.pi * FREQ_DEVIATION)
    amp = np.abs(bb_filt[1:])
    amp_thresh = 0.05 * np.median(amp)
    freq_norm[amp < amp_thresh] = 0.0
    return np.clip(freq_norm, -3.0, 3.0), amp

def gaussian_filter_fn(bt, sps, span=2):
    n_taps = int(2 * span * sps) + 1
    t = np.arange(n_taps) / sps - span
    alpha = np.sqrt(2 * np.log(2)) / bt
    h = np.sqrt(2 * np.pi) / alpha * np.exp(-2 * (np.pi * t / alpha) ** 2)
    h /= np.sum(h)
    return h

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

print("Waiting for signal (fire 'x')...")
prev_iq = None
prev_fd = None
captured_group = []

for buf_idx in range(300):
    iq = sdr.rx().astype(np.complex128)
    iq_rms = np.sqrt(np.mean(np.abs(iq) ** 2))

    if iq_rms > 100:
        fc = carrier_lock(iq)
        fd, amp = demod_full(iq, fc, lpf)
        captured_group.append((buf_idx, iq_rms, fc, fd, amp, iq))
        print(f"  Signal buf {buf_idx}: iq_rms={iq_rms:.0f} fc={fc/1e3:.2f}kHz")
        sys.stdout.flush()
    else:
        if len(captured_group) >= 2:
            print(f"Captured {len(captured_group)} signal buffers. Analyzing...")
            break
        elif len(captured_group) > 0:
            pass  # might be partial end of packet
        if buf_idx % 20 == 0:
            print(f"  buf {buf_idx}: quiet iq_rms={iq_rms:.1f}")
            sys.stdout.flush()

if len(captured_group) < 2:
    print("Not enough signal captured")
    sys.exit(1)

# Take the STRONGEST signal buffer for analysis
best = max(captured_group, key=lambda x: x[1])
buf_idx, iq_rms, fc, fd, amp, iq = best
print(f"\nAnalyzing strongest buffer: buf={buf_idx} iq_rms={iq_rms:.0f} fc={fc/1e3:.2f}kHz")
print()

# === 1. Show demod at every HALF-SYMBOL (50 samples = 5ms each) ===
print(f"=== FM demod at every half-symbol (50 samples) ===")
print(f"  {'sample':>8} {'t(ms)':>7} {'demod':>8}  {'amp':>6}  {'bar'}")
for i in range(0, min(len(fd), SPS * 200), SPS // 2):  # first 200 symbols
    t_ms = i / SAMPLE_RATE * 1000
    d = fd[i]
    a = amp[i]
    # bar showing ±1 expected range
    pos = int((d + 1.5) * 20)
    pos = max(0, min(59, pos))
    bar = ' ' * pos + '|'
    print(f"  {i:>8} {t_ms:>7.2f} {d:>+8.3f}  {a:>6.1f}  {bar}")

print()

# === 2. Look for preamble pattern by correlating raw demod (no Gaussian) with template ===
print(f"=== Raw correlation (no Gaussian filter) vs preamble template ===")
sync_bits = [(SYNC_WORD >> (15 - i)) & 1 for i in range(16)]
preamble_bits = [1, 0] * 32   # 64 preamble bits
search_bits = preamble_bits + sync_bits  # 80 bits

nrz = np.array([2.0 * b - 1.0 for b in search_bits])
template = np.repeat(nrz, SPS)

corr_raw = fftconvolve(fd, template[::-1], mode='valid')
template_energy = len(template)

# Show correlation vs position
max_pos = np.argmax(corr_raw)
max_neg = np.argmin(corr_raw)
best_pos = max_pos if corr_raw[max_pos] > -corr_raw[max_neg] else max_neg
best_val = corr_raw[best_pos]
polarity = 1 if best_val > 0 else -1
best_ratio = abs(best_val) / template_energy

print(f"  Template: {len(search_bits)} bits = {len(template)} samples "
      f"(energy={template_energy})")
print(f"  Best raw corr: pos={best_pos} val={best_val:.0f} ratio={best_ratio:.4f}")
print(f"  Polarity: {polarity:+d}")
print()

# Show correlation around the peak
print(f"  Corr around peak (+/-500 samples):")
for i in range(max(0, best_pos-500), min(len(corr_raw), best_pos+500), 50):
    t = i / SAMPLE_RATE * 1000
    r = corr_raw[i] * polarity / template_energy
    bar = '#' * int(abs(r) * 40)
    print(f"    pos={i:6d} ({t:6.2f}ms) ratio={r:.4f} {bar}")

# === 3. Now with Gaussian filter ===
g = gaussian_filter_fn(0.5, SPS, span=2)
fd_gauss = oaconvolve(fd, g, mode='same')

corr_gauss = fftconvolve(fd_gauss, template[::-1], mode='valid')
max_pos_g = np.argmax(corr_gauss)
max_neg_g = np.argmin(corr_gauss)
best_pos_g = max_pos_g if corr_gauss[max_pos_g] > -corr_gauss[max_neg_g] else max_neg_g
best_val_g = corr_gauss[best_pos_g]
polarity_g = 1 if best_val_g > 0 else -1
best_ratio_g = abs(best_val_g) / template_energy

print()
print(f"=== Gaussian-filtered correlation ===")
print(f"  Best Gaussian corr: pos={best_pos_g} val={best_val_g:.0f} "
      f"ratio={best_ratio_g:.4f}")
print(f"  Polarity: {polarity_g:+d}")

print()

# === 4. Verify by reading bits at best position ===
if abs(best_ratio_g) > 0.05 or abs(best_ratio) > 0.05:
    best_overall = best_pos_g if best_ratio_g > best_ratio else best_pos
    use_gauss = best_ratio_g > best_ratio
    sig = fd_gauss if use_gauss else fd
    pol = polarity_g if use_gauss else polarity

    # The peak position is the end of the template. Data starts after that.
    data_start = best_overall + len(template)
    print(f"=== Bits at data_start={data_start} (after best peak) ===")
    for b in range(40):
        center = data_start + int(b * SPS + SPS / 2)
        if center >= len(sig):
            break
        bit = 1 if sig[center] * pol > 0 else 0
        val = sig[center] * pol
        print(f"  bit {b:3d}: {bit}  val={val:+.3f}")

print()
print("Done.")
