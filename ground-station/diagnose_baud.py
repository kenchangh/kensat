#!/usr/bin/env python3
"""
Baud rate measurement: captures FM-demodulated signal and measures
actual symbol rate via power spectrum + autocorrelation.
Also shows raw FM demod waveform to see if preamble is visible.
"""
import sys
import numpy as np
import adi
from scipy.signal import firwin, lfilter

PLUTO_URI = "ip:192.168.3.1"
CENTER_FREQ = 437_080_000
PLUTO_XO_CORRECTION = 20800
LO_OFFSET = 100_000
SAMPLE_RATE = 960_000
BUF_SIZE = 65536
FREQ_DEVIATION = 7200

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

def demod(iq, fc, lpf):
    n = len(iq)
    bb = iq * np.exp(-1j * 2 * np.pi * fc * np.arange(n) / SAMPLE_RATE)
    bb_filt = lfilter(lpf, 1.0, bb)
    freq = np.angle(bb_filt[1:] * np.conj(bb_filt[:-1]))
    freq_norm = freq * SAMPLE_RATE / (2 * np.pi * FREQ_DEVIATION)
    amp = np.abs(bb_filt[1:])
    amp_thresh = 0.05 * np.median(amp)
    freq_norm[amp < amp_thresh] = 0.0
    return np.clip(freq_norm, -3.0, 3.0), amp

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

print("Monitoring... fire 'x' now!")
print()

all_demod = []
all_amp = []
capture_count = 0
MAX_CAPTURE = 6  # capture 6 signal buffers

for buf_idx in range(300):
    iq = sdr.rx().astype(np.complex128)
    iq_rms = np.sqrt(np.mean(np.abs(iq) ** 2))

    if iq_rms > 30:
        fc = carrier_lock(iq)
        fd, amp = demod(iq, fc, lpf)
        all_demod.append(fd)
        all_amp.append(amp)
        capture_count += 1
        print(f"  Captured buf {buf_idx}: iq_rms={iq_rms:.0f} fc={fc/1e3:.2f}kHz "
              f"demod_rms={np.sqrt(np.mean(fd**2)):.3f}")
        sys.stdout.flush()

        if capture_count >= MAX_CAPTURE:
            print(f"\nCaptured {MAX_CAPTURE} signal buffers. Analyzing...")
            break
    elif buf_idx % 20 == 0:
        print(f"  buf {buf_idx}: quiet iq_rms={iq_rms:.1f}")
        sys.stdout.flush()

if not all_demod:
    print("No signal found!")
    sys.exit(1)

# Concatenate all captured demod buffers
concat = np.concatenate(all_demod)
concat_amp = np.concatenate(all_amp)

print(f"\nTotal captured samples: {len(concat)} ({len(concat)/SAMPLE_RATE*1000:.1f} ms)")
print(f"Demod stats: min={concat.min():.2f} max={concat.max():.2f} "
      f"mean={concat.mean():.3f} rms={np.sqrt(np.mean(concat**2)):.3f}")
print()

# === 1. Power spectrum of demod signal ===
# Find dominant frequency in demod (preamble alternation appears at fs/2 relative to symbol rate)
fft_demod = np.abs(np.fft.rfft(concat)) ** 2
freqs_demod = np.fft.rfftfreq(len(concat), 1.0 / SAMPLE_RATE)

# Focus on 100 Hz to 20 kHz (covers 200 bps to 40000 bps symbol rate range)
mask_audio = (freqs_demod >= 100) & (freqs_demod <= 20000)
fft_audio = fft_demod[mask_audio]
freqs_audio = freqs_demod[mask_audio]

top5_idx = np.argsort(fft_audio)[::-1][:5]
print("=== Power spectrum of FM-demod (top 5 peaks, 100-20kHz) ===")
for i in top5_idx:
    f = freqs_audio[i]
    p = fft_audio[i]
    # If this is the preamble alternation, symbol rate = 2 * f
    print(f"  {f:8.1f} Hz  power={p:.2e}  → if preamble alt: {2*f:.0f} baud")

print()

# === 2. Sample the demod every 10 ms and show actual values ===
print("=== Demod waveform samples (every 5ms) ===")
step = int(SAMPLE_RATE * 0.005)  # every 5ms = 4800 samples
print("  t(ms)   demod   amp")
for i in range(0, min(len(concat), int(SAMPLE_RATE * 0.3)), step):
    t_ms = i / SAMPLE_RATE * 1000
    d_val = concat[i]
    a_val = concat_amp[i] if i < len(concat_amp) else 0
    print(f"  {t_ms:6.1f}   {d_val:+.3f}  {a_val:.1f}")

print()

# === 3. Zero-crossing analysis for baud rate measurement ===
# Find zero crossings in demod (indicates transitions)
sign_changes = np.where(np.diff(np.sign(concat)))[0]

if len(sign_changes) > 2:
    gaps = np.diff(sign_changes)
    # Half-period = minimum gap between crossings (≈ SPS/2 for clean alternating signal)
    # Full period = SPS
    median_half_period = np.median(gaps)
    median_period = 2 * median_half_period
    measured_baud = SAMPLE_RATE / median_period
    print(f"=== Zero-crossing baud rate estimate ===")
    print(f"  Median half-period: {median_half_period:.0f} samples = {median_half_period/SAMPLE_RATE*1000:.3f} ms")
    print(f"  Median full period: {median_period:.0f} samples")
    print(f"  Estimated baud rate: {measured_baud:.0f} baud")
    print(f"  SPS at this rate: {SAMPLE_RATE/measured_baud:.0f}")

    # Histogram of gaps to see distribution
    common_gaps = {}
    for g in gaps:
        key = round(g / 50) * 50  # bucket to nearest 50
        common_gaps[key] = common_gaps.get(key, 0) + 1
    print(f"  Gap histogram (top 10 buckets):")
    for k, v in sorted(common_gaps.items(), key=lambda x: -x[1])[:10]:
        baud = SAMPLE_RATE / (k * 2) if k > 0 else 0
        print(f"    {k:5d} samples: count={v:4d}  → {baud:.0f} baud if full period = 2×gap")

print()
print("Done.")
