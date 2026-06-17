#!/usr/bin/env python3
"""
Capture one signal buffer and dump:
1. FFT spectrum to identify spectral peaks
2. Raw FM demod values across the full buffer (no gating)
3. Brute-force bit scan at optimal timing offset
"""
import sys
import numpy as np
import adi
from scipy.signal import firwin, lfilter, fftconvolve

PLUTO_URI = "ip:192.168.3.1"
CENTER_FREQ = 437_080_000
PLUTO_XO_CORRECTION = 20800
LO_OFFSET = 100_000
SAMPLE_RATE = 960_000
BUF_SIZE = 65536
FREQ_DEVIATION = 7200
DATA_RATE = 9600
SPS = SAMPLE_RATE // DATA_RATE  # 100
SYNC_WORD = 0x2DD4

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

print("Waiting for signal. Fire 'x' now...")

best_buf = None
best_rms = 0

for buf_idx in range(200):
    iq = sdr.rx().astype(np.complex128)
    iq_rms = np.sqrt(np.mean(np.abs(iq) ** 2))
    if iq_rms > 30:
        print(f"  Signal buf {buf_idx}: iq_rms={iq_rms:.0f}")
        if iq_rms > best_rms:
            best_rms = iq_rms
            best_buf = iq.copy()
    elif best_buf is not None and iq_rms < 20:
        break
    elif buf_idx % 30 == 0:
        print(f"  quiet buf={buf_idx} rms={iq_rms:.1f}")
        sys.stdout.flush()

if best_buf is None:
    print("No signal captured.")
    sys.exit(1)

iq = best_buf
print(f"\nAnalyzing strongest buffer (iq_rms={best_rms:.0f})")

# 1. FFT spectrum
fft_data = np.fft.fftshift(np.fft.fft(iq))
freqs = np.fft.fftshift(np.fft.fftfreq(len(iq), 1.0 / SAMPLE_RATE))
power = np.abs(fft_data) ** 2

# Find peaks in [70-130 kHz]
mask = (freqs > 70000) & (freqs < 130000)
f_sub = freqs[mask]
p_sub = power[mask]
# Find top 5 peaks
indices = np.argsort(p_sub)[::-1][:10]
print(f"\nFFT peaks in 70-130 kHz band:")
for idx in indices[:5]:
    print(f"  f={f_sub[idx]/1e3:.2f}kHz power={p_sub[idx]:.0f}")

# Estimate carrier from the two main peaks
from scipy.signal import find_peaks as fp
peaks_i, _ = fp(p_sub, height=np.max(p_sub)*0.1, distance=50)
if len(peaks_i) >= 2:
    # Sort by power, take top 2
    top2 = sorted(peaks_i, key=lambda x: p_sub[x], reverse=True)[:2]
    top2_f = sorted([f_sub[i] for i in top2])
    fc = (top2_f[0] + top2_f[1]) / 2
    dev = (top2_f[1] - top2_f[0]) / 2
    print(f"  Carrier estimate: {fc/1e3:.2f}kHz, deviation: ±{dev:.0f}Hz")
else:
    fc = 100000.0
    print(f"  Only {len(peaks_i)} peaks found, using fc=100kHz")

# 2. FM demodulate WITHOUT amplitude gating
n = len(iq)
bb = iq * np.exp(-1j * 2 * np.pi * fc * np.arange(n) / SAMPLE_RATE)
bb_filt = lfilter(lpf, 1.0, bb)
freq_raw = np.angle(bb_filt[1:] * np.conj(bb_filt[:-1]))
freq_norm = freq_raw * SAMPLE_RATE / (2 * np.pi * FREQ_DEVIATION)
# No amplitude gating, just clip
freq_clipped = np.clip(freq_norm, -3.0, 3.0)

# Amplitude profile
amp = np.abs(bb_filt[1:])
amp_median = np.median(amp)

print(f"\nFM demod stats (no gating):")
print(f"  amp_median={amp_median:.0f}, amp_max={np.max(amp):.0f}")
print(f"  fm_rms={np.sqrt(np.mean(freq_clipped**2)):.3f}")
print(f"  fm_mean={np.mean(freq_clipped):.3f}")

# 3. Show FM values at 1-symbol intervals (UNGATED) for 200 bit positions
# At various timing offsets
print(f"\nFM values at 1-samp/sym (no gating), first 50 bit positions:")
print(f"Format: [bit_index] t=T: val val val ... (across 5 timing offsets 0,20,40,60,80)")
for t in [0, 20, 40, 60, 75, 80]:
    centers = np.arange(t + SPS // 2, len(freq_clipped), SPS)
    vals = freq_clipped[centers[:50]]
    bits = (vals > 0).astype(int)
    bit_str = ''.join(str(b) for b in bits)
    # Hamming to 0x2DD4 across windows
    sync_ref = [(SYNC_WORD >> (15-i)) & 1 for i in range(16)]
    min_errs = 16
    for i in range(len(bits)-15):
        errs = sum(bits[i+j] != sync_ref[j] for j in range(16))
        min_errs = min(min_errs, errs)
    # Find where amplitude is high (signal region)
    amp_at_centers = amp[centers[:50]] if len(amp) > centers[49] else amp[centers[:len(centers)]]
    sig_indices = np.where(amp_at_centers > 0.3 * amp_median)[0]
    print(f"  t={t:2d}: {bit_str} (min_sync_err={min_errs}, sig_bits={len(sig_indices)})")

# 4. Show amplitude profile at 1-samp/sym
print(f"\nAmplitude at 1 samp/sym (t=50), first 100 values:")
centers = np.arange(50, len(amp), SPS)
amp_vals = amp[centers[:100]]
print(f"  max={np.max(amp_vals):.0f}, min={np.min(amp_vals):.0f}")
# Show regions with signal
above = np.where(amp_vals > 0.1 * np.max(amp_vals))[0]
if len(above) > 0:
    print(f"  Signal present at bit positions: {above[0]}..{above[-1]}")

# 5. Full brute-force sync scan (no gating)
print(f"\nBrute-force sync scan (no amplitude gating):")
sync_ref = np.array([(SYNC_WORD >> (15-i)) & 1 for i in range(16)], dtype=np.uint8)
found_any = False
for t in range(SPS):
    centers = np.arange(t + SPS // 2, len(freq_clipped), SPS)
    bits = (freq_clipped[centers] > 0).astype(np.uint8)
    N = len(bits)
    for i in range(N - 15):
        errs = int(np.sum(bits[i:i+16] != sync_ref))
        if errs <= 2:
            found_any = True
            sw = int(''.join(str(int(b)) for b in bits[i:i+16]), 2)
            # Show 8 bytes after
            nb = min(8, (N - i - 16) // 1)
            after = [int(np.packbits(bits[i+16+j*8:i+16+(j+1)*8])[0]) if i+16+(j+1)*8 <= N else -1 for j in range(nb)]
            after_str = ' '.join(f'0x{b:02X}' for b in after if b >= 0)
            print(f"  t={t:3d} i={i:4d} sync=0x{sw:04X} err={errs} data[0..7]=[{after_str}]")

if not found_any:
    print("  No sync word found with ≤2 errors!")
    # Show best match
    best_t, best_i, best_errs = 0, 0, 16
    for t in range(SPS):
        centers = np.arange(t + SPS // 2, len(freq_clipped), SPS)
        bits = (freq_clipped[centers] > 0).astype(np.uint8)
        N = len(bits)
        for i in range(N - 15):
            errs = int(np.sum(bits[i:i+16] != sync_ref))
            if errs < best_errs:
                best_errs, best_t, best_i = errs, t, i
    print(f"  Best match: t={best_t} i={best_i} errors={best_errs}")

print("\nDone.")
