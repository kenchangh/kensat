#!/usr/bin/env python3
"""
Brute-force sync word scanner with DC bias correction.
For every timing offset t in 0..SPS-1, sample the FM demod at 1 sps and
scan for the 16-bit sync word 0x2DD4.
Tests both raw and DC-bias-corrected FM demod.
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
DATA_RATE = 9600
SPS = SAMPLE_RATE // DATA_RATE  # 100
SYNC_WORD = 0x2DD4

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

def scan_for_sync(fd, sps, polarity):
    """Sample fd at every 'sps' samples starting at each offset 0..sps-1,
    scan for 16-bit sync word. Returns list of (offset, bit_start)."""
    results = []
    sync_bits = [(SYNC_WORD >> (15 - i)) & 1 for i in range(16)]

    for t in range(sps):
        centers = np.arange(t + sps // 2, len(fd), sps)
        bits = np.where(fd[centers] * polarity > 0, 1, 0)
        for i in range(len(bits) - 15):
            if all(bits[i + j] == sync_bits[j] for j in range(16)):
                results.append((t, i))
    return results

def decode_bits(fd, t, bit_start, n_bits, polarity, sps):
    """Decode n_bits starting at bit_start with timing offset t."""
    centers = np.arange(t + sps // 2, len(fd), sps)
    bits = np.where(fd[centers] * polarity > 0, 1, 0)
    return list(bits[bit_start:bit_start + n_bits])

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

print(f"Scanning for sync word 0x{SYNC_WORD:04X} = "
      f"{''.join(str((SYNC_WORD >> (15-i)) & 1) for i in range(16))}")
print("Fire 'x' and wait...")
print()

captures = []
for buf_idx in range(300):
    iq = sdr.rx().astype(np.complex128)
    iq_rms = np.sqrt(np.mean(np.abs(iq) ** 2))

    if iq_rms > 30:
        fc = carrier_lock(iq)
        fd = demod(iq, fc, lpf)
        captures.append((buf_idx, iq_rms, fc, fd))
        print(f"  Signal buf {buf_idx}: iq_rms={iq_rms:.0f} fc={fc/1e3:.3f}kHz")
        sys.stdout.flush()
    elif captures:
        print("Signal ended, analyzing...")
        break
    elif buf_idx % 20 == 0:
        print(f"  buf {buf_idx}: quiet iq_rms={iq_rms:.1f}")
        sys.stdout.flush()

if not captures:
    print("No signal captured.")
    sys.exit(1)

# Concatenate all captured buffers
all_fd = np.concatenate([c[3] for c in captures])
print(f"\nTotal samples: {len(all_fd)} ({len(all_fd)/SAMPLE_RATE*1000:.1f} ms)")

# DC bias correction: carrier_lock sometimes returns LO_OFFSET (100kHz) as fallback
# even when the true carrier is lower (~93-95 kHz), creating a negative DC bias
# in the FM demod. The preamble (alternating bits) should have zero mean.
# Subtract mean of non-zero, non-clipped samples to correct.
nonzero = (np.abs(all_fd) > 0.01) & (np.abs(all_fd) < 2.9)
dc_bias = float(np.mean(all_fd[nonzero])) if nonzero.any() else 0.0
print(f"FM demod DC bias: {dc_bias:+.4f} ({dc_bias*FREQ_DEVIATION:+.0f} Hz carrier error)")
all_fd_corr = all_fd - dc_bias
print()

found_total = 0
for label, fd_use in [("original", all_fd), ("DC-corrected", all_fd_corr)]:
    print(f"=== {label} ===")
    for polarity, polname in [(1, "+1"), (-1, "-1")]:
        results = scan_for_sync(fd_use, SPS, polarity)
        if not results:
            print(f"  pol={polname}: no exact sync match")
            continue
        for t, bit_pos in results:
            found_total += 1
            print(f"\n  SYNC pol={polname} timing_offset={t} bit_pos={bit_pos}")
            # Decode 1 + 11 + 2 = 14 bytes = 112 bits after sync
            after_bits = decode_bits(fd_use, t, bit_pos + 16, 112, polarity, SPS)
            print(f"  Bytes after sync:")
            for i in range(0, len(after_bits) - 7, 8):
                byte_val = int(''.join(str(b) for b in after_bits[i:i+8]), 2)
                c = chr(byte_val) if 0x20 <= byte_val <= 0x7E else '.'
                print(f"    [{i//8:2d}] 0x{byte_val:02X} = {c}")

            # Show raw demod at first 8 data bits (should be length byte 0x0B)
            centers = np.arange(t + SPS // 2, len(fd_use), SPS)
            data_start = bit_pos + 16
            print(f"  FM demod at first 8 data bits (expected length 0x0B = 00001011):")
            for j in range(8):
                idx = centers[data_start + j] if data_start + j < len(centers) else -1
                if idx >= 0 and idx < len(fd_use):
                    val = fd_use[idx] * polarity
                    print(f"    data[{j}]: val={val:+.3f} bit={1 if val>0 else 0}")

if found_total == 0:
    print("\n=== No exact sync matches. Trying ≤2 bit errors ===")
    sync_bits_ref = [(SYNC_WORD >> (15 - i)) & 1 for i in range(16)]
    for fd_use, label in [(all_fd, "orig"), (all_fd_corr, "corr")]:
        for polarity in [1, -1]:
            for t in range(0, SPS, 5):
                centers = np.arange(t + SPS // 2, len(fd_use), SPS)
                bits = np.where(fd_use[centers] * polarity > 0, 1, 0)
                for i in range(len(bits) - 15):
                    errors = sum(bits[i+j] != sync_bits_ref[j] for j in range(16))
                    if errors <= 2:
                        sw = int(''.join(str(bits[i+j]) for j in range(16)), 2)
                        print(f"  {label} pol={polarity:+d} off={t} bit={i}: "
                              f"0x{sw:04X} errors={errors}")

print("\nDone.")
