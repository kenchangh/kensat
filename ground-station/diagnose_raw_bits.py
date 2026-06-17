#!/usr/bin/env python3
"""
Dump raw FM demod values for captured packets.
For each signal buffer, find the correlation peak and dump:
- FM values (raw and Gaussian) at each sync-word bit center
- First 30 bytes after sync, at the best timing offset
- Try ALL 100 timing offsets and show which gives fewest sync errors
"""
import sys
import numpy as np
import adi
from scipy.signal import firwin, lfilter, fftconvolve

PLUTO_URI = "ip:192.168.3.1"
CENTER_FREQ = 437_080_000
PLUTO_XO_CORRECTION = 26000     # matches pluto_rx.py working value
LO_OFFSET = 100_000
SAMPLE_RATE = 960_000
BUF_SIZE = 65536
FREQ_DEVIATION = 4800           # matches Si4463 config ±4.8 kHz
DATA_RATE = 9600
SPS = SAMPLE_RATE // DATA_RATE  # 100
SYNC_WORD = 0x2DD4
MAX_PACKET_LEN = 64

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

sync_bits_ref = [(SYNC_WORD >> (15 - i)) & 1 for i in range(16)]
nrz = np.array([2.0 * b - 1.0 for b in sync_bits_ref])
template = np.repeat(nrz, SPS)

print("Waiting for signal. Fire 'x' now...")

freq_prev = None
found = 0

for buf_idx in range(500):
    iq = sdr.rx().astype(np.complex128)
    iq_rms = np.sqrt(np.mean(np.abs(iq) ** 2))

    if iq_rms < 30:
        if buf_idx % 20 == 0:
            print(f"  buf {buf_idx}: quiet iq_rms={iq_rms:.1f}")
            sys.stdout.flush()
        freq_prev = None
        continue

    fc = carrier_lock(iq)
    fd_raw = demod(iq, fc, lpf)
    fd_g = fftconvolve(fd_raw, g, mode='same')

    print(f"\nSIGNAL buf={buf_idx} iq_rms={iq_rms:.0f} fc={fc/1e3:.2f}kHz")
    sys.stdout.flush()

    # Build 2-buffer window
    if freq_prev is not None:
        window_raw = np.concatenate([freq_prev[0], fd_raw])
        window_g   = np.concatenate([freq_prev[1], fd_g])
    else:
        window_raw = fd_raw
        window_g   = fd_g
    freq_prev = (fd_raw, fd_g)

    # Correlation on Gaussian-filtered
    corr_g = fftconvolve(window_g, template[::-1], mode='valid')

    for polarity in [1.0, -1.0]:
        pc = corr_g * polarity
        peak_threshold = len(template) * 0.35
        peaks = np.where(pc > peak_threshold)[0]
        if len(peaks) == 0:
            continue

        # Keep only local maxima
        local_max = []
        i = 0
        while i < len(peaks):
            j = i
            while j < len(peaks) - 1 and peaks[j+1] == peaks[j] + 1:
                j += 1
            best = peaks[i:j+1][np.argmax(pc[peaks[i:j+1]])]
            local_max.append(best)
            i = j + 1

        for peak_pos in sorted(local_max, key=lambda x: pc[x], reverse=True)[:3]:
            corr_ratio = pc[peak_pos] / len(template)

            # Fine timing search: try all t_offsets
            best_errors = 17
            best_t = peak_pos
            errors_by_t = {}
            for t_offset in range(-SPS // 2, SPS // 2 + 1):
                ss = peak_pos + t_offset
                if ss < 0 or ss + 16 * SPS > len(window_g):
                    continue
                errs = 0
                for b in range(16):
                    c = ss + b * SPS + SPS // 2
                    bit = 1 if window_g[c] * polarity > 0 else 0
                    if bit != sync_bits_ref[b]:
                        errs += 1
                errors_by_t[t_offset] = errs
                if errs < best_errors:
                    best_errors = errs
                    best_t = ss

            # Show error histogram
            error_counts = [0]*17
            for e in errors_by_t.values():
                error_counts[min(e,16)] += 1

            print(f"  pol={polarity:+.0f} peak={peak_pos} corr={corr_ratio:.3f} "
                  f"best_err={best_errors} best_t={best_t}")
            print(f"  error_histogram: ", end='')
            for e in range(6):
                if error_counts[e] > 0:
                    print(f"err={e}:{error_counts[e]} ", end='')
            print()

            # Dump sync word bits at best timing (raw and Gaussian)
            print(f"  Sync word at best_t={best_t} (raw / gaussian):")
            sync_decoded_raw = 0
            sync_decoded_g = 0
            for b in range(16):
                c = best_t + b * SPS + SPS // 2
                if c >= len(window_raw):
                    break
                val_raw = window_raw[c] * polarity
                val_g   = window_g[c]   * polarity
                bit_raw = 1 if val_raw > 0 else 0
                bit_g   = 1 if val_g   > 0 else 0
                expected = sync_bits_ref[b]
                sync_decoded_raw = (sync_decoded_raw << 1) | bit_raw
                sync_decoded_g   = (sync_decoded_g   << 1) | bit_g
                err_r = '!' if bit_raw != expected else ' '
                err_g = '!' if bit_g   != expected else ' '
                print(f"    bit[{b:2d}] exp={expected} "
                      f"raw={val_raw:+.3f}→{bit_raw}{err_r} "
                      f"g={val_g:+.3f}→{bit_g}{err_g}")
            print(f"  Decoded: raw=0x{sync_decoded_raw:04X} g=0x{sync_decoded_g:04X}")

            # Dump first 20 data bytes after sync
            # Show MSB-first and LSB-first interpretations to identify bit order
            data_start = best_t + 16 * SPS
            print(f"  First 20 data bytes [MSB-first / LSB-first] gaussian, data_start={data_start}:")
            for byte_idx in range(20):
                bits_g = []
                for b in range(8):
                    c = data_start + (byte_idx * 8 + b) * SPS + SPS // 2
                    if c >= len(window_g):
                        break
                    bits_g.append(1 if window_g[c] * polarity > 0 else 0)
                if len(bits_g) < 8:
                    break
                # MSB-first: bits[0] is MSB
                msb_first = int(''.join(str(b) for b in bits_g), 2)
                # LSB-first: bits[0] is LSB
                lsb_first = sum(bits_g[b] << b for b in range(8))
                c_msb = chr(msb_first) if 0x20 <= msb_first <= 0x7E else '.'
                c_lsb = chr(lsb_first) if 0x20 <= lsb_first <= 0x7E else '.'
                print(f"    [{byte_idx:2d}] MSB=0x{msb_first:02X}({c_msb}) LSB=0x{lsb_first:02X}({c_lsb})"
                      f"  bits={''.join(str(b) for b in bits_g)}")

            found += 1
            if found >= 5:
                break
        if found >= 5:
            break
    if found >= 5:
        break

print("\nDone.")
