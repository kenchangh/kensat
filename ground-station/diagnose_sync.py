#!/usr/bin/env python3
"""
Sync word sniffer: captures preamble, decodes 40 bits after preamble end,
shows what the actual on-air sync word and first data bytes are.
No assumed sync word — find preamble via autocorrelation only.
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

print("Waiting for signal (fire 'x')...")
prev_fd = None

for buf_idx in range(300):
    iq = sdr.rx().astype(np.complex128)
    iq_rms = np.sqrt(np.mean(np.abs(iq) ** 2))

    if iq_rms > 30:
        fc = carrier_lock(iq)
        fd = demod(iq, fc, lpf)

        if prev_fd is not None:
            window = np.concatenate([prev_fd, fd])
        else:
            window = fd

        print(f"  buf {buf_idx}: iq_rms={iq_rms:.0f} fc={fc/1e3:.2f}kHz")

        # === Find preamble via preamble-only template (no sync) ===
        # Use a 64-bit preamble template (±1 alternating × SPS)
        pre_bits = [1, 0] * 32  # 64 preamble bits
        nrz_pre = np.array([2.0 * b - 1.0 for b in pre_bits])
        template_pre = np.repeat(nrz_pre, SPS)  # 6400 samples

        corr = fftconvolve(window, template_pre[::-1], mode='valid')

        best_pos_plus = np.argmax(corr)
        best_pos_minus = np.argmin(corr)

        if corr[best_pos_plus] > -corr[best_pos_minus]:
            best_pos = best_pos_plus
            polarity = 1
        else:
            best_pos = best_pos_minus
            polarity = -1

        best_ratio = abs(corr[best_pos]) / len(template_pre)

        if best_ratio > 0.10:
            # The template end = best_pos + len(template_pre) - 1
            # Data starts after the template end = best_pos + len(template_pre)
            preamble_end = best_pos + len(template_pre)
            print(f"  Preamble found at template_end={preamble_end} ratio={best_ratio:.4f} pol={polarity:+d}")

            # Decode 64 bits starting from preamble_end
            print(f"  Bits after preamble (raw, polarity={polarity:+d}):")
            bits = []
            for b in range(64):
                center = preamble_end + int(b * SPS + SPS // 2)
                if center >= len(window):
                    break
                val = window[center] * polarity
                bit = 1 if val > 0 else 0
                bits.append(bit)

            # Show the 64 bits
            bit_str = ''.join(str(b) for b in bits)
            print(f"  '{bit_str}'")

            # Convert to hex bytes (MSB first)
            print(f"  As hex bytes:")
            for i in range(0, len(bits) - 7, 8):
                byte_val = 0
                for j in range(8):
                    byte_val = (byte_val << 1) | bits[i + j]
                print(f"    Byte {i//8:2d}: 0x{byte_val:02X} = {chr(byte_val) if 0x20 <= byte_val <= 0x7E else '.'}")

            # Show demod values at these positions
            print(f"  Raw demod at first 32 bit positions:")
            for b in range(32):
                center = preamble_end + int(b * SPS + SPS // 2)
                if center >= len(window):
                    break
                val = window[center] * polarity
                print(f"    bit {b:2d}: val={val:+.3f}  {'1' if val>0 else '0'}")

            print()
            sys.stdout.flush()

        prev_fd = fd
    else:
        if buf_idx % 20 == 0:
            print(f"  buf {buf_idx}: quiet iq_rms={iq_rms:.1f}")
            sys.stdout.flush()
        prev_fd = demod(iq, carrier_lock(iq), lpf)

print("Done.")
