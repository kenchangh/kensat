#!/usr/bin/env python3
"""
Brute-force bit search: capture a buffer during Si4463 TX,
demodulate with various carrier offsets, and search for 'KO6IKZ' in the bits.
If found, reports exact offset and polarity.
"""
import numpy as np, adi, sys
from scipy.signal import firwin, lfilter, oaconvolve

PLUTO_URI = "ip:192.168.3.1"
CENTER_FREQ = 437_080_000
PLUTO_XO_CORRECTION = 20800
LO_OFFSET = 100_000
SAMPLE_RATE = 960_000
FREQ_DEVIATION = 4800  # will try multiple values
SPS = 100
SYNC_WORD = 0xB42B

sdr = adi.Pluto(PLUTO_URI)
sdr.rx_lo = CENTER_FREQ + PLUTO_XO_CORRECTION - LO_OFFSET
sdr.rx_rf_bandwidth = SAMPLE_RATE
sdr.gain_control_mode_chan0 = 'manual'
sdr.rx_hardwaregain_chan0 = 40
sdr.sample_rate = SAMPLE_RATE
sdr.rx_buffer_size = 65536
for _ in range(5): sdr.rx()

target_bytes = b'KO6IKZ'
target_bits = []
for b in target_bytes:
    for i in range(7, -1, -1):
        target_bits.append((b >> i) & 1)

print(f"Target bits for 'KO6IKZ': {target_bits}")
print(f"Waiting for Si4463 TX (fire 'x' now)...")

lpf = firwin(501, 15000 / (SAMPLE_RATE / 2))

def demod_and_search(iq, fc, fdev_norm):
    """Returns demod bits and searches for target."""
    n = len(iq)
    bb = iq * np.exp(-1j * 2 * np.pi * fc * np.arange(n) / SAMPLE_RATE)
    bb_filt = lfilter(lpf, 1.0, bb)
    freq = np.angle(bb_filt[1:] * np.conj(bb_filt[:-1]))
    freq_norm = freq * SAMPLE_RATE / (2 * np.pi * fdev_norm)
    
    # Try both polarities, all bit offsets
    results = []
    for polarity in [1, -1]:
        for bit_offset in range(SPS):
            centers = np.arange(bit_offset + SPS//2, len(freq_norm) - SPS, SPS)
            if len(centers) < 50:
                continue
            bits = [1 if freq_norm[c] * polarity > 0 else 0 for c in centers]
            
            # Search for target_bits pattern
            L = len(target_bits)
            for start in range(len(bits) - L + 1):
                if bits[start:start+L] == target_bits:
                    results.append((polarity, bit_offset, start, bits[start:start+100]))
    return freq_norm, results

# Collect multiple signal buffers
iq_buffers = []
for i in range(200):
    iq = sdr.rx().astype(np.complex128)
    rms = np.sqrt(np.mean(np.abs(iq)**2))
    if rms < 50:
        if i % 20 == 0: print(f"  buf {i}: quiet rms={rms:.1f}")
        continue
    print(f"  SIGNAL buf {i}: rms={rms:.0f}")
    iq_buffers.append((i, iq, rms))
    if len(iq_buffers) >= 5:
        break

if not iq_buffers:
    print("No signal found.")
    sys.exit(1)

print(f"\nGot {len(iq_buffers)} signal buffers. Searching...")

# Try multiple carrier estimates and frequency deviations
for buf_idx, iq, rms in iq_buffers:
    n = len(iq)
    fft_data = np.fft.fftshift(np.fft.fft(iq))
    freqs = np.fft.fftshift(np.fft.fftfreq(n, 1.0/SAMPLE_RATE))

    # Centroid fc
    mask = (freqs > 70000) & (freqs < 130000)
    power = np.abs(fft_data[mask])**2
    fc_centroid = float(np.sum(freqs[mask] * power) / np.sum(power))

    # Find top two peaks in 85-115 kHz (GFSK sidebands)
    mask2 = (freqs > 85000) & (freqs < 115000)
    pwr2 = np.abs(fft_data[mask2])**2
    sorted_idx = np.argsort(pwr2)[::-1]
    top_freqs = freqs[mask2][sorted_idx[:20]]
    
    # Cluster the top frequencies into two groups
    # First peak: highest
    f1 = top_freqs[0]
    # Second peak: highest that's >2 kHz away from f1
    f2_candidates = [f for f in top_freqs[1:] if abs(f - f1) > 2000]
    f2 = f2_candidates[0] if f2_candidates else fc_centroid * 2 - f1
    
    fc_midpoint = (f1 + f2) / 2
    fdev_measured = abs(f1 - f2) / 2

    print(f"\n=== Buf {buf_idx}: rms={rms:.0f} ===")
    print(f"  fc_centroid={fc_centroid/1e3:.2f}kHz, peaks: {f1/1e3:.2f}kHz, {f2/1e3:.2f}kHz")
    print(f"  fc_midpoint={fc_midpoint/1e3:.3f}kHz, fdev_measured={fdev_measured/1e3:.3f}kHz")

    # Try with different carrier frequencies and deviation values
    for fc_try, fc_name in [(fc_centroid, 'centroid'), (fc_midpoint, 'midpoint'), (100000, 'nominal')]:
        for fdev_try, fdev_name in [(4800, '4800'), (fdev_measured, f'{fdev_measured:.0f}')]:
            freq_norm, results = demod_and_search(iq, fc_try, fdev_try)
            if results:
                print(f"  *** FOUND 'KO6IKZ' with fc={fc_name}={fc_try/1e3:.2f}kHz, fdev={fdev_name}!")
                for polarity, bit_offset, start_bit, bits_segment in results[:2]:
                    print(f"      polarity={polarity:+d}, bit_offset={bit_offset}, start_bit={start_bit}")
                    # Try to decode the 11-byte payload
                    print(f"      Bits[{start_bit}:{start_bit+88}]: {bits_segment[:88]}")
                    payload_bits = bits_segment[:88]
                    payload_bytes = bytes([sum(payload_bits[j*8+k] << (7-k) for k in range(8)) for j in range(len(payload_bits)//8)])
                    print(f"      Payload: {payload_bytes}")
            else:
                demod_rms = np.sqrt(np.mean(freq_norm**2))
                print(f"  fc={fc_name}, fdev={fdev_name}: NOT FOUND, demod_rms={demod_rms:.3f}")

print("\nDone.")
