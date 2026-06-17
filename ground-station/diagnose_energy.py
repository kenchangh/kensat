#!/usr/bin/env python3
"""
Non-coherent FSK energy detector.
Instead of FM discriminator, compute energy in mark/space bands.
Also shows raw FM demod waveform.
"""
import numpy as np, adi, sys
from scipy.signal import firwin, lfilter

PLUTO_URI = "ip:192.168.3.1"
CENTER_FREQ = 437_080_000
PLUTO_XO_CORRECTION = 20800
LO_OFFSET = 100_000
SAMPLE_RATE = 960_000
SPS = 100  # nominal

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
for b_val in target_bytes:
    for i in range(7, -1, -1):
        target_bits.append((b_val >> i) & 1)

print("Waiting for signal...")
lpf = firwin(501, 15000 / (SAMPLE_RATE / 2))

def energy_demod(iq, fc_mark, fc_space):
    """Non-coherent FSK: compute energy in mark/space bands."""
    n = len(iq)
    t = np.arange(n)
    # Bandpass around mark: mix down to DC at mark, LPF
    bb_mark = iq * np.exp(-1j * 2 * np.pi * fc_mark * t / SAMPLE_RATE)
    bb_space = iq * np.exp(-1j * 2 * np.pi * fc_space * t / SAMPLE_RATE)
    
    # LPF with 1 kHz bandwidth to isolate each tone
    lpf_narrow = firwin(101, 2000 / (SAMPLE_RATE / 2))
    mark_filt = lfilter(lpf_narrow, 1.0, bb_mark)
    space_filt = lfilter(lpf_narrow, 1.0, bb_space)
    
    # Envelope (instantaneous energy)
    mark_env = np.abs(mark_filt)**2
    space_env = np.abs(space_filt)**2
    
    return mark_env, space_env

found_any = 0
for buf_i in range(200):
    iq = sdr.rx().astype(np.complex128)
    rms = np.sqrt(np.mean(np.abs(iq)**2))
    if rms < 50:
        if buf_i % 20 == 0: print(f"  buf {buf_i}: quiet rms={rms:.1f}")
        continue

    n = len(iq)
    fft_data = np.fft.fftshift(np.fft.fft(iq))
    freqs = np.fft.fftshift(np.fft.fftfreq(n, 1.0/SAMPLE_RATE))
    
    # Find GFSK peaks
    mask = (freqs > 85000) & (freqs < 115000)
    pwr = np.abs(fft_data[mask])**2
    top2_idx = np.argsort(pwr)[-20:][::-1]
    top_freqs = freqs[mask][top2_idx]
    
    f1 = top_freqs[0]
    f2_cands = [f for f in top_freqs[1:] if abs(f - f1) > 2000]
    if not f2_cands:
        print(f"  buf {buf_i}: can't find two peaks, rms={rms:.0f}")
        continue
    f2 = f2_cands[0]
    fc_mark = max(f1, f2)
    fc_space = min(f1, f2)
    fdev_measured = (fc_mark - fc_space) / 2
    
    print(f"\n=== Buf {buf_i}: rms={rms:.0f}, mark={fc_mark/1e3:.2f}kHz, space={fc_space/1e3:.2f}kHz, fdev={fdev_measured:.0f}Hz")

    # Show raw FM demod waveform first 300 samples
    bb = iq * np.exp(-1j * 2 * np.pi * ((fc_mark+fc_space)/2) * np.arange(n) / SAMPLE_RATE)
    bb_filt = lfilter(lpf, 1.0, bb)
    freq_raw = np.angle(bb_filt[1:] * np.conj(bb_filt[:-1]))
    freq_norm = freq_raw * SAMPLE_RATE / (2 * np.pi * fdev_measured)
    
    print(f"  FM demod RMS (normalized to measured fdev): {np.sqrt(np.mean(freq_norm**2)):.3f}")
    print(f"  Raw FM demod waveform (samples 500-800, ~3 symbols):")
    vals = freq_norm[500:800:10]  # Every 10th sample in range 500-800
    print("  " + " ".join(f"{v:+.2f}" for v in vals))
    print(f"  At symbol centers (SPS=100, starting at offset 550): {[f'{freq_norm[550 + k*100]:+.3f}' for k in range(5)]}")

    # NON-COHERENT ENERGY DETECTION
    mark_env, space_env = energy_demod(iq, fc_mark, fc_space)
    
    # Try all bit_offsets and look for target
    found_this_buf = False
    for bit_offset in range(SPS):
        centers = np.arange(bit_offset + SPS//2 + 100, n - SPS, SPS)  # +100 for filter settling
        if len(centers) < 50: continue
        bits = [1 if mark_env[c] > space_env[c] else 0 for c in centers]
        
        # Search for target
        L = len(target_bits)
        for start in range(len(bits) - L + 1):
            if bits[start:start+L] == target_bits:
                print(f"  *** FOUND 'KO6IKZ' via energy detect! bit_offset={bit_offset}, start={start}")
                payload = bits[start:start+100]
                print(f"      Bits: {payload[:88]}")
                payload_bytes = bytes([sum(payload[j*8+k] << (7-k) for k in range(8)) for j in range(len(payload)//8)])
                print(f"      Bytes: {payload_bytes}")
                found_this_buf = True
                found_any += 1
        
        if found_this_buf:
            break

    if not found_this_buf:
        # Show mark/space ratio at some sample centers
        sample_marks = [mark_env[500 + k*100] for k in range(5)]
        sample_spaces = [space_env[500 + k*100] for k in range(5)]
        print(f"  Energy detection NOT found. Sample mark/space energies:")
        for m, s in zip(sample_marks, sample_spaces):
            ratio = m/s if s > 0 else float('inf')
            print(f"    mark={m:.2e} space={s:.2e} ratio={ratio:.2f}")

    if found_any >= 2 or buf_i > 30:
        break

print("\nDone.")
