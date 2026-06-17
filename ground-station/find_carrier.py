#!/usr/bin/env python3
"""Find actual Si4463 carrier frequency vs Pluto DC spike."""
import adi, numpy as np, sys

sdr = adi.Pluto("ip:192.168.3.1")
sdr.sample_rate = 960000
sdr.rx_rf_bandwidth = 200000
sdr.rx_lo = 437_080_000 + 26000 - 100000
sdr.rx_buffer_size = 65536
sdr.gain_control_mode_chan0 = "manual"
sdr.rx_hardwaregain_chan0 = 40
for _ in range(5): sdr.rx()

noise_fft_sum = None
for i in range(5):
    iq = sdr.rx().astype(np.complex128)
    fft = np.abs(np.fft.fftshift(np.fft.fft(iq)))**2
    noise_fft_sum = fft if noise_fft_sum is None else noise_fft_sum + fft
    print(f"silence buf {i}: iq_rms={np.sqrt(np.mean(np.abs(iq)**2)):.1f}")
noise_baseline = noise_fft_sum / 5
freqs = np.fft.fftshift(np.fft.fftfreq(65536, 1/960000))
mask = (freqs > 50000) & (freqs < 200000)
pk = np.argmax(noise_baseline[mask])
print(f"Noise baseline peak: {freqs[mask][pk]/1e3:.1f} kHz pwr={noise_baseline[mask][pk]:.0f}")
print("Capturing (looking for signal above noise floor)...")
sys.stdout.flush()
for i in range(180):
    iq = sdr.rx().astype(np.complex128)
    fft = np.abs(np.fft.fftshift(np.fft.fft(iq)))**2
    diff = fft - noise_baseline
    peak_diff = diff[mask].max()
    peak_freq = freqs[mask][np.argmax(diff[mask])]
    iq_rms = np.sqrt(np.mean(np.abs(iq)**2))
    if peak_diff > noise_baseline[mask].max() * 0.5:
        print(f"  buf {i:3d}: ** SIGNAL ** at {peak_freq/1e3:.1f}kHz  excess={peak_diff:.0f}  iq_rms={iq_rms:.1f}")
    elif i % 15 == 0:
        print(f"  buf {i:3d}: quiet   iq_rms={iq_rms:.1f}  max_diff={peak_diff:.0f} at {peak_freq/1e3:.1f}kHz")
    sys.stdout.flush()
print("Done.")
