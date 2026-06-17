#!/usr/bin/env python3
"""
Pluto SDR GFSK receiver for kensat_radio RF board.

Receives Si4463-compatible GFSK packets at 437.08 MHz transmitted
by the Si4463 on the RF board (in TX mode via 'x' command).

Packet format (matches Si4463 config):
    [Preamble 4B] [Sync 0x2DD4] [Len 1B] [Payload ≤64B] [CRC-16 2B]

Modulation:
    GFSK, 9600 bps, ±4.8 kHz deviation, BT=0.5

Usage:
    python pluto_rx.py [--rx-gain 50] [--duration 30]

Requirements:
    pip install pyadi-iio numpy scipy
"""

import argparse
import struct
import sys
import time

import adi
import numpy as np
from scipy.signal import lfilter

# ============================================================
# RF Configuration (must match Si4463 / pluto_tx.py)
# ============================================================

PLUTO_URI = "ip:192.168.3.1"
CENTER_FREQ = 437_080_000       # 437.08 MHz
PLUTO_XO_CORRECTION = 22400     # Pluto crystal is ~22.4 kHz low (~51 ppm)
DATA_RATE = 9600                # bps
FREQ_DEVIATION = 4800           # ±4.8 kHz
SAMPLE_RATE = 960_000           # 960 kHz
SYNC_WORD = 0x2DD4              # 16-bit sync word
MAX_PACKET_LEN = 64

SAMPLES_PER_SYMBOL = SAMPLE_RATE / DATA_RATE  # 100

CALLSIGN = "KO6IKZ"


# ============================================================
# CRC-16 ITU-T (polynomial 0x1021, seed 0xFFFF)
# ============================================================

def crc16_itu_t(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc = crc << 1
            crc &= 0xFFFF
    return crc


# ============================================================
# GFSK Demodulator
# ============================================================

def iq_lowpass(iq: np.ndarray) -> np.ndarray:
    """Low-pass filter IQ to isolate signal near baseband.
    Must be applied BEFORE FM demod to reject out-of-band interferers."""
    from scipy.signal import firwin, lfilter
    # 25 kHz cutoff — passes ±(deviation + bitrate/2) = ±9.6 kHz
    # with margin, while rejecting interferers at >50 kHz offset.
    num_taps = 201
    cutoff = 25000 / (SAMPLE_RATE / 2)
    lpf = firwin(num_taps, cutoff)
    iq_filt = lfilter(lpf, 1.0, iq)
    return iq_filt


def fm_demodulate(iq: np.ndarray) -> np.ndarray:
    """FM discriminator with soft clipping to suppress noise.

    Noise-only FM demod produces ±pi/sample → ±31.83 after scaling,
    which dominates correlation.  Soft-clipping to ±2 caps noise while
    preserving the ±1 signal, so correlation works even at low SNR.
    """
    # Differential phase: angle(iq[n] * conj(iq[n-1]))
    diff = iq[1:] * np.conj(iq[:-1])
    freq = np.angle(diff)

    # Normalize: scale so ±deviation maps to ±1.
    scale = SAMPLE_RATE / (2 * np.pi * FREQ_DEVIATION)
    freq *= scale

    # Soft clip: cap FM output to ±2.  Real GFSK signal is ±1;
    # noise is ±31.83.  Clipping to ±2 reduces noise energy by ~16×
    # while preserving the signal intact.
    freq = np.clip(freq, -2.0, 2.0)

    return freq


def bits_to_bytes(bits: list) -> bytes:
    """Convert bit list to bytes (MSB first per byte)."""
    out = []
    for i in range(0, len(bits) - 7, 8):
        byte = 0
        for j in range(8):
            byte = (byte << 1) | bits[i + j]
        out.append(byte)
    return bytes(out)


def find_sync_and_decode(freq: np.ndarray, sps: int) -> list:
    """Find sync words in FM-demodulated signal and extract packets.

    Returns list of (payload_bytes, crc_ok) tuples.
    """
    # Slice to bits using sign of frequency deviation
    # Sample at symbol centers — first downsample by taking every sps-th sample
    # Use a simple clock recovery: find peaks of correlation with sync word

    # Convert sync word to expected bit pattern (NRZ: 0→-1, 1→+1)
    sync_bits = []
    for i in range(15, -1, -1):
        sync_bits.append((SYNC_WORD >> i) & 1)

    # Prepend 2 bytes of preamble for robust correlation (32+16=48 bits)
    preamble_bits = [1, 0] * 16  # 2 bytes of 0xAA
    search_bits = preamble_bits + sync_bits  # 48 bits

    # Create expected waveform for correlation
    nrz = np.array([2.0 * b - 1.0 for b in search_bits])
    template = np.repeat(nrz, sps)

    # Correlate
    corr = np.correlate(freq, template, mode='valid')

    # Find peaks above threshold.
    # With 48 bits × 100 sps, a perfect match gives ~4800.
    # Require at least 70% correlation to reject false positives.
    min_threshold = len(template) * 0.15
    threshold = max(np.max(np.abs(corr)) * 0.7, min_threshold)
    if np.max(np.abs(corr)) < min_threshold:
        return []  # no signal

    # Also check negative correlation (inverted polarity)
    abs_corr = np.abs(corr)
    peaks = []
    min_gap = sps * 50  # minimum gap between packets

    i = 0
    while i < len(abs_corr):
        if abs_corr[i] > threshold:
            # Find local maximum
            j = i
            while j < len(abs_corr) - 1 and abs_corr[j + 1] > abs_corr[j]:
                j += 1
            peaks.append((j, corr[j] > 0))
            i = j + min_gap
        else:
            i += 1

    # For each sync position, extract bits
    packets = []
    for peak_pos, is_positive in peaks:
        # Data starts after the search pattern
        data_start = peak_pos + len(template)

        # We need at least 1 byte (length) + up to 64 payload + 2 CRC = 67 bytes = 536 bits
        max_bits = (1 + MAX_PACKET_LEN + 2) * 8
        samples_needed = max_bits * sps
        if data_start + samples_needed > len(freq):
            continue

        # Extract bits by sampling at symbol centers
        polarity = 1.0 if is_positive else -1.0
        raw_bits = []
        for b in range(max_bits):
            center = data_start + int(b * sps + sps / 2)
            if center >= len(freq):
                break
            val = freq[center] * polarity
            raw_bits.append(1 if val > 0 else 0)

        if len(raw_bits) < 24:  # need at least length + some payload + CRC
            continue

        raw_bytes = bits_to_bytes(raw_bits)

        # First byte is length
        payload_len = raw_bytes[0]
        if payload_len == 0 or payload_len > MAX_PACKET_LEN:
            continue

        total_bytes = 1 + payload_len + 2  # len + payload + CRC
        if len(raw_bytes) < total_bytes:
            continue

        pkt_bytes = raw_bytes[:total_bytes]
        payload = pkt_bytes[1:1 + payload_len]
        crc_rx = (pkt_bytes[-2] << 8) | pkt_bytes[-1]

        # CRC covers length + payload
        crc_calc = crc16_itu_t(pkt_bytes[:1 + payload_len])
        crc_ok = (crc_rx == crc_calc)

        packets.append((payload, crc_ok, crc_calc, crc_rx, peak_pos))

    return packets


# ============================================================
# Main receiver
# ============================================================

def receive(uri: str, rx_gain: float, duration: float):
    print(f"Connecting to Pluto SDR at {uri}...")
    try:
        sdr = adi.Pluto(uri=uri)
    except Exception as e:
        print(f"ERROR: Cannot connect to Pluto SDR: {e}")
        sys.exit(1)

    # Offset the LO to avoid DC/LO leakage at baseband center.
    # The signal will appear at +LO_OFFSET Hz in baseband and we
    # digitally mix it back to 0 Hz before demodulation.
    LO_OFFSET = 100_000  # 100 kHz offset
    sdr.rx_lo = CENTER_FREQ + PLUTO_XO_CORRECTION - LO_OFFSET
    sdr.rx_rf_bandwidth = SAMPLE_RATE
    sdr.gain_control_mode_chan0 = 'manual'
    sdr.rx_hardwaregain_chan0 = rx_gain
    sdr.sample_rate = SAMPLE_RATE
    sdr.rx_buffer_size = 2**16  # ~68 ms per buffer at 960 kHz

    sps = int(round(SAMPLES_PER_SYMBOL))

    print(f"  Frequency   : {CENTER_FREQ / 1e6:.2f} MHz")
    print(f"  LO offset  : {LO_OFFSET / 1e3:.0f} kHz (actual LO: {sdr.rx_lo / 1e6:.3f} MHz)")
    print(f"  Data rate   : {DATA_RATE} bps")
    print(f"  Deviation   : ±{FREQ_DEVIATION} Hz")
    print(f"  Modulation  : GFSK (BT=0.5)")
    print(f"  RX gain     : {rx_gain} dB")
    print(f"  Sample rate : {SAMPLE_RATE / 1e3:.0f} kHz")
    print(f"  Samp/symbol : {sps}")
    print(f"  Sync word   : 0x{SYNC_WORD:04X}")
    print(f"  Duration    : {duration}s")
    print(f"  Buffer size : {sdr.rx_buffer_size} samples ({sdr.rx_buffer_size/SAMPLE_RATE*1000:.1f} ms)")
    print()

    # Flush initial buffers
    for _ in range(5):
        sdr.rx()

    pkt_count = 0
    buf_count = 0
    sample_offset = 0  # track continuous phase for digital mixer
    start = time.time()

    print("Listening... (Ctrl+C to stop)")
    print("-" * 60)

    try:
        while time.time() - start < duration:
            iq_raw = sdr.rx()
            buf_count += 1

            # Digital downconversion: mix signal from +LO_OFFSET to 0 Hz
            n = len(iq_raw)
            t = (np.arange(n) + sample_offset) / SAMPLE_RATE
            mixer = np.exp(-1j * 2 * np.pi * LO_OFFSET * t)
            iq = iq_raw.astype(np.complex128) * mixer
            sample_offset += n

            # Low-pass filter IQ to reject out-of-band interferers
            iq = iq_lowpass(iq)

            # FM demodulate
            freq = fm_demodulate(iq)

            # Signal level (RMS of IQ magnitude)
            sig_power = 20 * np.log10(np.mean(np.abs(iq)) + 1e-10)

            # Search for packets
            packets = find_sync_and_decode(freq, sps)

            # Debug: check correlation level for sync detection
            sync_bits_dbg = []
            for i in range(15, -1, -1):
                sync_bits_dbg.append((SYNC_WORD >> i) & 1)
            preamble_dbg = [1, 0] * 16
            search_dbg = preamble_dbg + sync_bits_dbg
            nrz_dbg = np.array([2.0 * b - 1.0 for b in search_dbg])
            template_dbg = np.repeat(nrz_dbg, sps)
            corr_dbg = np.correlate(freq, template_dbg, mode='valid')
            max_corr = np.max(np.abs(corr_dbg))
            freq_rms = np.sqrt(np.mean(freq**2))
            template_energy = len(template_dbg)

            # Print periodic status
            if buf_count % 15 == 0:
                elapsed = time.time() - start
                print(f"  [{elapsed:5.1f}s] buf={buf_count}, "
                      f"pkts={pkt_count}, sig={sig_power:.1f} dB, "
                      f"freq_rms={freq_rms:.4f}, "
                      f"corr_max={max_corr:.0f}/{template_energy}")

            for payload, crc_ok, crc_calc, crc_rx, pos in packets:
                pkt_count += 1
                elapsed = time.time() - start

                # ASCII representation
                ascii_str = ''.join(
                    chr(b) if 0x20 <= b <= 0x7E else '.'
                    for b in payload
                )

                crc_str = "OK" if crc_ok else f"FAIL (rx=0x{crc_rx:04X} calc=0x{crc_calc:04X})"

                print(f"\n  >>> PACKET #{pkt_count} at {elapsed:.1f}s")
                print(f"      Length : {len(payload)}")
                print(f"      Hex    : {payload.hex()}")
                print(f"      ASCII  : {ascii_str}")
                print(f"      CRC    : {crc_str}")
                print()

    except KeyboardInterrupt:
        print("\nStopping RX...")

    elapsed = time.time() - start
    print("-" * 60)
    print(f"RX stopped. {pkt_count} packets in {elapsed:.1f}s "
          f"({buf_count} buffers). 73 de {CALLSIGN}")


def main():
    parser = argparse.ArgumentParser(
        description="Pluto SDR GFSK RX for kensat_radio (437.08 MHz)"
    )
    parser.add_argument("--uri", type=str, default=PLUTO_URI,
                        help=f"Pluto SDR URI (default: {PLUTO_URI})")
    parser.add_argument("--rx-gain", type=float, default=50,
                        help="RX gain in dB (default: 50)")
    parser.add_argument("--duration", type=float, default=60,
                        help="Listen duration in seconds (default: 60)")
    args = parser.parse_args()

    receive(uri=args.uri, rx_gain=args.rx_gain, duration=args.duration)


if __name__ == "__main__":
    main()
