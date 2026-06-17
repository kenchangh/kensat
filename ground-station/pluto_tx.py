#!/usr/bin/env python3
"""
Pluto SDR GFSK transmitter for kensat_radio RF board.

Generates Si4463-compatible GFSK packets at 437.08 MHz and transmits
them via ADALM-Pluto SDR in cyclic mode. The Si4463 on the RF board
(in RX mode) will decode these packets.

Packet format (matches Si4463 config):
    [Preamble] [Sync 0x2DD4] [Len 1B] [Payload ≤64B] [CRC-16 2B]

Modulation:
    GFSK, 9600 bps, ±4.8 kHz deviation, BT=0.5

The transmitter builds a single IQ buffer containing N packets
separated by silence gaps, then transmits in cyclic (DMA loop) mode.
Non-cyclic mode has DMA startup transients that corrupt short bursts.

Amateur radio compliance:
    - 437.08 MHz (70cm band, 420–450 MHz)
    - Callsign KO6IKZ included in every packet
    - FCC Part 97

Usage:
    python pluto_tx.py --packets 10 --gap-ms 200 --tx-gain 0

Requirements:
    pip install pyadi-iio numpy scipy
"""

import argparse
import struct
import sys
import time

import adi
import numpy as np
from scipy.signal import firwin

# ============================================================
# RF Configuration (must match Si4463 config.h)
# ============================================================

PLUTO_URI = "ip:192.168.3.1"
CENTER_FREQ = 437_080_000       # 437.08 MHz
PLUTO_XO_CORRECTION = 26000     # Pluto crystal offset (empirically tuned)
DATA_RATE = 9600                # bps
FREQ_DEVIATION = 4800           # ±4.8 kHz
BT = 0.5                       # Gaussian filter bandwidth-time product
PREAMBLE_BYTES = 128           # 128 bytes of 0xAA (2048 bits for reliable Si4463 AFC/BCR lock)
SYNC_WORD = 0x2DD4              # 16-bit sync word
MAX_PACKET_LEN = 64             # max payload bytes

CALLSIGN = "KO6IKZ"

# Pluto SDR settings
SAMPLE_RATE = 960_000           # 960 kHz — 100 samples/symbol
TX_GAIN_DB = -10                # dB attenuation (conservative default)

# Derived
SAMPLES_PER_SYMBOL = SAMPLE_RATE / DATA_RATE  # 100


# ============================================================
# CRC-16 CCITT (polynomial 0x1021, seed 0x0000)
# Matches Si4463 WDS PKT_CRC_SEED configuration
# ============================================================

def crc16_itu_t(data: bytes) -> int:
    """Compute CRC-16 CCITT over data bytes."""
    crc = 0x0000
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
# Packet Builder
# ============================================================

def build_packet_bits(payload: bytes) -> np.ndarray:
    """Build a complete Si4463-compatible packet as a bit array.

    Structure:
        [Preamble] [Sync 2B] [Length 1B] [Payload] [CRC-16 2B]

    All bytes sent MSB first — Si4463 CRC validates correctly with MSB-first.
    The CRC covers the length byte + payload (matching Si4463 field config).
    """
    if len(payload) > MAX_PACKET_LEN:
        raise ValueError(f"Payload too long: {len(payload)} > {MAX_PACKET_LEN}")

    preamble = bytes([0xAA] * PREAMBLE_BYTES)
    sync = struct.pack(">H", SYNC_WORD)         # MSB first
    length = bytes([len(payload)])

    # CRC covers length byte + payload
    crc_data = length + payload
    crc = crc16_itu_t(crc_data)
    crc_bytes = struct.pack(">H", crc)           # MSB first

    packet = preamble + sync + length + payload + crc_bytes
    return bytes_to_bits_msb(packet)


# ============================================================
# GFSK Modulator
# ============================================================

def bytes_to_bits_msb(data: bytes) -> np.ndarray:
    """Convert bytes to bit array (MSB first per byte)."""
    bits = []
    for byte in data:
        for i in range(7, -1, -1):
            bits.append((byte >> i) & 1)
    return np.array(bits, dtype=np.float64)


def bytes_to_bits_lsb(data: bytes) -> np.ndarray:
    """Convert bytes to bit array (LSB first per byte, matching Si4463 BIT_ORDER=0)."""
    bits = []
    for byte in data:
        for i in range(8):
            bits.append((byte >> i) & 1)
    return np.array(bits, dtype=np.float64)


def gaussian_filter(bt: float, samples_per_symbol: float, span: int = 4) -> np.ndarray:
    """Generate a Gaussian filter for GFSK modulation.

    Args:
        bt: bandwidth-time product
        samples_per_symbol: samples per symbol period
        span: filter span in symbol periods (each side)
    """
    n_taps = int(2 * span * samples_per_symbol) + 1
    t = np.arange(n_taps) / samples_per_symbol - span
    # Gaussian pulse shape
    alpha = np.sqrt(2 * np.log(2)) / bt
    h = np.sqrt(2 * np.pi) / alpha * np.exp(-2 * (np.pi * t / alpha) ** 2)
    h /= np.sum(h)  # normalize to unit area
    return h


def gfsk_modulate(bits: np.ndarray) -> np.ndarray:
    """GFSK-modulate a bit array into complex IQ samples.

    Steps:
        1. NRZ encode: 0 → -1, 1 → +1
        2. Upsample to sample rate
        3. Apply Gaussian filter (BT=0.5)
        4. Integrate to get instantaneous phase
        5. Generate complex IQ: exp(j * phase)
    """

    # NRZ encoding
    nrz = 2.0 * bits - 1.0

    # Upsample: repeat each symbol for samples_per_symbol
    sps = int(round(SAMPLES_PER_SYMBOL))
    upsampled = np.repeat(nrz, sps)

    # Gaussian filter
    g = gaussian_filter(BT, sps)
    filtered = np.convolve(upsampled, g, mode="same")

    # Frequency modulation: instantaneous phase
    # phase(t) = 2π * deviation * ∫ filtered(t) dt
    # With discrete integration: phase[n] = 2π * dev * Σ filtered[k] / fs
    phase = 2 * np.pi * FREQ_DEVIATION * np.cumsum(filtered) / SAMPLE_RATE

    # Generate IQ
    iq = np.exp(1j * phase)

    # Scale to Pluto DAC range (12-bit DAC, int16 transport, use 2^15 * 0.8 headroom)
    iq = iq * (2**15 * 0.8)

    return iq.astype(np.complex64)


# ============================================================
# Pluto SDR TX (cyclic mode with embedded gaps)
# ============================================================

def transmit(uri: str, tx_gain_db: float, packets: int, gap_ms: float,
             xo_correction: int = PLUTO_XO_CORRECTION):
    """Transmit GFSK packets via Pluto SDR in cyclic mode.

    Builds a single IQ buffer containing `packets` GFSK packets separated
    by `gap_ms` milliseconds of silence, then transmits the buffer in cyclic
    (DMA loop) mode. This avoids the DMA startup transients that corrupt
    non-cyclic short bursts.

    Args:
        uri: Pluto SDR URI (e.g., "ip:192.168.3.1")
        tx_gain_db: TX attenuation in dB (e.g., -10)
        packets: number of packets in the cyclic buffer
        gap_ms: silence gap between packets in milliseconds
    """
    print(f"Connecting to Pluto SDR at {uri}...")
    try:
        sdr = adi.Pluto(uri=uri)
    except Exception as e:
        print(f"ERROR: Cannot connect to Pluto SDR: {e}")
        print(f"Check that the Pluto is powered on and reachable at {uri}")
        sys.exit(1)

    # Configure TX
    # TX LO: apply XO correction to compensate for Pluto crystal offset.
    # Without correction, actual TX frequency is ~26 kHz low — outside Si4463 AFC range.
    sdr.tx_lo = CENTER_FREQ + xo_correction
    sdr.tx_rf_bandwidth = SAMPLE_RATE
    sdr.tx_hardwaregain_chan0 = tx_gain_db
    sdr.sample_rate = SAMPLE_RATE
    sdr.tx_cyclic_buffer = True  # cyclic DMA — loops the buffer continuously

    gap_samples = int(SAMPLE_RATE * gap_ms / 1000)

    print(f"  Frequency   : {CENTER_FREQ / 1e6:.2f} MHz")
    print(f"  Data rate   : {DATA_RATE} bps")
    print(f"  Deviation   : ±{FREQ_DEVIATION} Hz")
    print(f"  Modulation  : GFSK (BT={BT})")
    print(f"  TX gain     : {tx_gain_db} dB")
    print(f"  Sample rate : {SAMPLE_RATE / 1e3:.0f} kHz")
    print(f"  Callsign    : {CALLSIGN}")
    print(f"  Sync word   : 0x{SYNC_WORD:04X}")
    print(f"  Preamble    : {PREAMBLE_BYTES} bytes")
    print(f"  Packets     : {packets}")
    print(f"  Gap         : {gap_ms} ms ({gap_samples} samples)")
    print()

    # Build composite buffer: [pkt0][gap][pkt1][gap]...[pktN-1][gap]
    print("Building composite IQ buffer...")
    segments = []
    for i in range(packets):
        payload_str = f"{CALLSIGN} {i:04d}"
        payload = payload_str.encode("ascii")
        bits = build_packet_bits(payload)
        iq = gfsk_modulate(bits)

        n_bytes = len(bits) // 8
        pkt_duration_ms = len(iq) / SAMPLE_RATE * 1000
        print(f"  [{i:04d}] {payload_str}  "
              f"({n_bytes}B on-air, {len(iq)} IQ samples, "
              f"{pkt_duration_ms:.1f} ms)")

        segments.append(iq)
        segments.append(np.zeros(gap_samples, dtype=np.complex64))

    composite = np.concatenate(segments)
    total_duration_ms = len(composite) / SAMPLE_RATE * 1000

    print(f"\n  Total buffer: {len(composite)} samples "
          f"({total_duration_ms:.1f} ms per cycle)")
    print(f"  Duty cycle  : "
          f"{(total_duration_ms - packets * gap_ms) / total_duration_ms * 100:.1f}%")
    print()

    # Transmit
    print("Starting cyclic TX... (Ctrl+C to stop)")
    sdr.tx(composite)

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nStopping TX...")
    finally:
        sdr.tx_destroy_buffer()
        print(f"TX stopped. 73 de {CALLSIGN}")


# ============================================================
# CLI
# ============================================================

def main():
    parser = argparse.ArgumentParser(
        description="Pluto SDR GFSK TX for kensat_radio (437.08 MHz)"
    )
    parser.add_argument("--uri", type=str, default=PLUTO_URI,
                        help=f"Pluto SDR URI (default: {PLUTO_URI})")
    parser.add_argument("--tx-gain", type=float, default=TX_GAIN_DB,
                        help=f"TX attenuation in dB (default: {TX_GAIN_DB})")
    parser.add_argument("--packets", type=int, default=5,
                        help="Number of packets in cyclic buffer (default: 5)")
    parser.add_argument("--gap-ms", type=float, default=200,
                        help="Silence gap between packets in ms (default: 200)")
    parser.add_argument("--xo-correction", type=int, default=PLUTO_XO_CORRECTION,
                        help=f"Pluto XO frequency correction in Hz (default: {PLUTO_XO_CORRECTION})")
    args = parser.parse_args()

    transmit(
        uri=args.uri,
        tx_gain_db=args.tx_gain,
        packets=args.packets,
        gap_ms=args.gap_ms,
        xo_correction=args.xo_correction,
    )


if __name__ == "__main__":
    main()
