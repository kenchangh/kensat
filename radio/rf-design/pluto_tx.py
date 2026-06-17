#!/usr/bin/env python3
"""
Pluto SDR GFSK transmitter for kensat_radio RF board.

Generates Si4463-compatible GFSK packets at 437.08 MHz and transmits
them via ADALM-Pluto SDR. The Si4463 on the RF board (in RX mode)
will decode these packets.

Packet format (matches Si4463 config):
    [Preamble 4B] [Sync 0x2DD4] [Len 1B] [Payload ≤64B] [CRC-16 2B]

Modulation:
    GFSK, 9600 bps, ±4.8 kHz deviation, BT=0.5

Amateur radio compliance:
    - 437.08 MHz (70cm band, 420–450 MHz)
    - Callsign KO6IKZ included in every packet
    - FCC Part 97

Usage:
    python pluto_tx.py [--tx-gain -10] [--interval 1.0] [--count 0]

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
PLUTO_XO_CORRECTION = 22400     # Pluto crystal is ~22.4 kHz low (~51 ppm)
DATA_RATE = 9600                # bps
FREQ_DEVIATION = 4800           # ±4.8 kHz
BT = 0.5                       # Gaussian filter bandwidth-time product
PREAMBLE_BYTES = 4              # 4 bytes of 0xAA
SYNC_WORD = 0x2DD4              # 16-bit sync word
MAX_PACKET_LEN = 64             # max payload bytes

CALLSIGN = "KO6IKZ"

# Pluto SDR settings
SAMPLE_RATE = 960_000           # 960 kHz — 100 samples/symbol
TX_GAIN_DB = -10                # dB attenuation (conservative default)

# Derived
SAMPLES_PER_SYMBOL = SAMPLE_RATE / DATA_RATE  # ~26.67


# ============================================================
# CRC-16 ITU-T (polynomial 0x1021, seed 0xFFFF)
# ============================================================

def crc16_itu_t(data: bytes) -> int:
    """Compute CRC-16 ITU-T (CCITT) over data bytes."""
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
# Packet Builder
# ============================================================

def build_packet(payload: bytes) -> bytes:
    """Build a complete Si4463-compatible packet (preamble through CRC).

    Structure:
        [Preamble 4B] [Sync 2B] [Length 1B] [Payload] [CRC-16 2B]

    The CRC covers the length byte + payload (matching Si4463 field config:
    CRC starts at field 1, checked at field 2).
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

    return preamble + sync + length + payload + crc_bytes


# ============================================================
# GFSK Modulator
# ============================================================

def bytes_to_bits(data: bytes) -> np.ndarray:
    """Convert bytes to bit array (MSB first per byte)."""
    bits = []
    for byte in data:
        for i in range(7, -1, -1):
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
    alpha = np.sqrt(np.log(2) / 2) / bt
    h = np.sqrt(2 * np.pi) / alpha * np.exp(-2 * (np.pi * t / alpha) ** 2)
    h /= np.sum(h)  # normalize to unit area
    return h


def gfsk_modulate(packet_bytes: bytes) -> np.ndarray:
    """GFSK-modulate a packet into complex IQ samples.

    Steps:
        1. Convert bytes to bits
        2. NRZ encode: 0 → -1, 1 → +1
        3. Upsample to sample rate
        4. Apply Gaussian filter (BT=0.5)
        5. Integrate to get instantaneous phase
        6. Generate complex IQ: exp(j * phase)
    """
    bits = bytes_to_bits(packet_bytes)

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

    # Scale to Pluto DAC range (14-bit signed)
    iq = iq * (2**14 - 1)

    return iq.astype(np.complex64)


# ============================================================
# Pluto SDR TX
# ============================================================

def transmit(uri: str, tx_gain_db: float, interval: float, count: int):
    """Transmit GFSK packets via Pluto SDR.

    Args:
        uri: Pluto SDR URI (e.g., "ip:192.168.3.1")
        tx_gain_db: TX attenuation in dB (e.g., -10)
        interval: seconds between packets (0 = single burst)
        count: number of packets to send (0 = infinite)
    """
    print(f"Connecting to Pluto SDR at {uri}...")
    try:
        sdr = adi.Pluto(uri=uri)
    except Exception as e:
        print(f"ERROR: Cannot connect to Pluto SDR: {e}")
        print(f"Check that the Pluto is powered on and reachable at {uri}")
        sys.exit(1)

    # Configure TX
    sdr.tx_lo = CENTER_FREQ + PLUTO_XO_CORRECTION
    sdr.tx_rf_bandwidth = SAMPLE_RATE
    sdr.tx_hardwaregain_chan0 = tx_gain_db
    sdr.sample_rate = SAMPLE_RATE
    sdr.tx_cyclic_buffer = False  # we push new packets each time

    print(f"  Frequency   : {CENTER_FREQ / 1e6:.2f} MHz")
    print(f"  Data rate   : {DATA_RATE} bps")
    print(f"  Deviation   : ±{FREQ_DEVIATION} Hz")
    print(f"  Modulation  : GFSK (BT={BT})")
    print(f"  TX gain     : {tx_gain_db} dB")
    print(f"  Sample rate : {SAMPLE_RATE / 1e3:.0f} kHz")
    print(f"  Callsign    : {CALLSIGN}")
    print(f"  Sync word   : 0x{SYNC_WORD:04X}")
    print(f"  Interval    : {interval} s")
    print(f"  Count       : {'infinite' if count == 0 else count}")
    print()

    pkt_num = 0
    try:
        while count == 0 or pkt_num < count:
            # Build payload with callsign + counter
            payload_str = f"{CALLSIGN} {pkt_num:04d}"
            payload = payload_str.encode("ascii")

            # Build and modulate packet
            packet = build_packet(payload)
            iq_samples = gfsk_modulate(packet)

            # Pad with zeros (silence) for inter-packet gap
            gap_samples = int(SAMPLE_RATE * 0.05)  # 50 ms gap
            iq_with_gap = np.concatenate([
                iq_samples,
                np.zeros(gap_samples, dtype=np.complex64),
            ])

            # Transmit
            sdr.tx(iq_with_gap)

            print(f"  [{pkt_num:04d}] TX {len(payload)}B: {payload_str}  "
                  f"({len(iq_samples)} IQ samples, "
                  f"{len(packet)} packet bytes)")

            pkt_num += 1

            if interval > 0 and (count == 0 or pkt_num < count):
                time.sleep(interval)

    except KeyboardInterrupt:
        print("\nStopping TX...")
    finally:
        sdr.tx_destroy_buffer()
        print(f"TX stopped. Sent {pkt_num} packets. 73 de {CALLSIGN}")


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
    parser.add_argument("--interval", type=float, default=1.0,
                        help="Seconds between packets (default: 1.0)")
    parser.add_argument("--count", type=int, default=0,
                        help="Number of packets to send, 0=infinite (default: 0)")
    parser.add_argument("--message", type=str, default=None,
                        help="Custom payload (callsign is always prepended)")
    args = parser.parse_args()

    transmit(
        uri=args.uri,
        tx_gain_db=args.tx_gain,
        interval=args.interval,
        count=args.count,
    )


if __name__ == "__main__":
    main()
