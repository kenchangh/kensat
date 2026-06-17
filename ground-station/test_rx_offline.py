#!/usr/bin/env python3
"""
Offline test for pluto_rx.py decode pipeline.

Generates a synthetic Si4463-compatible GFSK packet (using pluto_tx.py's
modulator), places it at a simulated carrier offset, optionally adds AWGN
noise, then runs the full carrier_lock → downconvert_and_demod →
find_sync_and_decode pipeline and verifies CRC: OK.

This confirms the DSP chain works without needing live hardware.

Usage:
    python test_rx_offline.py [--snr 20] [--freq-offset 5000] [--preamble-bytes 64]
"""

import argparse
import sys
import numpy as np
from scipy.signal import firwin, lfilter

# Import DSP functions from the two scripts
sys.path.insert(0, '.')
from pluto_tx import gfsk_modulate, build_packet_bits, SAMPLE_RATE, FREQ_DEVIATION
from pluto_rx import (carrier_lock, downconvert_and_demod, find_sync_and_decode,
                      LO_OFFSET, SPS, MAX_PACKET_LEN)

BUF_SIZE = 65536  # samples per Pluto buffer


def add_awgn(signal: np.ndarray, snr_db: float) -> np.ndarray:
    """Add complex AWGN to IQ signal at specified SNR (dB)."""
    sig_power = np.mean(np.abs(signal) ** 2)
    noise_power = sig_power / (10 ** (snr_db / 10))
    noise = np.sqrt(noise_power / 2) * (
        np.random.randn(len(signal)) + 1j * np.random.randn(len(signal))
    )
    return signal + noise


def build_test_iq(payload_str: str, freq_offset: float, preamble_bytes: int,
                  snr_db: float, start_sample: int = 0) -> np.ndarray:
    """Build a full IQ buffer as the Pluto would receive it.

    Places the GFSK packet at freq_offset (simulating carrier drift),
    adds AWGN at the given SNR, and returns a contiguous IQ array.

    The IQ array is long enough to contain at least 2 full Pluto buffers.
    start_sample: offset (in samples) where the packet starts within the array.
    """
    # Build packet bits
    import struct

    SYNC_WORD = 0x2DD4
    preamble = bytes([0xAA] * preamble_bytes)
    sync = struct.pack(">H", SYNC_WORD)
    payload = payload_str.encode("ascii")
    length = bytes([len(payload)])
    from pluto_rx import crc16_itu_t
    crc = crc16_itu_t(length + payload)
    crc_bytes = struct.pack(">H", crc)
    packet_bytes = preamble + sync + length + payload + crc_bytes

    # Convert to bits and GFSK-modulate
    bits = np.array([], dtype=np.float64)
    for byte in packet_bytes:
        for i in range(7, -1, -1):
            bits = np.append(bits, (byte >> i) & 1)

    gfsk_iq = gfsk_modulate(bits).astype(np.complex128)
    n_pkt = len(gfsk_iq)

    # Total array: 2 full buffers, packet placed at start_sample
    total_len = 3 * BUF_SIZE
    iq = np.zeros(total_len, dtype=np.complex128)

    end_sample = start_sample + n_pkt
    if end_sample > total_len:
        n_pkt = total_len - start_sample
        gfsk_iq = gfsk_iq[:n_pkt]

    iq[start_sample:start_sample + n_pkt] = gfsk_iq * 1000  # scale to Pluto-like levels

    # Shift carrier to freq_offset
    t = np.arange(total_len) / SAMPLE_RATE
    iq = iq * np.exp(1j * 2 * np.pi * freq_offset * t)

    # Add AWGN
    iq = add_awgn(iq, snr_db)

    return iq


def run_test(payload_str: str, freq_offset: float, preamble_bytes: int,
             snr_db: float, start_sample: int):
    print(f"  Payload      : '{payload_str}'")
    print(f"  Carrier at   : {freq_offset / 1e3:.1f} kHz")
    print(f"  Preamble     : {preamble_bytes} bytes")
    print(f"  SNR          : {snr_db:.0f} dB")
    print(f"  Packet start : sample {start_sample} "
          f"(buffer phase = {start_sample % BUF_SIZE}/{BUF_SIZE})")

    # Generate synthetic IQ (Pluto ADC-level, carrier at freq_offset)
    full_iq = build_test_iq(payload_str, freq_offset, preamble_bytes,
                            snr_db, start_sample)

    # Pre-compute LPF (same as pluto_rx.py)
    lpf = firwin(501, 10000 / (SAMPLE_RATE / 2))

    # Simulate Pluto buffer reception with rolling 2-buffer window
    freq_prev = None
    total_samples = 0
    found_packets = []
    seen_pos = set()

    n_buffers = len(full_iq) // BUF_SIZE
    for buf_idx in range(n_buffers):
        iq_buf = full_iq[buf_idx * BUF_SIZE:(buf_idx + 1) * BUF_SIZE].copy()

        # Per-buffer FFT carrier lock
        fc = carrier_lock(iq_buf)

        # Downconvert + LPF + FM demod
        freq_new = downconvert_and_demod(iq_buf, fc, lpf)

        # Rolling 2-buffer window
        if freq_prev is not None:
            freq_history = np.concatenate([freq_prev, freq_new])
            history_start = total_samples - len(freq_prev)
        else:
            freq_history = freq_new
            history_start = total_samples

        total_samples += len(freq_new)
        freq_prev = freq_new

        for payload, crc_ok, crc_calc, crc_rx, local_peak in \
                find_sync_and_decode(freq_history, SPS):
            global_peak = history_start + local_peak
            pos_key = global_peak // SPS
            if pos_key in seen_pos:
                continue
            seen_pos.add(pos_key)

            ascii_str = ''.join(chr(b) if 0x20 <= b <= 0x7E else '.' for b in payload)
            crc_str = "OK" if crc_ok else f"FAIL (rx=0x{crc_rx:04X} calc=0x{crc_calc:04X})"
            found_packets.append((ascii_str, crc_ok, fc, buf_idx))
            print(f"  buf={buf_idx:2d} fc={fc/1e3:.1f}kHz → '{ascii_str}' CRC:{crc_str}")

    return found_packets


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--snr", type=float, default=20, help="SNR in dB (default: 20)")
    parser.add_argument("--freq-offset", type=float, default=100000,
                        help="Carrier offset Hz (default: 100000 = nominal)")
    parser.add_argument("--preamble-bytes", type=int, default=64,
                        help="Si4463 preamble length in bytes (default: 64)")
    parser.add_argument("--start-sample", type=int, default=-1,
                        help="Packet start sample (default: random in [0, 2*BUF_SIZE])")
    args = parser.parse_args()

    np.random.seed(42)

    if args.start_sample < 0:
        # Default: place packet so it spans a buffer boundary
        args.start_sample = BUF_SIZE - 30000  # preamble starts near end of buf 0

    print("=" * 60)
    print("Offline GFSK RX test")
    print("=" * 60)

    # Test 1: nominal conditions
    print("\n[Test 1] Nominal: packet spanning buffer boundary")
    pkts = run_test("KO6IKZ 0032", args.freq_offset, args.preamble_bytes,
                    args.snr, args.start_sample)
    ok1 = any(p[1] for p in pkts)
    if ok1:
        print("  PASS: packet found with CRC OK")
    else:
        print(f"  FAIL: found {len(pkts)} packet(s), CRC results: {[p[1] for p in pkts]}")

    # Test 2: frequency drift
    drift = args.freq_offset - 5400  # simulate 5.4 kHz offset (the bug this fixes)
    print(f"\n[Test 2] Carrier at {drift/1e3:.1f} kHz (off by 5.4 kHz from nominal)")
    pkts2 = run_test("KO6IKZ 0099", drift, args.preamble_bytes, args.snr,
                     args.start_sample)
    ok2 = any(p[1] for p in pkts2)
    if ok2:
        print("  PASS: FFT carrier lock corrected drift, CRC OK")
    else:
        print(f"  FAIL: found {len(pkts2)} packet(s), CRC: {[p[1] for p in pkts2]}")

    # Test 3: packet fully within single buffer (start early)
    print("\n[Test 3] Packet starting at sample 0 (fully in buffer 0)")
    pkts3 = run_test("KO6IKZ 0000", args.freq_offset, args.preamble_bytes,
                     args.snr, 0)
    ok3 = any(p[1] for p in pkts3)
    if ok3:
        print("  PASS: packet found with CRC OK")
    else:
        print(f"  FAIL: found {len(pkts3)} packet(s), CRC: {[p[1] for p in pkts3]}")

    print("\n" + "=" * 60)
    all_ok = ok1 and ok2 and ok3
    print("OVERALL:", "PASS" if all_ok else "FAIL")


if __name__ == "__main__":
    main()
