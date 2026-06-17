#!/usr/bin/env python3
"""
KENSAT beacon telemetry decoder.

Receives Si4463 GFSK beacon packets via Pluto SDR and decodes the
telemetry struct into human-readable output.

Beacon frame format (49 bytes):
    [callsign 6B][frame_type 1B][seq 2B][uptime 4B][reboot 1B]
    [obc_temp 2B][batt_v 2B][batt_i 2B][batt_t 2B][rails 1B]
    [jet_temp 2B][jet_state 1B][tx_pwr 1B][pa_temp 1B][rssi 1B]
    [tx_cnt 2B][rx_cnt 2B][crc_err 2B][pwr_set 1B][afc 2B]
    [llm_stat 1B][llm_lat 2B][llm_hash 4B][mode 1B][reserved 3B]

All multi-byte fields are little-endian (PIC24 native).
Signed fields: obc_temp, batt_current, batt_temp, jetson_temp, afc_offset.

Usage:
    python beacon_decoder.py [--rx-gain 50] [--duration 120]

Requirements:
    pip install pyadi-iio numpy scipy
"""

import argparse
import struct
import sys
import time

import adi
import numpy as np
from scipy.signal import firwin, lfilter

# ============================================================
# RF Configuration (matches Si4463 config)
# ============================================================

PLUTO_URI = "ip:192.168.3.1"
import os as _os
CENTER_FREQ = int(_os.environ.get("KENSAT_FREQ_HZ", "437080000"))
PLUTO_XO_CORRECTION = 26000
DATA_RATE = 9600
FREQ_DEVIATION = 4800
SAMPLE_RATE = 960_000
SYNC_WORD = 0x2DD4
MAX_PACKET_LEN = 64

SAMPLES_PER_SYMBOL = SAMPLE_RATE // DATA_RATE  # 100

# Beacon frame constants
TLM_FRAME_BEACON = 0x01
BEACON_STRUCT_SIZE = 49
CALLSIGN = "KO6IKZ"

# Mission mode names
MODE_NAMES = {
    0x00: "SAFE",
    0x01: "NOMINAL",
    0x02: "SCIENCE",
    0x03: "DOWNLINK",
}

# Jetson state names
JETSON_STATES = {
    0x00: "OFF",
    0x01: "BOOTING",
    0x02: "RUNNING",
    0xFF: "ERROR",
}


# ============================================================
# CRC-16 ITU-T (matches Si4463 / pluto_rx.py)
# ============================================================

def crc16_itu_t(data: bytes) -> int:
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
# Beacon telemetry parser
# ============================================================

def decode_beacon(payload: bytes) -> dict:
    """Parse a 49-byte beacon payload into a telemetry dict."""
    if len(payload) < BEACON_STRUCT_SIZE:
        return None

    # Unpack matching the C struct (little-endian, packed)
    # H = unsigned 16-bit, h = signed 16-bit
    # I = unsigned 32-bit, i = signed 32-bit
    # B = unsigned 8-bit, b = signed 8-bit
    fmt = '<'       # little-endian
    fmt += '6s'     # callsign [6]
    fmt += 'B'      # frame_type
    fmt += 'H'      # seq_num (unsigned int = 16-bit on PIC24)
    fmt += 'I'      # uptime_s (unsigned long = 32-bit)
    fmt += 'B'      # reboot_count
    fmt += 'h'      # obc_temp (int = signed 16-bit)
    fmt += 'H'      # batt_voltage
    fmt += 'h'      # batt_current (signed)
    fmt += 'h'      # batt_temp (signed)
    fmt += 'B'      # eps_rails
    fmt += 'h'      # jetson_temp (signed)
    fmt += 'B'      # jetson_state
    fmt += 'B'      # tx_power
    fmt += 'B'      # pa_temp
    fmt += 'B'      # last_rssi
    fmt += 'H'      # tx_count
    fmt += 'H'      # rx_count
    fmt += 'H'      # crc_err_count
    fmt += 'B'      # tx_pwr_setting
    fmt += 'h'      # afc_offset (signed)
    fmt += 'B'      # llm_status
    fmt += 'H'      # llm_latency_ms
    fmt += 'I'      # llm_hash
    fmt += 'B'      # mission_mode
    fmt += '3s'     # reserved

    expected_size = struct.calcsize(fmt)
    if expected_size != BEACON_STRUCT_SIZE:
        print(f"WARNING: struct size mismatch: expected {BEACON_STRUCT_SIZE}, got {expected_size}")

    fields = struct.unpack(fmt, payload[:BEACON_STRUCT_SIZE])

    tlm = {
        'callsign':       fields[0].decode('ascii', errors='replace'),
        'frame_type':     fields[1],
        'seq_num':        fields[2],
        'uptime_s':       fields[3],
        'reboot_count':   fields[4],
        'obc_temp':       fields[5] / 10.0,  # 0.1 degC -> degC
        'batt_voltage':   fields[6] / 1000.0,  # mV -> V
        'batt_current':   fields[7],  # mA
        'batt_temp':      fields[8] / 10.0,
        'eps_rails':      fields[9],
        'jetson_temp':    fields[10] / 10.0,
        'jetson_state':   fields[11],
        'tx_power':       fields[12],
        'pa_temp':        fields[13],
        'last_rssi':      fields[14],
        'tx_count':       fields[15],
        'rx_count':       fields[16],
        'crc_err_count':  fields[17],
        'tx_pwr_setting': fields[18],
        'afc_offset':     fields[19],
        'llm_status':     fields[20],
        'llm_latency_ms': fields[21],
        'llm_hash':       fields[22],
        'mission_mode':   fields[23],
    }

    return tlm


def print_beacon(tlm: dict, pkt_num: int, elapsed: float):
    """Pretty-print a decoded beacon frame."""
    rails = []
    if tlm['eps_rails'] & 0x01: rails.append("3.3V")
    if tlm['eps_rails'] & 0x02: rails.append("5V")
    if tlm['eps_rails'] & 0x04: rails.append("Jetson")
    rails_str = ", ".join(rails) if rails else "none"

    mode = MODE_NAMES.get(tlm['mission_mode'], f"0x{tlm['mission_mode']:02X}")
    jet_state = JETSON_STATES.get(tlm['jetson_state'], f"0x{tlm['jetson_state']:02X}")

    llm_flags = []
    if tlm['llm_status'] & 0x01: llm_flags.append("MODEL_LOADED")
    if tlm['llm_status'] & 0x02: llm_flags.append("INFERENCE_OK")
    llm_str = ", ".join(llm_flags) if llm_flags else "none"

    uptime_h = tlm['uptime_s'] // 3600
    uptime_m = (tlm['uptime_s'] % 3600) // 60
    uptime_sec = tlm['uptime_s'] % 60

    rssi_dbm = tlm['last_rssi'] / 2.0 - 134 if tlm['last_rssi'] else 0

    print(f"\n{'='*60}")
    print(f"  BEACON #{pkt_num} at {elapsed:.1f}s")
    print(f"{'='*60}")
    print(f"  Callsign      : {tlm['callsign']}")
    print(f"  Sequence      : {tlm['seq_num']}")
    print(f"  Mode          : {mode}")
    print(f"  Uptime        : {uptime_h}h {uptime_m}m {uptime_sec}s ({tlm['uptime_s']}s)")
    print(f"  Reboot count  : {tlm['reboot_count']}")
    print()
    print(f"  --- OBC ---")
    print(f"  Temperature   : {tlm['obc_temp']:.1f} C")
    print()
    print(f"  --- EPS ---")
    print(f"  Battery       : {tlm['batt_voltage']:.3f} V, {tlm['batt_current']} mA")
    print(f"  Batt temp     : {tlm['batt_temp']:.1f} C")
    print(f"  Rails active  : {rails_str}")
    print()
    print(f"  --- Jetson ---")
    print(f"  State         : {jet_state}")
    print(f"  Temperature   : {tlm['jetson_temp']:.1f} C")
    print()
    print(f"  --- Comms ---")
    print(f"  TX power      : 0x{tlm['tx_power']:02X} (setting 0x{tlm['tx_pwr_setting']:02X})")
    print(f"  PA temp       : {tlm['pa_temp']}")
    print(f"  Last RSSI     : {tlm['last_rssi']} (~{rssi_dbm:.0f} dBm)")
    print(f"  Packets       : TX={tlm['tx_count']} RX={tlm['rx_count']} CRC_ERR={tlm['crc_err_count']}")
    print(f"  AFC offset    : {tlm['afc_offset']} Hz")
    print()
    print(f"  --- LLM ---")
    print(f"  Status        : {llm_str}")
    print(f"  Latency       : {tlm['llm_latency_ms']} ms")
    print(f"  Output hash   : 0x{tlm['llm_hash']:08X}")
    print(f"{'='*60}")
    print()


# ============================================================
# GFSK Demodulator (same as pluto_rx.py)
# ============================================================

def iq_lowpass(iq: np.ndarray) -> np.ndarray:
    num_taps = 201
    cutoff = 25000 / (SAMPLE_RATE / 2)
    lpf = firwin(num_taps, cutoff)
    return lfilter(lpf, 1.0, iq)


def fm_demodulate(iq: np.ndarray) -> np.ndarray:
    diff = iq[1:] * np.conj(iq[:-1])
    freq = np.angle(diff)
    scale = SAMPLE_RATE / (2 * np.pi * FREQ_DEVIATION)
    freq *= scale
    freq = np.clip(freq, -2.0, 2.0)
    return freq


def bits_to_bytes(bits: list) -> bytes:
    out = []
    for i in range(0, len(bits) - 7, 8):
        byte = 0
        for j in range(8):
            byte = (byte << 1) | bits[i + j]
        out.append(byte)
    return bytes(out)


def find_sync_and_decode(freq: np.ndarray, sps: int) -> list:
    sync_bits = []
    for i in range(15, -1, -1):
        sync_bits.append((SYNC_WORD >> i) & 1)

    preamble_bits = [1, 0] * 16
    search_bits = preamble_bits + sync_bits

    nrz = np.array([2.0 * b - 1.0 for b in search_bits])
    template = np.repeat(nrz, sps)

    corr = np.correlate(freq, template, mode='valid')

    min_threshold = len(template) * 0.15
    threshold = max(np.max(np.abs(corr)) * 0.7, min_threshold)
    if np.max(np.abs(corr)) < min_threshold:
        return []

    abs_corr = np.abs(corr)
    peaks = []
    min_gap = sps * 50

    i = 0
    while i < len(abs_corr):
        if abs_corr[i] > threshold:
            j = i
            while j < len(abs_corr) - 1 and abs_corr[j + 1] > abs_corr[j]:
                j += 1
            peaks.append((j, corr[j] > 0))
            i = j + min_gap
        else:
            i += 1

    packets = []
    for peak_pos, is_positive in peaks:
        data_start = peak_pos + len(template)

        max_bits = (1 + MAX_PACKET_LEN + 2) * 8
        samples_needed = max_bits * sps
        if data_start + samples_needed > len(freq):
            continue

        polarity = 1.0 if is_positive else -1.0
        raw_bits = []
        for b in range(max_bits):
            center = data_start + int(b * sps + sps / 2)
            if center >= len(freq):
                break
            val = freq[center] * polarity
            raw_bits.append(1 if val > 0 else 0)

        if len(raw_bits) < 24:
            continue

        raw_bytes = bits_to_bytes(raw_bits)

        payload_len = raw_bytes[0]
        if payload_len == 0 or payload_len > MAX_PACKET_LEN:
            continue

        total_bytes = 1 + payload_len + 2
        if len(raw_bytes) < total_bytes:
            continue

        pkt_bytes = raw_bytes[:total_bytes]
        payload = pkt_bytes[1:1 + payload_len]
        crc_rx = (pkt_bytes[-2] << 8) | pkt_bytes[-1]
        crc_calc = crc16_itu_t(pkt_bytes[:1 + payload_len])
        crc_ok = (crc_rx == crc_calc)

        packets.append((payload, crc_ok, crc_calc, crc_rx, peak_pos))

    return packets


# ============================================================
# Main receiver
# ============================================================

def receive(uri: str, rx_gain: float, duration: float):
    print(f"KENSAT Beacon Decoder")
    print(f"Connecting to Pluto SDR at {uri}...")
    try:
        sdr = adi.Pluto(uri=uri)
    except Exception as e:
        print(f"ERROR: Cannot connect to Pluto SDR: {e}")
        sys.exit(1)

    LO_OFFSET = 100_000
    sdr.rx_lo = CENTER_FREQ + PLUTO_XO_CORRECTION - LO_OFFSET
    sdr.rx_rf_bandwidth = SAMPLE_RATE
    sdr.gain_control_mode_chan0 = 'manual'
    sdr.rx_hardwaregain_chan0 = rx_gain
    sdr.sample_rate = SAMPLE_RATE
    sdr.rx_buffer_size = 2**16

    sps = SAMPLES_PER_SYMBOL

    print(f"  Frequency   : {CENTER_FREQ / 1e6:.2f} MHz")
    print(f"  Data rate   : {DATA_RATE} bps (GFSK)")
    print(f"  RX gain     : {rx_gain} dB")
    print(f"  Beacon size : {BEACON_STRUCT_SIZE} bytes")
    print(f"  Duration    : {duration}s")
    print()

    for _ in range(5):
        sdr.rx()

    beacon_count = 0
    buf_count = 0
    sample_offset = 0
    start = time.time()

    print("Listening for beacons... (Ctrl+C to stop)")
    print("-" * 60)

    try:
        while time.time() - start < duration:
            iq_raw = sdr.rx()
            buf_count += 1

            n = len(iq_raw)
            t = (np.arange(n) + sample_offset) / SAMPLE_RATE
            mixer = np.exp(-1j * 2 * np.pi * LO_OFFSET * t)
            iq = iq_raw.astype(np.complex128) * mixer
            sample_offset += n

            iq = iq_lowpass(iq)
            freq = fm_demodulate(iq)

            packets = find_sync_and_decode(freq, sps)

            if buf_count % 30 == 0:
                elapsed = time.time() - start
                print(f"  [{elapsed:5.1f}s] Listening... beacons={beacon_count}")

            for payload, crc_ok, crc_calc, crc_rx, pos in packets:
                elapsed = time.time() - start

                if not crc_ok:
                    print(f"\n  [{elapsed:.1f}s] Packet CRC FAIL "
                          f"(rx=0x{crc_rx:04X} calc=0x{crc_calc:04X}), "
                          f"len={len(payload)}")
                    print(f"    Hex: {payload.hex()}")
                    continue

                if len(payload) < BEACON_STRUCT_SIZE:
                    print(f"\n  [{elapsed:.1f}s] Packet too short for beacon: "
                          f"{len(payload)} bytes")
                    ascii_str = ''.join(
                        chr(b) if 0x20 <= b <= 0x7E else '.'
                        for b in payload
                    )
                    print(f"    ASCII: {ascii_str}")
                    continue

                # Check frame type
                if payload[6] != TLM_FRAME_BEACON:
                    print(f"\n  [{elapsed:.1f}s] Non-beacon frame type: "
                          f"0x{payload[6]:02X}")
                    continue

                tlm = decode_beacon(payload)
                if tlm is None:
                    continue

                beacon_count += 1
                print_beacon(tlm, beacon_count, elapsed)

    except KeyboardInterrupt:
        print("\nStopping...")

    elapsed = time.time() - start
    print("-" * 60)
    print(f"Done. {beacon_count} beacons decoded in {elapsed:.1f}s. "
          f"73 de {CALLSIGN}")


def main():
    parser = argparse.ArgumentParser(
        description="KENSAT beacon telemetry decoder (437.08 MHz GFSK)"
    )
    parser.add_argument("--uri", type=str, default=PLUTO_URI,
                        help=f"Pluto SDR URI (default: {PLUTO_URI})")
    parser.add_argument("--rx-gain", type=float, default=50,
                        help="RX gain in dB (default: 50)")
    parser.add_argument("--duration", type=float, default=300,
                        help="Listen duration in seconds (default: 300)")
    args = parser.parse_args()

    receive(uri=args.uri, rx_gain=args.rx_gain, duration=args.duration)


if __name__ == "__main__":
    main()
