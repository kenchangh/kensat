#!/usr/bin/env python3
"""
Pluto SDR receiver for KENSAT AX.25 / G3RUH 9k6 GFSK beacon.

Decodes the AX.25 UI frames that task_beacon transmits at 437.080 MHz.
Replaces the old Si4463 packet format (see pluto_rx.py). On-air spec:

    9600 bps 2-GFSK, ±3 kHz deviation, BT=0.5, G3RUH scrambled, NRZI,
    bit-stuffed, HDLC 0x7E flag framing, 16-bit CRC-CCITT FCS.

Pipeline:
    Pluto IQ → carrier lock → downconvert → LPF → FM discriminator
    → Gaussian matched filter → SPS-grid bit slicing
    → G3RUH descramble (x^17+x^12+1) → un-NRZI
    → 0x7E flag scan → bit-destuff → FCS verify → AX.25 parse

The on-air format is identical to what Direwolf's "9600 G3RUH" preset
decodes, so this is functionally equivalent. Useful when you don't have
Direwolf installed or want everything in-process.

Usage:
    python pluto_rx_ax25.py [--rx-gain 60] [--duration 60]

Requirements:
    pip install pyadi-iio numpy scipy
"""

import argparse
import sys
import time

import adi
import numpy as np
from scipy.signal import firwin, lfilter, oaconvolve


# ============================================================
# RF configuration — must match the OBC firmware
# ============================================================

PLUTO_URI = "ip:192.168.3.1"
CENTER_FREQ = 437_080_000
PLUTO_XO_CORRECTION = 4000      # Pluto crystal offset (carrier lands ~LO_OFFSET in baseband)
DATA_RATE = 9600
FREQ_DEVIATION = 3000           # ±3 kHz peak (G3RUH 9k6 convention)
SAMPLE_RATE = 960_000
LO_OFFSET = 100_000
SPS = SAMPLE_RATE // DATA_RATE  # 100 samples/symbol

CARRIER_SEARCH_LO = 50_000
CARRIER_SEARCH_HI = 150_000
CARRIER_LEAK_GUARD = 2_500
CARRIER_TOL = 40_000


# ============================================================
# RF front-end (carrier lock + FM demod, same as pluto_rx.py)
# ============================================================

def carrier_lock(iq):
    """Power-weighted spectral centroid in [50, 150] kHz, masking the
    narrow LO leakage tone. Falls back to LO_OFFSET if no signal energy."""
    n = len(iq)
    fft_data = np.fft.fftshift(np.fft.fft(iq))
    freqs = np.fft.fftshift(np.fft.fftfreq(n, 1.0 / SAMPLE_RATE))
    power = np.abs(fft_data) ** 2

    mask = ((freqs > CARRIER_SEARCH_LO) & (freqs < CARRIER_SEARCH_HI)
            & (np.abs(freqs - LO_OFFSET) > CARRIER_LEAK_GUARD))
    if not mask.any():
        return float(LO_OFFSET)
    p = np.where(mask, power, 0.0)
    total = float(np.sum(p))
    if total <= 0.0:
        return float(LO_OFFSET)
    fc = float(np.sum(freqs * p) / total)
    if abs(fc - LO_OFFSET) > CARRIER_TOL:
        return float(LO_OFFSET)
    return fc


def downconvert_and_demod(iq, fc, lpf):
    """Mix IQ to exact baseband, LPF, FM discriminator, normalize so
    ±1 ≈ ±FREQ_DEVIATION. Length is len(iq) - 1."""
    n = len(iq)
    bb = iq * np.exp(-1j * 2 * np.pi * fc * np.arange(n) / SAMPLE_RATE)
    bb_filt = lfilter(lpf, 1.0, bb)
    freq = np.angle(bb_filt[1:] * np.conj(bb_filt[:-1]))
    freq = freq * SAMPLE_RATE / (2 * np.pi * FREQ_DEVIATION)
    return np.clip(freq, -2.0, 2.0)


def gaussian_filter(bt, sps, span=2):
    """Gaussian RX matched filter for GFSK with given BT product."""
    n_taps = int(2 * span * sps) + 1
    t = np.arange(n_taps) / sps - span
    alpha = np.sqrt(2 * np.log(2)) / bt
    h = np.sqrt(2 * np.pi) / alpha * np.exp(-2 * (np.pi * t / alpha) ** 2)
    return h / np.sum(h)


# ============================================================
# AX.25 / G3RUH decoder
# ============================================================

def ax25_fcs(data):
    """CRC-16-CCITT reflected (init 0xFFFF, poly 0x8408, final XOR 0xFFFF).
    The standard HDLC FCS used by AX.25."""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0x8408
            else:
                crc >>= 1
    return crc ^ 0xFFFF


def descramble_g3ruh(line_bits):
    """G3RUH multiplicative descrambler: out = in ⊕ in[-12] ⊕ in[-17].
    The 17-bit shift register holds the last 17 SCRAMBLED bits (LSB =
    most recent). First ~17 output bits are garbage as the SR fills,
    but the leading flag preamble in the TX output covers that."""
    sr = 0
    out = np.empty(len(line_bits), dtype=np.int8)
    for i, b in enumerate(line_bits):
        b = int(b) & 1
        out[i] = b ^ ((sr >> 11) & 1) ^ ((sr >> 16) & 1)
        sr = ((sr << 1) | b) & 0x1FFFF
    return out


def un_nrzi(scrambled_out):
    """Inverse of TX-side NRZI. TX rule: bit=0 → line transitions,
    bit=1 → line stays. So RX: data_bit = 1 if line[n]==line[n-1] else 0.
    Initial line state choice doesn't affect alignment past bit 1."""
    out = np.empty(len(scrambled_out), dtype=np.int8)
    prev = 1
    for i, b in enumerate(scrambled_out):
        out[i] = 1 if b == prev else 0
        prev = b
    return out


# HDLC flag 0x7E, LSB-first on the wire = 0,1,1,1,1,1,1,0
_FLAG = np.array([0, 1, 1, 1, 1, 1, 1, 0], dtype=np.int8)


def find_flag_positions(bits):
    """Return indices i such that bits[i:i+8] == 0x7E (LSB-first)."""
    if len(bits) < 8:
        return []
    # Vectorized 8-bit pattern match
    bits = np.asarray(bits, dtype=np.int8)
    # Build sliding window via stride tricks
    from numpy.lib.stride_tricks import sliding_window_view
    win = sliding_window_view(bits, 8)
    matches = np.all(win == _FLAG, axis=1)
    return np.nonzero(matches)[0].tolist()


def destuff_and_pack(bits, start, end, require_aligned=True):
    """Bit-destuff bits[start:end] and pack LSB-first into bytes.

    Bits are received from the air in AX.25 per-byte LSB-first order
    (the encoder packs MSB-first into its output bytes specifically so
    that Si4463's MSB-first FIFO transmission produces this on-air order).
    Pack LSB-first here so we recover the original AX.25 byte values.

    If require_aligned=True (default), returns None when not byte-aligned
    after destuffing — appropriate for FCS-checked frame extraction where
    a non-aligned tail means the closing flag wasn't actually a flag.
    If require_aligned=False, returns whatever complete bytes were emitted
    (drops the partial trailing byte) — appropriate for partial decodes
    where we destuff past the real frame end into post-TX noise."""
    out = bytearray()
    cur = 0
    cur_bit = 0
    ones = 0
    for i in range(start, end):
        b = int(bits[i])
        if b == 1:
            ones += 1
            cur |= (1 << cur_bit)
            cur_bit += 1
            if cur_bit == 8:
                out.append(cur)
                cur = 0
                cur_bit = 0
        else:
            if ones == 5:
                # stuffed 0 — drop it
                ones = 0
            else:
                cur_bit += 1
                if cur_bit == 8:
                    out.append(cur)
                    cur = 0
                    cur_bit = 0
                ones = 0
    if require_aligned and cur_bit != 0:
        return None
    return bytes(out)


def extract_ax25_frames(bits, accept_partial=True):
    """Locate flag-delimited regions in `bits`, destuff each, and return
    a list of (frame_bytes, fcs_ok) tuples.

    Strict mode (accept_partial=False): only FCS-passing frame pairs.
    Partial mode (default): also returns frames where no trailing flag was
    found but the data starting from a long run of leading flags decodes
    to a plausible AX.25 address (printable callsign chars). At 9k6 / 12 dB
    SNR the late bits of a 1000-bit frame are often corrupted by accumulated
    timing drift, but the addr+ctrl+pid+early-info portion is usually clean.
    """
    flags = find_flag_positions(bits)
    if not flags:
        return []

    out = []
    seen = set()

    # ---- Strict: full-flag-pair + FCS ----
    # AX.25 callsigns are alphanumeric uppercase (after shifting >> 1). Reject
    # CRC-passing frames with non-callsign addresses — at 1-in-65k per random
    # destuff and thousands of flag pairs tried, false-positive FCS hits are
    # otherwise common.
    def callsign_chars_look_valid(raw_bytes):
        for b in raw_bytes[:6] + raw_bytes[7:13]:
            c = b >> 1
            if not (c == 0x20 or 0x30 <= c <= 0x39 or 0x41 <= c <= 0x5A):
                return False
        return True

    for k in range(len(flags) - 1):
        start = flags[k] + 8
        end = flags[k + 1]
        if end - start < (14 + 2 + 2) * 8:
            continue
        raw = destuff_and_pack(bits, start, end)
        if raw is None or len(raw) < 16:
            continue
        fcs_rx = raw[-2] | (raw[-1] << 8)
        fcs_calc = ax25_fcs(raw[:-2])
        if fcs_rx != fcs_calc:
            continue
        if not callsign_chars_look_valid(raw):
            continue   # FCS-coincidence false positive
        if raw not in seen:
            seen.add(raw)
            out.append((raw, True))

    if not accept_partial or out:
        return out

    # ---- Partial: longest run of leading flags → forward destuff ----
    # Find the longest run of consecutive flag bytes (the leading preamble).
    longest_run_end = 0
    longest_run_len = 0
    i = 0
    while i < len(flags):
        run_len = 1
        while i + run_len < len(flags) and flags[i + run_len] == flags[i] + 8 * run_len:
            run_len += 1
        if run_len > longest_run_len:
            longest_run_len = run_len
            longest_run_end = flags[i] + 8 * run_len
        i += run_len
    if longest_run_len < 4:
        return out   # no plausible leading preamble found

    # Destuff from end of preamble through the rest of the bit stream.
    # Partial mode — don't require byte alignment (tail is post-TX noise).
    raw = destuff_and_pack(bits, longest_run_end, len(bits),
                           require_aligned=False)
    if raw is None or len(raw) < 16:
        return out

    # Validate by inspecting the would-be address field: first 14 bytes
    # >> 1 should be space-padded ASCII (callsign chars). If they are,
    # treat the prefix as a real (FCS-unchecked) frame.
    addr_chars = [b >> 1 for b in raw[:14]]
    if all(0x20 <= c <= 0x7E for c in addr_chars):
        if raw not in seen:
            seen.add(raw)
            out.append((raw, False))
    return out


def parse_ax25_ui(frame, fcs_ok=True):
    """Parse a UI frame. In partial mode (fcs_ok=False) the info field is
    truncated to its leading printable-ASCII run — everything after that
    point is post-TX noise that the destuffer chewed through, not part of
    the actual frame."""
    if len(frame) < 17:
        return None
    try:
        dst = ''.join(chr(b >> 1) for b in frame[:6]).rstrip()
        dst_ssid = (frame[6] >> 1) & 0x0F
        src = ''.join(chr(b >> 1) for b in frame[7:13]).rstrip()
        src_ssid = (frame[13] >> 1) & 0x0F
    except ValueError:
        return None
    ctrl = frame[14]
    pid = frame[15]
    if fcs_ok:
        info_bytes = bytes(frame[16:-2])
        fcs_bytes = bytes(frame[-2:])
        raw = bytes(frame)
    else:
        info_raw = bytes(frame[16:])
        cut = len(info_raw)
        for i, b in enumerate(info_raw):
            if not (0x20 <= b <= 0x7E):
                cut = i
                break
        info_bytes = info_raw[:cut]
        fcs_bytes = b''
        # Trimmed raw = addr(14) + ctrl + pid + info_printable
        raw = bytes(frame[:16]) + info_bytes
    return {
        'dst': dst, 'dst_ssid': dst_ssid,
        'src': src, 'src_ssid': src_ssid,
        'ctrl': ctrl, 'pid': pid,
        'info': info_bytes,
        'fcs': fcs_bytes,
        'fcs_ok': fcs_ok,
        'raw': raw,
    }


def _hexdump(data, width=16):
    """Return a multi-line hex+ASCII dump of `data`."""
    lines = []
    for off in range(0, len(data), width):
        chunk = data[off:off + width]
        hexpart = ' '.join(f'{b:02x}' for b in chunk)
        hexpart = hexpart.ljust(width * 3 - 1)
        asciipart = ''.join(chr(b) if 0x20 <= b <= 0x7E else '.' for b in chunk)
        lines.append(f"      {off:04x}  {hexpart}  {asciipart}")
    return '\n'.join(lines)


# ============================================================
# Bit slicing — brute-force timing offset search
# ============================================================

def decode_buffer(freq_demod, matched_filter):
    """Run the full pipeline against an FM-demod buffer. Returns a list of
    (frame_bytes, fcs_ok) tuples — fcs_ok=True for fully validated frames,
    False for partial decodes (long leading-flag run found + plausible
    callsign address but no trailing flag / FCS check)."""
    freq_f = oaconvolve(freq_demod, matched_filter, mode='same').astype(np.float32)

    out = []
    seen = set()

    for t in range(SPS):
        centers = np.arange(t + SPS // 2, len(freq_f), SPS)
        if len(centers) < 200:
            continue
        bits = (freq_f[centers] > 0).astype(np.int8)
        if bits.sum() == 0 or bits.sum() == len(bits):
            continue

        scrambled = descramble_g3ruh(bits)
        data_bits = un_nrzi(scrambled)
        for frame, fcs_ok in extract_ax25_frames(data_bits):
            key = frame[:32]  # dedupe by first 32 bytes (addr+ctrl+pid+early info)
            if key in seen:
                continue
            seen.add(key)
            out.append((frame, fcs_ok))
    return out


# ============================================================
# Main RX loop
# ============================================================

def receive(uri, rx_gain, duration):
    print(f"Connecting to Pluto SDR at {uri}...")
    try:
        sdr = adi.Pluto(uri=uri)
    except Exception as e:
        print(f"ERROR: cannot connect to Pluto: {e}")
        sys.exit(1)

    sdr.rx_lo = CENTER_FREQ + PLUTO_XO_CORRECTION - LO_OFFSET
    sdr.rx_rf_bandwidth = SAMPLE_RATE
    sdr.gain_control_mode_chan0 = 'manual'
    sdr.rx_hardwaregain_chan0 = rx_gain
    sdr.sample_rate = SAMPLE_RATE
    # 131072 samples ≈ 137 ms. A 100-char AX.25 info field = ~150 encoded
    # bytes = 1250 bits = 130 ms at 9600 baud, so one buffer holds a frame
    # comfortably. Rolling-2 window below covers buffer-straddling frames.
    BUF_SIZE = 2 ** 17
    sdr.rx_buffer_size = BUF_SIZE

    print(f"  Frequency   : {CENTER_FREQ / 1e6:.3f} MHz")
    print(f"  LO offset   : +{LO_OFFSET / 1e3:.0f} kHz (carrier expected at this offset)")
    print(f"  Data rate   : {DATA_RATE} bps")
    print(f"  Deviation   : ±{FREQ_DEVIATION} Hz")
    print(f"  RX gain     : {rx_gain} dB")
    print(f"  Sample rate : {SAMPLE_RATE / 1e3:.0f} kHz ({SPS} samp/sym)")
    print(f"  Buffer      : {BUF_SIZE} samples ({BUF_SIZE / SAMPLE_RATE * 1e3:.0f} ms)")
    print(f"  Duration    : {duration:.0f} s")
    print()

    # Narrow LPF — passes the ±3 kHz deviation comfortably, rejects adjacent noise.
    lpf = firwin(501, 10000 / (SAMPLE_RATE / 2))
    matched = gaussian_filter(0.5, SPS, span=2)

    # Flush settling buffers
    for _ in range(5):
        sdr.rx()

    # Session dedup map: gkey -> length of longest info seen
    seen_frames = {}
    pkt_count = 0
    buf_count = 0
    start = time.time()
    prev_freq = None

    print("Listening... (Ctrl+C to stop)")
    print("-" * 72)

    try:
        while time.time() - start < duration:
            iq = sdr.rx().astype(np.complex128)
            buf_count += 1
            fc = carrier_lock(iq)
            freq_new = downconvert_and_demod(iq, fc, lpf)

            # Rolling 2-buffer concatenation: AX.25 beacon frames are ~130 ms
            # and the buffer is ~137 ms, so a frame can straddle the boundary.
            window = freq_new if prev_freq is None else np.concatenate([prev_freq, freq_new])
            prev_freq = freq_new

            # Decode + collapse duplicates (same timing-jittered decode of one
            # beacon often produces several near-identical partials with the
            # same printable info prefix — keep the longest, drop the rest).
            new_decodes = decode_buffer(window, matched)
            parsed_list = []
            for frame, fcs_ok in new_decodes:
                p = parse_ax25_ui(frame, fcs_ok=fcs_ok)
                if p is None:
                    continue
                # Reject decodes with implausibly short info — these are
                # mostly bit-error artifacts at the addr/ctrl boundary.
                if not fcs_ok and len(p['info']) < 8:
                    continue
                parsed_list.append((p, fcs_ok))

            # Dedup by (src, dst, info bytes) — different timing offsets that
            # recovered the same prefix collapse to one. Keep the longest.
            by_key = {}
            for p, fcs_ok in parsed_list:
                key = (p['src'], p['src_ssid'], p['dst'], p['dst_ssid'], p['info'])
                if key in by_key:
                    continue
                # Squash if some other entry has this info as a prefix and is longer.
                squashed = False
                for k2 in list(by_key.keys()):
                    if k2[:4] == key[:4] and k2[4].startswith(p['info']):
                        squashed = True
                        break
                    if k2[:4] == key[:4] and p['info'].startswith(k2[4]):
                        del by_key[k2]
                if not squashed:
                    by_key[key] = (p, fcs_ok)

            # Session-level dedup: collapse partial decodes of the same beacon
            # that appear in successive rolling windows. We key on the first
            # 16 chars of info (typically "KO6IKZ BCN seq=N "), and only keep
            # the longest variant seen so far for that prefix.
            for (p, fcs_ok) in by_key.values():
                gkey = (p['src'], p['src_ssid'], p['info'][:16])
                prev = seen_frames.get(gkey)
                if prev is not None and len(p['info']) <= prev:
                    continue
                seen_frames[gkey] = len(p['info'])
                pkt_count += 1

                elapsed = time.time() - start
                tag = "AX.25" if fcs_ok else "AX.25 (partial)"
                print(f"\n  >>> {tag} #{pkt_count}  t={elapsed:5.1f}s  "
                      f"fc={fc/1e3:.1f} kHz")
                print(f"      {p['src']}-{p['src_ssid']} → "
                      f"{p['dst']}-{p['dst_ssid']}   "
                      f"ctrl=0x{p['ctrl']:02X} pid=0x{p['pid']:02X}")
                info_ascii = p['info'].decode('ascii', errors='replace')
                print(f"      INFO ({len(p['info'])} B): \"{info_ascii}\"")
                if p['fcs']:
                    print(f"      FCS: {p['fcs'].hex()}  "
                          f"({'OK' if fcs_ok else 'BAD'})")
                print(f"      --- hex dump (addr+ctrl+pid+info"
                      f"{'+fcs' if fcs_ok else ''}) ---")
                print(_hexdump(p['raw']))
                print()

            if buf_count % 10 == 0:
                elapsed = time.time() - start
                fft_pwr = np.abs(np.fft.fftshift(np.fft.fft(iq))) ** 2
                fft_f = np.fft.fftshift(np.fft.fftfreq(len(iq), 1.0 / SAMPLE_RATE))
                cm = (fft_f > 70_000) & (fft_f < 130_000)
                sig_peak = float(np.max(fft_pwr[cm])) if cm.any() else 0.0
                noise_med = float(np.median(fft_pwr))
                snr_db = 10 * np.log10(sig_peak / noise_med) if noise_med > 0 else 0.0
                print(f"  [{elapsed:5.1f}s] buf={buf_count}, frames={pkt_count}, "
                      f"carrier={fc/1e3:.1f} kHz, snr={snr_db:.1f} dB")

    except KeyboardInterrupt:
        print("\nStopping RX...")

    elapsed = time.time() - start
    print("-" * 72)
    print(f"Stopped. {pkt_count} unique frames in {elapsed:.1f}s ({buf_count} buffers).")


def main():
    p = argparse.ArgumentParser(description="Pluto SDR AX.25 / G3RUH 9k6 RX")
    p.add_argument("--uri", default=PLUTO_URI, help=f"default {PLUTO_URI}")
    p.add_argument("--rx-gain", type=float, default=60, help="RX gain dB (default 60)")
    p.add_argument("--duration", type=float, default=60, help="listen seconds (default 60)")
    args = p.parse_args()
    receive(args.uri, args.rx_gain, args.duration)


if __name__ == "__main__":
    main()
