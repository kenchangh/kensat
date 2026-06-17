#!/usr/bin/env python3
"""
Pluto SDR GFSK receiver for kensat_radio RF board.

Receives Si4463-compatible GFSK packets at 437.08 MHz transmitted
by the Si4463 on the RF board (in TX mode via 'x' command).

Packet format (matches Si4463 config):
    [Preamble 64B] [Sync 0x2DD4] [Len 1B] [Payload ≤64B] [CRC-16 2B]

Modulation:
    GFSK, 9600 bps, ±4.8 kHz deviation, BT=0.5

Key DSP:
    - Per-buffer FFT carrier lock: finds actual carrier in 70-130 kHz range
      (carrier drifts from nominal 100 kHz due to Si4463 crystal tolerance)
    - 10 kHz LPF at 960 kHz sample rate removes adjacent interference
    - FM discriminator normalized to ±1 = ±4.8 kHz deviation
    - Gaussian matched filter (BT=0.5, span=2) on FM demod output
    - Rolling 2-buffer window: the 64B preamble makes packets ~66.7 ms,
      nearly equal to the 68.3 ms buffer period, so packets routinely span
      buffer boundaries — concatenating adjacent buffers captures the CRC
    - Global-position deduplication prevents double-reporting from overlap

Usage:
    python pluto_rx.py [--rx-gain 40] [--duration 30]

Requirements:
    pip install pyadi-iio numpy scipy
"""

import argparse
import sys
import time

import adi
import numpy as np
from scipy.signal import firwin, lfilter, fftconvolve, oaconvolve

# ============================================================
# RF Configuration (must match Si4463 / pluto_tx.py)
# ============================================================

PLUTO_URI = "ip:192.168.3.1"
CENTER_FREQ = 437_080_000       # 437.08 MHz
PLUTO_XO_CORRECTION = 4000      # Pluto crystal offset (carrier lands ~100 kHz in baseband)
DATA_RATE = 9600                # bps
FREQ_DEVIATION = 4800           # ±4.8 kHz
SAMPLE_RATE = 960_000           # 960 kHz
SYNC_WORD = 0x2DD4              # 16-bit sync word (0xB42B on wire, bit-reversed)
MAX_PACKET_LEN = 64
LO_OFFSET = 100_000             # Offset LO from center to avoid DC spike
SPS = SAMPLE_RATE // DATA_RATE  # 100 samples/symbol

CALLSIGN = "KO6IKZ"


# ============================================================
# CRC-16 CCITT (polynomial 0x1021, seed 0x0000)
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
# DSP Functions
# ============================================================

def gaussian_filter(bt: float, sps: int, span: int = 4) -> np.ndarray:
    """Gaussian pulse shape filter matched to GFSK BT."""
    n_taps = int(2 * span * sps) + 1
    t = np.arange(n_taps) / sps - span
    alpha = np.sqrt(2 * np.log(2)) / bt
    h = np.sqrt(2 * np.pi) / alpha * np.exp(-2 * (np.pi * t / alpha) ** 2)
    h /= np.sum(h)
    return h


CARRIER_SEARCH_LO = 50_000
CARRIER_SEARCH_HI = 150_000
CARRIER_LEAK_GUARD = 2_500   # exclude ±this around LO_OFFSET (Pluto LO/DC leakage)
CARRIER_TOL = 40_000


def carrier_lock(iq: np.ndarray) -> float:
    """Carrier lock via power-weighted spectral centroid.

    The Pluto leaks a narrow tone at LO_OFFSET (LO feedthrough / DC) that
    biases peak-based locks. We compute the centroid of the power spectrum
    over [50, 150] kHz, masking out a narrow guard around LO_OFFSET. This is
    naturally robust to GFSK h=1 (suppressed carrier, two sideband peaks),
    h<1 (carrier line plus sidebands), and arbitrary data imbalance.
    Falls back to LO_OFFSET when there's no signal energy.
    """
    n = len(iq)
    fft_data = np.fft.fftshift(np.fft.fft(iq))
    freqs = np.fft.fftshift(np.fft.fftfreq(n, 1.0 / SAMPLE_RATE))
    power = np.abs(fft_data) ** 2

    in_search = (freqs > CARRIER_SEARCH_LO) & (freqs < CARRIER_SEARCH_HI)
    not_leak = (np.abs(freqs - LO_OFFSET) > CARRIER_LEAK_GUARD)
    mask = in_search & not_leak
    if not mask.any():
        return float(LO_OFFSET)

    masked_power = np.where(mask, power, 0.0)
    total = float(np.sum(masked_power))
    if total <= 0.0:
        return float(LO_OFFSET)
    fc = float(np.sum(freqs * masked_power) / total)
    if abs(fc - LO_OFFSET) > CARRIER_TOL:
        return float(LO_OFFSET)
    return fc


def downconvert_and_demod(iq: np.ndarray, fc: float, lpf: np.ndarray) -> np.ndarray:
    """Mix to exact baseband, LPF, FM discriminator, normalize to ±1.

    Returns FM demod output in units of FREQ_DEVIATION (±1 = ±4.8 kHz).
    Length is len(iq) - 1 (differential phase drops one sample).
    """
    n = len(iq)
    bb = iq * np.exp(-1j * 2 * np.pi * fc * np.arange(n) / SAMPLE_RATE)
    bb_filt = lfilter(lpf, 1.0, bb)
    freq = np.angle(bb_filt[1:] * np.conj(bb_filt[:-1]))
    freq = freq * SAMPLE_RATE / (2 * np.pi * FREQ_DEVIATION)
    return np.clip(freq, -2.0, 2.0)


def bits_to_bytes(bits: list) -> bytes:
    """Convert bit list to bytes (MSB first per byte)."""
    out = []
    for i in range(0, len(bits) - 7, 8):
        byte = 0
        for j in range(8):
            byte = (byte << 1) | bits[i + j]
        out.append(byte)
    return bytes(out)


def _track_data_bits(freq_matched: np.ndarray, start_sample: float,
                     polarity: float, n_bits: int, sps: float,
                     threshold: float) -> tuple:
    """Mueller-Muller-tracked symbol slicer for the data portion of a packet.

    Starts at `start_sample` (the first data symbol center, established by
    the brute-force sync search at fixed sps) and walks forward `n_bits`
    symbols, adjusting the step size symbol-by-symbol via decision-directed
    M&M TED. This tracks the residual Si4463-vs-Pluto crystal drift that a
    fixed-grid slice cannot follow over an 80+ bit packet.

    M&M is used (not Gardner) because alternating bit patterns are its
    natural domain — Gardner has a half-symbol-ambiguity failure mode on
    tone-like inputs.

    Returns (bits, end_sps) where bits is a list of 0/1 and end_sps is the
    final tracked symbol period (useful for diagnostics / next-packet
    bootstrap).
    """
    bits = []
    pos = float(start_sample)
    sps_inst = float(sps)
    integ = 0.0
    prev_sym = 0.0
    prev_dec = 0.0
    kp = 0.005
    ki = 0.00005

    n = len(freq_matched)
    for _ in range(n_bits):
        if pos < 0 or pos + 1 >= n:
            break
        ti = int(pos)
        tf = pos - ti
        raw = (1.0 - tf) * freq_matched[ti] + tf * freq_matched[ti + 1]
        sym = raw * polarity
        # Slicer decision (against threshold).
        bit = 1 if sym > threshold else 0
        bits.append(bit)
        dec = 1.0 if bit else -1.0

        # Mueller-Muller TED:  e = (sym - threshold) * prev_dec
        #                        - (prev_sym - threshold) * dec
        ted = (sym - threshold) * prev_dec - (prev_sym - threshold) * dec

        integ += ki * ted
        sps_inst -= kp * ted + integ

        prev_sym = sym
        prev_dec = dec
        pos += sps_inst

    return bits, sps_inst


def find_sync_and_decode(freq: np.ndarray, sps: int) -> list:
    """Brute-force sync search at fixed sps, then M&M timing tracking for data.

    Two-pass design:
      1. Gaussian RX matched filter, then scan all `sps` integer timing
         offsets for the 16-bit sync (Hamming distance ≤ 2). Cheap and
         vectorized; finds the symbol grid origin within ±1 sample.
      2. For each sync candidate, run Mueller-Muller decision-directed
         timing tracking from the first data symbol forward. Each symbol
         step is adjusted by the M&M error so accumulated rate drift
         (Si4463 vs Pluto crystal) doesn't flip later bits.

    CRC is the final validity filter.

    Returns list of (payload, crc_ok, crc_calc, crc_rx, peak_pos).
    """
    g = gaussian_filter(0.5, sps, span=2)
    freq_matched = oaconvolve(freq, g, mode='same').astype(np.float32)

    sync_ref = np.array([(SYNC_WORD >> (15 - i)) & 1 for i in range(16)],
                        dtype=np.float32)
    nrz_sync = 2.0 * sync_ref - 1.0  # ±1

    packets = []
    seen_peak_pos = set()

    for t in range(sps):
        centers = np.arange(t + sps // 2, len(freq_matched), sps)
        if len(centers) < 17:
            continue
        filt_vals = freq_matched[centers].astype(np.float32)

        for polarity in [1.0, -1.0]:
            nrz_bits = filt_vals * polarity
            hard = (nrz_bits > 0).astype(np.float32) * 2.0 - 1.0

            # Vectorized sync search: corr[i] = 16 - 2*HammingDist.
            # Require HD=0 (corr==16): a perfect 16-bit sync match has
            # negligible false-positive rate in noise (~2^-16), so we don't
            # need the noisy HD≤2 candidates that dominated the output.
            corr = np.convolve(hard, nrz_sync[::-1], mode='valid')
            sync_positions = np.where(corr >= 16.0)[0]

            for i in sync_positions:
                data_start_bit = i + 16
                N = len(centers)
                if data_start_bit + 8 > N:
                    continue

                # Preamble (alternating bits → true mean 0) just before sync
                # gives the residual FM-demod DC offset. Slice data against
                # that, not zero, to absorb residual carrier bias.
                pre_lo = max(0, i - 64)
                if i - pre_lo >= 16:
                    threshold = float(np.median(nrz_bits[pre_lo:i]))
                else:
                    threshold = 0.0

                # First 8 bits = length, taken from the fixed-grid slice
                # (8 bits is too short to need timing tracking).
                payload_len = 0
                for b in range(8):
                    payload_len = (payload_len << 1) | (
                        1 if nrz_bits[data_start_bit + b] > threshold else 0)

                if payload_len == 0 or payload_len > MAX_PACKET_LEN:
                    continue

                total_bits = (1 + payload_len + 2) * 8
                if data_start_bit + total_bits > N:
                    continue

                # Track timing across the full data region: M&M starts at
                # the first data symbol center and walks `total_bits` ahead,
                # so accumulated rate drift over 80+ bits doesn't flip bits.
                first_data_sample = float(t + sps // 2 + data_start_bit * sps)
                tracked_bits, _ = _track_data_bits(
                    freq_matched, first_data_sample, polarity,
                    total_bits, float(sps), threshold)

                if len(tracked_bits) < total_bits:
                    continue

                raw_bytes = bits_to_bytes(tracked_bits)
                if len(raw_bytes) < 1 + payload_len + 2:
                    continue

                pkt_bytes = raw_bytes[:1 + payload_len + 2]
                payload = pkt_bytes[1:1 + payload_len]
                crc_rx = (pkt_bytes[-2] << 8) | pkt_bytes[-1]
                crc_calc = crc16_itu_t(pkt_bytes[:1 + payload_len])
                crc_ok = (crc_rx == crc_calc)

                peak_pos = t + i * sps
                if crc_ok:
                    pos_key = peak_pos // sps
                    if pos_key in seen_peak_pos:
                        continue
                    seen_peak_pos.add(pos_key)
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

    # LO placed LO_OFFSET below center so Si4463 carrier lands at ~LO_OFFSET
    # in baseband, away from DC leakage. Per-buffer FFT lock finds actual offset.
    sdr.rx_lo = CENTER_FREQ + PLUTO_XO_CORRECTION - LO_OFFSET
    sdr.rx_rf_bandwidth = SAMPLE_RATE
    sdr.gain_control_mode_chan0 = 'manual'
    sdr.rx_hardwaregain_chan0 = rx_gain
    sdr.sample_rate = SAMPLE_RATE
    BUF_SIZE = 2**16  # 65536 samples = 68.3 ms
    sdr.rx_buffer_size = BUF_SIZE

    print(f"  Frequency   : {CENTER_FREQ / 1e6:.2f} MHz")
    print(f"  LO offset   : +{LO_OFFSET / 1e3:.0f} kHz (carrier expected ~{LO_OFFSET/1e3:.0f} kHz)")
    print(f"  Data rate   : {DATA_RATE} bps")
    print(f"  Deviation   : ±{FREQ_DEVIATION} Hz")
    print(f"  RX gain     : {rx_gain} dB")
    print(f"  Sample rate : {SAMPLE_RATE / 1e3:.0f} kHz, {SPS} samp/sym")
    print(f"  Sync word   : 0x{SYNC_WORD:04X}")
    print(f"  Duration    : {duration}s")
    print()

    # 15 kHz LPF — passes ±9.6 kHz GFSK sidebands, rejects adjacent noise
    lpf = firwin(501, 15000 / (SAMPLE_RATE / 2))

    # Flush initial Pluto buffers (ADC settling)
    for _ in range(5):
        sdr.rx()

    pkt_count = 0
    buf_count = 0
    start = time.time()

    # Rolling 3-buffer FM demod history.
    # Si4463 64B preamble + 16B data = ~64000 samples ≈ 1 buffer (65536).
    # Packets routinely have their CRC in the next buffer, and a packet
    # ending late in the current buffer needs the *following* buffer's
    # samples for M&M timing-tracked CRC slicing. Three buffers cover both:
    # any packet whose preamble starts in the oldest buffer is fully
    # decodable through CRC, regardless of where its tail lands.
    freq_hist = []            # list of FM demod outputs, length ≤ 3
    total_samples = 0         # cumulative count of demod samples received

    # Dedup: global sample position of each found packet (symbol-level precision).
    # The same packet appears at the same global position in consecutive windows,
    # so tracking by position prevents double-reporting from the 2-buffer overlap.
    seen_pos = set()

    print("Listening... (Ctrl+C to stop)")
    print("-" * 60)

    try:
        while time.time() - start < duration:
            iq_raw = sdr.rx()
            buf_count += 1
            iq = iq_raw.astype(np.complex128)

            # Per-buffer FFT carrier lock
            fc = carrier_lock(iq)

            # Downconvert to exact baseband, LPF, FM demod (normalized to ±1)
            freq_new = downconvert_and_demod(iq, fc, lpf)

            # Status every 15 buffers (~1s)
            if buf_count % 15 == 0:
                elapsed = time.time() - start
                fft_pwr = np.abs(np.fft.fftshift(np.fft.fft(iq)))**2
                fft_f = np.fft.fftshift(np.fft.fftfreq(len(iq), 1.0/SAMPLE_RATE))
                carrier_mask = (fft_f > 70000) & (fft_f < 130000)
                sig_peak = float(np.max(fft_pwr[carrier_mask])) if carrier_mask.any() else 0
                noise_med = float(np.median(fft_pwr))
                snr_db = 10*np.log10(sig_peak/noise_med) if noise_med > 0 else 0
                print(f"  [{elapsed:5.1f}s] buf={buf_count}, pkts={pkt_count}, "
                      f"carrier={fc / 1e3:.1f} kHz, "
                      f"snr={snr_db:.1f} dB")

            # Build rolling 3-buffer window.
            # history_start is the global sample index of freq_history[0].
            freq_hist.append(freq_new)
            if len(freq_hist) > 3:
                freq_hist.pop(0)
            freq_history = (np.concatenate(freq_hist)
                            if len(freq_hist) > 1 else freq_hist[0])
            history_start = total_samples - sum(len(b) for b in freq_hist[:-1])

            total_samples += len(freq_new)

            # Search for packets in the combined window
            for payload, crc_ok, crc_calc, crc_rx, local_peak in \
                    find_sync_and_decode(freq_history, SPS):

                # Map local peak position to global sample index for dedup.
                # Only dedup successful packets — a CRC-failed candidate may
                # become a CRC-OK decode in the next iteration's window where
                # the packet's CRC tail is no longer past the buffer end.
                global_peak = history_start + local_peak
                # Dedup tolerance = 1 byte (8 symbols). The timing-offset
                # scan picks slightly different t per iteration, so the same
                # packet's peak_pos can drift ±1 symbol across windows.
                # Real packets are >2000 symbols apart (277 ms cadence), so
                # 8-symbol bins won't collapse distinct packets.
                pos_key = global_peak // (SPS * 8)

                # Soft-accept: rx_crc == 0 means the transmitter did not
                # append CRC bytes (Si4463 packet handler with current config
                # leaves them off-air — the bytes M&M reads in their place
                # are the post-TX silence demod = all zero). With perfect
                # 16/16 sync (HD=0) the false-positive rate is ~2^-16, so a
                # sync-locked candidate with all-printable ASCII payload is
                # the same packet, just without a CRC tail.
                soft_ok = (not crc_ok and crc_rx == 0 and
                           all(0x20 <= b <= 0x7E for b in payload))
                accepted = crc_ok or soft_ok

                if accepted and pos_key in seen_pos:
                    continue

                elapsed = time.time() - start
                # Si4463 TX scrambles the payload by XOR'ing each byte with
                # 0xAA so 9600 bps GFSK doesn't lose bit-clock lock through
                # long zero runs in the beacon struct. Un-scramble for
                # display. CRC was already validated against scrambled bytes
                # above, so this is just the inverse for human-readable output.
                payload_unscr = bytes(b ^ 0xAA for b in payload)
                ascii_str = ''.join(
                    chr(b) if 0x20 <= b <= 0x7E else '.' for b in payload_unscr
                )

                if accepted:
                    seen_pos.add(pos_key)
                    pkt_count += 1
                    crc_status = "OK" if crc_ok else "(none — TX did not append)"
                    print(f"\n  >>> PACKET #{pkt_count} at {elapsed:.1f}s")
                    print(f"      ASCII  : {ascii_str}")
                    print(f"      Hex    : {payload_unscr.hex()}")
                    print(f"      Carrier: {fc / 1e3:.1f} kHz")
                    print(f"      CRC    : {crc_status}")
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
    parser.add_argument("--rx-gain", type=float, default=60,
                        help="RX gain in dB (default: 40)")
    parser.add_argument("--duration", type=float, default=60,
                        help="Listen duration in seconds (default: 60)")
    args = parser.parse_args()

    receive(uri=args.uri, rx_gain=args.rx_gain, duration=args.duration)


if __name__ == "__main__":
    main()
