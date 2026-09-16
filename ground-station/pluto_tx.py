#!/usr/bin/env python3
"""
Pluto SDR GFSK transmitter for kensat_radio RF board.

Generates Si4463-compatible GFSK packets at 437.08 MHz and transmits
them via ADALM-Pluto SDR in cyclic mode. The Si4463 on the RF board
(in RX mode) will decode these packets.

Packet format (matches the OBC uplink config, si4463_pkt_cfg_rx()):
    [Preamble] [Sync 0x2DD4] [Len 2B, big-endian] [Payload ≤300B] [CRC-16 2B]

The CRC covers the length bytes + payload.  --len-bytes 1 gives the older
1-byte-length frame (payload ≤64B) used by the ESP32 radio bench firmware.

Telecommands (--cmd / --prompt) are "KO6IKZ <auth> <VERB>[ <ARG>]".  A
prompt goes up in ONE frame:

    "KO6IKZ <auth> PROMPT " + 256-byte field (printable ASCII, NUL-padded)

and the OBC runs inference on it as soon as it arrives, acknowledging on the
AX.25 downlink with "ACK PROMPT <n>B h=<crc32>".  This script prints the
CRC-32 to expect; stop sending once you hear it (a repeat after the run has
finished would run the prompt again).

Preamble: the OBC polls the radio every 400 ms and the Si4463 RX FIFO is
only 64 B, so a ~280 B PROMPT frame is drained while it arrives -- which only
works if the poll sees the preamble first.  The default 768 B preamble
(640 ms) outlasts the poll period with margin; don't shorten it for uplink.

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
    python pluto_tx.py --packets 10 --gap-ms 200 --tx-gain 0   # test packets, loops
    python pluto_tx.py --cmd PING                               # one telecommand x N
    python pluto_tx.py --prompt "Describe the Earth from orbit."
    python pluto_tx.py --prompt-file prompt.txt --repeat 10
    python pluto_tx.py --selftest                               # no radio needed

Requirements:
    pip install pyadi-iio numpy scipy
"""

import argparse
import binascii
import os
import struct
import sys
import time
import zlib

import numpy as np

# pyadi-iio is imported in transmit() so --selftest runs without it.

# ============================================================
# RF Configuration (must match Si4463 config.h)
# ============================================================

PLUTO_URI = "ip:192.168.3.1"
CENTER_FREQ = 437_080_000       # 437.08 MHz
PLUTO_XO_CORRECTION = 26000     # Pluto crystal offset (empirically tuned)
DATA_RATE = 9600                # bps
FREQ_DEVIATION = 4800           # ±4.8 kHz
BT = 0.5                       # Gaussian filter bandwidth-time product
PREAMBLE_BYTES = 768           # 768 B of 0xAA = 640 ms: must outlast the OBC's 400 ms RX poll
SYNC_WORD = 0x2DD4              # 16-bit sync word
MAX_PAYLOAD_LEN = {1: 64,       # 1-byte length: ESP32 bench radio (FIFO-bound)
                   2: 300}      # 2-byte length: OBC uplink (SI4463_RX_MAX_PAYLOAD)

CALLSIGN = "KO6IKZ"

# Telecommand prefix token (UPLINK_AUTH in firmware/obc/app/task_beacon.c).
# Set KENSAT_UPLINK_AUTH once the flight value is changed and kept off the repo.
UPLINK_AUTH = os.environ.get("KENSAT_UPLINK_AUTH", "KENSAT1")

PROMPT_FIELD_LEN = 256          # UP_PROMPT_FIELD in task_beacon.c

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

def build_frame(payload: bytes, len_bytes: int = 2) -> bytes:
    """Si4463 packet-handler frame, without the preamble.

    Structure:
        [Sync 2B] [Length 1B or 2B big-endian] [Payload] [CRC-16 2B]

    All bytes sent MSB first — Si4463 CRC validates correctly with MSB-first.
    The CRC covers the length bytes + payload (matching Si4463 field config).
    """
    if len_bytes not in MAX_PAYLOAD_LEN:
        raise ValueError(f"len_bytes must be 1 or 2, not {len_bytes}")
    if len(payload) > MAX_PAYLOAD_LEN[len_bytes]:
        raise ValueError(f"Payload too long: {len(payload)} > "
                         f"{MAX_PAYLOAD_LEN[len_bytes]}")

    sync = struct.pack(">H", SYNC_WORD)         # MSB first
    length = (struct.pack(">H", len(payload)) if len_bytes == 2
              else bytes([len(payload)]))
    crc_bytes = struct.pack(">H", crc16_itu_t(length + payload))  # MSB first
    return sync + length + payload + crc_bytes


def build_packet_bits(payload: bytes, len_bytes: int = 2,
                      preamble_bytes: int = PREAMBLE_BYTES) -> np.ndarray:
    """Preamble + build_frame(), as a bit array ready to modulate."""
    preamble = bytes([0xAA] * preamble_bytes)
    return bytes_to_bits_msb(preamble + build_frame(payload, len_bytes))


# ============================================================
# Telecommands
# ============================================================

def command_payload(verb: str, auth: str = UPLINK_AUTH) -> bytes:
    """ "KO6IKZ <auth> <verb>" -- e.g. verb "PING", "BCN ON", "RUN 3"."""
    return f"{CALLSIGN} {auth} {verb}".encode("ascii")


def prompt_field(text: str) -> bytes:
    """The PROMPT telecommand's fixed 256-byte field: printable ASCII,
    NUL-padded.  Raises ValueError for anything the OBC would NAK."""
    raw = text.encode("ascii", errors="strict") if text.isascii() else None
    if raw is None:
        raise ValueError("prompt must be ASCII")
    if not raw:
        raise ValueError("prompt is empty")
    if len(raw) > PROMPT_FIELD_LEN:
        raise ValueError(f"prompt is {len(raw)} chars; the field holds "
                         f"{PROMPT_FIELD_LEN}")
    for i, b in enumerate(raw):
        if b < 0x20 or b > 0x7E:
            raise ValueError(f"non-printable byte 0x{b:02X} at {i} "
                             f"(one line, printable ASCII only)")
    return raw + bytes(PROMPT_FIELD_LEN - len(raw))


def prompt_payload(text: str, auth: str = UPLINK_AUTH) -> bytes:
    return command_payload("PROMPT ", auth) + prompt_field(text)


def expected_prompt_ack(text: str) -> str:
    """What the OBC should send back for this prompt."""
    raw = text.encode("ascii")
    return f"ACK PROMPT {len(raw)}B h={zlib.crc32(raw) & 0xFFFFFFFF:08X}"


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
             xo_correction: int = PLUTO_XO_CORRECTION,
             payload: bytes = None, len_bytes: int = 2,
             preamble_bytes: int = PREAMBLE_BYTES):
    """Transmit GFSK packets via Pluto SDR in cyclic mode.

    Builds a single IQ buffer containing `packets` GFSK packets separated
    by `gap_ms` milliseconds of silence, then transmits the buffer in cyclic
    (DMA loop) mode. This avoids the DMA startup transients that corrupt
    non-cyclic short bursts.

    With payload=None the packets are numbered test packets ("KO6IKZ 0000",
    ...) and the buffer loops until Ctrl+C.  With a payload (a telecommand)
    every packet is that payload and the buffer is sent ONCE, then TX
    stops -- looping a PROMPT forever would re-run it after each inference.

    Args:
        uri: Pluto SDR URI (e.g., "ip:192.168.3.1")
        tx_gain_db: TX attenuation in dB (e.g., -10)
        packets: number of packets in the cyclic buffer
        gap_ms: silence gap between packets in milliseconds
    """
    import adi

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
    print(f"  Preamble    : {preamble_bytes} bytes "
          f"({preamble_bytes * 8 / DATA_RATE * 1000:.0f} ms)")
    print(f"  Length field: {len_bytes} byte(s)")
    print(f"  Packets     : {packets}")
    print(f"  Gap         : {gap_ms} ms ({gap_samples} samples)")
    print()

    # Build composite buffer: [pkt0][gap][pkt1][gap]...[pktN-1][gap]
    print("Building composite IQ buffer...")
    segments = []
    cmd_iq = None
    for i in range(packets):
        if payload is None:
            this = f"{CALLSIGN} {i:04d}".encode("ascii")
            iq = gfsk_modulate(build_packet_bits(this, len_bytes, preamble_bytes))
        else:
            this = payload
            if cmd_iq is None:   # identical every time: modulate once
                cmd_iq = gfsk_modulate(build_packet_bits(this, len_bytes,
                                                         preamble_bytes))
            iq = cmd_iq

        shown = this.rstrip(b"\x00").decode("ascii", errors="replace")
        if len(shown) > 60:
            shown = shown[:57] + "..."
        pkt_duration_ms = len(iq) / SAMPLE_RATE * 1000
        print(f"  [{i:04d}] {shown}  "
              f"({len(this)}B payload, {pkt_duration_ms:.1f} ms on-air)")

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
    if payload is None:
        print("Starting cyclic TX... (Ctrl+C to stop)")
    else:
        print(f"Sending {packets} copies once "
              f"({total_duration_ms / 1000:.1f} s)...")
    sdr.tx(composite)

    try:
        if payload is None:
            while True:
                time.sleep(1)
        else:
            # One pass of the cyclic buffer.  Stopping a little late at most
            # clips the start of a second pass, which fails CRC and is dropped.
            time.sleep(total_duration_ms / 1000 + 0.05)
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
    what = parser.add_mutually_exclusive_group()
    what.add_argument("--cmd", type=str,
                      help='send a telecommand, e.g. "PING", "BCN ON", "RUN 3"')
    what.add_argument("--prompt", type=str,
                      help="uplink this LLM prompt (<=256 printable ASCII chars); "
                           "the OBC runs it on arrival")
    what.add_argument("--prompt-file", type=str,
                      help="like --prompt, read from a file (newlines become spaces)")
    what.add_argument("--selftest", action="store_true",
                      help="check the frame builders offline and exit (no radio)")
    parser.add_argument("--repeat", type=int, default=8,
                        help="copies of a --cmd/--prompt frame to send (default: 8)")
    parser.add_argument("--cmd-gap-ms", type=float, default=1500,
                        help="gap between telecommand copies; leaves room for the "
                             "OBC's ACK downlink (default: 1500)")
    parser.add_argument("--auth", type=str, default=UPLINK_AUTH,
                        help="telecommand auth token (default: $KENSAT_UPLINK_AUTH "
                             "or KENSAT1)")
    parser.add_argument("--len-bytes", type=int, choices=(1, 2), default=2,
                        help="frame length-field size: 2 = OBC uplink (default), "
                             "1 = ESP32 bench radio")
    parser.add_argument("--preamble-bytes", type=int, default=PREAMBLE_BYTES,
                        help=f"preamble length (default: {PREAMBLE_BYTES} = "
                             f"{PREAMBLE_BYTES * 8 / DATA_RATE * 1000:.0f} ms)")
    args = parser.parse_args()

    if args.selftest:
        sys.exit(selftest())

    payload = None
    packets, gap_ms = args.packets, args.gap_ms
    if args.prompt is not None or args.prompt_file is not None:
        text = args.prompt
        if text is None:
            with open(args.prompt_file, "r", encoding="ascii") as f:
                text = " ".join(f.read().split())
        try:
            payload = prompt_payload(text, args.auth)
        except ValueError as e:
            parser.error(str(e))
        print(f"Prompt: {len(text)} chars. Expect on the downlink: "
              f"{expected_prompt_ack(text)}")
        print("(\"... dup\" = a repeat of a run already queued; "
              "\"NAK PROMPT busy\" = payload busy with something else)\n")
    elif args.cmd is not None:
        payload = command_payload(args.cmd, args.auth)
    if payload is not None:
        packets, gap_ms = args.repeat, args.cmd_gap_ms
        if args.len_bytes == 1 and len(payload) > MAX_PAYLOAD_LEN[1]:
            parser.error("a telecommand this long needs --len-bytes 2")

    transmit(
        uri=args.uri,
        tx_gain_db=args.tx_gain,
        packets=packets,
        gap_ms=gap_ms,
        xo_correction=args.xo_correction,
        payload=payload,
        len_bytes=args.len_bytes,
        preamble_bytes=args.preamble_bytes,
    )


# ============================================================
# Self-test (no radio)
# ============================================================

def parse_frame(frame: bytes, len_bytes: int = 2) -> bytes:
    """Receive-side model of si4463_rx_poll(): returns the payload, or raises.
    Used only by the self-test, as an independent reading of the frame."""
    if frame[:2] != struct.pack(">H", SYNC_WORD):
        raise ValueError("bad sync")
    hdr = frame[2:2 + len_bytes]
    plen = int.from_bytes(hdr, "big")
    body = frame[2 + len_bytes:2 + len_bytes + plen]
    crc = frame[2 + len_bytes + plen:2 + len_bytes + plen + 2]
    if len(body) != plen or len(crc) != 2:
        raise ValueError("short frame")
    # binascii.crc_hqx is CRC-16/XMODEM: poly 0x1021, init 0 -- the Si4463's
    # CRC_CONFIG 0x05 with seed 0, computed by code that isn't ours.
    if binascii.crc_hqx(hdr + body, 0) != int.from_bytes(crc, "big"):
        raise ValueError("CRC mismatch")
    return body


def selftest() -> int:
    text = "Describe the Earth from orbit in one line."
    pl = prompt_payload(text, "KENSAT1")
    assert len(pl) == 15 + 7 + 256 == 278, len(pl)
    assert pl.startswith(b"KO6IKZ KENSAT1 PROMPT Describe")
    assert pl[22 + len(text):] == bytes(256 - len(text))

    fr = build_frame(pl, 2)
    assert fr[2:4] == b"\x01\x16", fr[2:4]          # 278, big-endian
    assert parse_frame(fr, 2) == pl
    assert len(fr) == 2 + 2 + 278 + 2

    full = prompt_payload("x" * 256, "KENSAT1")       # no NUL at all
    assert full.endswith(b"x" * 256) and parse_frame(build_frame(full), 2) == full

    for bad in ("", "x" * 257, "line\nbreak", "caf\u00e9"):
        try:
            prompt_field(bad)
        except ValueError:
            pass
        else:
            raise AssertionError(f"accepted bad prompt {bad!r}")

    short = command_payload("PING", "KENSAT1")
    assert parse_frame(build_frame(short, 1), 1) == short
    assert parse_frame(build_frame(short, 2), 2) == short
    try:
        build_frame(pl, 1)
    except ValueError:
        pass
    else:
        raise AssertionError("1-byte length accepted a 278 B payload")

    ack = expected_prompt_ack(text)
    assert ack == f"ACK PROMPT 42B h={zlib.crc32(text.encode()):08X}", ack

    on_air_ms = (PREAMBLE_BYTES + len(fr)) * 8 / DATA_RATE * 1000
    print(f"selftest OK: PROMPT frame {len(fr)} B + {PREAMBLE_BYTES} B preamble "
          f"= {on_air_ms:.0f} ms on air; {ack}")
    return 0


if __name__ == "__main__":
    main()
