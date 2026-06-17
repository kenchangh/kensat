#!/usr/bin/env python3
"""
PlutoSDR live waterfall/spectrogram — 437 MHz band.

Shows real-time scrolling waterfall of the RF spectrum around 437.08 MHz.
GFSK bursts appear as bright horizontal stripes near the carrier marker.

Modes:
  Si4463 TX mode (default):
    --port triggers Si4463 TX via UART ('x'/'q'), Pluto RX shows waterfall.
    --burst-period pulses TX on/off for discrete visible bursts.

  Pluto TX mode (--tx):
    Pluto transmits GFSK packets AND simultaneously shows the waterfall.
    Full-duplex: TX cyclic DMA + RX streaming. You'll see your own
    transmitted packets on the waterfall. Use --port to put Si4463 in
    RX mode ('r' command).

Usage:
    # Si4463 TX -> Pluto RX waterfall
    python pluto_waterfall.py --port /dev/tty.usbserial-BG03U9VQ

    # Pluto TX -> waterfall (see your own GFSK bursts)
    python pluto_waterfall.py --tx --tx-gain -10 --packets 5 --gap-ms 500

    # Pluto TX + Si4463 RX
    python pluto_waterfall.py --tx --port /dev/tty.usbserial-BG03U9VQ

Requirements:
    pip install pyadi-iio numpy matplotlib scipy pyserial
"""

import argparse
import struct
import sys
import time
import threading

import adi
import numpy as np
from scipy.signal import firwin
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# ============================================================
# RF Configuration (must match Si4463 / pluto_rx.py)
# ============================================================

PLUTO_URI = "ip:192.168.3.1"
CENTER_FREQ = 437_080_000       # 437.08 MHz
PLUTO_XO_CORRECTION = 26000     # Pluto crystal offset
DATA_RATE = 9600                # bps
FREQ_DEVIATION = 4800           # +/-4.8 kHz
BT = 0.5                       # Gaussian filter BT product
SAMPLE_RATE = 960_000           # 960 kHz
LO_OFFSET = 100_000             # Baseband carrier offset
BUF_SIZE = 2**16                # 65536 samples = 68.3 ms
SYNC_WORD = 0x2DD4              # 16-bit sync word
MAX_PACKET_LEN = 64
PREAMBLE_BYTES = 64
SAMPLES_PER_SYMBOL = SAMPLE_RATE / DATA_RATE  # 100

CALLSIGN = "KO6IKZ"

UART_PORT = "/dev/tty.usbserial-BG03U9VQ"
UART_BAUD = 9600

DEFAULT_FFT_SIZE = 1024
DEFAULT_WATERFALL_ROWS = 200
DEFAULT_RX_GAIN = 60
FLUSH_BUFFERS = 5


# ============================================================
# UART control
# ============================================================

def open_uart(port):
    """Open UART to PIC24 for TX/RX control."""
    import serial
    try:
        ser = serial.Serial(port, UART_BAUD, timeout=0.1)
        time.sleep(0.2)
        print(f"  UART opened: {port}")
        return ser
    except Exception as e:
        print(f"  WARNING: Cannot open UART {port}: {e}")
        print(f"  Running without UART control.")
        return None


def uart_send(ser, cmd, label=""):
    """Send a single character command to PIC24."""
    if ser is None:
        return
    ser.write(cmd.encode('ascii'))
    ser.flush()
    if label:
        print(f"  [UART] Sent '{cmd}' — {label}")


class BurstPulser(threading.Thread):
    """Periodically toggles Si4463 TX on/off so bursts are visible."""

    def __init__(self, ser, period, stop_event):
        super().__init__(daemon=True)
        self.ser = ser
        self.period = period
        self.stop_event = stop_event

    def run(self):
        time.sleep(self.period / 2)
        while not self.stop_event.is_set():
            uart_send(self.ser, 'x', 'TX on')
            if self.stop_event.wait(timeout=1.0):
                break
            uart_send(self.ser, 'q', 'TX off')
            if self.stop_event.wait(timeout=max(0.1, self.period - 1.0)):
                break


# ============================================================
# GFSK Modulator (from pluto_tx.py)
# ============================================================

def crc16_itu_t(data: bytes) -> int:
    crc = 0x0000
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc


def bytes_to_bits_msb(data: bytes) -> np.ndarray:
    bits = []
    for byte in data:
        for i in range(7, -1, -1):
            bits.append((byte >> i) & 1)
    return np.array(bits, dtype=np.float64)


def build_packet_bits(payload: bytes) -> np.ndarray:
    preamble = bytes([0xAA] * PREAMBLE_BYTES)
    sync = struct.pack(">H", SYNC_WORD)
    length = bytes([len(payload)])
    crc = crc16_itu_t(length + payload)
    crc_bytes = struct.pack(">H", crc)
    packet = preamble + sync + length + payload + crc_bytes
    return bytes_to_bits_msb(packet)


def gaussian_filter_tx(bt, sps, span=4):
    n_taps = int(2 * span * sps) + 1
    t = np.arange(n_taps) / sps - span
    alpha = np.sqrt(2 * np.log(2)) / bt
    h = np.sqrt(2 * np.pi) / alpha * np.exp(-2 * (np.pi * t / alpha) ** 2)
    h /= np.sum(h)
    return h


def gfsk_modulate(bits: np.ndarray) -> np.ndarray:
    nrz = 2.0 * bits - 1.0
    sps = int(round(SAMPLES_PER_SYMBOL))
    upsampled = np.repeat(nrz, sps)
    g = gaussian_filter_tx(BT, sps)
    filtered = np.convolve(upsampled, g, mode="same")
    phase = 2 * np.pi * FREQ_DEVIATION * np.cumsum(filtered) / SAMPLE_RATE
    iq = np.exp(1j * phase)
    iq = iq * (2**14 - 1)
    return iq.astype(np.complex64)


def build_tx_buffer(packets, gap_ms):
    """Build cyclic TX buffer with N GFSK packets separated by gaps."""
    gap_samples = int(SAMPLE_RATE * gap_ms / 1000)
    segments = []
    for i in range(packets):
        payload = f"{CALLSIGN} {i:04d}".encode("ascii")
        bits = build_packet_bits(payload)
        iq = gfsk_modulate(bits)
        segments.append(iq)
        segments.append(np.zeros(gap_samples, dtype=np.complex64))
        pkt_ms = len(iq) / SAMPLE_RATE * 1000
        print(f"  TX pkt [{i}] '{CALLSIGN} {i:04d}' — {pkt_ms:.0f} ms")
    composite = np.concatenate(segments)
    cycle_ms = len(composite) / SAMPLE_RATE * 1000
    print(f"  TX buffer: {len(composite)} samples ({cycle_ms:.0f} ms/cycle)")
    return composite


# ============================================================
# SDR Setup
# ============================================================

def setup_sdr(uri, rx_gain, tx_mode=False, tx_gain=-10):
    """Connect to PlutoSDR and configure RX (and optionally TX)."""
    print(f"Connecting to Pluto SDR at {uri}...")
    try:
        sdr = adi.Pluto(uri=uri)
    except Exception as e:
        print(f"ERROR: Cannot connect to Pluto SDR: {e}")
        sys.exit(1)

    # RX config
    sdr.rx_lo = CENTER_FREQ + PLUTO_XO_CORRECTION - LO_OFFSET
    sdr.rx_rf_bandwidth = SAMPLE_RATE
    sdr.gain_control_mode_chan0 = 'manual'
    sdr.rx_hardwaregain_chan0 = rx_gain
    sdr.sample_rate = SAMPLE_RATE
    sdr.rx_buffer_size = BUF_SIZE

    print(f"  RX LO       : {sdr.rx_lo / 1e6:.3f} MHz")
    print(f"  RX gain     : {rx_gain} dB")

    # TX config (full-duplex)
    if tx_mode:
        sdr.tx_lo = CENTER_FREQ + PLUTO_XO_CORRECTION
        sdr.tx_rf_bandwidth = SAMPLE_RATE
        sdr.tx_hardwaregain_chan0 = tx_gain
        sdr.tx_cyclic_buffer = True
        print(f"  TX LO       : {sdr.tx_lo / 1e6:.3f} MHz")
        print(f"  TX gain     : {tx_gain} dB")

    print(f"  Sample rate : {SAMPLE_RATE / 1e3:.0f} kHz")
    print(f"  Buffer      : {BUF_SIZE} samples ({BUF_SIZE / SAMPLE_RATE * 1000:.1f} ms)")

    # Flush initial buffers (ADC settling)
    print("  Flushing ADC...")
    for _ in range(FLUSH_BUFFERS):
        sdr.rx()

    return sdr


# ============================================================
# Spectrum Computation
# ============================================================

def compute_freq_axis(fft_size):
    """Return RF frequency axis in MHz for fftshift'd FFT output."""
    freqs_bb = np.fft.fftshift(np.fft.fftfreq(fft_size, 1.0 / SAMPLE_RATE))
    rx_lo = CENTER_FREQ + PLUTO_XO_CORRECTION - LO_OFFSET
    return (freqs_bb + rx_lo) / 1e6


def compute_spectrum_linear(iq, fft_size, window):
    """Welch-like averaged power spectrum (linear scale).

    Splits the IQ buffer into segments of fft_size, applies window,
    FFTs, and averages the power across all segments.
    DC bin and neighbors are interpolated to remove PlutoSDR LO leakage.
    """
    n_segments = len(iq) // fft_size
    if n_segments == 0:
        return np.ones(fft_size) * 1e-20
    iq_trimmed = iq[:n_segments * fft_size].reshape(n_segments, fft_size)
    windowed = iq_trimmed * window[np.newaxis, :]
    ffts = np.fft.fftshift(np.fft.fft(windowed, axis=1), axes=1)
    power = np.mean(np.abs(ffts) ** 2, axis=0)
    # Null DC leakage: interpolate center bins from neighbors
    c = fft_size // 2
    dc_width = max(1, fft_size // 256)
    lo = max(0, c - dc_width)
    hi = min(fft_size, c + dc_width + 1)
    if lo > 0 and hi < fft_size:
        power[lo:hi] = np.linspace(power[lo - 1], power[hi], hi - lo)
    return power


def compute_spectrum_db(iq, fft_size, window, noise_floor_db=None):
    """Power spectrum in dB, optionally with noise floor subtracted."""
    power = compute_spectrum_linear(iq, fft_size, window)
    power_db = 10.0 * np.log10(power + 1e-20)
    if noise_floor_db is not None:
        power_db -= noise_floor_db
    return power_db


def capture_noise_floor(sdr, n_buffers, fft_size, window):
    """Average n_buffers in linear domain, return noise floor in dB."""
    print(f"  Capturing noise floor ({n_buffers} buffers)...")
    accum = np.zeros(fft_size)
    for _ in range(n_buffers):
        iq = sdr.rx().astype(np.complex128)
        accum += compute_spectrum_linear(iq, fft_size, window)
    avg = accum / n_buffers
    return 10.0 * np.log10(avg + 1e-20)


# ============================================================
# Main
# ============================================================

def main():
    parser = argparse.ArgumentParser(
        description="PlutoSDR live waterfall — 437 MHz band"
    )
    parser.add_argument("--uri", type=str, default=PLUTO_URI,
                        help=f"Pluto SDR URI (default: {PLUTO_URI})")
    parser.add_argument("--rx-gain", type=float, default=DEFAULT_RX_GAIN,
                        help=f"RX gain in dB (default: {DEFAULT_RX_GAIN})")
    parser.add_argument("--duration", type=float, default=0,
                        help="Duration in seconds (0 = infinite, default: 0)")
    parser.add_argument("--fft-size", type=int, default=DEFAULT_FFT_SIZE,
                        help=f"FFT size per segment (default: {DEFAULT_FFT_SIZE})")
    parser.add_argument("--waterfall-rows", type=int, default=DEFAULT_WATERFALL_ROWS,
                        help=f"Number of time rows (default: {DEFAULT_WATERFALL_ROWS})")
    parser.add_argument("--colormap", type=str, default="inferno",
                        help="Matplotlib colormap (default: inferno)")
    parser.add_argument("--noise-subtract", type=int, default=0,
                        help="Average N initial buffers as noise floor (0 = off)")
    parser.add_argument("--vmin", type=float, default=None,
                        help="Color scale min in dB (default: auto)")
    parser.add_argument("--vmax", type=float, default=None,
                        help="Color scale max in dB (default: auto)")
    parser.add_argument("--port", type=str, default=None,
                        help="PIC24 UART port for Si4463 control")
    parser.add_argument("--burst-period", type=float, default=0,
                        help="Pulse Si4463 TX on/off every N seconds (0 = continuous)")

    # Pluto TX mode
    parser.add_argument("--tx", action="store_true",
                        help="Enable Pluto TX (full-duplex: TX + waterfall)")
    parser.add_argument("--tx-gain", type=float, default=-10,
                        help="Pluto TX gain in dB (default: -10)")
    parser.add_argument("--packets", type=int, default=5,
                        help="Number of GFSK packets in TX buffer (default: 5)")
    parser.add_argument("--gap-ms", type=float, default=500,
                        help="Gap between TX packets in ms (default: 500)")
    args = parser.parse_args()

    fft_size = args.fft_size
    waterfall_rows = args.waterfall_rows

    # --- UART ---
    ser = None
    if args.port:
        ser = open_uart(args.port)

    # --- SDR ---
    sdr = setup_sdr(args.uri, args.rx_gain, tx_mode=args.tx,
                    tx_gain=args.tx_gain)

    # --- Pre-compute ---
    window = np.hanning(fft_size)
    freqs_mhz = compute_freq_axis(fft_size)

    # --- Stop Si4463 TX before calibration ---
    if ser and not args.tx:
        uart_send(ser, 'q', 'stop Si4463 TX for calibration')
        time.sleep(0.5)
        for _ in range(5):
            sdr.rx()

    # --- Put Si4463 in RX mode if we're doing Pluto TX ---
    if ser and args.tx:
        uart_send(ser, 'r', 'Si4463 RX mode')
        time.sleep(0.3)

    noise_floor_db = None
    if args.noise_subtract > 0:
        noise_floor_db = capture_noise_floor(sdr, args.noise_subtract,
                                             fft_size, window)

    # --- Auto-scale: measure noise floor (before TX starts) ---
    print("  Measuring noise level for color scale...")
    cal_levels = []
    for _ in range(10):
        iq = sdr.rx().astype(np.complex128)
        spec = compute_spectrum_db(iq, fft_size, window, noise_floor_db)
        cal_levels.append(np.median(spec))
    noise_median = np.median(cal_levels)

    vmin = args.vmin if args.vmin is not None else noise_median - 3
    vmax = args.vmax if args.vmax is not None else noise_median + 30
    print(f"  Noise median: {noise_median:.1f} dB -> scale [{vmin:.0f}, {vmax:.0f}] dB")

    # --- Start TX after calibration ---
    stop_event = threading.Event()
    pulser = None

    if args.tx:
        # Pluto TX: build GFSK buffer, start cyclic DMA
        print()
        print("Building Pluto TX buffer...")
        tx_buf = build_tx_buffer(args.packets, args.gap_ms)
        print("Starting Pluto TX (full-duplex)...")
        sdr.tx(tx_buf)
    elif ser:
        # Si4463 TX mode
        if args.burst_period > 0:
            print(f"  Burst mode: Si4463 TX pulses every {args.burst_period}s")
            pulser = BurstPulser(ser, args.burst_period, stop_event)
            pulser.start()
        else:
            uart_send(ser, 'x', 'start Si4463 continuous TX')

    # --- Waterfall data array ---
    waterfall_data = np.full((waterfall_rows, fft_size), vmin,
                            dtype=np.float64)

    # --- Figure setup ---
    fig, ax = plt.subplots(figsize=(12, 6))
    extent = [freqs_mhz[0], freqs_mhz[-1], waterfall_rows, 0]
    img = ax.imshow(waterfall_data, aspect='auto', extent=extent,
                    cmap=args.colormap, vmin=vmin, vmax=vmax,
                    interpolation='nearest', origin='upper')

    # Carrier marker at 437.08 MHz
    ax.axvline(x=CENTER_FREQ / 1e6, color='white', linestyle='--',
               linewidth=0.8, alpha=0.7)
    ax.text(CENTER_FREQ / 1e6, 3, ' 437.08', color='white', fontsize=8,
            alpha=0.7, va='top')

    mode_str = "Pluto TX + RX" if args.tx else "Si4463 TX"
    ax.set_xlabel('Frequency (MHz)')
    ax.set_ylabel('Time (newest at bottom)')
    ax.set_title(f'PlutoSDR Waterfall — 437 MHz Band ({mode_str})')
    fig.colorbar(img, ax=ax, label='Power (dB)', shrink=0.8)
    fig.tight_layout()

    # --- State for animation ---
    start_time = time.time()
    frame_count = [0]

    def update(frame):
        if args.duration > 0 and (time.time() - start_time) > args.duration:
            plt.close(fig)
            return [img]

        iq = sdr.rx().astype(np.complex128)
        power_db = compute_spectrum_db(iq, fft_size, window, noise_floor_db)

        waterfall_data[:-1] = waterfall_data[1:]
        waterfall_data[-1] = power_db
        img.set_data(waterfall_data)

        frame_count[0] += 1
        if frame_count[0] % 50 == 0:
            elapsed = time.time() - start_time
            fps = frame_count[0] / elapsed
            print(f"  [{elapsed:.0f}s] {frame_count[0]} frames, {fps:.1f} fps")

        return [img]

    print()
    print("Waterfall running... close window or Ctrl+C to stop.")
    print(f"  Mode        : {mode_str}")
    print(f"  FFT size    : {fft_size} ({SAMPLE_RATE/fft_size:.0f} Hz/bin)")
    print(f"  Segments    : {BUF_SIZE // fft_size} averaged per row")
    print(f"  Time/row    : {BUF_SIZE / SAMPLE_RATE * 1000:.1f} ms")
    print(f"  Freq range  : {freqs_mhz[0]:.3f} — {freqs_mhz[-1]:.3f} MHz")
    print()

    ani = FuncAnimation(fig, update, interval=1, blit=True,
                        cache_frame_data=False)

    try:
        plt.show()
    except KeyboardInterrupt:
        pass

    # Cleanup
    stop_event.set()
    if args.tx:
        sdr.tx_destroy_buffer()
        print("  Pluto TX stopped.")
    if ser:
        uart_send(ser, 'q', 'stop')
        ser.close()

    elapsed = time.time() - start_time
    print(f"\nStopped. {frame_count[0]} frames in {elapsed:.1f}s.")


if __name__ == "__main__":
    main()
