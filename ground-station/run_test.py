#!/usr/bin/env python3
"""
Combined TX trigger + Pluto RX test script.

Sends 'x' on the PIC24 debug UART to start Si4463 TX, then runs the
Pluto SDR receiver inline. Auto-retriggers TX if it stops (every
RETRIGGER_INTERVAL seconds). Prints a pass/fail summary on exit.

Usage:
    python run_test.py [--port /dev/tty.usbserial-BG03U9VQ] [--rx-gain 60] [--duration 60]
"""

import argparse
import sys
import time
import threading

import serial
import adi
import numpy as np
from scipy.signal import firwin, lfilter, fftconvolve, oaconvolve

# ============================================================
# RF / UART Configuration
# ============================================================

UART_PORT     = "/dev/tty.usbserial-BG03U9VQ"
UART_BAUD     = 9600

PLUTO_URI           = "ip:192.168.3.1"
CENTER_FREQ         = 437_080_000
PLUTO_XO_CORRECTION = 26000
DATA_RATE           = 9600
FREQ_DEVIATION      = 4800
SAMPLE_RATE         = 960_000
SYNC_WORD           = 0x2DD4
MAX_PACKET_LEN      = 64
LO_OFFSET           = 100_000
SPS                 = SAMPLE_RATE // DATA_RATE   # 100

RETRIGGER_INTERVAL  = 9999.0 # disabled — firmware TX loop is infinite until 'q'


# ============================================================
# CRC-16 CCITT
# ============================================================

def crc16_itu_t(data: bytes) -> int:
    crc = 0x0000
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc


# ============================================================
# DSP
# ============================================================

def gaussian_filter(bt, sps, span=2):
    n_taps = int(2 * span * sps) + 1
    t = np.arange(n_taps) / sps - span
    alpha = np.sqrt(2 * np.log(2)) / bt
    h = np.sqrt(2 * np.pi) / alpha * np.exp(-2 * (np.pi * t / alpha) ** 2)
    h /= np.sum(h)
    return h


def carrier_lock(iq):
    n = len(iq)
    fft_data = np.fft.fftshift(np.fft.fft(iq))
    freqs = np.fft.fftshift(np.fft.fftfreq(n, 1.0 / SAMPLE_RATE))
    power = np.abs(fft_data) ** 2
    mask_lo = (freqs > 70000) & (freqs < LO_OFFSET)
    mask_hi = (freqs > LO_OFFSET) & (freqs < 130000)
    if not mask_lo.any() or not mask_hi.any():
        return float(LO_OFFSET)
    f_lo = float(freqs[mask_lo][np.argmax(power[mask_lo])])
    f_hi = float(freqs[mask_hi][np.argmax(power[mask_hi])])
    if abs(f_lo - LO_OFFSET) > 15000 or abs(f_hi - LO_OFFSET) > 15000:
        return float(LO_OFFSET)
    return (f_lo + f_hi) / 2.0


def downconvert_and_demod(iq, fc, lpf):
    n = len(iq)
    bb = iq * np.exp(-1j * 2 * np.pi * fc * np.arange(n) / SAMPLE_RATE)
    bb_filt = lfilter(lpf, 1.0, bb)
    freq = np.angle(bb_filt[1:] * np.conj(bb_filt[:-1]))
    freq = freq * SAMPLE_RATE / (2 * np.pi * FREQ_DEVIATION)
    return np.clip(freq, -2.0, 2.0)


def bits_to_bytes(bits):
    out = []
    for i in range(0, len(bits) - 7, 8):
        byte = 0
        for j in range(8):
            byte = (byte << 1) | bits[i + j]
        out.append(byte)
    return bytes(out)


def find_sync_and_decode(freq):
    g = gaussian_filter(0.5, SPS, span=2)
    freq_matched = oaconvolve(freq, g, mode='same')

    sync_ref = np.array([(SYNC_WORD >> (15 - i)) & 1 for i in range(16)], dtype=np.float32)
    nrz_sync = 2.0 * sync_ref - 1.0

    packets = []
    seen_peak_pos = set()

    for t in range(SPS):
        centers = np.arange(t + SPS // 2, len(freq_matched), SPS)
        if len(centers) < 17:
            continue
        filt_vals = freq_matched[centers].astype(np.float32)

        for polarity in [1.0, -1.0]:
            nrz_bits = filt_vals * polarity
            hard = (nrz_bits > 0).astype(np.float32) * 2.0 - 1.0

            corr = np.convolve(hard, nrz_sync[::-1], mode='valid')
            sync_positions = np.where(corr >= 12.0)[0]

            for i in sync_positions:
                data_start_bit = i + 16
                N = len(centers)
                if data_start_bit + 8 > N:
                    continue

                payload_len = 0
                for b in range(8):
                    payload_len = (payload_len << 1) | (1 if nrz_bits[data_start_bit + b] > 0 else 0)

                if payload_len == 0 or payload_len > MAX_PACKET_LEN:
                    continue

                total_bits = (1 + payload_len + 2) * 8
                if data_start_bit + total_bits > N:
                    continue

                raw_bits = [1 if nrz_bits[data_start_bit + b] > 0 else 0 for b in range(total_bits)]
                raw_bytes = bits_to_bytes(raw_bits)
                if len(raw_bytes) < 1 + payload_len + 2:
                    continue

                pkt_bytes = raw_bytes[:1 + payload_len + 2]
                payload = pkt_bytes[1:1 + payload_len]
                crc_rx = (pkt_bytes[-2] << 8) | pkt_bytes[-1]
                crc_calc = crc16_itu_t(pkt_bytes[:1 + payload_len])
                crc_ok = (crc_rx == crc_calc)

                peak_pos = t + i * SPS
                if crc_ok:
                    pos_key = peak_pos // SPS
                    if pos_key in seen_peak_pos:
                        continue
                    seen_peak_pos.add(pos_key)
                packets.append((payload, crc_ok, crc_calc, crc_rx, peak_pos))

    return packets


# ============================================================
# UART TX trigger (runs in background thread)
# ============================================================

class TxTrigger(threading.Thread):
    """Sends 'x' to start TX, monitors UART output, retriggers periodically."""

    def __init__(self, port, baud, retrigger_interval, stop_event, verbose=True):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.retrigger_interval = retrigger_interval
        self.stop_event = stop_event
        self.verbose = verbose
        self.last_trigger = 0.0
        self.uart_lines = []
        self.lock = threading.Lock()

    def trigger(self, ser):
        ser.write(b'x')
        ser.flush()
        self.last_trigger = time.time()
        print(f"  [UART] Sent 'x' to trigger TX")
        sys.stdout.flush()

    def run(self):
        try:
            ser = serial.Serial(self.port, self.baud, timeout=0.1)
        except Exception as e:
            print(f"  [UART] ERROR opening {self.port}: {e}")
            return

        time.sleep(0.2)
        self.trigger(ser)

        while not self.stop_event.is_set():
            # Drain any UART output
            line = ser.readline()
            if line:
                text = line.decode('ascii', errors='replace').rstrip()
                if self.verbose and text:
                    print(f"  [PIC24] {text}")
                    sys.stdout.flush()
                with self.lock:
                    self.uart_lines.append(text)

            # Retrigger if interval elapsed
            if time.time() - self.last_trigger > self.retrigger_interval:
                self.trigger(ser)

        # Stop TX cleanly
        ser.write(b'q')
        ser.flush()
        print(f"  [UART] Sent 'q' to stop TX")
        ser.close()


# ============================================================
# Main
# ============================================================

def run(port, rx_gain, duration, retrigger_interval):
    stop_event = threading.Event()

    # Start UART trigger thread
    trigger = TxTrigger(port, UART_BAUD, retrigger_interval, stop_event)
    trigger.start()

    # Connect to Pluto
    print(f"Connecting to Pluto at {PLUTO_URI}...")
    try:
        sdr = adi.Pluto(uri=PLUTO_URI)
    except Exception as e:
        print(f"ERROR: {e}")
        stop_event.set()
        sys.exit(1)

    sdr.rx_lo = CENTER_FREQ + PLUTO_XO_CORRECTION - LO_OFFSET
    sdr.rx_rf_bandwidth = SAMPLE_RATE
    sdr.gain_control_mode_chan0 = 'manual'
    sdr.rx_hardwaregain_chan0 = rx_gain
    sdr.sample_rate = SAMPLE_RATE
    BUF_SIZE = 2**16
    sdr.rx_buffer_size = BUF_SIZE

    print(f"  Frequency  : {CENTER_FREQ/1e6:.3f} MHz")
    print(f"  RX gain    : {rx_gain} dB")
    print(f"  Duration   : {duration}s")
    print(f"  Retrigger  : every {retrigger_interval}s")
    print()

    lpf = firwin(501, 15000 / (SAMPLE_RATE / 2))

    for _ in range(5):
        sdr.rx()

    pkt_ok = 0
    pkt_fail = 0
    buf_count = 0
    start = time.time()
    freq_prev = None
    total_samples = 0
    seen_pos = set()

    print("Listening... (Ctrl+C to stop)")
    print("-" * 60)

    try:
        while time.time() - start < duration:
            iq = sdr.rx().astype(np.complex128)
            buf_count += 1

            fc = carrier_lock(iq)
            freq_new = downconvert_and_demod(iq, fc, lpf)

            if buf_count % 15 == 0:
                elapsed = time.time() - start
                fft_pwr = np.abs(np.fft.fftshift(np.fft.fft(iq)))**2
                fft_f = np.fft.fftshift(np.fft.fftfreq(len(iq), 1.0/SAMPLE_RATE))
                carrier_mask = (fft_f > 70000) & (fft_f < 130000)
                sig_peak = float(np.max(fft_pwr[carrier_mask])) if carrier_mask.any() else 0
                noise_med = float(np.median(fft_pwr))
                snr_db = 10*np.log10(sig_peak/noise_med) if noise_med > 0 else 0
                print(f"  [{elapsed:5.1f}s] buf={buf_count} pkts={pkt_ok}ok/{pkt_fail}fail "
                      f"carrier={fc/1e3:.1f}kHz snr={snr_db:.1f}dB")
                sys.stdout.flush()

            if freq_prev is not None:
                freq_history = np.concatenate([freq_prev, freq_new])
                history_start = total_samples - len(freq_prev)
            else:
                freq_history = freq_new
                history_start = total_samples

            total_samples += len(freq_new)
            freq_prev = freq_new

            for payload, crc_ok, crc_calc, crc_rx, local_peak in find_sync_and_decode(freq_history):
                global_peak = history_start + local_peak
                pos_key = global_peak // SPS
                if pos_key in seen_pos:
                    continue
                seen_pos.add(pos_key)

                elapsed = time.time() - start
                ascii_str = ''.join(chr(b) if 0x20 <= b <= 0x7E else '.' for b in payload)

                if crc_ok:
                    pkt_ok += 1
                    print(f"\n  >>> PACKET #{pkt_ok} at {elapsed:.1f}s  '{ascii_str}'  carrier={fc/1e3:.1f}kHz  CRC:OK\n")
                else:
                    pkt_fail += 1
                    print(f"  [FAIL] '{ascii_str}' calc=0x{crc_calc:04X} rx=0x{crc_rx:04X}")
                sys.stdout.flush()

    except KeyboardInterrupt:
        print("\nStopping...")

    stop_event.set()

    elapsed = time.time() - start
    print("-" * 60)
    total = pkt_ok + pkt_fail
    if total > 0:
        rate = 100.0 * pkt_ok / total
        print(f"RESULT: {pkt_ok}/{total} CRC-OK ({rate:.0f}%) in {elapsed:.1f}s  ({buf_count} buffers)")
        if pkt_ok > 0:
            print("PASS")
        else:
            print("FAIL — no CRC-OK packets")
    else:
        print(f"RESULT: 0 packets detected in {elapsed:.1f}s")
        print("FAIL — no packets found (check RF cable and gain)")


def main():
    parser = argparse.ArgumentParser(description="Combined TX trigger + Pluto RX test")
    parser.add_argument("--port",      default=UART_PORT,          help="PIC24 UART port")
    parser.add_argument("--rx-gain",   type=float, default=60,     help="Pluto RX gain dB")
    parser.add_argument("--duration",  type=float, default=60,     help="Test duration seconds")
    parser.add_argument("--retrigger", type=float, default=RETRIGGER_INTERVAL,
                        help="TX retrigger interval seconds")
    parser.add_argument("--quiet",     action="store_true",        help="Suppress PIC24 UART output")
    args = parser.parse_args()

    run(args.port, args.rx_gain, args.duration, args.retrigger)


if __name__ == "__main__":
    main()
