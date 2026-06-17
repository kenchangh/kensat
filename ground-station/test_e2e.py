#!/usr/bin/env python3
"""
End-to-end test: PIC24 triggers Jetson inference, receives response,
broadcasts via Si4463 radio.

Single command that orchestrates the full flow:
  1. SSH to Jetson: restart kensat daemon
  2. Open PIC24 serial, send 'j' to trigger inference
  3. Monitor PIC24 debug output for state transitions
  4. Report pass/fail with timing

Usage:
    python test_e2e.py [--port /dev/cu.usbserial-BG03U9VQ]
"""

import argparse
import os
import fcntl
import termios
import time
import select
import subprocess
import sys
import re

# ============================================================
# Configuration
# ============================================================

UART_PORT = os.environ.get("KENSAT_UART_PORT", "/dev/cu.usbserial-BG03U9VQ")
UART_BAUD = 9600
# Jetson SSH target and password are read from the environment so no
# credentials live in the repo. Example:
#   export KENSAT_JETSON_HOST="ken@192.168.1.50"
#   export KENSAT_JETSON_PW="..."
JETSON_HOST = os.environ.get("KENSAT_JETSON_HOST", "ken@jetson.local")
JETSON_PW = os.environ.get("KENSAT_JETSON_PW", "")
OVERALL_TIMEOUT = 180  # 3 minutes max

# Expected state transitions in PIC24 debug output
MILESTONES = [
    ("inference requested", "Inference requested"),
    ("IO.16 HIGH",         "IO.16 powered on"),
    ("Got READY",          "Received READY from Jetson"),
    ("Sent RUN",           "Sent RUN command"),
    ("Got DONE",           "Received DONE (inference complete)"),
    ("Response [",         "Got inference response"),
    ("TELEM: cpu=",        "Received telemetry"),
    ("beacon: TX 49 bytes", "Beacon transmitted"),
]


def ssh_cmd(cmd, timeout=15):
    """Run a command on the Jetson via SSH."""
    return subprocess.run(
        ["sshpass", "-p", JETSON_PW, "ssh",
         "-o", "StrictHostKeyChecking=no",
         "-o", "ConnectTimeout=5",
         JETSON_HOST, cmd],
        capture_output=True, text=True, timeout=timeout
    )


def open_serial(port):
    """Open serial port in raw mode at 9600 baud."""
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    fcntl.fcntl(fd, fcntl.F_SETFL, fcntl.fcntl(fd, fcntl.F_GETFL) & ~os.O_NONBLOCK)
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0  # iflag
    attrs[1] = 0  # oflag
    attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL  # cflag
    attrs[3] = 0  # lflag
    attrs[4] = termios.B9600  # ispeed
    attrs[5] = termios.B9600  # ospeed
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 1
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIOFLUSH)
    return fd


def read_lines(fd, timeout_s):
    """Read serial data, yield complete lines as they arrive."""
    buf = b""
    end = time.time() + timeout_s
    while time.time() < end:
        r, _, _ = select.select([fd], [], [], 0.5)
        if r:
            d = os.read(fd, 4096)
            if d:
                buf += d
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    yield line.decode("ascii", errors="replace").strip()


def drain(fd, secs=1):
    """Drain any buffered serial data."""
    end = time.time() + secs
    while time.time() < end:
        r, _, _ = select.select([fd], [], [], 0.2)
        if r:
            os.read(fd, 4096)


def main():
    parser = argparse.ArgumentParser(description="KENSAT E2E inference test")
    parser.add_argument("--port", default=UART_PORT, help="PIC24 serial port")
    parser.add_argument("--skip-daemon", action="store_true",
                        help="Skip daemon restart (assume already running)")
    args = parser.parse_args()

    t0 = time.time()
    hit = {pat: None for pat, _ in MILESTONES}

    def elapsed():
        return f"{time.time() - t0:.1f}s"

    def status(msg):
        print(f"  [{elapsed():>6s}] {msg}", flush=True)

    print("=" * 60)
    print("KENSAT E2E Test: Inference -> Beacon TX")
    print("=" * 60)

    # ---- Step 1: Check Jetson connectivity ----
    jetson_reachable = False
    status("Checking Jetson SSH...")
    try:
        r = ssh_cmd("echo OK", timeout=8)
        if r.returncode == 0:
            jetson_reachable = True
            status("Jetson reachable.")
        else:
            status("Jetson not reachable — will boot via IO.16.")
    except subprocess.TimeoutExpired:
        status("Jetson SSH timed out — will boot via IO.16.")

    # ---- Step 2: Open serial, send 'j' ----
    status("Opening serial port...")
    fd = open_serial(args.port)
    drain(fd, 1)

    def check_milestones(line):
        """Check line against milestones, print if matched."""
        for pat, desc in MILESTONES:
            if pat in line and hit[pat] is None:
                hit[pat] = time.time() - t0
                print(f"  [{hit[pat]:6.1f}s] >>> {desc}", flush=True)
                return True
        return False

    status("Sending 'j' to PIC24...")
    os.write(fd, b"j")

    # Read initial response — check if we need to send 'j' twice
    need_second_j = False
    saw_io16_high = False
    for line in read_lines(fd, 10):
        if not line.strip():
            continue
        status(f"PIC24: {line}")
        check_milestones(line)
        if "hard power cut" in line or "hard cut" in line:
            need_second_j = True
        if "IO.16 HIGH" in line:
            saw_io16_high = True
            break

    if need_second_j:
        status("Hard cut detected — waiting 3s then sending 'j' again...")
        time.sleep(3)
        os.write(fd, b"j")
        for line in read_lines(fd, 10):
            if not line.strip():
                continue
            status(f"PIC24: {line}")
            check_milestones(line)
            if "IO.16 HIGH" in line:
                saw_io16_high = True
                break

    # ---- Step 3: Restart daemon AFTER PIC24 enters JS_BOOT ----
    # PIC24 is now waiting for READY. Restart daemon so it sends READY.
    # Only if Jetson was already on — if cold booting, daemon starts with OS.
    if not args.skip_daemon and saw_io16_high and jetson_reachable:
        status("PIC24 waiting for READY — restarting daemon...")
        ssh_cmd(f"echo '{JETSON_PW}'|sudo -S systemctl restart kensat 2>/dev/null",
                timeout=10)
        status("Daemon restarted.")
    elif saw_io16_high and not jetson_reachable:
        status("Jetson cold booting — daemon will start with OS (~100s).")

    # ---- Step 4: Monitor full cycle ----
    status("Monitoring inference cycle...")
    print()

    end_time = time.time() + OVERALL_TIMEOUT
    cycle_done = False
    prompt_id = None
    response_text = None
    capturing_response = False
    telem_line = None         # "TELEM: cpu=... gpu=... pwr=... infer=..." from task_jetson
    beacon_hex_rows = {}      # row_offset (int) -> "XX XX ..."

    for line in read_lines(fd, OVERALL_TIMEOUT):
        if not line.strip():
            continue

        # Capture the response body — it's the line after "Response [prompt XX, N bytes]:"
        # The PIC24 prints it raw (no task_jetson prefix)
        if capturing_response:
            # Response line has no "task_" prefix — it's raw text from the Jetson
            if "task_" not in line and line.strip():
                response_text = line.strip()
            capturing_response = False

        # Extract prompt ID from "Sent RUN:XX"
        m = re.search(r'Sent RUN:(\d+)', line)
        if m:
            prompt_id = m.group(1)

        # Flag to capture the next non-task line as the response body
        if "Response [" in line:
            capturing_response = True

        # Capture Jetson telemetry CSV line
        m = re.search(r'TELEM:\s*cpu=.*', line)
        if m:
            telem_line = m.group(0)

        # Capture beacon hex-dump rows (one per "beacon hex NN: XX XX ...")
        m = re.search(r'beacon hex ([0-9A-F]{2}):\s*([0-9A-F ]+)', line)
        if m:
            beacon_hex_rows[int(m.group(1), 16)] = m.group(2).strip()

        # Check milestones
        if not check_milestones(line):
            # Print other task_jetson lines (skip noisy beacon/sdlog lines)
            if "task_jetson" in line:
                print(f"  [{time.time() - t0:6.1f}s]     {line}")

        # Check for cycle end
        if "powered off" in line or "Returning to IDLE" in line:
            cycle_done = True
            break
        if "hard cut" in line and hit.get("Got READY") is not None:
            # Timeout after READY means something went wrong
            cycle_done = True
            break

    os.close(fd)

    # ---- Step 5: Results ----
    print()
    print("=" * 60)
    print("RESULTS")
    print("=" * 60)

    passed = 0
    failed = 0
    for pat, desc in MILESTONES:
        t = hit[pat]
        if t is not None:
            print(f"  PASS  {desc:40s}  ({t:.1f}s)")
            passed += 1
        else:
            # beacon TX is optional (depends on timing)
            if pat == "beacon: TX":
                print(f"  SKIP  {desc:40s}  (not observed)")
            else:
                print(f"  FAIL  {desc:40s}")
                failed += 1

    total_time = time.time() - t0
    print()
    if failed == 0:
        print(f"  ALL PASSED ({passed}/{passed}) in {total_time:.1f}s")
    else:
        print(f"  {failed} FAILED, {passed} passed in {total_time:.1f}s")

    # ---- Prompt & Response ----
    if prompt_id is not None:
        print()
        print("-" * 60)
        print(f"PROMPT (id={prompt_id}):")
        try:
            r = ssh_cmd(f"cat /opt/kensat/prompts/{prompt_id}.txt", timeout=5)
            if r.returncode == 0 and r.stdout.strip():
                for pline in r.stdout.strip().split("\n"):
                    print(f"  {pline}")
            else:
                print(f"  (could not read prompt file)")
        except Exception:
            print(f"  (Jetson not reachable)")
        print()
        print("RESPONSE:")
        if response_text:
            # Word-wrap at 70 chars for readability
            words = response_text.split()
            line = "  "
            for w in words:
                if len(line) + len(w) + 1 > 72:
                    print(line)
                    line = "  " + w
                else:
                    line = line + " " + w if line.strip() else "  " + w
            if line.strip():
                print(line)
        else:
            print("  (no response captured)")
        print("-" * 60)

    # ---- Telemetry CSV + beacon hex dump ----
    if telem_line or beacon_hex_rows:
        print()
        print("-" * 60)
        if telem_line:
            print(f"TELEMETRY: {telem_line}")
        if beacon_hex_rows:
            flat = []
            for offset in sorted(beacon_hex_rows):
                flat.extend(beacon_hex_rows[offset].split())
            print(f"BEACON HEX ({len(flat)} bytes):")
            # 16 bytes per line, aligned
            for i in range(0, len(flat), 16):
                chunk = " ".join(flat[i:i + 16])
                print(f"  {i:02X}:  {chunk}")
        print("-" * 60)

    # Check Jetson daemon logs for additional info
    if hit.get("Got DONE") is not None or hit.get("Sent RUN") is not None:
        print()
        print("Jetson daemon log (last 10 lines):")
        try:
            r = ssh_cmd(f"echo '{JETSON_PW}'|sudo -S journalctl -u kensat --no-pager -n 10 2>&1",
                        timeout=8)
            for line in r.stdout.strip().split("\n"):
                if "kensat_daemon" in line or "kensat]" in line:
                    print(f"  {line}")
        except Exception:
            print("  (could not fetch logs)")

    print("=" * 60)
    sys.exit(0 if failed == 0 else 1)


if __name__ == "__main__":
    main()
