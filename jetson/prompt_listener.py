#!/usr/bin/env python3
"""Jetson UART listener for PIC24 OBC prompts with TinyLlama inference.

Protocol:
  PIC24 sends: PING\r\n
  Jetson replies: PONG\r\n
  PIC24 sends: <prompt text>\r\n
  Jetson replies: <inference response>\r\n

Requires llama-server running:
  ~/build/bin/llama-server \
    -m tinyllama-1.1b-chat-v1.0.Q4_K_M.gguf \
    -ngl 99 -c 1024 --host 0.0.0.0 --port 8080

Uses /dev/ttyTHS1 at 9600 N81.
"""
import json
import serial
import sys
import urllib.request
import urllib.error

PORT = "/dev/ttyTHS1"
BAUD = 9600
LLAMA_URL = "http://localhost:8080/v1/chat/completions"
MAX_RESPONSE_CHARS = 120  # PIC24 RX buffer is 128, leave room for \r\n


def log(msg):
    print(msg, flush=True)


def query_tinyllama(prompt):
    """Send prompt to llama-server and return the response text."""
    payload = json.dumps({
        "messages": [
            {"role": "user", "content": prompt}
        ],
        "max_tokens": 64,
    }).encode("utf-8")

    req = urllib.request.Request(
        LLAMA_URL,
        data=payload,
        headers={"Content-Type": "application/json"},
    )

    try:
        with urllib.request.urlopen(req, timeout=30) as resp:
            body = json.loads(resp.read().decode("utf-8"))
            text = body["choices"][0]["message"]["content"]
            # Collapse to single line, trim to fit PIC24 buffer
            text = " ".join(text.split())
            return text[:MAX_RESPONSE_CHARS]
    except urllib.error.URLError as e:
        log(f"llama-server error: {e}")
        return "[error] llama-server unreachable"
    except (KeyError, IndexError, json.JSONDecodeError) as e:
        log(f"Response parse error: {e}")
        return "[error] bad response from llama-server"


def main():
    ser = serial.Serial(PORT, BAUD, timeout=1)
    ser.reset_input_buffer()
    log(f"Listening on {PORT} at {BAUD} baud...")

    while True:
        line = ser.readline().decode("ascii", errors="replace").strip()
        if not line:
            continue

        log(f"RX: [{line}]")

        if line == "PING":
            ser.write(b"PONG\r\n")
            ser.flush()
            log("TX: PONG")
        else:
            log(f"Querying TinyLlama...")
            response = query_tinyllama(line)
            ser.write(response.encode("ascii", errors="replace") + b"\r\n")
            ser.flush()
            log(f"TX: {response}")


if __name__ == "__main__":
    main()
