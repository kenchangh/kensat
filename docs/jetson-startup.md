# Jetson Orin Nano — Startup & Inference Process

## Overview

The Jetson Orin Nano is the AI payload on the radio board (slot 3). It runs TinyLlama inference on demand, controlled entirely by the PIC24 OBC. The Jetson has no autonomous scheduling — it boots when told, runs one inference, and shuts down.

## Power Control

The PIC24 controls Jetson power via IO.16 (RD8), which drives an AP22811 high-side load switch on the radio board. This gates the 12V boost rail to the Jetson's barrel jack input.

- **IO.16 HIGH** = Jetson powered on (5V rail active)
- **IO.16 LOW** = Jetson powered off (hard cut)

The Jetson's eMMC is mounted read-only, so hard power cuts are safe and do not risk filesystem corruption.

## Boot Sequence

```
PIC24 drives IO.16 HIGH
        |
        v
Jetson powers on, Linux boots (~35 s)
        |
        v
systemd starts kensat.service
        |
        v
kensat_daemon.sh runs:
  1. Sets 7W power mode (nvpmodel -m 3)
  2. Starts llama-server, loads TinyLlama INT8 into GPU (~2 s)
  3. Sends "READY\n" over UART to PIC24
        |
        v
PIC24 receives READY, sends "RUN:XX\n" (prompt ID 00-19)
        |
        v
Daemon reads /opt/kensat/prompts/XX.txt
Daemon sends HTTP request to local llama-server
Inference runs (~17 s at 7W for 256 tokens)
        |
        v
Daemon sends "DONE:<length>\n<response>\n" over UART
        |
        v
PIC24 sends "OK\n" then "OFF\n"
        |
        v
Daemon runs "shutdown -h now"
PIC24 waits 10 s, then drives IO.16 LOW
```

**Total Jetson on-time: ~65 seconds at 7W mode.**

## UART Protocol

UART0 at 9600 8N1. PIC24 pins IO.4 (TX) / IO.14 (RX) through TXB0102 level shifter.

| Direction | Message | Meaning |
|-----------|---------|---------|
| Jetson -> PIC24 | `READY\n` | Daemon is running, model loaded, ready for prompt |
| PIC24 -> Jetson | `RUN:XX\n` | Run prompt XX (00-19, zero-padded) |
| Jetson -> PIC24 | `DONE:<len>\n<response>\n` | Inference complete, response follows |
| Jetson -> PIC24 | `ERR:NO_PROMPT\n` | Prompt file not found |
| Jetson -> PIC24 | `ERR:INFERENCE_FAIL\n` | Inference timed out or crashed |
| Jetson -> PIC24 | `ERR:SERVER_FAIL\n` | llama-server failed to start |
| PIC24 -> Jetson | `OK\n` | Response received |
| PIC24 -> Jetson | `OFF\n` | Shut down now |

## PIC24 State Machine

```
IDLE --> BOOT --> SEND_RUN --> READ_RESP --> SHUTDOWN --> IDLE
          |          |             |
       (60s)     (120s)        (30s)        timeouts = hard cut IO.16 LOW
```

| State | Action | Timeout |
|-------|--------|---------|
| IDLE | Wait for `jetson_run_requested` flag (set by 'j' command) | -- |
| BOOT | IO.16 HIGH, wait for `READY\n` | 60 s |
| SEND_RUN | Send `RUN:XX\n`, wait for `DONE:\n` | 120 s |
| READ_RESP | Accumulate response bytes until `\n` | 30 s |
| SHUTDOWN | Send `OK\n` + `OFF\n`, wait 10 s, IO.16 LOW | -- |

Any timeout triggers a hard power cut (IO.16 LOW) and returns to IDLE.

## Daemon Files (on Jetson eMMC)

| Path | Purpose |
|------|---------|
| `/opt/kensat/kensat_daemon.sh` | Main daemon script |
| `/opt/kensat/model.gguf` | Symlink to TinyLlama Q4_K_M GGUF |
| `/opt/kensat/prompts/00.txt` - `19.txt` | 20 pre-loaded prompt files |
| `/etc/systemd/system/kensat.service` | Systemd unit (enabled, starts on boot) |
| `/home/ken/llama.cpp/build/bin/llama-server` | llama.cpp inference server (CUDA) |

Source files in the repo: `jetson/kensat_daemon.sh`, `jetson/kensat.service`.

## Inference Engine

The daemon uses `llama-server` (llama.cpp HTTP server) rather than `llama-cli`, because this version of llama.cpp (b8702) only ships `llama-server` as the main binary. The flow is:

1. Daemon starts `llama-server` on `127.0.0.1:8080` with `-ngl 99` (full GPU offload)
2. Waits for `/health` endpoint to return OK (model loaded)
3. Sends a `/v1/chat/completions` request with the prompt
4. Parses the JSON response and extracts the assistant's reply
5. Kills `llama-server` after inference

This avoids keeping the server running longer than needed.

## Power Mode Comparison

Measured on Jetson Orin Nano Super (8 GB), TinyLlama 1.1B Q4_K_M, 256 output tokens. Measurements are for the server start + inference phase only (not Linux boot or shutdown).

| Metric | 7W Mode | 25W Mode |
|--------|---------|----------|
| Server start + model load | 1.6 s | 10.2 s |
| Inference (256 tokens) | 16.8 s | 4.0 s |
| Generation speed | 15.3 tok/s | 60.3 tok/s |
| Total active time | 18.2 s | 14.1 s |
| Average power draw | 6.42 W | 8.25 W |
| Peak power draw | 6.75 W | 17.25 W |
| **Total energy** | **117.5 J (32.6 mWh)** | **119.5 J (33.2 mWh)** |

### Decision: 7W mode

Total energy consumption is essentially identical (~33 mWh either way). The 25W mode draws more power but finishes faster; the 7W mode draws less power but runs longer. The joules cancel out.

The tiebreakers all favor 7W:

1. **Peak current**: 6.75 W vs 17.25 W. The AP22811 load switch and 2S2P 18650 battery pack have current limits. Lower peak draw reduces stress on the power system.
2. **Thermal**: Less heat generation in a vacuum-insulated CubeSat enclosure where convection cooling is unavailable.
3. **Battery voltage sag**: At the 7.0 V trigger threshold, a 17 W spike risks voltage droop below the Jetson's minimum input. 6.75 W is well within safe margins.

The daemon enforces 7W mode (`nvpmodel -m 3`) on every boot as a safety net, regardless of the Jetson's stored default.

### Full Cycle Energy Budget (estimated)

| Phase | Duration | Power | Energy |
|-------|----------|-------|--------|
| Linux boot | ~35 s | ~4 W (idle) | 140 J |
| Model load + inference | ~18 s | ~6.4 W (avg) | 117 J |
| Shutdown | ~10 s | ~4 W | 40 J |
| **Total** | **~63 s** | | **~297 J (82 mWh, 0.08 Wh)** |
