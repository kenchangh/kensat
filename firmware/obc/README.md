# OBC Flight Software

Flight software for the KENSAT on-board computer: a **PIC24FJ256GB210** on a
Pumpkin PPM D1 module, running the **Salvo** cooperative RTOS. The OBC is the
always-on command-and-data-handling computer — I²C/SPI master to the EPS and
radio boards, UART link to the Jetson payload, and driver of the antenna burn
wires.

## Layout

```
firmware/obc/
├── app/    Application tasks and mission logic (original work)
├── bsp/    Board-support drivers (csk_*) implemented on the PIC24 peripherals
└── test/   Bench-test fixtures (ESP32 mock radio sketch)
```

### `app/` — mission logic

| Module | Responsibility |
|--------|----------------|
| `main.c` / `main.h` | Boot, init, build flags (`FLIGHT_MODE`/`DEPLOY_ARMED`), idle-mode power saving, task registration |
| `task_beacon.c` | Beacon cadence, RF-silence hold, uplink telecommands, LLM result downlink, and the antenna-deployment state machine |
| `task_si4463.c` | Si4463 UHF transceiver driver (SPI, TX/RX, config) |
| `task_jetson.c` | Power-gating, prompt dispatch and UART protocol with the Jetson edge-AI payload |
| `task_cmd.c` | Command parser / debug console |
| `task_bmi088.c` | BMI088 IMU (attitude / tumble sensing) |
| `task_sdlog.c` | Telemetry logging to SD card; `INFER.LOG`; beacon-counter checkpoints |
| `ax25.c` | AX.25 frame construction for beacons and downlink |
| `deploy.h`, `bootcount.h`, `commstate.h` | Flash-persisted deployment, boot-count and comms state (implemented in the vendor example tree's `init.c`; see [BUILD.md](BUILD.md)) |
| `task_wdt.c`, `task_status.c`, `task_usb.c`, `task_mhx.c`, `task_5sec.c` | Watchdog, status, USB/serial, MHX radio, periodic housekeeping |

### `bsp/` — board support

`csk_*` drivers (I²C, SPI/SD, IO, UART, power, watchdog, RNG, USB) written
directly against PIC24 peripheral registers, plus the EFFS-THIN MMC SPI driver
glue. These build on Pumpkin CubeSat Kit header conventions.

## The radio link

`task_beacon.c` owns the Si4463, so it is where the whole link lives. The
satellite is half-duplex with one radio: each ~10 s cycle it transmits, then
spends the remainder of the cycle in RX listening for telecommands.

### Downlink frames

All downlink is AX.25 UI, GFSK 1k2, NRZI + G3RUH, source `KO6IKZ`, and plain
ASCII in the info field — decodable by any 1k2 G3RUH station (Direwolf,
gr-satellites, SatNOGS) with no custom software. Frames are distinguished by
their first token:

| Frame | When | Contents |
|-------|------|----------|
| `BCN` | every cycle | `seq=` (cumulative), `bc=` boot count, `rst=` reset cause, `up=` uptime in minutes, `gx/gy/gz=` body rates, `rx=` uplink frames received |
| `LLM` | once per inference | prompt id, Jetson CPU/GPU temps and peaks, power, RAM, model-load and inference durations, status, `h=` CRC-32 of the result text, `txt=<bytes>/<frames>` |
| `TXT` | one chunk per cycle until the result is out | `s=` result sequence, `c=<k>/<m>` chunk index, `h=` CRC-32 of the complete text, then that chunk of the generated text |
| `ACK` | on an accepted telecommand | which command, plus `seq=` and `bc=` |

An inference result is up to 512 bytes and one info field holds 180, so the
text goes out as `TXT` chunks of 144 bytes — one per beacon cycle, and the
whole set twice. That is deliberate: back-to-back chunks would push the
transmitter duty cycle past 60 % and leave no listening window, while
spreading them over ~40 s of sky and repeating the set means a single fade
cannot take out a whole result on a link with no ARQ. The reasoning is
written out above `beacon_tx_llm_text()`.

Reassembly on the ground is three rules — group by `s=`, order by `c=`,
verify `h=` (standard CRC-32, i.e. Python's `zlib.crc32`) — implemented in
[ground-station/llm_downlink.py](../../ground-station/llm_downlink.py).

### Uplink telecommands

Uplink uses the Si4463's own packet handler (preamble + sync `0x2DD4` +
length + payload + CRC-16), payload capped at 63 bytes, ASCII:

```
KO6IKZ <AUTH> <VERB>[ <ARG>]
```

The callsign keeps the spacecraft from acting on other traffic and the token
after it is a shared telecommand code — FCC Part 97.211(b) permits codes that
obscure the meaning of telecommand messages, and a CRC alone is not a gate on
a command that can silence the spacecraft. **Change `UPLINK_AUTH` in
`task_beacon.c` before flight and keep the flight value out of the repo.**

| Verb | Effect |
|------|--------|
| `BCN OFF` | Inhibit the transmitter. Persisted in program flash, no auto-expiry, and gates every transmit path including acknowledgements — so there is no reply; the confirmation is that the carrier stops. |
| `BCN ON` | Resume beaconing. Acknowledged. |
| `PING` | Liveness check. Acknowledged. |
| `PR <k>/<m> <text>` | Chunk *k* of *m* of an uplinked LLM prompt (≤ 40 chars per chunk, ≤ 6 chunks = 240 chars). Acknowledged with the bitmap of chunks held, so the ground can re-send only what was lost. Chunks may arrive in any order. |
| `PR?` | Report prompt-assembly state. Acknowledged. |
| `PRX` | Discard the partially assembled prompt. Acknowledged. |
| `RUN` | Run an inference now — on the assembled uplinked prompt if one is complete, otherwise on the next prompt in the Jetson's on-board library. |
| `RUN <id>` | Run library prompt `<id>` (0–19) now. |

A `RUN` the payload cannot take (already powered, or a bad index) is
acknowledged as `NAK`, not dropped: repeating a telecommand is how you get
one through a fading uplink, and the operator has to be able to tell "heard,
declined" from "not heard".

The uplinked prompt is held in RAM only and never persisted — a reset falls
back to the on-board prompt library, so the mission keeps producing results
with no uplink at all. The library cursor does not advance for an uplinked
prompt, so experimenting from the ground never skips part of the planned
prompt set.

On the bench, `u`, `p` and `n` on the debug console push telecommands through
the real uplink dispatcher (prompt chunks, `RUN`, inhibit, ping) with no
ground transmitter; only the demodulator is bypassed. They are compiled out
of flight builds — see **Known issues** below.

## Building & flashing

The PIC24 toolchain (Microchip XC16/C30) and the vendor SDKs are required and
are **not** in this repo — see **[BUILD.md](BUILD.md)**.

## Key design notes

- **Deployment is fail-safe and idempotent.** State lives in PIC24 program
  flash and survives resets; the burn is bounded to 3 attempts and never
  re-fires after success. Full state machine in
  [docs/system-architecture.md](../../docs/system-architecture.md).
- **Si4463 SPI** runs at 1–2 MHz (derated for the two-connector bus crossing),
  SPI Mode 0, crystal mode (not TCXO). See
  [docs/debug-si4463.md](../../docs/debug-si4463.md).
- **Persistent state is split by write rate.** The beacon/TX/RX counters are
  checkpointed to the SD card after every beacon, because program flash is
  rated ~10 k erase cycles and a write every 10 s would destroy the page in
  about a day. The commanded beacon-inhibit flag goes the other way — it is
  written a handful of times in the whole mission, and must survive a card
  that is missing or dead, so it lives in flash (`commstate.h`).
- **One inference, two downlink routes.** The `LLM` frame says how a run went;
  the generated text goes out separately as `TXT` chunks. Both carry the same
  sequence number and the same CRC-32, which is the only end-to-end integrity
  check the text has on a link with no ARQ.

## Known issues

- **`OS_Delay()` must stay at the task's top level.** On this Salvo PIC24
  build (OST library, `OSVIA_OSDISPATCH`), yielding from inside a function the
  task calls corrupts the return path: prints from inside the helper come out
  fine, then the chip traps on the helper's epilogue. Hence the unrolled
  `OS_Delay()` chains in `task_beacon.c` rather than loops, and busy-waits
  (`busy_ms()`) in sensor-init helpers such as `task_bmi088.c`'s
  `bmi088_init()`. Every helper that must be callable from a task documents
  that it contains no `OS_Delay`.
- **Phantom console commands from stale Salvo semaphore counts.** Salvo
  semaphores are counting, and the UART RX ISR signals one per received
  character, so a burst of N characters leaves the count at N. `task_cmd_do()`
  still uses `OS_WaitSem(SEM_CMD_CHAR_P)` and therefore wakes spuriously after
  a long console response, re-running whatever character is stale. Receive
  loops written since then poll with `OS_Delay` + `csk_uart0_getchar()`
  instead — see the pattern in `task_jetson.c`. Until `task_cmd` is converted,
  the bench-only uplink-injection commands (`u`, `p`, `n`) are compiled out of
  flight builds, so a phantom command can never silence the transmitter on
  orbit.
- **`csk_uart1_msg_ts()` destabilises busy tasks.** It busy-waits on TX ring
  space; called early or often it hangs the system. New tasks should `sprintf`
  into a task-local buffer and call `csk_uart1_puts()` directly.
- **`URXISEL` must be cleared after `csk_uart0_open()`.** The BSP leaves it at
  `01` (interrupt at 3/4 FIFO full), which loses data when fewer than three
  characters arrive; `init.c` sets `U1STAbits.URXISEL = 0`.
