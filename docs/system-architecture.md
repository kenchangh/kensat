# KENSAT System Architecture

**2U CubeSat | Rev 2 | April 2026**

## 1. Mission Overview

KENSAT is a 2U CubeSat whose primary mission is to demonstrate **edge-AI compute in orbit**: running large-language-model inference on a power-gated NVIDIA Jetson Orin Nano payload and downlinking the results. The objective is to operate a GPU-class accelerator and a quantized LLM within the power, thermal, and reliability envelope of a small satellite — scheduling the Jetson as a fault-tolerant load, running an inference inside a bounded energy budget, and getting its output to the ground over an open UHF link. The satellite operates in a 520 km sun-synchronous orbit (97.5° inclination, ~95 min period, 0 eccentricity). The program is operated under Swordholder Technologies Inc. (Delaware corporation).

The satellite uses UHF-only communications at 437.08 MHz (100 kbps, 2 kHz bandwidth) for beaconing, telemetry downlink, and command uplink. Telemetry — including downlinked inference results — is unencrypted so amateur operators can receive it; the station operates under an FCC amateur radio license (Technician class).

## 2. Physical Stack

From bottom to top:

| Slot | Board | Description |
|------|-------|-------------|
| 1 (bottom) | Pumpkin MB (P/N 710-00484 Rev E) + PPM D1 (P/N 710-00527) | OBC — always-on command & data handling |
| 2 | Custom EPS Board | Power system with 2S2P 18650 batteries on underside |
| 3 | Custom Radio Board | Si4463 transceiver + Jetson Orin Nano (payload) |
| 4 (top) | Custom Burn-Wire Board | Antenna-release deployment driver — resistive heating elements melt the tether |

All boards interconnect via the 104-pin CubeSat Kit Bus (headers H1 and H2, Samtec ESQ-126 series). The MB uses non-stackthrough connectors (slot 0 endpoint); the EPS, radio, and burn-wire boards use stackthrough connectors.

**SPI signal integrity note:** The Si4463 SPI bus (IO.8–IO.11) crosses two board-to-board connectors (MB → EPS → Radio). Run the SPI clock at **1–2 MHz max** (not the Si4463's 10 MHz limit). On the radio board, keep traces from the bus connector to the Si4463 short (<20 mm), referenced to a ground plane, and add 33 Ω series resistors on MOSI and SCLK.

### 2.1 Antenna

A tape measure dipole antenna is used for UHF at 437.08 MHz. The antenna elements are spring steel tape measure strips that stow flat against the chassis during launch, restrained by a fishing-line tether. After ejection from the deployer the PIC24 waits 120 s (deployer-safety hold), then drives **IO.24** and **IO.25** HIGH to energise two redundant nichrome burn wires that melt through the tether and release the elements; the elements then spring outward into a straight dipole. The deployment is one-shot and persistent — it is recorded in program flash so it is never re-fired after success, and retried at most three times if interrupted (see [§2.2 Boot & Deployment Sequence](#22-boot--deployment-sequence)).

| Parameter | Value |
|-----------|-------|
| Type | Tape measure dipole (half-wave) |
| Frequency | 437.08 MHz |
| Element length | ~16.4 cm per arm (quarter-wave each, trimmed to frequency) |
| Material | Spring steel tape measure strip (~6 mm wide) |
| Feed point | Radio board, via U.FL coax from Si4463 matching network |
| Mounting | Top face of chassis (+Z), elements stowed flat inside deployer rail |
| Deployment | Active — 120 s post-boot hold, then nichrome burn wires on IO.24 + IO.25 (doubled for redundancy) melt the restraint tether |
| Burn duration | ~10 s, both pins driven HIGH simultaneously |
| Polarization | Linear |

Deployment is gated by the `DEVELOPMENT` build flag in [main.h](../firmware/obc/app/main.h) (aliased to `BENCH_TEST`): `DEVELOPMENT=1` (default for bench work) skips the burn and the RF-silence hold, beaconing immediately; `DEVELOPMENT=0` is the flight build that runs the full sequence in [§2.2](#22-boot--deployment-sequence). The current setting is printed to the debug console at boot.

**RF path:** Si4463 TX/RX pin → matching network on radio board → U.FL connector → U.FL coax cable → antenna feed point on chassis top panel.

### 2.2 Boot & Deployment Sequence

The deployment and first-transmit logic lives in the beacon task ([task_beacon.c](../firmware/obc/app/task_beacon.c)) and is built for resilience against unexpected resets (e.g. a brownout from the high-current burn). All decision state is persisted in PIC24 program flash via [deploy.h](../firmware/obc/app/deploy.h) / `init.c` and survives power cycles.

**Persistent state (program flash).** A dedicated, page-aligned reserved page holds four fields, written as a single read-modify-write so updates never clobber each other:

| Field | Meaning |
|-------|---------|
| `boot count` | Incremented once per power-on (separate page); visible via the `b` command. |
| `deployed` | Set to 1 only *after* a burn fully completes. Once set, the wires are never re-fired. |
| `attempts` | Number of burn attempts *started*. Bumped *before* energising, capped at `DEPLOY_MAX_ATTEMPTS` (3). |
| `rf_unlocked` | Set to 1 the first time the 125 s RF-silence hold is satisfied. Once set, later boots transmit immediately. |

A reflash of the `.hex` resets the reserved pages to blank, so a freshly programmed board is treated as *not deployed, 0 attempts, RF-locked*. The `y` debug command clears all deployment state for bench re-testing; `d` prints the current values.

**Per-boot flow:**

1. **`init()`** — increment the persistent boot counter, configure peripherals (clocks, I/O directions, UARTs, SPI), and print the boot banner (firmware version, `DEVELOPMENT`/`BENCH_TEST` setting, boot count) to the debug console.
2. **Salvo scheduler starts.** The beacon task delays ~5 s, initialises the Si4463 into a TX-ready state, then enters its main loop (~30 s cadence). `uptime` below means seconds since *this* power-on — it resets to 0 on every boot.
3. **Each loop pass:**
   - Collect telemetry (uptime, EPS rails, attitude/tumble).
   - **RF-silence gate** (flight only; bench always transmits):
     - if `rf_unlocked` is already set → transmit;
     - else if `uptime ≥ 125 s` → persist `rf_unlocked`, then transmit;
     - else → stay silent (console logs `RF silent (uptime …)`).
   - **Transmit** the AX.25 beacon (and any queued Jetson inference frame) if allowed. TX counters advance only on a real transmission.
   - **Deployment** (flight only, one-shot per boot) fires when *all* hold: `deployed == 0` **and** `attempts < 3` **and** `uptime ≥ 120 s`. It then: bump `attempts` in flash → drive IO.24/IO.25 HIGH for 10 s → drive LOW → set `deployed`. The attempt is recorded *before* the wires energise, so a reset mid-burn still counts toward the 3-try cap. Deployment is GPIO, not RF, so it proceeds normally even during the RF-silence hold.
   - Delay ~30 s to the next beacon.

**Key behaviours and edge cases:**

- **Beacon is independent of deployment.** Once past the RF-silence hold, the satellite beacons whether the antenna deployed, gave up after 3 attempts, or was already deployed — so a (possibly faint, pre-deploy) signal always goes out.
- **Deploy retries, bounded.** If the OBC resets during a burn, the next boot waits its 120 s hold and retries, up to 3 total attempts; after that it stops trying and just keeps beaconing. The `deployed` flag is written only after a burn completes, so a half-finished burn is never recorded as success.
- **RF silence is one-time.** The 125 s hold must be satisfied by **≥125 s of continuous uptime in a single boot** (it is *not* cumulative across boots). The unlock latches on the first loop iteration at/after 125 s. Once latched, every subsequent reboot transmits immediately with no wait.
- **Short first boot (`uptime < 125 s`).** `rf_unlocked` is never written, so that boot stays fully RF-silent and the full 125 s hold restarts from scratch on the next boot — repeating until one boot survives long enough. The design fails *safe*: any uncertainty resolves toward staying silent / waiting again, never toward transmitting early.
- **Short first boot (`uptime < 120 s`).** No deploy attempt is started that boot, so the attempt count is not consumed.

## 3. Processor — PPM D1 (PIC24FJ256GB210)

The PIC24 on the PPM D1 is the always-on flight computer. It runs continuously at ~20 mA from +5V_SYS, handling:

- Telemetry collection from EPS over I²C
- Beacon transmission and command reception via Si4463
- Jetson Orin Nano power control and data exchange
- Timekeeping via the MB's onboard RTC
- Data logging to SD card and/or 64 Mbit serial flash

Key resources:

- **256 KB** program flash, **16 KB** SRAM
- **4 UARTs**, 3 SPIs, 3 I²Cs
- **16-channel 10-bit ADC**
- Peripheral Pin Select allows remapping of digital peripherals to available I/O pins

## 4. Interface Summary

| Interface | Bus | Purpose |
|-----------|-----|---------|
| PIC24 ↔ EPS | I²C (SDA_SYS / SCL_SYS) | EPS telemetry: battery voltage, current, temperature |
| PIC24 ↔ Si4463 | SPI1 (IO.8–IO.11) + GPIO | Radio control: beacon, downlink, uplink |
| PIC24 ↔ Jetson | UART0 (IO.4 / IO.5) | Command/response when Jetson is powered on |
| PIC24 ↔ RTC | I²C (SDA_SYS / SCL_SYS) | Timekeeping (M41T81S, shared I²C bus with EPS) |
| PIC24 ↔ SD Card | SPI0 (IO.0–IO.3) | Mass storage for telemetry logs and downlink queues |
| PIC24 ↔ Serial Flash | SPI2 (internal to PPM) | 64 Mbit AT25DF641, additional onboard storage |
| PIC24 → Jetson 5V rail | GPIO IO.16 | Power on/off the Jetson via a 2.7A high-side load switch (SOT-25, active-HIGH enable) on the radio board |

## 5. Detailed Pinouts

### 5.1 PIC24 ↔ EPS — I²C Bus

The EPS provides telemetry (battery voltage, cell temperatures, solar input current, rail status) as an I²C slave. The MB's RTC (M41T81S) shares this bus. Bus pull-ups (4.7 kΩ to VCC_SYS) are on the MB.

| Signal | CubeSat Kit Bus Pin | PIC24 Pin | Direction |
|--------|-------------------|-----------|-----------|
| SDA_SYS | H1.41 | SDA1 (U1.56) | Bidirectional |
| SCL_SYS | H1.43 | SCL1 (U1.57) | Output from PIC24 |

### 5.2 PIC24 ↔ Si4463 — SPI1 (Radio)

The Si4463 transceiver on the radio board is controlled via the second SPI interface (SPI1) routed over the CubeSat Kit Bus. Two additional GPIO lines are required: SDN (shutdown control) and nIRQ (interrupt from Si4463 to PIC24).

| Signal | Function | CubeSat Kit Bus Pin | PIC24 Pin | Direction |
|--------|----------|-------------------|-----------|-----------|
| IO.8 | -SS1 / nSEL (chip select) | H1.16 | RP27 (U1.14) | Output from PIC24 |
| IO.9 | SDO1 / MOSI (data to Si4463) | H1.15 | RP19 (U1.12) | Output from PIC24 |
| IO.10 | SDI1 / MISO (data from Si4463) | H1.14 | RP26 (U1.11) | Input to PIC24 |
| IO.11 | SCK1 / SCLK (SPI clock) | H1.13 | RP21 (U1.10) | Output from PIC24 |
| IO.30 | Si4463 SDN (shutdown) | H2.18 | RPI33 (U1.18) | Output from PIC24 |
| IO.31 | Si4463 nIRQ (interrupt) | H2.17 | RPI34 (U1.19) | Input to PIC24 |

**SPI clock speed:** 1–2 MHz max due to two connector hops (see Section 2).

### 5.3 PIC24 ↔ Jetson Orin Nano — UART0

The Jetson is normally powered off. When the PIC24 enables the 5V rail, the Jetson boots and communicates with the PIC24 over UART. Both PIC24 I/O and Jetson GPIO are 3.3V — direct connection is acceptable.

The Jetson UART is `/dev/ttyTHS1` (UART1), broken out on the 40-pin expansion header.

| Signal | Function | CubeSat Kit Bus Pin | PIC24 Pin | Jetson 40-pin Header | Direction |
|--------|----------|-------------------|-----------|---------------------|-----------|
| IO.4 | UTX0 (PIC24 TX → Jetson RX) | H1.20 | RP4/RD9 (U1.69) | Pin 10 (UART1_RXD) | PIC24 → Jetson |
| IO.14 | URX0 (Jetson TX → PIC24 RX) | H1.10 | RP31/RF13 (U1.39) | Pin 8 (UART1_TXD) | Jetson → PIC24 |
| GND | Ground reference | H2.29/30/32 (DGND) | — | Pin 6 (or any GND) | — |

**Note:** IO.5 (H1.19) was the original Jetson RX pin but the path is broken on this unit. Jetson RX is remapped to IO.14 via PPS.

**UART parameters:** 9600 baud, 8N1, no flow control.

### 5.3.1 Debug Console — UART1 (PIC24 UART2)

The debug console provides command-line access to the PIC24 over a USB-serial adapter. Connect adapter TX/RX to H1 pins 11/12 (IO.13/IO.12). These pins are directly adjacent to the Si4463 SPI pins on the H1 connector.

| Signal | Function | CubeSat Kit Bus Pin | PIC24 Pin | Direction |
|--------|----------|-------------------|-----------|-----------|
| IO.12 | UTX1 (PIC24 debug TX) | H1.12 | RP5/RD15 (U1.48) | Output from PIC24 |
| IO.13 | URX1 (debug RX → PIC24) | H1.11 | RPI43/RD14 (U1.47) | Input to PIC24 |

**UART parameters:** 9600 baud, 8N1, no flow control.

**Connection:** `screen /dev/tty.usbserial-XXXX 9600`

### 5.4 PIC24 ↔ SD Card — SPI0

Directly on the MB. Used for telemetry logs and downlink data queues.

| Signal | Function | CubeSat Kit Bus Pin | PIC24 Pin | Direction |
|--------|----------|-------------------|-----------|-----------|
| IO.0 | -CS_SD (chip select) | H1.24 | RE5 (U1.3) | Output from PIC24 |
| IO.1 | SDO0 / MOSI | H1.23 | RP15 (U1.53) | Output from PIC24 |
| IO.2 | SDI0 / MISO | H1.22 | RPI44 (U1.54) | Input to PIC24 |
| IO.3 | SCK0 / SCLK | H1.21 | RP4 (U1.69) | Output from PIC24 |

SD card power is controlled by **-ON_SD** (RE4/U1.100), active LOW.

### 5.5 PIC24 ↔ Serial Flash (AT25DF641) — SPI2

Onboard the OBC module. Not routed through the CubeSat Kit Bus.

| Signal | Function | PIC24 Pin |
|--------|----------|-----------|
| -CS_SFM | Chip select | RD13 (U1.80) |
| SDO_SFM | MOSI | RP11 (U1.72) |
| SDI_SFM | MISO | RP3 (U1.70) |
| SCK_SFM | SPI clock | RP12 (U1.71) |
| -WP_SFM | Write protect | RD6 (U1.83) |

### 5.6 EPS Power Rails on CubeSat Kit Bus

| Signal | Bus Pin(s) | Voltage | Description |
|--------|-----------|---------|-------------|
| +5V_SYS | H2.25, H2.26 | 5.0 V, 3 A max | Main system rail — powers MB, PPM, radio board logic, Jetson (gated by load switch) |
| VCC_SYS | H2.27, H2.28 | 3.3 V, 1.2 A max | 3.3V system rail |
| VBATT | H2.45, H2.46 | 5.0–8.4 V | Raw battery voltage (2S Li-ion) |
| DGND | H2.29, H2.30, H2.32 | 0 V | Digital ground |
| AGND | H2.31 | 0 V | Analog ground (tied to DGND at one point) |

### 5.7 EPS Control & Status on CubeSat Kit Bus

| Signal | Bus Pin | Direction | Description |
|--------|---------|-----------|-------------|
| -RESET | H1.29 | Bus → PIC24 | Active LOW reset |
| OFF_VCC | H1.31 | Bus → PIC24 | Active HIGH, controls PPM power circuits |
| -FAULT | H1.25 | PIC24 → Bus | Open-collector overcurrent fault |
| SENSE | H1.27 | PIC24 → Bus | Current sense output |

### 5.8 RBF & Separation Switches

Both switches are wired in series in the battery power path on the EPS. Both must be in the "allow" position for the satellite to receive power.

| Signal | Bus Pin(s) | Switch | Terminal |
|--------|-----------|--------|----------|
| S0 | H2.33, H2.34 | RBF | NC |
| S1 | H2.35, H2.36 | Separation | NC |
| S2 | H2.37, H2.38 | RBF | NO |
| S3 | H2.39, H2.40 | Separation | NO |
| S4 | H2.41, H2.42 | RBF | Common |
| S5 | H2.43, H2.44 | Separation | Common |

## 6. Operational Modes

### 6.1 Normal Mode (Jetson OFF)

This is the primary operating mode. The PIC24 runs continuously.

- EPS charges batteries from solar panels via 4× MPPT channels
- PIC24 polls EPS telemetry over I²C every 30 seconds
- PIC24 transmits beacon packets via Si4463 (call sign, battery voltage, temperatures, uptime)
- PIC24 listens for ground station uplink commands
- PIC24 sends I²C heartbeat to EPS every 60 seconds (see Section 7.1)
- Jetson is powered off — IO.16 held LOW
- Estimated power: ~0.5 W (PIC24 + Si4463 idle/beacon)

### 6.2 Inference Mode

The satellite runs TinyLlama inference **once per day automatically**, cycling through a pre-loaded prompt library. No uplink is required for the mission to produce results. If the ground station sends an uplink command, it overrides which prompt runs next.

#### Prompt Library (pre-launch)

Prompts are stored on the Jetson's eMMC before launch:

```
/opt/kensat/prompts/
├── 00.txt    "What is the meaning of life?"
├── 01.txt    "Write a haiku about Earth from orbit."
├── 02.txt    "Explain quantum computing in one paragraph."
│   ...
└── 19.txt    (up to 20 prompts)
```

The PIC24 never touches prompt content — it only sends a 1-byte prompt ID to the Jetson.

#### Scheduling

The PIC24 maintains two persistent variables in its program flash (survive resets and watchdog power-cycles):

| Variable | Size | Description |
|----------|------|-------------|
| `next_prompt_id` | 1 byte | 0–19, wraps around. Incremented after each successful inference. |
| `last_inference_time` | 4 bytes | RTC epoch timestamp of last completed inference. |

**Trigger logic** (checked every 30 seconds during normal telemetry polling):

```
if (rtc_now - last_inference_time >= 86400) AND (vbatt >= 7.0V):
    run_inference(next_prompt_id)
```

That's it. The satellite wakes the Jetson once per day, runs the next prompt in sequence, and retransmits the result for 1 hour. If the battery is too low, it retries on the next 30-second check until voltage recovers.

After 20 days the prompts cycle back to `00.txt` and repeat. The ground station receives a fresh inference response every day without any uplink.

#### Uplink Override (optional)

If the ground station wants to run a specific prompt, it sends a short command during a pass:

**Uplink packet (8 bytes total):**

| Field | Size | Description |
|-------|------|-------------|
| Sync + preamble | 4 bytes | Si4463 hardware |
| Command ID | 1 byte | `0x01` = set next prompt |
| Prompt ID | 1 byte | `0x00`–`0x13` (prompt 0–19) |
| CRC-16 | 2 bytes | Error check |

**PIC24 processing:**

1. Si4463 fires nIRQ (IO.31) → PIC24 services INT2 ISR
2. PIC24 reads RX FIFO over SPI1, validates CRC-16
3. Overwrites `next_prompt_id` with the received value
4. Beacons ACK with the prompt ID echoed back

The override only changes which prompt runs next — it does not trigger an immediate inference. The daily timer handles the rest. This keeps the logic simple: one code path for inference, regardless of how the prompt was selected.

#### Inference Execution

When the daily trigger fires:

**Step 1 — Power on Jetson:**
PIC24 drives IO.16 HIGH → load switch enables +5V_SYS to Jetson. PIC24 starts a 60-second boot timeout.

**Step 2 — UART handshake & run:**

```
Jetson → PIC24:  READY\n
PIC24  → Jetson: RUN:05\n            (prompt ID as 2-digit ASCII)
```

The Jetson daemon reads `/opt/kensat/prompts/05.txt`, loads TinyLlama (Q4_K_M quantized), runs inference, and returns:

```
Jetson → PIC24:  DONE:<length>\n<response_bytes>
PIC24  → Jetson: OK\n
PIC24  → Jetson: OFF\n
```

Inference timeout: 120 seconds. If exceeded, PIC24 hard-cuts IO.16 LOW.

**Step 3 — Shut down Jetson:**
Jetson daemon runs `shutdown -h now`. PIC24 waits 10 seconds, then drives IO.16 LOW. Safe against hard cuts because eMMC is mounted read-only.

**Step 4 — Update state:**
PIC24 writes response to SD card, updates `last_inference_time` to `rtc_now`, increments `next_prompt_id` (wrapping 19 → 0), and writes both to flash.

**Total Jetson on-time:** ~45–90 seconds.

#### Downlink — 1-Hour Retransmission

After inference, PIC24 retransmits the result for 1 hour:

**Downlink packet:**

| Field | Size | Description |
|-------|------|-------------|
| Sync + preamble | 4 bytes | Si4463 hardware |
| Packet type | 1 byte | `0x10` = inference result |
| Prompt ID | 1 byte | Which prompt was run |
| Payload length | 2 bytes | Response length in bytes |
| Payload | N bytes | Response text (≤ 1.5 KB typical) |
| CRC-16 | 2 bytes | Error check |

At 100 kbps, a 1.5 KB response takes ~120 ms to transmit. No fragmentation — the Si4463 FIFO is refilled by the PIC24 via nIRQ as it drains.

```
retransmit_until = rtc_now + 3600

while rtc_now < retransmit_until:
    transmit inference result     (~120 ms)
    wait 5 seconds
    transmit beacon               (~10 ms)
    wait 5 seconds
```

~360 retransmissions per hour. Ground station needs one clean copy. No sequence numbers, no reassembly.

**Power cost:** ~0.5–1 Wh over the full hour.

#### State Machine

```
                    ┌─────────────────────────┐
                    │                         │
                    ▼                         │
                  IDLE                        │
                    │                         │
          ┌────────┴────────┐                 │
          │                 │                 │
    (uplink received)  (24h elapsed?)         │
          │                 │                 │
   set next_prompt_id  (vbatt ≥ 7.0V?)       │
   beacon ACK               │                │
   stay in IDLE         NO: wait             │
                        YES: ──→ JETSON_BOOT  │
                                    │         │
                             (READY in 60s?)  │
                              /        \      │
                           NO          YES    │
                            │            │    │
                      HARD_CUT    RUN_INFERENCE
                            │            │    │
                            │     (DONE in 120s?)
                            │      /         \│
                            │   NO          YES
                            │    │            │
                            │  HARD_CUT  JETSON_OFF
                            │    │            │
                            │    │     update prompt_id
                            │    │     write to flash
                            │    │            │
                            │    │     DOWNLINK_LOOP
                            │    │      (1 hour)
                            │    │            │
                            └────┴────────────┘
```

All error paths return to IDLE and resume normal beaconing. A failed inference does **not** increment `next_prompt_id` — the same prompt retries the next day.

#### Power Budget — Inference Cycle

| Phase | Duration | Power | Energy |
|-------|----------|-------|--------|
| Jetson boot | 15–30 s | 7–10 W | ~0.08 Wh |
| Inference | 10–17 s | 15–25 W | ~0.12 Wh |
| Jetson shutdown | 10 s | 5 W | ~0.01 Wh |
| 1-hour retransmission | 3600 s | 0.5–1 W | ~1.0 Wh |
| **Total** | **~61 min** | — | **~1.2 Wh** |

At one cycle per day (~1.2 Wh) plus continuous beaconing (~12 Wh/day at 0.5 W), the 50 Wh battery can sustain the satellite for roughly 3–4 days without any solar input. With solar charging, the mission runs indefinitely.

## 7. Reliability

### 7.1 EPS Hardware Watchdog

If the PIC24 hangs, the entire satellite goes silent — no beacon, no uplink reception, no recovery. The EPS must include a hardware watchdog independent of the PIC24.

**Implementation:** The EPS microcontroller (or a dedicated watchdog timer IC) monitors a heartbeat from the PIC24. The PIC24 sends an I²C write to a specific EPS register every 60 seconds. If the EPS does not receive a heartbeat within 180 seconds (3 missed beats), the EPS power-cycles the +5V_SYS rail, forcing a full PIC24 reboot.

**Requirements for the EPS contract:**

| Parameter | Value |
|-----------|-------|
| Heartbeat interface | I²C register write (address and register TBD by EPS designer) |
| Heartbeat interval | 60 seconds (PIC24 sends) |
| Timeout | 180 seconds (EPS triggers reset) |
| Reset method | Cycle +5V_SYS LOW for 1 second, then restore |
| Watchdog must survive | PIC24 crash, bus lockup, I²C hang |
| Fallback | If I²C bus is hung (SCL held LOW >10 seconds), EPS power-cycles +5V_SYS |

### 7.2 Jetson Fault Tolerance

The Jetson is treated as an unreliable peripheral. The PIC24 never depends on it.

- **Boot timeout (60s):** If Jetson doesn't send `READY`, hard-cut 5V.
- **Inference timeout (120s):** If Jetson doesn't send `DONE`, hard-cut 5V.
- **Read-only rootfs:** Jetson's eMMC is mounted read-only in flight. Hard power cuts cannot corrupt the filesystem.
- **Prompt library is static:** Pre-loaded before launch, never modified in orbit.

### 7.3 Si4463 Recovery

If the Si4463 becomes unresponsive (no nIRQ after TX/RX command):

1. PIC24 asserts SDN (IO.30) HIGH for 10 ms → Si4463 full hardware reset
2. PIC24 re-runs Si4463 power-up configuration sequence over SPI
3. Resume normal beaconing

## 8. Thermal Considerations

### 8.1 Jetson Orin Nano in Vacuum

The Jetson dissipates 15–25 W during inference. In vacuum there is no convective cooling — heat can only leave by radiation and conduction.

**Mitigation:**

- **Thermal pad:** Mount the Jetson's heatsink surface directly to the CubeSat's aluminum structure via a thermal interface pad (e.g., Bergquist Gap Pad). The structure acts as a thermal mass and radiates to space.
- **Short duty cycle:** The Jetson runs for < 90 seconds per inference. At 25 W for 90 seconds, total energy is ~0.6 Wh (2.25 kJ). Spread across the thermal mass of the structure (~200 g aluminum), this raises temperature by roughly 12°C — well within the Jetson's operating range.
- **Cooldown between inferences:** Allow at least 30 minutes between inference cycles so the structure can radiate accumulated heat.

### 8.2 Passive Thermal — Rest of Satellite

The PIC24, EPS, and Si4463 all dissipate < 1 W continuously. At these power levels, the satellite's thermal environment is dominated by orbital solar/albedo/Earth-IR heating, not internal dissipation. Standard CubeSat thermal design (surface coatings, internal conduction paths) is sufficient.

Battery heaters may be needed if cell temperatures drop below 0°C during eclipse. The INR18650-35E is rated for discharge down to -20°C but charging below 0°C causes lithium plating. The EPS should inhibit charging when battery temperature is below 0°C.

## 9. Pin Allocation Summary

| IO Pin | PPM D1 Function | KENSAT Assignment |
|--------|----------------|-------------------|
| IO.0 | -CS_SD | SD card chip select |
| IO.1 | SDO0 | SD card MOSI |
| IO.2 | SDI0 | SD card MISO |
| IO.3 | SCK0 | SD card SCLK |
| IO.4 | UTX0 | UART TX → Jetson RX |
| IO.5 | URX0 | ~~Broken on this unit~~ — see IO.14 |
| IO.6 | UTX1 | Reserved (MHX/USB interface on MB — do not use) |
| IO.7 | URX1 | Reserved (MHX/USB interface on MB — do not use) |
| IO.8 | -SS1 | Si4463 nSEL (chip select) |
| IO.9 | SDO1 | Si4463 MOSI |
| IO.10 | SDI1 | Si4463 MISO |
| IO.11 | SCK1 | Si4463 SCLK (1–2 MHz max) |
| IO.12 | GPIO (RP5/RD15) | **Debug console TX** (UART2 TX, H1.12) |
| IO.13 | GPIO (RPI43/RD14) | **Debug console RX** (UART2 RX, H1.11) |
| IO.14 | GPIO (RP31/RF13) | **Jetson UART RX** (UART0 RX, remapped from IO.5 via PPS) |
| IO.15 | -UCTS1 | Unassigned |
| IO.16 | GPIO (RP2) | **Jetson 5V enable** (active HIGH) |
| IO.17–IO.23 | GPIO | Unassigned |
| IO.24 | GPIO (RF1) | **Antenna nichrome burn wire A** (active HIGH, redundant with IO.25) |
| IO.25 | GPIO (RF0) | **Antenna nichrome burn wire B** (active HIGH, redundant with IO.24) |
| IO.26–IO.29 | GPIO | Unassigned |
| IO.30 | INT1 (RPI33) | **Si4463 SDN** (shutdown control) |
| IO.31 | INT2 (RPI34) | **Si4463 nIRQ** (interrupt input) |
| IO.32–IO.39 | AN8–AN15 | Unassigned (available for analog telemetry) |
| IO.40–IO.47 | AN0–AN7 | Unassigned (available for analog telemetry) |
| SDA_SYS | I²C data | EPS telemetry + RTC + EPS watchdog heartbeat |
| SCL_SYS | I²C clock | EPS telemetry + RTC + EPS watchdog heartbeat |