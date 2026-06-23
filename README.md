# KENSAT — Edge-AI Compute in Orbit

> A 2U CubeSat that runs a large language model on-orbit. A power-gated NVIDIA
> Jetson Orin Nano executes neural-network inference in space, and the results
> are downlinked over a custom UHF radio link to amateur ground stations.

KENSAT's mission is to demonstrate **autonomous AI compute on a small
satellite** — taking a GPU-class accelerator and a quantized LLM out of the
data center and running them inside the tight power, thermal, and reliability
envelope of a 2U CubeSat. The on-board computer wakes the Jetson only when
needed, runs an inference within a measured energy budget, and frames the
output for downlink. Everything else in the spacecraft — the flight software,
the UHF radio and its RF matching networks, the power system, and the
antenna-deployment hardware — exists to get that compute on orbit, keep it
alive, and get its results to the ground.

**Why it's hard.** Edge AI in orbit means treating a multi-watt accelerator as
a scheduled, fault-tolerant *load*: powering it from a battery-backed bus only
in defined windows, dissipating its heat passively in vacuum, recovering it if
it hangs, and surviving the brownouts and resets that come with switching that
much current on a small satellite. KENSAT is built end to end around that
problem.

**Mission profile:** a 520 km sun-synchronous orbit, operating UHF amateur
radio at **437.08 MHz** for beaconing, telemetry downlink, and command uplink.
Telemetry — including downlinked inference results — is intentionally
**unencrypted** so amateur operators worldwide can receive and decode it.

---

## System at a glance

```
        ┌──────────────────────────────────────────────┐
        │  Burn-Wire Board (custom)                     │  Slot 4 (top)
        │  Antenna-release driver · resistive heating   │
        ├──────────────────────────────────────────────┤
        │  Radio Board (custom)                         │  Slot 3
        │  Si4463 UHF transceiver + AS179 T/R switch    │
        │  Jetson Orin Nano (edge-AI payload)           │
        ├──────────────────────────────────────────────┤
        │  EPS Board (custom)                           │  Slot 2
        │  2S2P 18650 pack · rails · hardware watchdog  │
        ├──────────────────────────────────────────────┤
        │  Pumpkin MB + PPM D1 (PIC24FJ256GB210)        │  Slot 1 (bottom)
        │  OBC — always-on C&DH, Salvo RTOS             │
        └──────────────────────────────────────────────┘
              104-pin CubeSat Kit Bus (H1/H2)

   UHF dipole ── U.FL ── matching network ── AS179 ── Si4463 ── SPI ── PIC24
```

Boards interconnect over the 104-pin CubeSat Kit Bus. The PIC24 is the
always-on flight computer; it talks to the EPS over I²C, drives the Si4463
radio over SPI, controls and exchanges data with the Jetson over UART, and
fires the antenna-deployment burn wires over GPIO.

See **[docs/system-architecture.md](docs/system-architecture.md)** for the full
spec: pinouts, bus map, operational modes, deployment state machine, power
budget, and thermal analysis.

---

## Subsystems

| Subsystem | What it is | Highlights |
|-----------|-----------|------------|
| **[firmware/obc](firmware/obc)** | PIC24 flight software (C, Salvo RTOS) | Fault-tolerant antenna deployment with flash-persisted state, AX.25 beacons, Si4463 driver, IMU, SD/serial-flash logging, command parser |
| **[radio](radio)** | Custom 437 MHz UHF radio board | Si4463 + AS179 T/R switch, ESP32-S3 firmware, KiCad 2-layer design |
| **[radio/rf-design](radio/rf-design)** | RF matching-network design & test | Custom scikit-rf optimizer over LCSC-stocked parts, CPWG parasitic modeling, ADALM-Pluto SDR test bench |
| **[eps](eps)** | Electrical power system board | 2S2P 18650 pack, regulated rails, hardware watchdog (KiCad) |
| **[burnwire](burnwire)** | Antenna-release deployment board | Redundant nichrome burn-wire driver (KiCad) |
| **[jetson](jetson)** | Edge-AI payload | Jetson Orin Nano running a quantized LLM (llama.cpp/TinyLlama) over UART, systemd daemons, thermal management |
| **[ground-station](ground-station)** | SDR ground station & decoders | ADALM-Pluto RX/TX, AX.25 beacon decoder, DSP diagnostics, end-to-end test harness |
| **[docs](docs)** | System documentation | Architecture spec, radio guide, Jetson startup, debug notes |

---

## Engineering highlights

- **An LLM running in orbit.** The payload is an NVIDIA Jetson Orin Nano running
  a quantized TinyLlama model under llama.cpp. The always-on PIC24 keeps it
  powered down, then — on schedule or command — switches on its rail through a
  high-side load switch, waits for the Jetson to boot and signal `READY` over
  UART, dispatches a prompt, collects the generated text, powers the Jetson back
  off, and queues the result for downlink. The whole cycle is bounded by a
  measured **per-inference energy budget** so a single run can't drain the pack.
  ([docs/system-architecture.md §6.2](docs/system-architecture.md), [jetson/](jetson))

- **Treating a GPU as a spacecraft load.** Running multi-watt edge compute on a
  2U bus is a systems problem: the EPS sizes a battery-backed rail and a
  hardware watchdog around the Jetson's draw, the OBC power-gates it and can
  recover a hung payload by cutting power, and the design accounts for
  dissipating its heat **passively in vacuum**, where there's no convection.
  ([docs/system-architecture.md §8.1](docs/system-architecture.md))

- **Fault-tolerant by construction.** Both the inference cycle and the antenna
  burn-wire release are one-shot, flash-persisted state machines that survive
  the brownouts and resets that come with switching large currents: attempt
  counts are written *before* energizing, success flags only *after* completion,
  and the RF-silence hold fails *safe* toward staying quiet.
  ([details](docs/system-architecture.md))

- **A complete UHF link to get results down.** TX / RX / antenna matching
  networks were designed with a custom scikit-rf grid-search optimizer that
  models PCB traces as CPWG transmission lines (elliptic-integral method) and
  snaps to real LCSC part values, then validated on an ADALM-Pluto SDR loopback
  bench (gain sweep, harmonics, insertion loss). Inference output is framed as
  AX.25 and downlinked openly. ([radio/rf-design](radio/rf-design))

- **Signal-integrity-aware bus design.** The Si4463 SPI bus crosses two
  board-to-board connectors; the design derates the clock and adds series
  termination accordingly. ([docs/spi-debug-notes.md](docs/spi-debug-notes.md))

---

## Repository layout

```
kensat/
├── docs/             System architecture spec + guides
├── firmware/obc/     PIC24 flight software (app/ + bsp/)
├── radio/            UHF radio: ESP32 firmware, KiCad board, RF design
├── eps/              Electrical power system board (KiCad)
├── burnwire/         Antenna deployment board (KiCad)
├── jetson/           Edge-AI payload daemon + systemd units
└── ground-station/   SDR scripts, beacon decoder, diagnostics
```

## Building

- **OBC firmware** depends on proprietary vendor SDKs (Pumpkin CubeSat Kit BSP,
  Salvo RTOS) that are **not** redistributed here. See
  **[firmware/obc/BUILD.md](firmware/obc/BUILD.md)**.
- **Radio firmware** builds with arduino-cli for the ESP32-S3.
- **RF design / ground-station** tools are Python; see the `requirements.txt`
  in each directory.

## License

Original work in this repository is released under the [MIT License](LICENSE).
Proprietary vendor SDKs the firmware builds against are **not** included and are
**not** covered by that license — see the LICENSE file and
[firmware/obc/BUILD.md](firmware/obc/BUILD.md).

---

*KENSAT — UHF amateur satellite, 437.08 MHz. Telemetry is open and unencrypted
by design, in the tradition of the amateur radio community.*
