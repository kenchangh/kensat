# KENSAT — A Single-Engineer 2U CubeSat

> An end-to-end 2U CubeSat — flight software, RF communications, power, attitude
> sensing, antenna deployment, and an on-board edge-AI payload — designed,
> built, and tested from first principles by one engineer.

KENSAT is an independent 2U CubeSat developed outside any institutional
framework. Every subsystem in this repository — the flight computer firmware,
the custom UHF radio board and its RF matching networks, the electrical power
system, the antenna-deployment hardware, the Jetson edge-AI payload, and the
software-defined-radio ground station — was engineered, fabricated, and
bench-tested by a single person on a COTS budget.

**Mission:** a privately-owned satellite in a 520 km sun-synchronous orbit,
operating UHF amateur radio at **437.08 MHz** for beaconing, telemetry
downlink, and command uplink. Telemetry is intentionally **unencrypted** so
amateur operators worldwide can receive and decode it.

---

## System at a glance

```
        ┌──────────────────────────────────────────────┐
        │  Radio Board (custom)                         │  Slot 3 (top)
        │  Si4463 UHF transceiver + AS179 T/R switch    │
        │  Jetson Orin Nano (edge-AI payload)           │
        ├──────────────────────────────────────────────┤
        │  EPS Board (custom)                           │  Slot 2 (middle)
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

- **Fault-tolerant antenna deployment.** The burn-wire release sequence is a
  one-shot, flash-persisted state machine that survives brownouts and resets:
  attempt counts are written *before* energizing, the "deployed" flag only
  *after* a complete burn, and an RF-silence hold fails *safe* toward staying
  quiet. Bounded to 3 retries. ([details](docs/system-architecture.md))

- **RF from first principles.** TX / RX / antenna matching networks designed
  with a custom scikit-rf grid-search optimizer that models PCB traces as CPWG
  transmission lines (elliptic-integral method) and snaps to real LCSC part
  values, then validated on an ADALM-Pluto SDR loopback bench
  (gain sweep, harmonics, insertion loss). ([radio/rf-design](radio/rf-design))

- **Edge AI in orbit.** A Jetson Orin Nano payload runs a quantized LLM,
  triggered and power-gated by the PIC24, with results downlinked as AX.25
  frames — within a measured per-inference power budget.

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
