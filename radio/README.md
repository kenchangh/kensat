# KENSAT Radio — UHF Transceiver Frontend for CubeSat Communications

A custom-designed 437 MHz UHF radio frontend for the KENSAT 2U CubeSat, engineered from first principles to deliver high-performance bidirectional satellite communications using commercially available components. This board is the communications link for KENSAT's edge-AI mission: it carries telemetry, commands, and the inference results produced by the satellite's on-board Jetson payload down to amateur ground stations.

## Technical Overview

The KENSAT Radio is a compact transceiver frontend built around the Silicon Labs Si4463 ISM-band transceiver, an AS179-92LF SPDT RF switch for TX/RX path selection, and a set of impedance matching networks optimized through computational electromagnetic simulation. The board interfaces with an ESP32-S3 microcontroller over SPI and is designed for fabrication at JLCPCB with standard 2-layer FR4.

### RF Architecture

```
                    ┌─────────────┐
                    │   Si4463    │
                    │  Transceiver│
                    │             │
              TX out│             │RX in
                    └──┬───────┬──┘
                       │       │
                 TX Match   RX Match
                  Network    Network
                       │       │
                       ▼       ▼
                    ┌─────────────┐
     Antenna ───────│  AS179-92LF │
     (SMA, 50Ω)    │  SPDT Switch│
                    └─────────────┘
```

**Three discrete signal paths, each with dedicated impedance matching:**

| Path | Function | Source Z | Target Z | Topology |
|------|----------|----------|----------|----------|
| TX | Class-E PA output to antenna | 26.9 Ω | 50 Ω | 5-element LC ladder |
| RX | Antenna to LNA input | 50 Ω | 240 Ω (single-ended) | 3-element LC |
| ANT | 50 Ω line to RF switch | 50 Ω | 50 Ω | 3-element LC |

### Matching Network Design

Impedance matching networks were designed using a custom scikit-rf optimizer (`rf-design/matching_network_skrf.py`) that performs discrete grid search over LCSC-stocked component values. The optimizer models PCB trace parasitics as coplanar waveguide with ground (CPWG) transmission line segments, computed via elliptic integral methods on the actual board stackup:

- **Substrate:** 2-layer FR4, 1.465 mm thickness, εr = 4.6
- **Trace geometry:** 0.3–0.5 mm width, 0.508 mm coplanar ground gap
- **Parasitic extraction:** Each trace segment between components is modeled as a CPWG element with frequency-dependent impedance and propagation constant

The PA output impedance is derived from Class-E switching amplifier theory per Silicon Labs AN648, with drain-source capacitance of 2.5 pF and a DC feed choke of 220 nH.

### Transceiver Configuration

| Parameter | Value |
|-----------|-------|
| Center frequency | 437.08 MHz |
| Modulation | GFSK |
| Data rate | 9600 bps |
| Frequency deviation | ±4.8 kHz |
| RX bandwidth | 100 kHz |
| Sync word | 0x2DD4 (16-bit) |
| Max packet size | 64 bytes |
| Error detection | CRC-16 ITU-T |
| TX power | Programmable (0x00–0x7F) |

### Component Summary

| Ref | Part | Function |
|-----|------|----------|
| U1 | Si4463 (QFN-20) | UHF transceiver, integrated PA + LNA |
| IC1 | AS179-92LF (SOT-65) | SPDT RF switch, TX/RX antenna routing |
| Y1 | 30 MHz XTAL | Transceiver reference oscillator |
| J1 | 2×10 pin header | ESP32-S3 interface (SPI + GPIO) |
| J2 | SMA (Amphenol 132289) | 50 Ω antenna connector |
| L1–L5 | Thin-film inductors | Matching network (120nH, 27nH, 22nH, 16nH, 39nH) |
| C1–C13 | 0402 capacitors | Matching, decoupling, DC blocking (1.0pF–100nF) |

## Firmware

The ESP32-S3 firmware (`esp32_si4463/`) provides full SPI control of the Si4463 transceiver and GPIO-based TX/RX path switching through the AS179:

- **SPI bus:** SCLK=12, MOSI=11, MISO=13, CS=10 @ 4 MHz
- **Control:** SDN=9 (shutdown), NIRQ=14 (interrupt)
- **RF switch control:** Si4463 GPIO2/GPIO3 directly drive AS179 V1/V2 lines — no additional logic required

## Test Infrastructure

Automated RF characterization is performed via an ADALM-Pluto SDR loopback testbench (`rf-design/tx_test_suite.py`):

- **TX gain sweep:** Power linearity across the full programmable range
- **Harmonic distortion:** 2nd and 3rd harmonic levels in dBc
- **Insertion loss:** End-to-end path loss characterization
- **CW tone injection:** 100 kHz offset at 1 MSPS for spectral analysis

## Mission Context

This board is the RF subsystem of the KENSAT mission — a 2U CubeSat built to run **edge-AI compute in orbit**, executing LLM inference on a power-gated Jetson Orin Nano payload and downlinking the results. The radio is what closes the loop, carrying that compute output to the ground. Key mission characteristics:

- **AI payload downlink** — inference results from the on-board Jetson are framed as AX.25 and transmitted over this link alongside telemetry
- **Frequency coordination** through IARU for 437 MHz amateur satellite allocation
- **Open telemetry architecture** — non-encrypted protocol stack enabling global amateur radio operators to receive and decode satellite data, including AI output, in real time
- **Ground station integration** with commercial providers (RBC Signals) for global coverage
- **Commercial launch** via SpaceX rideshare program
- **Full COTS architecture** — demonstrating that meaningful space missions, and on-orbit AI compute, can be executed at component cost

## Manufacturing

Designed for fabrication at JLCPCB on standard 2-layer FR4. The BOM (with LCSC
part numbers) and component-placement files are generated from the KiCad project
with `rf-design/convert_to_jlcpcb.py`.

## Repository Structure

- `hardware/` — KiCad board design
  - `kensat_radio.kicad_pro` / `.kicad_sch` / `.kicad_pcb` — project, schematic, PCB
  - `rf.kicad_sch` — RF front-end sub-schematic
  - `kensat_radio.pdf`, `schematic.png` — rendered schematic
- `firmware/esp32_si4463/` — ESP32-S3 transceiver firmware
  - `esp32_si4463.ino`, `si4463.cpp` / `si4463.h`, `config.h`
- `rf-design/` — RF matching-network design and test
  - `matching_network_skrf.py` — scikit-rf matching network optimizer
  - `vna_smith_plot.py`, `rf_block_diagram.py` — analysis / plotting
  - `tx_test_pluto.py`, `tx_test_suite.py` — ADALM-Pluto SDR loopback tests
  - `convert_to_jlcpcb.py` — BOM/CPL conversion for JLCPCB
  - `*.png` — Smith charts, gain sweep, harmonics, S-parameter results
  - `requirements.txt` — Python dependencies

## License

This project is open hardware. Telemetry protocols and data formats are intentionally unencrypted to support the amateur radio community's tradition of open access to satellite communications.
