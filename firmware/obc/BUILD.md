# Building the OBC firmware

The OBC firmware is written for the **PIC24FJ256GB210** and targets the
Microchip XC16/C30 toolchain inside an MPLAB X project, on top of two
proprietary vendor SDKs that are **not** redistributed in this repository.

## Prerequisites (obtain separately)

| Component | Source | Notes |
|-----------|--------|-------|
| Microchip **XC16** (a.k.a. C30) compiler | Microchip | C compiler for PIC24 |
| **MPLAB X IDE** | Microchip | Project / build / debug |
| Pumpkin **CubeSat Kit BSP** (PIC24) | Pumpkin, Inc. | Board support package + EFFS-THIN filesystem |
| Pumpkin **Salvo RTOS** (Pro, PIC24/dsPIC) | Pumpkin, Inc. | Cooperative RTOS the tasks run under |
| **PICkit 5** + MPLAB IPE | Microchip | Programming / flashing |

These commercial products are licensed by their vendors and are intentionally
omitted from this repo. The original mission code here (`app/`, `bsp/`) is what
you would drop into the vendor example tree.

## Layout in a working build tree

In the development environment the sources in `app/` live inside the CubeSat Kit
example project (`.../CubeSatKit_Dev_Board/Test/Test1/`) and the `bsp/` drivers
extend the kit's `PIC24/` BSP. New source files are registered in the MPLAB X
project's `configurations.xml` under the `Project` logical folder.

## Build & flash (development workflow)

PIC24 builds run under MPLAB X / XC16 (historically on a Windows VM); the
resulting `.hex` is flashed with a PICkit 5 via MPLAB IPE.

## Build flags

- `DEVELOPMENT` / `BENCH_TEST` (in `app/main.h`):
  - `1` (bench default) — skip the burn-wire deploy and the RF-silence hold;
    beacon immediately.
  - `0` (flight) — run the full post-eject deployment sequence and 125 s
    RF-silence hold.

## Si4463 radio settings (do not change blindly)

- `XTAL_OPTIONS = 0x00` (crystal mode), `XO_FREQ = 30 MHz`. **Not** TCXO
  (`0x01`) — the board has a real crystal; TCXO mode causes CTS timeout.
- SPI **Mode 0** (CKP=0, CKE=1); SPI clock derated to 1–2 MHz for the
  two-connector bus crossing.
