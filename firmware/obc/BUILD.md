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

All of these live in `app/main.h`.

| Flag | Default | Effect |
|------|---------|--------|
| `FLIGHT_MODE` | `1` in this snapshot | The switch that arms antenna deployment. `0` (bench) compiles the burn-wire block out entirely — the nichrome gate FETs on IO.24/IO.25 can never be energised. `1` fires them once past `FLIGHT_HOLD_S`, subject to the persistent `DEPLOYED` flag and the 3-attempt cap. |
| `DEPLOY_ARMED` | `1` | Second, independent gate on the burn-wire block, so "am I debugging?" and "is this build going to space?" cannot be confused. `0` disarms deployment *even in a `FLIGHT_MODE` build*; everything else about flight mode stays live. **Must be `1` for flight** — the boot banner reports it, so do not read `FLIGHT_MODE=true` as meaning the wires are live. |
| `FLIGHT_HOLD_S` | `125` | Post-power-on hold, in seconds, used for both the deployer-safety hold and the RF-silence compliance hold. |
| `DEPLOY_TEST_MODE` | `0` | Bench burn test: pulses IO.24+IO.25 HIGH for 10 s, once, 10 s after startup. Energises nichrome — fit a dummy load. Must be `0` otherwise. |
| `POWER_IDLE` | `1` | Executes `PWRSAV #1` (Idle) after every `OSSched()` pass, halting the core between beacons while Timer2/UART/SPI/I²C keep running. Set to `0` by hand if an attached ICD needs the core running continuously. |
| `CONSOLE_VERBOSE_DEFAULT` | `0` | Debug-console verbosity at boot; toggle at runtime with `q`. Verbose adds the rails/tumble lines and an AX.25 hex dump of every frame (~0.5 s of busy-waited UART per cycle at 9600 baud). |
| `DEVELOPMENT` | `!FLIGHT_MODE` | Legacy alias. Its one remaining job is gating the bench-only uplink-injection console commands (`u`, `p`, `n`) out of flight builds — see "Known issues" in [README.md](README.md). |

Note that the RF-silence hold and the beacon itself are **not** gated on
`FLIGHT_MODE`: the beacon task starts ~5 s after boot and transmits from
`FLIGHT_HOLD_S` onwards in both builds. One consequence worth knowing — a
bench session longer than the hold latches `rf_unlocked` in program flash, and
a board that ships in that state transmits from its first orbital loop with no
wait. Clear it with the `y` console command (`deploy_reset()`) as the last
step before flight, or reflash, then confirm with `d`.

Before flight, also change `UPLINK_AUTH` in `app/task_beacon.c`: it is the
shared token that gates every telecommand, including the one that can silence
the transmitter, and the value in this repo is public.

## Si4463 radio settings (do not change blindly)

- `XTAL_OPTIONS = 0x00` (crystal mode), `XO_FREQ = 30 MHz`. **Not** TCXO
  (`0x01`) — the board has a real crystal; TCXO mode causes CTS timeout.
- SPI **Mode 0** (CKP=0, CKE=1); SPI clock derated to 1–2 MHz for the
  two-connector bus crossing.
- The modem register block is a **WDS-generated set for 1200 bps GFSK**. Do
  not hand-edit it: the `CHFLT` values are FIR filter taps, not scalars, and a
  partial edit yields a radio that still transmits but silently stops hearing
  commands. Regenerate the whole block if the baud rate or deviation changes,
  and keep the deviation in step with the `deviation:` field of the SatNOGS
  SatYAML entry — gr-satellites derives both its pre-demod cutoff and its
  quadrature-demod gain from it.
