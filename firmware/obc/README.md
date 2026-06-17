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
| `main.c` / `main.h` | Boot, init, build flags (`DEVELOPMENT`/`BENCH_TEST`), task registration |
| `task_beacon.c` | Beacon cadence, RF-silence hold, and the antenna-deployment state machine |
| `task_si4463.c` | Si4463 UHF transceiver driver (SPI, TX/RX, config) |
| `task_jetson.c` | Power-gating and UART protocol with the Jetson edge-AI payload |
| `task_cmd.c` | Command parser / debug console |
| `task_bmi088.c` | BMI088 IMU (attitude / tumble sensing) |
| `task_sdlog.c` | Telemetry logging to SD card / serial flash |
| `ax25.c` | AX.25 frame construction for beacons and downlink |
| `deploy.h`, `bootcount.h` | Flash-persisted deployment & boot-count state |
| `task_wdt.c`, `task_status.c`, `task_usb.c`, `task_mhx.c`, `task_5sec.c` | Watchdog, status, USB/serial, MHX radio, periodic housekeeping |

### `bsp/` — board support

`csk_*` drivers (I²C, SPI/SD, IO, UART, power, watchdog, RNG, USB) written
directly against PIC24 peripheral registers, plus the EFFS-THIN MMC SPI driver
glue. These build on Pumpkin CubeSat Kit header conventions.

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
