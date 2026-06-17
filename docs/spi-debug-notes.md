# SPI Debug Notes — PIC24 to Si4463 (via ESP32 Mock)

**Date:** April 2026
**Symptom:** ESP32 mock radio received only `0x00` on every SPI transaction. SCLK and MOSI never toggled from the ESP32's perspective, even though CS and SDN worked.

## Root Causes

### 1. Watchdog Timer (WDT) Causing Silent Reboots

The PIC24FJ256GB210's CONFIG1 fuse defaults to `FWDTEN=ON`. When this bit is set at the hardware level, the WDT is **always enabled and cannot be disabled by software**. The BSP's `csk_wdt_off()` (which calls `EnableWDT(WDT_DISABLE)`) has no effect — it only works when `FWDTEN=OFF` in CONFIG1, which allows the software `SWDTEN` bit to control the WDT.

The WDT timeout is approximately 2 seconds with default prescaler settings. Any NOP-based delay loop or tight polling loop that runs longer than this without calling `__builtin_clrwdt()` triggers a reset.

**Affected code:**
- `si4463_wait_cts()` — 1000 CTS poll iterations, each ~3 ms = ~3 seconds total
- `si4463_reset()` — 6 x 50000 NOP loops for POR delay
- Any future long-running SPI or delay sequence

**Fix:** Add `__builtin_clrwdt()` at the top of every loop that may exceed ~1 second without yielding to the RTOS (whose timer hook clears the WDT).

**Proper long-term fix:** Add `FWDTEN_OFF` to `_CONFIG1()` in `init.c` so the WDT can be controlled by software, or ensure the Salvo WDT hook (`salvohook_wdt_MCC30_clrwdt.c`) runs frequently enough. Since `init.c` is shared BSP code, the safer approach is to keep the WDT enabled and clear it explicitly.

### 2. task_exercise_io Toggling SPI Pins

The BSP demo task `task_exercise_io` (in `CubeSatKit/Example/PIC24/PIC24FJ256GA110/.../task_exercise_io.c`) cycles through every CubeSat Kit IO pin, calling `csk_ioN_high()` and `csk_ioN_low()` with 100-tick (1 second) delays between each. This includes our SPI pins:

| IO Signal | Function | BSP call | Effect |
|-----------|----------|----------|--------|
| IO.8 | CS (RG9) | `csk_io8_high/low` → `PORTG \|= BIT9` | Spurious CS assertions |
| IO.9 | MOSI (RG8) | `csk_io9_high/low` → `PORTG \|= BIT8` | Drives MOSI |
| IO.11 | SCLK (RG6) | `csk_io11_high/low` → `PORTG \|= BIT6` | Drives SCLK |
| IO.30 | SDN (RE8) | `csk_io30_high/low` → `PORTE \|= BIT8` | Fake radio resets |

This task is a demo/test feature and must be disabled when using any IO pins for real peripherals.

**Fix:** Comment out `OSCreateTask(task_exercise_io, ...)` in `main.c`. The `TASK_EXERCISE_IO_P` define must remain in `tasks.h` because `task_cmd.c` references it (the 'i' serial command starts/stops the IO exercise).

### 3. RTOS Yield Instability (Unresolved)

Calling `si4463_usb_msg()` — which acquires a binary semaphore, opens the USB interface, sends a UART message, delays, and releases the semaphore — causes the PIC24 to crash when the RTOS scheduler runs other tasks during the yield. The exact cause is not yet identified but is likely related to `task_usb_talk` or the reduced task configuration (several original tasks were disabled).

**Workaround:** Use NOP-based delays instead of `OS_Delay()` in the Si4463 task, and avoid calling `si4463_usb_msg()`. This sacrifices USB debug output but keeps SPI communication stable.

**TODO:** Investigate the RTOS yield crash. Likely needs checking `task_usb_talk` initialization, semaphore state with fewer tasks, or stack allocation.

## Gotchas and Lessons

- **`init.c` drives CS LOW at boot.** The line `PORTG = 0x0000;` in `init.c` sets all PORTG pins LOW, including RG9 (CS). This creates a spurious CS assertion that the ESP32 sees before `spi2_init()` runs. It's benign but shows up as a `CMD: 0x00` transaction on first boot.

- **ANSG was not the problem.** The `init.c` GB210 branch correctly clears `ANSG = 0x0000;` before any tasks run. We added a redundant clear in `spi2_init()` as belt-and-suspenders, but it was never the root cause.

- **PPS pins work fine as GPIO.** Setting `RPOR9bits.RP19R = 0` and `RPOR10bits.RP21R = 0` correctly disconnects PPS peripheral output and allows GPIO control via LAT registers. The PPS unlock/lock sequence with `IOL1WAY_OFF` works as expected.

- **ESP32 diagnostic buffer fills during CS setup delay.** The 64-sample GPIO diagnostic in `spi_slave_byte()` captures during the CS-to-SCLK setup delay (before SCLK starts toggling), so `SCLK_HI=0/64` is normal even when SPI is working correctly. The received byte values are the real indicator of success.

- **Serial output blocks the ESP32 main loop.** Printing full 64-line GPIO diagnostics at 115200 baud takes ~280 ms, during which the ESP32 misses SPI transactions. Keep ESP32 serial output minimal when debugging SPI timing.

## Working SPI Configuration

| Parameter | Value |
|-----------|-------|
| Mode | SPI Mode 0 (CPOL=0, CPHA=0) |
| Implementation | Bit-bang GPIO (no SPI peripheral) |
| SCLK half-period | ~50 us (200 NOPs at 4 MIPS) |
| CS-to-SCLK setup | ~1.25 ms (5000 NOPs) |
| CTS poll inter-delay | ~0.25 ms (1000 NOPs) |
| WDT clearing | Every CTS poll iteration and before/during long delays |
