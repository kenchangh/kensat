# Si4463 Debugging Guide

Reference for diagnosing Si4463 transceiver issues on the Pumpkin CubeSat Kit OBC. Lessons learned from debugging sessions getting GFSK RX working between a Pluto SDR and Si4463.

## GFSK Gaussian Filter Bug (pluto_tx.py)

**Root cause of Pluto TX not being decodable by Si4463.**

The Gaussian filter alpha constant was wrong by a factor of 2, producing rectangular FSK instead of Gaussian-shaped FSK:

```python
# BUG (sqrt(ln2/2) = 0.59 -- filter 2x too narrow, no filtering):
alpha = np.sqrt(np.log(2) / 2) / bt

# FIX (sqrt(2*ln2) = 1.18 -- correct BT=0.5 shaping):
alpha = np.sqrt(2 * np.log(2)) / bt
```

**Symptoms:** Strong RSSI (120-150) from Pluto GFSK packets, but PREAMBLE_DETECT never fired. MODEM_PEND showed only INVALID_SYNC (0x04) + RSSI_JUMP (0x08).

**Why it was hard to find:**
- Software loopback passed (FM demod doesn't care about pulse shape)
- CW test confirmed RF path worked (RSSI 36 -> 108)
- RSSI confirmed signal arrival, ruling out cable/connector issues
- `sqrt(ln2/2)` vs `sqrt(2*ln2)` look similar and both are plausible Gaussian constants

**How to verify:** Check the filtered preamble peak amplitude. With BT=0.5, the Gaussian filter should reduce the alternating NRZ preamble from +/-1.0 to approximately +/-0.75. If the peak is >0.95, the filter is doing nothing.

## DSA Causes RSSI Hysteresis

After the Si4463 sees a strong signal, DSA (Digital Sensitivity Adjustment) raises the noise floor from ~36 RSSI to ~85 RSSI and does not recover without a full re-init ('s' command). This can make RSSI appear "stuck" and mask signal path problems. Always re-init before comparing RSSI with/without signal.

## MODEM_PEND Is Essential for Debugging

The standard nIRQ/PH_PEND approach only shows packet-level events. For diagnosing WHY packets aren't being received, read MODEM_PEND from GET_INT_STATUS (response byte 4) or GET_MODEM_STATUS (response byte 0):

| Bit | Name | Meaning |
|-----|------|---------|
| 0 | RSSI | RSSI above threshold |
| 1 | POSTAMBLE_DETECT | End of transmission detected |
| 2 | INVALID_SYNC | Sync search failed / preamble timeout |
| 3 | RSSI_JUMP | Sudden RSSI change |
| 4 | PREAMBLE_DETECT | Valid preamble found |
| 5 | SYNC_DETECT | Valid sync word found |
| 6 | DSA_DETECT | DSA event |

Typical failure modes:
- `pnd=0x08` only: Signal present but modem can't demodulate at all (wrong GLOBAL_CONFIG, wrong frequency band)
- `pnd=0x0C` (RSSI_JUMP + INVAL_SYNC): Signal present, preamble detector searching but failing (wrong modulation shape, frequency offset too large)
- `pnd=0x14` (PREAMBLE + INVAL_SYNC): Preamble found but sync word doesn't match (wrong sync word encoding, too many bit errors)
- `pnd=0x34` then PH=0x10: Full success (preamble + sync + packet received)

## Diagnostic Scripts

| Script | Purpose |
|--------|---------|
| `scripts/pluto_tx.py` | GFSK packet transmitter (with corrected Gaussian filter) |
| `scripts/pluto_rx.py` | GFSK packet receiver/demodulator |
| `scripts/pluto_cw.py` | CW carrier for RF path testing |
| `scripts/pluto_freq_sweep.py` | Sweep TX frequency, read Si4463 RSSI from serial |
| `scripts/serial_rssi_monitor.py` | Real-time RSSI display with change detection |

## Critical Rule: Use WDS Configs Verbatim

**Never hand-calculate Si4463 register values.** Always use Silicon Labs WDS (Wireless Development Suite) to generate a complete `radio_config.h`, and apply every property group from it. The register API is full of non-obvious encodings and interdependencies.

Known traps:
- `FREQ_CONTROL_FRAC` uses the formula `f = (INTE + FRAC/2^19) * 2 * XTAL / outdiv`. Plugging in frequency naively gives wrong FRAC values.
- `MODEM_DATA_RATE` encoding depends on `TX_NCO_MODE`. WDS value `0x05DC00` and hand-calculated value `0x002580` both claim to be 9600 bps but only the WDS value actually produces 9600 bps.
- `SYNC_BITS` register stores the **bit-reversed** version of the on-air sync word. Register `0xB4,0x2B` = on-air `0x2D,0xD4`. WDS handles this automatically.
- `AFC_FREQ_OFFSET` in `GET_MODEM_STATUS` reports a constant default value (~20737) when AFC hasn't locked. This is NOT a real frequency measurement.

## WDS Config Must Be Complete

**Partial WDS configs break reception.** The modem properties, packet handler, preamble, sync word, channel filter, and frequency settings are all interdependent. In our case:

- Mixing custom `PKT_CONFIG1 = 0x80` (PH_FIELD_SPLIT) with WDS modem values prevented preamble detection entirely.
- WDS sets `PKT_CONFIG1 = 0x02` and `PKT_LEN = 0x2A` with different field/CRC arrangements that the modem expects.
- TX worked fine with partial configs, but RX silently failed — no error, just no preamble detection.

**Always apply the complete `RADIO_CONFIGURATION_DATA_ARRAY`** from WDS output, including every property group: GLOBAL, INT_CTL, FRR, PREAMBLE, SYNC, PKT (all sub-properties), MODEM (all), CHFLT, PA, SYNTH, MATCH, FREQ_CONTROL.

## GLOBAL_CONFIG = 0x20 Is Mandatory

WDS generates modem parameters assuming high-performance divisor mode (`Hi_pfm_div: 1`). This requires `GLOBAL_CONFIG` (group 0x00, property 0x03) = `0x20`.

Without this:
- The modem's internal clock rates are wrong
- IF frequency, decimation, and BCR all operate at incorrect rates
- RSSI still responds to signals (wideband measurement), but the narrowband modem chain is completely misconfigured
- The chip doesn't report any error — it silently fails

This property must be set **before** any modem properties.

## RSSI Does Not Mean Demodulation Works

Strong RSSI (even -30 dBm) does NOT mean the modem can demodulate. RSSI measures total power at the wideband IF stage, before the narrowband channel filter. The preamble detector operates after:
1. Channel filter (narrowband)
2. Decimation
3. Bit clock recovery (BCR)

A signal can show -30 dBm RSSI but be completely invisible to the preamble detector if any of: frequency offset exceeds filter bandwidth, clock rates are wrong due to missing GLOBAL_CONFIG, or BCR parameters don't match the actual data rate.

## AFC Configuration

For cross-device communication (different crystals), AFC is essential.

### WDS AFC Settings
- Set `AFC_en: 1` in WDS and `Crystal tolerance: 50 ppm` (or your actual tolerance)
- WDS will automatically widen the channel filter (e.g., 46 kHz → 103 kHz) and set appropriate AFC limiter values
- AFC is enabled via bit 7 of `MODEM_AFC_GAIN_1` (0x80), not through `AFC_GEAR`
- WDS may set `AFC_GEAR = 0x00` even with AFC enabled — this is normal, the gain register controls enablement

### AFC Limiter
The `AFC_LIMITER` property (group 0x20, properties 0x30-0x31) sets the maximum frequency offset AFC can correct:
```
AFC_LIMITER = offset_Hz * 2^19 * outdiv / XTAL_Hz
```
For 17 kHz with 30 MHz XTAL and outdiv=8: `17000 * 524288 * 8 / 30000000 = 2388 = 0x0954`.

If the crystal offset exceeds the AFC limiter, preamble detection fails completely.

## Pluto SDR Crystal Offset

The ADALM-Pluto has ~50 ppm crystal tolerance. At 437 MHz this is ~22 kHz.

### Measuring the Offset
1. Put Si4463 in TX mode (command `x`)
2. Set Pluto RX LO above the nominal frequency (e.g., 437.28 MHz) to avoid DC/LO leakage
3. Capture IQ, take FFT, find the peak frequency offset
4. Our measurement: ~26 kHz (Pluto crystal is low)

### Correcting the Offset
Add the correction to the Pluto's LO setting:
```python
PLUTO_XO_CORRECTION = 26000  # Hz, positive because Pluto crystal is low
sdr.tx_lo = int(437.08e6 + PLUTO_XO_CORRECTION)
sdr.rx_lo = int(437.08e6 + PLUTO_XO_CORRECTION - LO_OFFSET)
```

Without correction, the signal lands near the edge of the Si4463's channel filter, making preamble detection unreliable.

## Si4463 Rev C2 Does Not Need a ROM Patch

- Our chip: Si4463 Rev C2, CHIPREV=0x22, ROMID=0x06
- All patch files found for ROMID=0x06 have `FUNCTION=TEST` (Silicon Labs internal testing only)
- No production patch (`FUNCTION=MAIN`) exists for Rev C2
- Applying the test patch (PATCHID=0xCA90) **crashes the chip**: RSSI=0, device state UNKNOWN, nIRQ stuck
- WDS generates `BOOT_OPTIONS=0x81` (with patch flag) in its `RF_POWER_UP` but this is for WDS's internal test flow
- Use `BOOT_OPTIONS=0x01` (no patch) for production firmware

## Frequency Configuration

### The FRAC Bug
The original ESP32 code had `FREQ_CONTROL_FRAC = 0x02380C` with a comment saying "437.08 MHz". The actual frequency with this value:
```
f = (57 + 0x02380C/524288) * 2 * 30e6 / 8 = 429.58 MHz
```
That's **7.5 MHz off target**. The correct WDS-generated value is `FRAC = 0x0A37FA`:
```
f = (57 + 0x0A37FA/524288) * 2 * 30e6 / 8 = 437.08 MHz
```

### MODEM_CLKGEN_BAND
Must match the frequency band:
- 420-470 MHz: `CLKGEN_BAND = 0x0A` (BAND=2, outdiv=8, SY_SEL=1)
- 850-1050 MHz: `CLKGEN_BAND = 0x08` (BAND=0, outdiv=4)

The original code had `0x08` for a 437 MHz signal — wrong band, wrong outdiv. This misconfigures the modem clock generator even though the synthesizer output frequency may appear correct on a spectrum analyzer.

## RF Switch GPIO Configuration

The CubeSat Kit board uses an AS179 RF switch controlled by Si4463 GPIO2/GPIO3. The correct configuration determined by RSSI sweep:
- **RX**: GPIO2=HIGH (0x03), GPIO3=LOW (0x02)
- **TX**: GPIO2=LOW (0x02), GPIO3=HIGH (0x03)

The firmware includes a 3-config RSSI sweep test during RX initialization to verify the correct GPIO polarity. If the sweep shows signal on a different config than expected, the GPIO assignments need to be swapped.

## Debugging Checklist

When Si4463 RX doesn't receive packets:

1. **Verify TX frequency** — Use Pluto as spectrum analyzer. If the peak isn't at the expected frequency, check `FREQ_CONTROL_FRAC` and `CLKGEN_BAND`.

2. **Check RSSI during signal** — If RSSI doesn't rise when TX is active, the RF path is broken (antenna, cable, RF switch GPIO).

3. **Check MODEM_STATUS** — Read `GET_MODEM_STATUS` response byte 1:
   - Bit 0: PREAMBLE_DETECT
   - Bit 1: SYNC_DETECT
   - Bit 3: RSSI above threshold
   - If only bit 3 is set: signal present but modem can't demodulate → check GLOBAL_CONFIG, AFC, channel filter.

4. **Check RSSI_THRESH** — Property 0x20:0x4A. If set to 0xFF (-6.5 dBm), preamble detection is blocked for any normal signal level. Set to 0x14 (-124 dBm) or let WDS choose.

5. **Verify GLOBAL_CONFIG = 0x20** — Missing this makes all WDS modem values wrong.

6. **Verify AFC is enabled and limiter is wide enough** — `AFC_GAIN_1` bit 7 must be set. `AFC_LIMITER` must exceed the expected crystal offset.

7. **Check for partial WDS config** — If you customized any PKT properties from the WDS defaults, try loading the complete WDS config verbatim to see if that fixes it.

8. **Don't apply ROM patches for Rev C2** — There is no production patch for ROMID=0x06. Test patches crash the chip.

## WDS Configuration Parameters

The working WDS input parameters for our application:
```
Crystal frequency: 30.000 MHz
Crystal tolerance RX/TX: 50 ppm
Base frequency: 437.08 MHz
Modulation: 2GFSK
Data rate: 9.600 kbps
Deviation: 4.800 kHz
Enable PLL AFC: YES
Optimize RX: High sensitivity
Packet: Variable length payload with CRC
  Preamble: 8 bytes TX, 20 bit RX threshold, Std 1010
  Sync word: 2 bytes, on-air 2D D4
  Field 1: 1 byte (length), CRC enabled over field
  Field 2: 63 bytes max (variable), CRC transmit + enabled
  CRC: CCITT-16 (X16+X12+X5+1), seed 0x0000
  Variable length: enabled, field 2 varies, length in field 1
GPIO 2/3: DONOTHING (firmware controls RF switch separately)
```

## SPI and Timing Notes

- SPI Mode 0 (CKP=0, CKE=1), clock ~1 MHz on 16 MHz Fcy
- POWER_UP command: use `0x01` for BOOT_OPTIONS (no patch), `0x00` for XTAL_OPTIONS (crystal, not TCXO)
- After POWER_UP, wait for CTS before any other command
- Insert `__builtin_clrwdt()` every 3-4 SET_PROPERTY commands during configuration — the full WDS config has ~30 commands and takes several hundred milliseconds
- The `set_properties` function handles max 12 properties per call (Si4463 API limit)
