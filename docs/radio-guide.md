# Si4463 GFSK Radio Guide

Practical guide for operating the Si4463 GFSK radio link between the Pumpkin CubeSat Kit OBC and an ADALM-Pluto SDR (or future ground station). Documents what works, what doesn't, and the non-obvious lessons from getting Pluto TX to Si4463 RX working at 437.08 MHz.

For register-level debugging reference, see `docs/debug-si4463.md`.

## Working Configuration

### RF Parameters
| Parameter | Value |
|-----------|-------|
| Frequency | 437.08 MHz (70cm amateur band) |
| Modulation | GFSK |
| Data rate | 9600 bps |
| Deviation | +/- 4800 Hz |
| BT (Gaussian) | 0.5 |
| Modulation index | 1.0 |

### Packet Format (on-air)
```
[Preamble 64B] [Sync 2B: 0x2DD4] [Length 1B] [Payload <=64B] [CRC-16 2B]
```

- Preamble: 64 bytes of 0xAA (512 bits). Required for reliable AFC, BCR, and DSA lock.
- Sync word: 0x2DD4 on-air. Si4463 stores it bit-reversed as 0xB4, 0x2B.
- Length byte: payload length (0-63). Consumed by Si4463 packet handler (Field 1).
- CRC-16: CCITT polynomial 0x1021, seed 0x0000, big-endian. Covers length + payload.

> **OBC uplink differs.** This section is the bench link that was brought up
> first. The flight OBC's uplink receiver (`si4463_pkt_cfg_rx()` in
> `firmware/obc/app/task_si4463.c`) uses a **2-byte big-endian length**
> (`PKT_LEN = 0x3A`) so a frame can carry the ~280-byte `PROMPT`
> telecommand, and needs a preamble longer than its 400 ms poll period
> (768 bytes by default). `pluto_tx.py` builds that frame by default;
> `--len-bytes 1` gives the format above.

### Si4463 Config Highlights (task_si4463.c)
| Register | Value | Notes |
|----------|-------|-------|
| PKT_CRC_CONFIG | 0x03 | CRC-16 CCITT. WDS generates 0x05 (CRC-32) -- must override. |
| FRR_CTL_A/B | 0x04, 0x07 | FRR_A=PH_PEND, FRR_B=MODEM_PEND for fast nIRQ triage |
| SYNC_CONFIG | 0x31 | 2-byte sync, allow 3 bit errors |
| DSA_QUAL | 0x40 | DSA enabled. Required for preamble detection. |
| MODEM_AFC_GAIN_1 | 0x80 | AFC enabled (bit 7) |
| AFC_LIMITER | 0x0954 | +/- 17 kHz max AFC correction |
| GPIO2/3 (RX) | HIGH/LOW | RF switch: 0x03/0x02 (cfg0) |
| RXVALID_STATE | READY | Host re-enters RX after each packet |

### Pluto SDR Settings (pluto_tx.py)
| Setting | Value |
|---------|-------|
| Sample rate | 960 kHz (100 samples/symbol) |
| XO correction | 26000 Hz (Pluto crystal is low by ~60 ppm) |
| TX gain | 0 dB with 10 dB inline attenuator |
| Preamble | 64 bytes (0xAA) |
| Cyclic TX | Yes -- single DMA buffer loops continuously |

## Performance

With the current configuration and 10 dB attenuator:
- **80% unique packet coverage** (24/30 unique IDs received over 90 seconds)
- **0% CRC error rate** -- every received packet has valid CRC
- **~5-6 packets per cycle** out of 30 transmitted per cycle
- AFC locks at approximately -1350 units (~9.7 kHz correction)

## The CRC-32 vs CRC-16 Bug

**This was the root cause of zero packet reception for most of the debugging effort.**

WDS (Wireless Development Suite) generated `PKT_CRC_CONFIG = 0x05`. In the Si4463 API:
- 0x03 = CRC-16 CCITT (polynomial 0x1021)
- 0x05 = CRC-32 ITU-T (Ethernet CRC)

With CRC-32 enabled, the Si4463 expected 4 CRC bytes after the payload. The Pluto sent 2 (CRC-16). The Si4463 would read 2 CRC bytes plus 2 bytes of the next packet's preamble, CRC check would fail silently, and no PH_PEND event would fire. No CRC_ERROR either -- the packet handler just timed out.

**How it was found:** Switched to fixed-length mode (20 bytes, no CRC) and received perfect packets with correct payload. Computed CRC-16 offline -- it matched the received bytes. Changed PKT_CRC_CONFIG to 0x03 and packets started arriving with valid CRC.

**Lesson:** Always verify the CRC polynomial index against the Si4463 API documentation. Don't trust the WDS default if your TX side uses a different CRC.

## FRR Fast Response Registers

The single biggest performance improvement. Without FRR, every nIRQ event (including false preamble detects from noise) triggered a full `GET_INT_STATUS` SPI transaction: 4 bytes out + 8 bytes in = ~100 microseconds. With hundreds of false preamble detects per second, the PIC24 spent most of its time servicing SPI instead of catching real packets.

**Fix:** Configure FRR_A = PH_PEND (0x04), FRR_B = MODEM_PEND (0x07). Read FRR on every nIRQ (~3 microseconds, no CTS polling needed). Only call full GET_INT_STATUS when FRR shows a PH event (PACKET_RX or CRC_ERROR).

```c
// FRR read: send 0x50, clock out 2 bytes, done
static void si4463_read_frr(unsigned char *frr_a, unsigned char *frr_b) {
    SI4463_CS_LOW();
    spi2_transfer(0x50);  // FRR_A_READ
    *frr_a = spi2_transfer(0x00);
    *frr_b = spi2_transfer(0x00);
    SI4463_CS_HIGH();
}

// In nIRQ handler:
if (SI4463_NIRQ_READ() == 0) {
    unsigned char frr_ph, frr_mdm;
    si4463_read_frr(&frr_ph, &frr_mdm);
    if (!(frr_ph & 0x30))  // No PACKET_RX or CRC_ERROR
        goto skip;          // Don't waste time on preamble events
    // ... full GET_INT_STATUS + packet handling ...
}
```

**FRR register mapping (Si4463 API):**
| Value | Register |
|-------|----------|
| 0x00 | Disabled |
| 0x04 | PH_PEND |
| 0x05 | PH_STATUS |
| 0x07 | MODEM_PEND |
| 0x08 | MODEM_STATUS |
| 0x0A | CHIP_PEND |

Note: 0x0A is CHIP_PEND, NOT PH_PEND. Getting this wrong means the FRR never sees packet events and all packets are silently dropped.

## DSA (Digital Sensitivity Adjustment)

DSA gates the preamble detector. It detects signal arrival and only starts the preamble search after DSA triggers. This prevents the preamble detector from chasing noise.

**DSA must be enabled (0x40) for the WDS config to work.** Without DSA, the modem sees signal (RSSI is elevated) but the preamble detector never triggers. This was verified empirically: disabling DSA (0x00) with RSSI=82-89 produced zero preamble detects. Re-enabling DSA immediately restored detection.

**DSA hysteresis problem:** After exposure to a strong signal (e.g., CW at RSSI=125), DSA raises the internal noise floor to ~85 and does not recover without a full chip reset (POWER_UP). This manifests as:
- RSSI stuck at 75-85 even with no signal present
- No preamble detection despite strong packets arriving
- RSSI doesn't drop to the normal noise floor of 40-50

**Mitigation:** Use a 10 dB inline attenuator between Pluto TX and Si4463 antenna. This keeps the signal in the 60-90 RSSI range where DSA works properly. Without the attenuator, the direct connection produces RSSI=125 which saturates DSA.

## Things That Don't Work

### Disabling DSA for SDR Compatibility
Attempted because cyclic TX from the Pluto confuses DSA's signal arrival detection. Result: zero preamble detects even with strong signal. The WDS modem configuration assumes DSA is enabled.

### NRZ Polarity Inversion
Tested `exp(-j*phase)` instead of `exp(j*phase)` in the Pluto GFSK modulator. No improvement -- preamble detection is polarity-agnostic (0xAA = 0x55 in frequency domain), so this change has no effect on preamble. Sync word detection was still failing for other reasons (CRC-32 bug).

### RXVALID_STATE = RX (Auto-Restart)
Setting RXVALID/RXINVALID to RX (stay in RX) instead of READY (return to READY) caused the firmware to lock up. After the first packet, the chip auto-restarted RX and immediately began servicing preamble detects from the next cyclic packet. The FIFO from the just-received packet wasn't read in time. The firmware spin-locked on nIRQ with PH_PEND stuck. Use READY and explicitly re-enter RX from the host after reading the FIFO.

### Re-Entering RX on Every Modem nIRQ
Calling `si4463_set_mode(RX)` for every preamble-only nIRQ event consumed all CPU time. With hundreds of false preamble detects per second, the PIC24 never reached the periodic status poll or the PACKET_RX handler. Removing this call (modem events don't change the chip's RX state) was the fix.

### GPIO Sweep with GFSK Signal
The 3-config GPIO RSSI sweep was designed to auto-select the RF switch path. It works with CW but fails with GFSK because the 2-second sweep window rarely catches the short packet bursts. The sweep shows noise-floor RSSI for all configs. Hardcoding the known-good config (cfg0: GPIO2=HIGH, GPIO3=LOW) is more reliable.

### Pluto Frequency Sweep (pluto_freq_sweep.py)
The automated frequency sweep script was unreliable because:
1. The Si4463 RSSI poll interval (~2.5s) and the sweep dwell time (2.5s) frequently misaligned
2. Only 0-1 RSSI samples per step, producing noisy data
3. The peak was often at the wrong frequency due to sampling luck

Manual CW tests with longer dwell times gave better results.

## RF Path Requirements

### Physical Setup
```
Pluto TX port --> [10 dB attenuator] --> [F-F SMA adapter] --> Si4463 antenna connector
```

- **Must use the TX port** on the Pluto (not RX). The Pluto's RX port was connected by mistake during one session, producing zero signal.
- **10 dB attenuator is required** to prevent receiver saturation and DSA hysteresis. Without it, RSSI=125 during CW permanently corrupts the DSA state.
- Female-to-female SMA adapter works fine. Ensure finger-tight connections on both ends -- loose SMA caused intermittent signal loss during one debugging session.

### Expected RSSI Values (with 10 dB attenuator, TX gain 0 dB)
| Condition | RSSI | dBm |
|-----------|------|-----|
| No signal (noise floor) | 40-55 | -112 to -107 |
| CW carrier | 100-120 | -84 to -74 |
| GFSK packet peak | 75-95 | -97 to -87 |
| GFSK gap (between packets) | 60-75 | -104 to -97 |

If RSSI doesn't rise above 60 with CW active, check cables and attenuator.

## Pluto SDR XO Correction

The Pluto's crystal is off by approximately 60 ppm (26 kHz at 437 MHz). The correction is applied to the LO frequency:

```python
sdr.tx_lo = CENTER_FREQ + PLUTO_XO_CORRECTION  # 437080000 + 26000
```

This was determined empirically by sweeping XO corrections from 0 to 70000 and finding which value produced packet reception. 26000 Hz works, though the optimal value may drift with temperature.

The Si4463's AFC can correct up to +/- 17 kHz (AFC_LIMITER = 0x0954). Without the XO correction, the 26 kHz offset exceeds the AFC range and reception fails completely.

## Cyclic TX Mode

The Pluto uses cyclic DMA mode (`tx_cyclic_buffer = True`). This builds the entire packet sequence (N packets + gaps) into one IQ buffer and transmits it in a loop. This avoids DMA startup transients that corrupt the first few packets in non-cyclic mode.

Gaps between packets are `np.zeros()` (zero IQ = no RF output). The gap duration is configurable via `--gap-ms`. Shorter gaps (200ms) give more packet opportunities per cycle but reduce the time the Si4463 has to re-acquire.

**Important:** The Pluto IIO daemon can hang if a previous TX session wasn't properly cleaned up (e.g., Python process crashed). If `adi.Pluto()` hangs, kill all Python processes accessing the Pluto and retry. The Pluto is reachable at `192.168.3.1`.

## What's Left for 100% Reception

The current 80% hit rate (24/30 unique IDs) plateaus because ~6 packet positions in the cyclic buffer are consistently missed. The likely cause is DSA's adaptation to the cyclic pattern -- some packet positions fall during DSA's gain-settling transition. Potential improvements:

1. **Non-cyclic TX** -- Send packets individually with `sdr.tx()` calls per packet. Eliminates the cyclic pattern that DSA adapts to. Trade-off: DMA startup transients may corrupt short packets.
2. **DSA timing parameters** -- Adjust DSA_DECAY (0x78) and DSA_ARRSWTCH (0x04) to change how fast DSA adapts. Underdocumented by Silicon Labs.
3. **Longer preamble** -- Already at 64 bytes. Going to 128 would give even more AFC/DSA settling time but doubles the on-air time per packet.
4. **Use the real ground station** -- The Pluto SDR is a development tool. The actual ground station UHF radio will have better frequency stability, higher TX power, and proper RF chain. Many Pluto-specific workarounds (XO correction, attenuator, cyclic TX quirks) won't apply.

## Quick Start

### Receiving Packets on Si4463 from Pluto

1. Connect: Pluto TX port --> 10 dB atten --> Si4463 antenna
2. Flash the PIC24 firmware with the CRC-16 + FRR fixes
3. Open serial at 9600 baud to the PIC24 debug console
4. Send `s` to enter Si4463 RX mode
5. Start Pluto TX:
   ```bash
   cd scripts
   .venv/bin/python pluto_tx.py --packets 30 --gap-ms 200 --tx-gain 0 --xo-correction 26000
   ```
6. Watch for `RX [12]:` and `ASC: KO6IKZ NNNN` lines on the serial console

### Verifying the RF Path

1. Start CW: `.venv/bin/python pluto_cw.py --tx-gain 0`
2. Send `s` on serial to enter RX mode
3. RSSI should read 100-120 with CW active, 40-55 without
4. If RSSI doesn't change, check SMA connections and verify TX port

### Building and Flashing

```bash
~/Desktop/kensat_utils/build_project.sh -c -f  # clean build + flash
```

Source files are in `~/Pumpkin/` (main repo), NOT the worktree. The build script uses the main repo path.
