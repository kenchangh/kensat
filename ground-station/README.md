# Ground Station & SDR Tooling

Software-defined-radio ground-station tools for receiving, decoding, and
debugging the KENSAT UHF link at 437.08 MHz, built around an **ADALM-Pluto**
SDR. These scripts capture and demodulate the GFSK / AX.25 beacon, characterize
the link, and run end-to-end tests against the flight hardware.

## Receive & decode

| Script | Purpose |
|--------|---------|
| `pluto_rx.py`, `pluto_rx_ax25.py` | Pluto receive; AX.25 frame decode |
| `beacon_decoder.py` | Decode the KENSAT AX.25 telemetry beacon |
| `llm_downlink.py` | Reassemble a downlinked LLM inference result from its `TXT` chunk frames and verify the CRC-32 (`--selftest` needs no radio) |
| `pluto_iq_capture.py` | Capture raw IQ to file for offline analysis |
| `pluto_waterfall.py`, `pluto_wide_scan.py`, `pluto_freq_sweep.py` | Spectrum waterfall / scanning |

## Transmit & stimulus

| Script | Purpose |
|--------|---------|
| `pluto_tx.py` | Pluto transmit |
| `pluto_cw.py` | CW tone for path / antenna testing |
| `send_x.py`, `serial_rssi_monitor.py` | Inject test traffic; monitor RSSI over serial |

## Test & visualization

| Script | Purpose |
|--------|---------|
| `test_e2e.py` | Full end-to-end test: trigger Jetson inference on the OBC → radio downlink → decode (host/credentials via `KENSAT_JETSON_HOST` / `KENSAT_JETSON_PW`) |
| `test_rx_offline.py`, `run_test.py` | Offline RX decode against captured IQ; test runner |
| `test_ax25_preamble.py` | Guards the 16-flag TX preamble that third-party SatNOGS/gr-satellites decode depends on (no hardware needed) |
| `imu_visualizer.py` | Visualize BMI088 IMU telemetry |

## Setup

```sh
pip install -r requirements.txt
```

Requires an ADALM-Pluto SDR (libiio / pyadi-iio) for the live-radio scripts;
the offline decoders run against captured IQ files without hardware.

## Receiving an inference result

The OBC downlinks a housekeeping beacon (`BCN`), one summary frame per
inference (`LLM`), and the generated text itself as numbered `TXT` chunk
frames — one chunk per beacon cycle, with the whole set repeated, since the
link has no ARQ. Group by `s=`, order by `c=`, check `h=`:

```sh
python pluto_rx_ax25.py --duration 600 | python llm_downlink.py
```

`llm_downlink.py` accepts either live decoder output or a saved session log,
reports which chunks are missing, and verifies the reassembled text against
the CRC-32 the spacecraft computed on board. Frame formats are documented in
[firmware/obc/README.md](../firmware/obc/README.md#downlink-frames).
