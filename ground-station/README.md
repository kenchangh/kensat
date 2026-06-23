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
| `imu_visualizer.py` | Visualize BMI088 IMU telemetry |

## Setup

```sh
pip install -r requirements.txt
```

Requires an ADALM-Pluto SDR (libiio / pyadi-iio) for the live-radio scripts;
the offline decoders run against captured IQ files without hardware.
