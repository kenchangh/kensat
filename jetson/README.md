# Jetson Edge-AI Payload

The KENSAT payload is an **NVIDIA Jetson Orin Nano** running a quantized LLM
on-orbit. It is normally powered **off**; the PIC24 OBC power-gates it via a
high-side load switch and exchanges commands and results over UART. When woken,
the Jetson runs an inference from a pre-loaded prompt library and returns the
text, which the OBC frames as AX.25 and downlinks.

- **Compute:** Jetson Orin Nano (8 GB), JetPack 6 / L4T R36.4.4, CUDA
- **Model:** TinyLlama 1.1B (Q4_K_M) under llama.cpp (`llama-server`, HTTP API)
- **Link to OBC:** UART at 9600 8N1 (`/dev/ttyTHS1`)
- **Power/thermal:** runs in a low-power mode; thermal behaviour in vacuum is
  analysed in [docs/system-architecture.md](../docs/system-architecture.md) §8.1

See [docs/jetson-startup.md](../docs/jetson-startup.md) for the boot/inference
timing and bring-up notes.

## Files

| File | Purpose |
|------|---------|
| `kensat_daemon.sh` | Main payload daemon: starts `llama-server`, runs inference over HTTP, returns result on UART |
| `prompt_listener.py` | UART prompt listener |
| `devmem_uart.py` | UART pin-mux / register setup |
| `thermal_profile.sh` | Thermal characterization helper |
| `kensat.service`, `kensat-heartbeat.service`, `uart-pinmux.service` | systemd units (deploy under `/etc/systemd/system/`) |

## Configuration / secrets

No credentials are committed. The daemon escalates privilege through an
environment variable:

```sh
export SUDO_PASS="..."     # only if passwordless sudo is unavailable
```

The end-to-end test harness in
[ground-station/test_e2e.py](../ground-station/test_e2e.py) reads the Jetson
host and password from `KENSAT_JETSON_HOST` / `KENSAT_JETSON_PW`.
