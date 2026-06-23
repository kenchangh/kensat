# Electrical Power System (EPS)

Custom EPS board for KENSAT — Slot 2 in the 2U stack, above the OBC. Generates and
distributes the satellite's power rails, manages the battery pack, and provides
a hardware watchdog that can power-cycle the rest of the spacecraft.

- **Battery:** 2S2P 18650 Li-ion pack (mounted on the board underside)
- **Rails:** regulated buses distributed over the 104-pin CubeSat Kit Bus
- **Telemetry:** battery voltage / current / temperature, read by the OBC over I²C
- **Reliability:** hardware watchdog (see
  [docs/system-architecture.md](../docs/system-architecture.md) §7.1)

## Files

```
eps/hardware/
├── KENSAT_EPS_v1.2.kicad_pro   KiCad project
├── KENSAT_EPS_v1.2.kicad_sch   Schematic
└── KENSAT_EPS_v1.2.kicad_pcb   PCB layout
```

Open with [KiCad](https://www.kicad.org/) 7+. Power-rail and bus pin
assignments are documented in
[docs/system-architecture.md](../docs/system-architecture.md) §5.6–5.7.
