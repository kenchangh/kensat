# Burn-Wire Deployment Board

Antenna-release board for KENSAT. On command from the OBC, it energizes
**redundant nichrome burn wires** that melt through the fishing-line tether
restraining the tape-measure dipole; the elements then spring outward into a
straight half-wave dipole.

The release is driven by GPIO **IO.24** and **IO.25** from the PIC24, doubled
for redundancy, held HIGH for ~10 s. The deployment sequence — including the
flash-persisted, fail-safe, bounded-retry state machine — is documented in
[docs/system-architecture.md](../docs/system-architecture.md) §2.1–2.2 and
implemented in [firmware/obc/app/task_beacon.c](../firmware/obc/app/task_beacon.c).

## Files

```
burnwire/hardware/
├── burnwire.kicad_pro / .kicad_sch / .kicad_pcb   KiCad design
├── burnwire.csv                                   BOM
├── burnwire_bom_jlc.csv / burnwire_cpl_jlc.csv    JLCPCB fabrication BOM + placement
└── burnwire-all-pos.csv                           component placement
```

Open with [KiCad](https://www.kicad.org/) 7+.
