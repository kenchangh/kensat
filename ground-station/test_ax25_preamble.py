#!/usr/bin/env python3
"""
Regression guard for the AX.25 TX preamble length.

The OBC sends AX25_TX_LEADING_FLAGS HDLC flags ahead of every downlink
frame. That preamble exists for *third-party* receivers, not for us:
stock gr-satellites (what SatNOGS ground stations run) has to pull a
symbol-sync loop into lock in real time and gets one pass at it.
Measured on clean synthetic frames at 48 kSps:

    leading flags | frames decoded by gr-satellites
    --------------+--------------------------------
           4      |  3 / 6
           8      |  4 / 6
          10      |  6 / 6
          16      |  6 / 6   <-- shipped, with margin

Trimming 16 -> 8 saves 53 ms of airtime per beacon at 1200 bps (~10% of
a 550 ms beacon) and loses about a third of every frame at every SatNOGS
station that hears us.

Nothing else in this repo will catch that. pluto_rx_ax25.py brute-forces
every symbol timing phase over a captured buffer, so it decodes happily
at 4 flags. This test and the compile-time floor in ax25.h are the only
two things standing between a well-meaning airtime optimization and a
silent third-party reception loss.

No hardware and no dependencies. Run directly, or under pytest:

    python3 ground-station/test_ax25_preamble.py
"""

import re
import sys
from pathlib import Path

# Minimum leading flags. Below this, third-party decode measurably drops.
MIN_LEADING_FLAGS = 16

REPO_ROOT = Path(__file__).resolve().parent.parent
AX25_H = REPO_ROOT / "firmware" / "obc" / "app" / "ax25.h"
AX25_C = REPO_ROOT / "firmware" / "obc" / "app" / "ax25.c"

WHY = __doc__.split("Measured on clean", 1)[1]
WHY = "Measured on clean" + WHY.split("No hardware", 1)[0].rstrip()


def read_leading_flags() -> int:
    """Return the AX25_TX_LEADING_FLAGS value defined in ax25.h."""
    text = AX25_H.read_text(encoding="utf-8")
    m = re.search(r"^\s*#define\s+AX25_TX_LEADING_FLAGS\s+(\d+)\s*$",
                  text, re.MULTILINE)
    if not m:
        raise AssertionError(
            f"AX25_TX_LEADING_FLAGS not found in {AX25_H}.\n"
            "It must stay defined there as a plain integer literal.\n" + WHY)
    return int(m.group(1))


def test_leading_flags_meet_floor():
    """The preamble must not be trimmed below the measured floor."""
    n = read_leading_flags()
    assert n >= MIN_LEADING_FLAGS, (
        f"AX25_TX_LEADING_FLAGS is {n}, below the floor of "
        f"{MIN_LEADING_FLAGS}.\n"
        "This does not break our own decoder. It breaks other people's.\n"
        + WHY)


def test_encoder_still_uses_the_constant():
    """ax25.c must emit the preamble from the constant, not a literal.

    Guards the other half of the regression: leaving the #define at 16
    while trimming the emit loop in ax25.c would pass the header check
    and the compile-time floor alike.
    """
    text = AX25_C.read_text(encoding="utf-8")
    assert re.search(r"i\s*<\s*AX25_TX_LEADING_FLAGS\s*;.*bs_emit_flag", text), (
        f"The leading-flag emit loop in {AX25_C} no longer reads\n"
        "AX25_TX_LEADING_FLAGS. The preamble length must come from the\n"
        "constant so the header check and the compile-time floor in\n"
        "ax25.h actually govern what goes on the air.\n" + WHY)


def main() -> int:
    failures = 0
    for fn in (test_leading_flags_meet_floor, test_encoder_still_uses_the_constant):
        try:
            fn()
        except AssertionError as e:
            print(f"FAIL  {fn.__name__}\n{e}\n", file=sys.stderr)
            failures += 1
        else:
            print(f"ok    {fn.__name__}")
    if failures:
        print(f"{failures} check(s) failed", file=sys.stderr)
        return 1
    print(f"AX25_TX_LEADING_FLAGS = {read_leading_flags()} "
          f"(floor {MIN_LEADING_FLAGS}) -- preamble intact")
    return 0


if __name__ == "__main__":
    sys.exit(main())
