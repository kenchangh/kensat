#!/usr/bin/env python3
"""Reassemble KENSAT LLM inference results from decoded AX.25 frames.

The spacecraft downlinks the text its on-board LLM generated as a numbered
series of AX.25 UI frames, because a result (up to 512 bytes) does not fit in
one 180-byte info field:

    TXT s=<seq> c=<k>/<m> h=<crc32> <text>

  s=  result sequence number, shared with the "LLM seq=..." housekeeping
      frame for the same inference
  c=  chunk k of m
  h=  CRC-32 (IEEE, == Python's zlib.crc32) of the COMPLETE result text, as
      computed on board over exactly the bytes it transmitted
  then a space, then that chunk's slice of the text

Those four fields are the entire reassembly protocol: group by s=, order by
c=, concatenate, check h=. There is no ARQ on this link and no way to ask for
a repeat, so the spacecraft simply sends each result twice (chunks
interleaved one per beacon cycle); duplicates are expected and are dropped
here.

The matching housekeeping frame, if it was heard, carries how the inference
went plus "txt=<bytes>/<frames>", which tells you what a complete result for
that sequence number looks like even if you missed some of the text:

    LLM seq=7 pid=99 up=412m cpu=47 gpu=45 pk=61/58 mw=6820 ram=3104
        ml=0s inf=14s lat=14000ms st=03 h=1C2A9F40 txt=300B/3

Usage
-----
    # live, piped straight from the AX.25 receiver
    python pluto_rx_ax25.py --duration 600 | python llm_downlink.py

    # from a saved session log
    python llm_downlink.py session.log

    # no radio, no capture: round-trip synthetic frames
    python llm_downlink.py --selftest

Input is free-form: any line containing a TXT or LLM frame is picked up,
whether it is a bare info field or one of pluto_rx_ax25.py's
    INFO (123 B): "TXT s=7 c=1/3 h=1C2A9F40 ..."
lines. Everything else is ignored, so pointing this at a whole session log
is fine.

Exit status is 0 if at least one result reassembled completely with a
matching CRC, else 1.
"""

import argparse
import re
import sys
import zlib

# One TXT frame. The payload runs to the end of the field and may be empty.
TXT_RE = re.compile(
    r"TXT s=(?P<seq>\d+) c=(?P<k>\d+)/(?P<m>\d+) h=(?P<crc>[0-9A-Fa-f]{8}) ?"
    r"(?P<text>.*)$"
)

# The housekeeping frame for the same result. Only the fields this tool uses
# are named; the rest of the line is left alone.
LLM_RE = re.compile(
    r"LLM seq=(?P<seq>\d+) .*?h=(?P<crc>[0-9A-Fa-f]{8})"
    r"(?: txt=(?P<bytes>\d+)B/(?P<frames>\d+))?"
)

# pluto_rx_ax25.py's frame dump: the info field is inside the quotes.
INFO_RE = re.compile(r'INFO \(\d+ B\): "(?P<info>.*)"\s*$')


class Result:
    """Chunks collected so far for one result sequence number."""

    def __init__(self, seq):
        self.seq = seq
        self.total = None          # m, from the chunks
        self.crc = None            # h=, from the chunks
        self.chunks = {}           # k -> text
        self.copies = 0            # frames accepted, including repeats
        self.conflicts = []        # k where two frames disagreed
        self.meta = None           # the LLM housekeeping line, if heard

    def add_chunk(self, k, m, crc, text):
        self.copies += 1
        self.total = m
        self.crc = crc
        if k in self.chunks and self.chunks[k] != text:
            # Two frames claim the same chunk with different bytes: at least
            # one is corrupt in a way that still passed the CRC-16 on the
            # frame. Keep the first and say so rather than silently picking.
            self.conflicts.append(k)
            return
        self.chunks[k] = text

    @property
    def missing(self):
        if self.total is None:
            return []
        return [k for k in range(1, self.total + 1) if k not in self.chunks]

    @property
    def complete(self):
        return self.total is not None and not self.missing

    def text(self):
        return "".join(self.chunks[k] for k in sorted(self.chunks))

    def crc_ok(self):
        if not self.complete or self.crc is None:
            return False
        return zlib.crc32(self.text().encode("ascii", "replace")) == self.crc


def harvest(lines):
    """Pull TXT/LLM frames out of arbitrary text. Returns {seq: Result}."""
    results = {}

    for line in lines:
        m = INFO_RE.search(line)
        field = m.group("info") if m else line.rstrip("\r\n")

        t = TXT_RE.search(field)
        if t:
            seq = int(t.group("seq"))
            r = results.setdefault(seq, Result(seq))
            r.add_chunk(int(t.group("k")), int(t.group("m")),
                        int(t.group("crc"), 16), t.group("text"))
            continue

        h = LLM_RE.search(field)
        if h:
            seq = int(h.group("seq"))
            r = results.setdefault(seq, Result(seq))
            r.meta = field.strip()
            if r.crc is None:
                r.crc = int(h.group("crc"), 16)
            if r.total is None and h.group("frames"):
                r.total = int(h.group("frames"))

    return results


def report(results, out=sys.stdout):
    """Print one block per result. Returns the number fully verified."""
    verified = 0

    for seq in sorted(results):
        r = results[seq]
        total = r.total if r.total is not None else "?"

        print(f"\n=== result #{seq} "
              f"({len(r.chunks)}/{total} chunks, {r.copies} frames heard) ===",
              file=out)
        if r.meta:
            print(f"  {r.meta}", file=out)

        if r.conflicts:
            print(f"  WARNING: disagreeing copies of chunk(s) "
                  f"{', '.join(str(k) for k in sorted(set(r.conflicts)))} "
                  f"— kept the first", file=out)

        if not r.complete:
            miss = ", ".join(str(k) for k in r.missing) or "unknown"
            print(f"  INCOMPLETE — missing chunk(s): {miss}", file=out)
            if r.chunks:
                print(f"  partial text: {r.text()!r}", file=out)
            continue

        text = r.text()
        if r.crc_ok():
            verified += 1
            print(f"  CRC-32 {r.crc:08X} OK ({len(text)} bytes)", file=out)
        else:
            got = zlib.crc32(text.encode("ascii", "replace"))
            print(f"  CRC-32 MISMATCH: frames say {r.crc:08X}, "
                  f"text gives {got:08X} — text below is suspect", file=out)
        print(f"  {text}", file=out)

    if not results:
        print("No TXT or LLM frames found in the input.", file=out)

    return verified


def build_frames(seq, text, chunk=144, passes=2):
    """Build the frames the spacecraft would send for one result.

    Mirrors beacon_tx_llm_text() in firmware/obc/app/task_beacon.c: same
    header, same 144-byte chunking, same repeat pass. Used by --selftest.
    """
    crc = zlib.crc32(text.encode("ascii"))
    pieces = [text[i:i + chunk] for i in range(0, len(text), chunk)] or [""]
    frames = []
    for _ in range(passes):
        for k, piece in enumerate(pieces, start=1):
            frames.append(f"TXT s={seq} c={k}/{len(pieces)} h={crc:08X} {piece}")
    return frames


def selftest():
    """Round-trip synthetic frames, including lossy and corrupt cases."""
    text = ("Earth from orbit is a thin blue line above a black horizon, "
            "and every city at night is a smear of orange light. " * 3)[:300]
    frames = build_frames(7, text)
    fails = 0

    def check(cond, what):
        nonlocal fails
        print(f"  {'ok  ' if cond else 'FAIL'}  {what}")
        if not cond:
            fails += 1

    print("clean capture:")
    r = harvest(frames)[7]
    check(r.complete, "all chunks present")
    check(r.text() == text, "text matches")
    check(r.crc_ok(), "CRC verifies")

    print("as printed by pluto_rx_ax25.py:")
    wrapped = [f'      INFO ({len(f)} B): "{f}"' for f in frames]
    r = harvest(["Listening...", "-" * 72] + wrapped)[7]
    check(r.complete and r.text() == text and r.crc_ok(),
          "parsed out of decoder output")

    print("one chunk lost in both passes:")
    lost = [f for f in frames if " c=2/" not in f]
    r = harvest(lost)[7]
    check(not r.complete, "reported incomplete")
    check(r.missing == [2], f"names the missing chunk (got {r.missing})")

    print("one chunk lost in the first pass only:")
    half = [f for f in frames[:3] if " c=2/" not in f] + frames[3:]
    r = harvest(half)[7]
    check(r.complete and r.text() == text, "repeat pass fills the gap")

    print("corrupted text, intact header:")
    bad = list(frames[:3])
    bad[1] = bad[1][:-5] + "xxxxx"
    r = harvest(bad)[7]
    check(r.complete, "still reassembles")
    check(not r.crc_ok(), "CRC catches the corruption")

    print("housekeeping frame alone:")
    r = harvest([f"LLM seq=9 pid=99 up=1m cpu=40 gpu=40 pk=1/1 mw=100 ram=1 "
                 f"ml=0s inf=1s lat=1000ms st=03 h=DEADBEEF txt=300B/3"])[9]
    check(r.total == 3 and not r.complete,
          "expected chunk count known, result incomplete")

    print("empty input:")
    check(harvest(["nothing to see here"]) == {}, "no results invented")

    print("\nselftest FAILED\n" if fails else "\nselftest passed\n")
    return 1 if fails else 0


def main():
    ap = argparse.ArgumentParser(
        description="Reassemble KENSAT LLM inference results from decoded "
                    "AX.25 frames.")
    ap.add_argument("logfile", nargs="?",
                    help="file of decoded frames / decoder output "
                         "(default: stdin)")
    ap.add_argument("--selftest", action="store_true",
                    help="round-trip synthetic frames; needs no radio")
    args = ap.parse_args()

    if args.selftest:
        return selftest()

    if args.logfile:
        with open(args.logfile, "r", errors="replace") as fh:
            results = harvest(fh)
    else:
        if sys.stdin.isatty():
            print("Reading frames from stdin; Ctrl-D to finish "
                  "(or pass a log file).", file=sys.stderr)
        results = harvest(sys.stdin)

    return 0 if report(results) else 1


if __name__ == "__main__":
    sys.exit(main())
