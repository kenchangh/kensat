#!/usr/bin/env python3
"""Set UART1 pinmux on Jetson Orin Nano dev board.

UART1_TX (PR2) and UART1_RX (PR3) default to GPIO mode on boot.
This writes PADCTL registers to select the uarta function.

Run with sudo. Installed as a systemd service (uart-pinmux.service)
so it runs automatically at boot.
"""
import mmap
import os
import struct

PADCTL_BASE = 0x02430000
TX_OFFSET   = 0x4020       # UART1_TX_PR2 at 0x02434020
RX_OFFSET   = 0x4028       # UART1_RX_PR3 at 0x02434028

PAGE_SIZE = 0x10000
MAP_BASE  = PADCTL_BASE & ~(PAGE_SIZE - 1)

fd = os.open("/dev/mem", os.O_RDWR | os.O_SYNC)
mm = mmap.mmap(fd, PAGE_SIZE, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE, offset=MAP_BASE)

# clear_mask: bits to clear in each pad.
#   [1:0] function-select -> 0 = uarta (both pins)
#   [4]   TRISTATE        -> 0 = output driver enabled (TX only; RX stays an input)
# Without clearing TRISTATE on TX, the pin is muxed to uarta but its output
# driver is off, so no data ever drives out (logic analyzer sees only noise).
for name, offset, clear_mask in [
    ("UART1_TX_PR2", TX_OFFSET, 0x3 | 0x10),
    ("UART1_RX_PR3", RX_OFFSET, 0x3),
]:
    addr = (PADCTL_BASE + offset) - MAP_BASE
    old_val = struct.unpack("<I", mm[addr:addr+4])[0]
    new_val = old_val & ~clear_mask
    struct.pack_into("<I", mm, addr, new_val)
    check = struct.unpack("<I", mm[addr:addr+4])[0]
    print(f"{name}: 0x{old_val:08x} -> 0x{check:08x}")

mm.close()
os.close(fd)
print("UART1 pinmux set to uarta mode.")
