#!/usr/bin/env python3
"""Send 'x' command to PIC24 serial port to trigger Si4463 TX (100 packets)."""
import serial
import time
import sys

port = '/dev/cu.usbserial-BG03U9VQ'
try:
    ser = serial.Serial(port, 9600, timeout=1,
                        xonxoff=False, rtscts=False, dsrdtr=False)
    time.sleep(0.1)
    ser.write(b'x')
    print(f"Sent 'x' to {port}")
    time.sleep(0.5)
    resp = ser.read(ser.in_waiting or 0)
    if resp:
        print(f"Response: {repr(resp)}")
    ser.close()
except Exception as e:
    print(f"ERROR: {e}")
    sys.exit(1)
