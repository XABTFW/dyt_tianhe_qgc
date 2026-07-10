#!/usr/bin/env python3
"""
Radar Bridge fixed-frame replay sender.

Sends a fixed list of pre-built frames verbatim (byte-for-byte) over a serial
port, one frame every N seconds (default 30 s), looping forever.

These are the exact bytes provided by the user; the script does NOT rebuild or
re-CRC them, it just transmits them as-is. The trailing 2 bytes of each frame
are the CRC that was already computed for that frame (verified to be MAVLink
X25 / CRC16_MCRF4XX over all bytes except the leading SOF and the CRC itself).

Usage:
    python3 radar_replay_sender.py --port /dev/pts/5 --baud 115200 --interval 30
    python3 radar_replay_sender.py --dry-run          # just print, no serial
"""

import argparse
import time

# The five frames to replay, in order. Each string is whitespace-separated hex.
FRAMES_HEX = [
    "FD 10 38 01 01 C5 CE C2 19 00 00 00 28 A7 6C 14 40 1F F1 40 C0 F2 FC FF 51 3C",
    "FD 10 39 01 01 C5 CE C2 19 00 00 00 70 93 93 EB C0 39 F7 40 C0 F2 FC FF 13 64",
    "FD 10 3A 01 01 C5 CE C2 19 00 00 00 4F D1 56 14 C0 6F 5C 05 80 02 87 00 DD 75",
    "FD 10 3B 01 01 C5 CE C2 19 00 00 00 B0 D1 8A FD 40 E1 72 34 00 00 00 00 77 7D",
    "FD 10 40 01 01 C5 CE C2 19 00 00 00 40 C3 2A 2C 00 5C BE 3F 60 79 FE FF 89 23",
]


def hex_to_bytes(s: str) -> bytes:
    return bytes(int(tok, 16) for tok in s.split())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", help="serial port, e.g. /dev/ttyUSB0, /dev/pts/5, COM3")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--interval", type=float, default=30.0,
                    help="seconds between frames (default 30)")
    ap.add_argument("--once", action="store_true",
                    help="send each frame once then stop (no looping)")
    ap.add_argument("--dry-run", action="store_true",
                    help="print frames instead of sending over serial")
    args = ap.parse_args()

    frames = [hex_to_bytes(h) for h in FRAMES_HEX]

    ser = None
    if not args.dry_run:
        import serial  # pip install pyserial
        ser = serial.Serial(args.port, args.baud, timeout=0)

    idx = 0
    n = len(frames)
    while True:
        frame = frames[idx]
        label = f"frame {idx + 1}/{n} (SEQ=0x{frame[2]:02X})"
        if args.dry_run:
            print(label, frame.hex(" "))
        else:
            ser.write(frame)
            ser.flush()
            print(f"sent {label}: {frame.hex(' ')}")

        idx += 1
        if idx >= n:
            idx = 0
            if args.once:
                break
        time.sleep(args.interval)


if __name__ == "__main__":
    main()
