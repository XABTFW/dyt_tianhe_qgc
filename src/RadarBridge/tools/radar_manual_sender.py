#!/usr/bin/env python3
"""
Radar Bridge MANUAL POSITION sender.

Lets you type a specific latitude / longitude / altitude, builds a POSITION
frame in the real on-wire format, sends it over a (virtual or real) serial
port, and prints the exact bytes it sent as an upper-case hex string.

The ground station (QGC Radar Bridge window) parses the same frame and shows
the identical hex under "Parsed frame (hex)" plus the decoded lat/lon/alt, so
you can confirm both ends agree byte-for-byte.

Frame layout (matches RadarProtocolParser), fixed 26 bytes:
    SOF(FD) | VERSION(10) | SEQ | 01 | 01 | MSG_ID(C5 CE C2) |
    time_ms(u32) lat_e7(i32) lon_e7(i32) alt_mm(i32) | CRC16(LE)
CRC = MAVLink X25 / CRC16_MCRF4XX over bytes [1..23] (everything except SOF and CRC).

Usage:
    # Interactive: keeps prompting for lat lon alt
    python3 radar_manual_sender.py --port /dev/pts/3 --baud 115200

    # One-shot from the command line
    python3 radar_manual_sender.py --port /dev/pts/3 --lat 34.2665 --lon 108.9544 --alt -200

    # Just print the hex, don't open a serial port
    python3 radar_manual_sender.py --dry-run --lat 47.1234567 --lon 8.7654321 --alt 123.45

    # Send a raw hex frame verbatim (the ground station parses it to lat/lon/alt)
    python3 radar_manual_sender.py --port /dev/pts/3 --hex "FD 10 38 01 01 C5 CE C2 19 00 00 00 28 A7 6C 14 40 1F F1 40 C0 F2 FC FF 51 3C"

In interactive mode you can type EITHER three numbers (lat lon alt) OR a raw
hex frame (e.g. FD 10 00 01 ...) on each line; the script auto-detects which.
"""

import argparse
import struct
import time

SOF = 0xFD
VERSION = 0x10           # byte[1], fixed (0x10)
FIXED3 = 0x01            # byte[3], fixed
FIXED4 = 0x01            # byte[4], fixed
MSG_ID_POSITION = b"\xC5\xCE\xC2"   # bytes[5..7], POSITION identifier


def crc_calculate(data: bytes) -> int:
    """MAVLink X25 / CRC16_MCRF4XX."""
    crc = 0xFFFF
    for b in data:
        tmp = b ^ (crc & 0xFF)
        tmp ^= (tmp << 4) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


def build_position(seq: int, t_ms: int, lat_deg: float, lon_deg: float, alt_m: float) -> bytes:
    # Fixed 26-byte frame:
    #   SOF | VERSION | SEQ | 01 | 01 | MSG_ID(C5 CE C2) | payload(16) | CRC(2)
    # payload(16): time_ms(u32) lat_e7(i32) lon_e7(i32) alt_mm(i32), little-endian.
    payload = struct.pack("<I i i i",
                          t_ms & 0xFFFFFFFF,
                          int(round(lat_deg * 1e7)),
                          int(round(lon_deg * 1e7)),
                          int(round(alt_m * 1000)))
    # CRC covers bytes [1..23]: VERSION SEQ 01 01 MSG_ID(3) payload(16) = 23 bytes.
    crc_region = bytes([VERSION, seq & 0xFF, FIXED3, FIXED4]) + MSG_ID_POSITION + payload
    crc = crc_calculate(crc_region)
    return bytes([SOF]) + crc_region + struct.pack("<H", crc)


def hex_upper(frame: bytes) -> str:
    # Same formatting as QGC's "Parsed frame (hex)": upper-case, space-separated.
    return frame.hex(" ").upper()


_HEXDIGITS = set("0123456789abcdefABCDEF")


def looks_like_hex(line: str) -> bool:
    """True if the line looks like a raw hex frame rather than 'lat lon alt'."""
    compact = line.replace(",", " ").replace("0x", " ").replace("0X", " ")
    tokens = compact.split()
    if not tokens:
        return False
    # Case 1: several space-separated 1-2 digit hex bytes (a frame is ~26 bytes).
    if len(tokens) >= 4 and all(len(t) <= 2 and all(c in _HEXDIGITS for c in t) for t in tokens):
        return True
    # Case 2: one long contiguous hex string, even length.
    if len(tokens) == 1:
        t = tokens[0]
        if len(t) >= 8 and len(t) % 2 == 0 and all(c in _HEXDIGITS for c in t):
            return True
    return False


def hex_to_bytes(line: str) -> bytes:
    """Parse a hex string (spaces/commas/0x prefixes allowed) into raw bytes."""
    compact = line.replace(",", " ").replace("0x", " ").replace("0X", " ").split()
    if len(compact) == 1:  # one contiguous string like FD1000...
        s = compact[0]
        return bytes(int(s[i:i + 2], 16) for i in range(0, len(s), 2))
    return bytes(int(tok, 16) for tok in compact)


def send_raw(ser, frame: bytes):
    """Send pre-built raw bytes verbatim (no CRC recompute)."""
    if ser is not None:
        ser.write(frame)
        ser.flush()
    else:
        print(hex_upper(frame))


def send_one(ser, seq, lat, lon, alt):
    t_ms = int(time.time() * 1000) & 0xFFFFFFFF
    frame = build_position(seq, t_ms, lat, lon, alt)
    # Confirmation line for what was sent.
    print(f"lat={lat:.7f} lon={lon:.7f} alt={alt:.3f}  (SEQ={seq})")
    if ser is not None:
        ser.write(frame)
        ser.flush()
    else:
        # Dry-run: also show the raw hex for inspection.
        print(hex_upper(frame))
    return frame


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", help="serial port, e.g. /dev/pts/3, /dev/ttyUSB0, COM3")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--lat", type=float, help="latitude in degrees")
    ap.add_argument("--lon", type=float, help="longitude in degrees")
    ap.add_argument("--alt", type=float, help="altitude in metres")
    ap.add_argument("--hex", dest="hexstr",
                    help="send this raw hex frame verbatim, e.g. \"FD 10 00 01 00 ...\"")
    ap.add_argument("--dry-run", action="store_true", help="print hex, do not open serial")
    args = ap.parse_args()

    ser = None
    if not args.dry_run:
        import serial  # pip install pyserial
        ser = serial.Serial(args.port, args.baud, timeout=0)

    seq = 0

    # One-shot: raw hex frame from the command line.
    if args.hexstr is not None:
        send_raw(ser, hex_to_bytes(args.hexstr))
        return

    # One-shot: lat/lon/alt from the command line.
    if args.lat is not None and args.lon is not None and args.alt is not None:
        send_one(ser, seq, args.lat, args.lon, args.alt)
        return

    # Interactive mode. Two accepted input forms per line:
    #   1) three numbers  -> "lat lon alt"  (built into a POSITION frame)
    #   2) a raw hex frame -> "FD 10 00 01 ..." (sent verbatim)
    print("Interactive mode. Enter either:")
    print("  - three numbers:  lat lon alt      (e.g. 34.2665 108.9544 -200)")
    print("  - a raw hex frame: FD 10 00 01 ...  (sent as-is)")
    print("Type 'q' to quit.")
    while True:
        try:
            s = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if s.lower() in ("q", "quit", "exit"):
            break
        if not s:
            continue

        # Form 2: raw hex frame -> send verbatim.
        if looks_like_hex(s):
            try:
                frame = hex_to_bytes(s)
            except ValueError:
                print("  invalid hex, try again")
                continue
            send_raw(ser, frame)
            continue

        # Form 1: lat lon alt -> build a POSITION frame.
        parts = s.replace(",", " ").split()
        if len(parts) != 3:
            print("  enter 3 numbers (lat lon alt) or a hex frame")
            continue
        try:
            lat, lon, alt = (float(parts[0]), float(parts[1]), float(parts[2]))
        except ValueError:
            print("  invalid number(s), try again")
            continue
        send_one(ser, seq, lat, lon, alt)
        seq = (seq + 1) & 0xFF


if __name__ == "__main__":
    main()
