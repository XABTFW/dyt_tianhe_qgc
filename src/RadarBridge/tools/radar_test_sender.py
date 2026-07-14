#!/usr/bin/env python3
"""
Radar Bridge protocol test-frame generator / sender.

Builds valid 0xFD frames (MAVLink X25 CRC / CRC16_MCRF4XX) for POSITION (0x10) and
RADAR_SCAN_2D (0x20) and streams them to a serial port so you can exercise
the QGC RadarBridge module end-to-end without real hardware.

Usage:
    # Send to a real/virtual serial port at 10 Hz:
    python3 radar_test_sender.py --port /dev/ttyUSB0 --baud 115200 --hz 10

    # Just print hex frames (no serial):
    python3 radar_test_sender.py --dry-run

Tip: create a virtual serial pair with socat for loopback testing:
    socat -d -d pty,raw,echo=0 pty,raw,echo=0
    # then point QGC at one end and this script at the other.
"""

import argparse
import struct
import time
import math

SOF = 0xFD
VERSION = 0x10                       # byte[1], fixed (0x10)
FIXED3 = 0x01                        # byte[3], fixed
FIXED4 = 0x01                        # byte[4], fixed
MSG_ID_POSITION = b"\xC5\xCE\xC2"    # bytes[5..7], POSITION identifier


X25_INIT_CRC = 0xFFFF


def crc_accumulate(data: int, crc: int) -> int:
    """MAVLink crc_accumulate(): X25 / CRC16_MCRF4XX single-byte step."""
    tmp = data ^ (crc & 0xFF)
    tmp ^= (tmp << 4) & 0xFF
    crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


def crc_calculate(data: bytes) -> int:
    """MAVLink X25 CRC / CRC16_MCRF4XX over a byte buffer."""
    crc = X25_INIT_CRC
    for b in data:
        crc = crc_accumulate(b, crc)
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
    # CRC covers bytes [1..23]: VERSION SEQ 01 01 MSG_ID(3) payload(16).
    crc_region = bytes([VERSION, seq & 0xFF, FIXED3, FIXED4]) + MSG_ID_POSITION + payload
    crc = crc_calculate(crc_region)          # MAVLink X25 / CRC16_MCRF4XX
    return bytes([SOF]) + crc_region + struct.pack("<H", crc)


def meters_to_lat_lon(base_lat_deg: float, base_lon_deg: float, north_m: float, east_m: float):
    # Good enough for short test tracks: convert local N/E metres to WGS84 deg.
    metres_per_deg_lat = 111_320.0
    metres_per_deg_lon = metres_per_deg_lat * math.cos(math.radians(base_lat_deg))
    return (base_lat_deg + north_m / metres_per_deg_lat,
            base_lon_deg + east_m / metres_per_deg_lon)


def square_track(t_s: float, base_lat_deg: float, base_lon_deg: float, base_alt_m: float,
                 side_m: float, speed_m_s: float, altitude_step_m: float):
    side_m = max(1.0, side_m)
    speed_m_s = max(0.1, speed_m_s)
    leg_s = side_m / speed_m_s
    cycle_s = 4.0 * leg_s
    cycle_index = int(t_s / cycle_s)
    cycle_t = t_s - cycle_index * cycle_s
    leg_index = min(3, int(cycle_t / leg_s))
    leg_t = cycle_t - leg_index * leg_s
    u = max(0.0, min(1.0, leg_t / leg_s))

    half = side_m * 0.5
    corners = [
        (-half, -half),
        ( half, -half),
        ( half,  half),
        (-half,  half),
    ]
    start_n, start_e = corners[leg_index]
    end_n, end_e = corners[(leg_index + 1) % 4]
    north_m = start_n + (end_n - start_n) * u
    east_m = start_e + (end_e - start_e) * u

    altitude_phase = cycle_index % 3
    if altitude_phase == 0:
        alt_m = base_alt_m
    elif altitude_phase == 1:
        alt_m = base_alt_m + altitude_step_m
    else:
        alt_m = base_alt_m

    lat, lon = meters_to_lat_lon(base_lat_deg, base_lon_deg, north_m, east_m)
    return lat, lon, alt_m, cycle_index, leg_index


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", help="serial port, e.g. /dev/ttyUSB0 or COM3")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--hz", type=float, default=10.0)
    ap.add_argument("--dry-run", action="store_true", help="print frames instead of sending")
    ap.add_argument("--fixed", action="store_true",
                    help="send constant, known values so you can compare them directly "
                         "with what QGC displays (proves the parser is correct)")
    ap.add_argument("--corrupt", type=int, default=0, metavar="N",
                    help="corrupt 1 byte of every Nth frame to prove CRC rejection "
                         "(CRC Errors should climb, no bad data should appear). 0=off")
    ap.add_argument("--base-lat", type=float, default=47.397972,
                    help="square-track centre latitude in degrees")
    ap.add_argument("--base-lon", type=float, default=8.546164,
                    help="square-track centre longitude in degrees")
    ap.add_argument("--base-alt", type=float, default=3.0,
                    help="square-track base altitude in metres")
    ap.add_argument("--side-m", type=float, default=40.0,
                    help="square-track side length in metres")
    ap.add_argument("--speed", type=float, default=5.0,
                    help="target speed along square edges in m/s")
    ap.add_argument("--alt-step", type=float, default=10.0,
                    help="altitude change after each square lap in metres")
    ap.add_argument("--print-every", type=int, default=10,
                    help="print one decoded target line every N frames; 0 disables")
    args = ap.parse_args()

    # Fixed, easy-to-verify constants used by --fixed mode.
    FIXED_LAT = 47.1234567
    FIXED_LON = 8.7654321
    FIXED_ALT = 123.450          # metres  -> alt_mm = 123450
    FIXED_DISTS = [101, 202, 303, 404, 505]  # point_count=5, min=101, max=505

    if args.fixed:
        print("=== FIXED mode: QGC should show EXACTLY these values ===")
        print("Last POSITION:")
        print(f"  Latitude  = {FIXED_LAT:.7f}")
        print(f"  Longitude = {FIXED_LON:.7f}")
        print(f"  Altitude  = {FIXED_ALT:.2f}")
        print("Last RADAR_SCAN_2D:")
        print(f"  point_count     = {len(FIXED_DISTS)}")
        print(f"  min_distance_cm = {min(FIXED_DISTS)}")
        print(f"  max_distance_cm = {max(FIXED_DISTS)}")
        print("(time_ms will keep increasing; everything else stays constant)")
        print("========================================================")

    ser = None
    if not args.dry_run:
        import serial  # pip install pyserial
        ser = serial.Serial(args.port, args.baud, timeout=0)

    seq = 0
    frame_no = 0
    t0 = time.time()
    period = 1.0 / args.hz

    def maybe_corrupt(frame: bytes) -> bytes:
        # Flip one payload byte so the CRC no longer matches. The parser must
        # reject it (crcErrors++), so it must NOT change QGC's Last* values.
        nonlocal frame_no
        frame_no += 1
        if args.corrupt > 0 and (frame_no % args.corrupt == 0):
            b = bytearray(frame)
            # Byte index 8 is inside the payload for both message types.
            idx = 8 if len(b) > 10 else len(b) // 2
            b[idx] ^= 0xFF
            print(f"[corrupt] frame #{frame_no}: flipped byte {idx} (should be dropped by CRC)")
            return bytes(b)
        return frame

    while True:
        t_ms = int((time.time() - t0) * 1000)

        if args.fixed:
            lat, lon, alt = FIXED_LAT, FIXED_LON, FIXED_ALT
            dists = FIXED_DISTS
        else:
            # Fly a square in WGS84. After each square lap, alternate between
            # base altitude and base+alt_step so the target climbs 10 m for one
            # square, descends 10 m for the next, and repeats.
            lat, lon, alt, cycle_index, leg_index = square_track(
                t_ms / 1000.0,
                args.base_lat,
                args.base_lon,
                args.base_alt,
                args.side_m,
                args.speed,
                args.alt_step)
            # A 40-point scan with a sweeping near obstacle.
            dists = [800 + int(300 * math.sin((i + t_ms / 200.0) / 5.0)) for i in range(40)]

        # Only POSITION (0x10) is sent: its real on-wire format is confirmed and
        # the QGC parser frames it. RADAR_SCAN_2D's real layout is not confirmed
        # yet, so we do not emit it (it would show up as Bad Frames).
        _ = dists  # reserved for when SCAN is enabled
        pos = build_position(seq, t_ms, lat, lon, alt)
        seq = (seq + 1) & 0xFF

        pos = maybe_corrupt(pos)

        if args.dry_run:
            print("POSITION ", pos.hex())
        else:
            ser.write(pos)

        if args.print_every > 0 and frame_no % args.print_every == 0:
            print(f"target lat={lat:.7f} lon={lon:.7f} alt={alt:.1f}m "
                  f"lap={locals().get('cycle_index', 0)} leg={locals().get('leg_index', 0)}")

        time.sleep(period)


if __name__ == "__main__":
    main()
