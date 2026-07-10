/****************************************************************************
 *
 * Radar Bridge module for QGroundControl
 *
 * Common types shared across the RadarBridge module.
 *
 * Real POSITION frame (fixed 26 bytes, all multi-byte ints little-endian):
 *
 *   [0]     SOF        = 0xFD (fixed)
 *   [1]     VERSION    (fixed, e.g. 0x01) - not strictly enforced; CRC validates
 *   [2]     SEQ        rolling 0..255
 *   [3]     fixed byte (0x01)
 *   [4]     fixed byte (0x01)
 *   [5..7]  MSG_ID     3-byte message identifier (POSITION = C5 CE C2)
 *   [8..11] time_ms    uint32  (ms)
 *   [12..15]lat_e7     int32   (deg * 1e7)
 *   [16..19]lon_e7     int32   (deg * 1e7)
 *   [20..23]alt_mm     int32   (m * 1000)
 *   [24..25]CRC16      MAVLink X25 / CRC16_MCRF4XX, little-endian,
 *                      computed over bytes [1..23] (everything except SOF and CRC)
 *
 ****************************************************************************/

#pragma once

#include <QtCore/QByteArray>
#include <QtCore/QMetaType>
#include <QtCore/QVector>
#include <cstdint>
#include <cstring>

namespace RadarProtocol {

// ---- Framing constants ----------------------------------------------------
constexpr uint8_t kSOF        = 0xFD;   ///< Start-of-frame marker
constexpr uint8_t kVersion    = 0x10;   ///< VERSION byte value (informational; CRC validates)

constexpr int kFrameLen = 26;           ///< fixed total frame length
constexpr int kCrcLen   = 2;            ///< CRC16 length

// Byte offsets within the frame.
constexpr int kOffSof     = 0;
constexpr int kOffVersion = 1;
constexpr int kOffSeq     = 2;
constexpr int kOffFixed1  = 3;
constexpr int kOffFixed2  = 4;
constexpr int kOffMsgId   = 5;   ///< 3-byte message identifier
constexpr int kMsgIdLen   = 3;
constexpr int kOffPayload = 8;   ///< 16-byte payload starts here
constexpr int kPayloadLen = 16;  ///< time(4)+lat(4)+lon(4)+alt(4)
constexpr int kOffCrc     = 24;  ///< CRC16 (LE) at [24..25]

// The 3-byte identifier for the POSITION message (C5 CE C2).
constexpr uint8_t kMsgIdPosition[kMsgIdLen] = { 0xC5, 0xCE, 0xC2 };

// Payload field offsets (relative to the start of the 16-byte payload).
constexpr int kPayOffTimeMs = 0;
constexpr int kPayOffLat    = 4;
constexpr int kPayOffLon    = 8;
constexpr int kPayOffAlt    = 12;

// ---------------------------------------------------------------------------
// Decoded POSITION payload.
// ---------------------------------------------------------------------------
struct PositionData {
    uint32_t timeMs = 0;
    int32_t  latE7  = 0;   ///< degrees * 1e7
    int32_t  lonE7  = 0;   ///< degrees * 1e7
    int32_t  altMm  = 0;   ///< millimetres
    bool     valid  = false;

    double latDeg() const { return latE7 / 1e7; }
    double lonDeg() const { return lonE7 / 1e7; }
    double altM()   const { return altMm / 1000.0; }
};

// ---------------------------------------------------------------------------
// RADAR_SCAN_2D (kept for future use; not produced by the current parser as the
// real wire format for this message has not been provided).
// ---------------------------------------------------------------------------
struct RadarScan2DData {
    uint32_t          timeMs        = 0;
    int16_t           angleMinCd    = 0;
    uint16_t          angleIncCd    = 0;
    uint16_t          minDistanceCm = 0;
    uint16_t          maxDistanceCm = 0;
    uint8_t           pointCount    = 0;
    uint8_t           reserved      = 0;
    QVector<uint16_t> distancesCm;
    bool              valid         = false;
};

// ---------------------------------------------------------------------------
// Statistics exposed to the QML settings/monitor page.
// ---------------------------------------------------------------------------
struct Statistics {
    quint64 receivedBytes    = 0;   ///< total bytes read from the serial port
    quint64 parsedFrames     = 0;   ///< frames that passed CRC and framing
    quint64 crcErrors        = 0;   ///< frames dropped due to CRC mismatch
    quint64 badFrames        = 0;   ///< resync / framing errors
    quint64 seqDrops         = 0;   ///< missing sequence numbers detected
    quint64 mavlinkSentCount = 0;   ///< MAVLink messages sent to the vehicle

    void reset() { *this = Statistics(); }
};

// ---------------------------------------------------------------------------
// Little-endian reader helper.
// ---------------------------------------------------------------------------
class LeReader {
public:
    LeReader(const uint8_t *data, int len) : _data(data), _len(len) {}

    bool ok() const { return _ok; }
    void seek(int pos) { _pos = pos; }

    uint8_t u8() {
        if (_pos + 1 > _len) { _ok = false; return 0; }
        return _data[_pos++];
    }
    int16_t i16()  { return static_cast<int16_t>(u16()); }
    uint16_t u16() {
        if (_pos + 2 > _len) { _ok = false; return 0; }
        uint16_t v = static_cast<uint16_t>(_data[_pos]) |
                     (static_cast<uint16_t>(_data[_pos + 1]) << 8);
        _pos += 2;
        return v;
    }
    int32_t i32()  { return static_cast<int32_t>(u32()); }
    uint32_t u32() {
        if (_pos + 4 > _len) { _ok = false; return 0; }
        uint32_t v = static_cast<uint32_t>(_data[_pos]) |
                     (static_cast<uint32_t>(_data[_pos + 1]) << 8) |
                     (static_cast<uint32_t>(_data[_pos + 2]) << 16) |
                     (static_cast<uint32_t>(_data[_pos + 3]) << 24);
        _pos += 4;
        return v;
    }

private:
    const uint8_t *_data = nullptr;
    int _len = 0;
    int _pos = 0;
    bool _ok = true;
};

} // namespace RadarProtocol

Q_DECLARE_METATYPE(RadarProtocol::PositionData)
Q_DECLARE_METATYPE(RadarProtocol::RadarScan2DData)
