/****************************************************************************
 *
 * MAVLink X25 CRC / CRC16_MCRF4XX implementation for the RadarBridge protocol.
 *
 * This is the same checksum used by MAVLink itself (crc_accumulate /
 * crc_calculate in checksum.h):
 *
 *   Init : 0xFFFF (X25_INIT_CRC)
 *   Byte accumulation:
 *       tmp   = data ^ (crc & 0xFF)
 *       tmp  ^= (tmp << 4)
 *       crc   = (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)
 *
 * NOTE:
 *   - Radar protocol CRC: used to validate the custom 0xFD protocol frames
 *     (this class).
 *   - MAVLink CRC: used to validate MAVLink message frames themselves and is
 *     handled automatically by the MAVLink library.
 *
 ****************************************************************************/

#pragma once

#include <QtCore/QByteArray>
#include <cstddef>
#include <cstdint>

namespace RadarProtocol {

/// MAVLink X25 / CRC16_MCRF4XX checksum.
class Crc16
{
public:
    static constexpr uint16_t kInitCrc     = 0xFFFF; ///< X25_INIT_CRC
    static constexpr uint16_t kValidateCrc = 0xF0B8; ///< X25_VALIDATE_CRC

    /// Accumulate a single byte into @a crcAccum (mirrors crc_accumulate()).
    static inline void accumulate(uint8_t data, uint16_t &crcAccum)
    {
        uint8_t tmp = data ^ static_cast<uint8_t>(crcAccum & 0xFF);
        tmp ^= (tmp << 4);
        crcAccum = static_cast<uint16_t>((crcAccum >> 8)
                                         ^ (static_cast<uint16_t>(tmp) << 8)
                                         ^ (static_cast<uint16_t>(tmp) << 3)
                                         ^ (static_cast<uint16_t>(tmp) >> 4));
    }

    /// Calculate the checksum over @a length bytes at @a data.
    static uint16_t calculate(const uint8_t *data, size_t length);

    /// Calculate the checksum over @a data (Qt convenience overload).
    static uint16_t calculate(const QByteArray &data);
};

} // namespace RadarProtocol
