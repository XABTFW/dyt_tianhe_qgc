/****************************************************************************
 *
 * MAVLink X25 CRC / CRC16_MCRF4XX implementation for the RadarBridge protocol.
 *
 ****************************************************************************/

#include "Crc16.h"

namespace RadarProtocol {

uint16_t Crc16::calculate(const uint8_t *data, size_t length)
{
    // Equivalent to MAVLink's crc_calculate(): init to 0xFFFF then accumulate
    // every byte with the X25 / MCRF4XX byte mixing.
    uint16_t crc = kInitCrc;
    for (size_t i = 0; i < length; ++i) {
        accumulate(data[i], crc);
    }
    return crc;
}

uint16_t Crc16::calculate(const QByteArray &data)
{
    return calculate(reinterpret_cast<const uint8_t *>(data.constData()),
                     static_cast<size_t>(data.size()));
}

} // namespace RadarProtocol
