/****************************************************************************
 *
 * RadarProtocolParser implementation.
 *
 ****************************************************************************/

#include "RadarProtocolParser.h"
#include "Crc16.h"

using namespace RadarProtocol;

RadarProtocolParser::RadarProtocolParser(QObject *parent)
    : QObject(parent)
{
}

void RadarProtocolParser::reset()
{
    _buffer.clear();
    _lastSeq = -1;
}

void RadarProtocolParser::addData(const QByteArray &data)
{
    if (data.isEmpty()) {
        return;
    }

    _stats.receivedBytes += static_cast<quint64>(data.size());
    _buffer.append(data);

    // Guard against unbounded growth if we somehow never find a valid frame.
    const int maxBuffer = 8 * kFrameLen;
    if (_buffer.size() > maxBuffer) {
        _buffer.remove(0, _buffer.size() - maxBuffer);
    }

    _processBuffer();
    emit statisticsChanged();
}

void RadarProtocolParser::_processBuffer()
{
    // Fixed 26-byte frame:
    //   [0]SOF [1]VER [2]SEQ [3]01 [4]01 [5..7]MSG_ID(C5 CE C2)
    //   [8..11]time [12..15]lat [16..19]lon [20..23]alt [24..25]CRC(LE)
    while (true) {
        const int available = _buffer.size();
        if (available < 1) {
            return;
        }

        const auto *buf = reinterpret_cast<const uint8_t *>(_buffer.constData());

        // --- Resynchronise on SOF ---------------------------------------
        if (buf[kOffSof] != kSOF) {
            int sofIndex = -1;
            for (int i = 1; i < available; ++i) {
                if (buf[i] == kSOF) { sofIndex = i; break; }
            }
            _stats.badFrames++;
            if (sofIndex < 0) {
                _buffer.clear();
                return;
            }
            _buffer.remove(0, sofIndex);
            continue; // re-evaluate from the new SOF
        }

        if (available < kFrameLen) {
            return; // full frame not here yet -> wait (half packet)
        }

        // --- CRC check (MAVLink X25 / CRC16_MCRF4XX) --------------------
        // CRC covers bytes [1..23] (everything except SOF and the 2 CRC bytes).
        const int crcRegionLen = kOffCrc - 1;                // 24 - 1 = 23
        const uint16_t calcCrc = Crc16::calculate(buf + 1, static_cast<size_t>(crcRegionLen));
        const uint16_t frameCrc =
            static_cast<uint16_t>(buf[kOffCrc]) |
            (static_cast<uint16_t>(buf[kOffCrc + 1]) << 8);

        if (calcCrc != frameCrc) {
            // CRC mismatch: drop the SOF byte, resync, count the error.
            _stats.crcErrors++;
            _buffer.remove(0, 1);
            continue;
        }

        // --- Valid frame -------------------------------------------------
        const uint8_t seq = buf[kOffSeq];
        _stats.parsedFrames++;
        _updateSeq(seq);

        // Only POSITION (MSG_ID C5 CE C2) is decoded; others are consumed but
        // not emitted (extend here if more 26-byte message ids appear).
        const bool isPosition =
            (buf[kOffMsgId]     == kMsgIdPosition[0]) &&
            (buf[kOffMsgId + 1] == kMsgIdPosition[1]) &&
            (buf[kOffMsgId + 2] == kMsgIdPosition[2]);

        if (isPosition) {
            const QByteArray payload  = _buffer.mid(kOffPayload, kPayloadLen);
            const QByteArray rawFrame = _buffer.left(kFrameLen);
            emit frameParsed(seq, payload, rawFrame);
        }

        // Consume the whole frame and continue (handles sticky packets).
        _buffer.remove(0, kFrameLen);
    }
}

void RadarProtocolParser::_updateSeq(quint8 seq)
{
    if (_lastSeq >= 0) {
        const quint8 expected = static_cast<quint8>((_lastSeq + 1) & 0xFF);
        if (seq != expected) {
            // Number of packets skipped (modulo 256). This is a heuristic that
            // works for forward gaps; large gaps or reordering are approximated.
            const int gap = (static_cast<int>(seq) - expected + 256) & 0xFF;
            _stats.seqDrops += static_cast<quint64>(gap);
        }
    }
    _lastSeq = seq;
}
