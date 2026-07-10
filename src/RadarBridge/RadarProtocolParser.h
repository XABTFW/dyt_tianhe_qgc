/****************************************************************************
 *
 * RadarProtocolParser
 *
 * Incremental state-machine parser for the custom 0xFD binary protocol.
 * It is fed raw bytes as they arrive from the serial port and emits fully
 * validated frames. It transparently handles:
 *   - Packet fragmentation (half packets): incomplete data is buffered.
 *   - Packet coalescing (sticky packets): multiple frames in one read.
 *   - Framing errors / garbage: resynchronises on the next SOF.
 *   - CRC errors: frame dropped, counter incremented.
 *   - Bad PAYLOAD_LEN: rejected and resynchronised.
 *   - Sequence gap detection: counts dropped packets.
 *
 ****************************************************************************/

#pragma once

#include "RadarTypes.h"

#include <QtCore/QByteArray>
#include <QtCore/QObject>

class RadarProtocolParser : public QObject
{
    Q_OBJECT

public:
    explicit RadarProtocolParser(QObject *parent = nullptr);

    /// Feed newly received bytes into the parser. Emits frameParsed() for
    /// every complete, CRC-valid frame found.
    void addData(const QByteArray &data);

    /// Drop any buffered partial data and reset the sequence tracker.
    void reset();

    const RadarProtocol::Statistics &statistics() const { return _stats; }
    void resetStatistics() { _stats.reset(); }

signals:
    /// Emitted once per valid POSITION frame.
    ///   @a seq      the frame sequence number
    ///   @a payload  the 16-byte payload (time_ms, lat_e7, lon_e7, alt_mm)
    ///   @a rawFrame the complete frame as received (SOF..CRC), for hex display
    void frameParsed(quint8 seq, const QByteArray &payload, const QByteArray &rawFrame);

    /// Emitted whenever any statistic changes so the UI can refresh.
    void statisticsChanged();

private:
    // Attempt to extract as many complete frames as the buffer currently holds.
    void _processBuffer();
    // Track SEQ continuity and update the drop counter.
    void _updateSeq(quint8 seq);

    QByteArray                _buffer;         ///< accumulation buffer
    RadarProtocol::Statistics _stats;          ///< running statistics
    int                       _lastSeq = -1;   ///< last accepted SEQ (-1 = none yet)
};
