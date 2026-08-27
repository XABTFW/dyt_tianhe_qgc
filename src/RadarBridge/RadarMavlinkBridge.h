/****************************************************************************
 *
 * RadarMavlinkBridge
 *
 * Converts decoded radar payloads into MAVLink messages and sends them to
 * PX4 over QGC's *existing* vehicle data link. It never opens its own
 * connection to the flight controller: it reuses the active Vehicle and its
 * primary LinkInterface, exactly like RTCMMavlink / MissionManager do.
 *
 *   POSITION       -> UAV_INFO         (radar target for cooperative_rendezvous)
 *   RADAR_SCAN_2D  -> OBSTACLE_DISTANCE (72-point array, downsample/pad as needed)
 *
 ****************************************************************************/

#pragma once

#include "RadarTypes.h"

#include <QtCore/QObject>

class Vehicle;

class RadarMavlinkBridge : public QObject
{
    Q_OBJECT

public:
    explicit RadarMavlinkBridge(QObject *parent = nullptr);

    /// Send a POSITION as a MAVLink UAV_INFO radar-target message to the active vehicle.
    /// @return true if a message was actually sent.
    bool sendPositionAsUavInfo(const RadarProtocol::PositionData &pos, quint32 targetId);

    /// Optional: also send the position as GLOBAL_POSITION_INT so it shows up
    /// on the QGC map. NOTE: GLOBAL_POSITION_INT conventionally represents the
    /// *vehicle's own* fused position, so PX4/QGC may treat it as the drone's
    /// location rather than an external target. Use with care; GPS_INPUT is the
    /// correct channel for feeding an external position source into PX4.
    bool sendPositionAsGlobalPositionInt(const RadarProtocol::PositionData &pos);

    /// Send a RADAR_SCAN_2D as a MAVLink OBSTACLE_DISTANCE message.
    /// @return true if a message was actually sent.
    bool sendRadarScanAsObstacleDistance(const RadarProtocol::RadarScan2DData &scan);

signals:
    /// Emitted after each successfully transmitted MAVLink message.
    void mavlinkMessageSent();

private:
    // Resolve the active vehicle + its primary link. Returns nullptr if none.
    Vehicle *_activeVehicle() const;
};
