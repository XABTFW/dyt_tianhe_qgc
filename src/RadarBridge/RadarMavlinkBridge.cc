/****************************************************************************
 *
 * RadarMavlinkBridge implementation.
 *
 ****************************************************************************/

#include "RadarMavlinkBridge.h"
#include "MAVLinkProtocol.h"        // pulls in MAVLinkLib.h (mavlink message packers)
#include "MultiVehicleManager.h"
#include "Vehicle.h"
#include "VehicleLinkManager.h"
#include "LinkInterface.h"

#include <algorithm>
#include <cmath>

RadarMavlinkBridge::RadarMavlinkBridge(QObject *parent)
    : QObject(parent)
{
}

Vehicle *RadarMavlinkBridge::_activeVehicle() const
{
    // Reuse QGC's current active vehicle. If there is no connected vehicle we
    // simply drop the sample (nothing to send to).
    return MultiVehicleManager::instance()->activeVehicle();
}

bool RadarMavlinkBridge::sendPositionAsUavInfo(const RadarProtocol::PositionData &pos, quint32 targetId)
{
    const double lat = pos.latDeg();
    const double lon = pos.lonDeg();
    const double alt = pos.altM();

    if (!pos.valid || targetId == 0 || !std::isfinite(lat) || !std::isfinite(lon) || !std::isfinite(alt)
        || lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0 || alt < 0.0) {
        return false;
    }

    Vehicle *const vehicle = _activeVehicle();
    if (!vehicle) {
        return false;
    }

    // Grab the vehicle's primary link (the real data link to PX4).
    const SharedLinkInterfacePtr sharedLink = vehicle->vehicleLinkManager()->primaryLink().lock();
    if (!sharedLink) {
        return false;
    }

    // is_leader=0 makes the custom PX4 receiver publish this sample as
    // follower_info. The 26-byte radar protocol has no velocity or yaw fields.
    mavlink_message_t msg;
    (void) mavlink_msg_uav_info_pack_chan(
        MAVLinkProtocol::instance()->getSystemId(),
        MAVLinkProtocol::getComponentId(),
        sharedLink->mavlinkChannel(),
        &msg,
        targetId,
        0, // group_id
        0, // is_leader
        static_cast<float>(lat),
        static_cast<float>(lon),
        0.0f, // yaw unknown
        0.0f, // yaw speed unknown
        static_cast<float>(alt), // radar height AGL
        0.0f, // vx unknown
        0.0f, // vy unknown
        0.0f, // vz unknown
        0);   // land/at_target flags

    if (!vehicle->sendMessageOnLinkThreadSafe(sharedLink.get(), msg)) {
        return false;
    }

    emit mavlinkMessageSent();
    return true;
}

bool RadarMavlinkBridge::sendPositionAsGlobalPositionInt(const RadarProtocol::PositionData &pos)
{
    if (!pos.valid) {
        return false;
    }

    Vehicle *const vehicle = _activeVehicle();
    if (!vehicle) {
        return false;
    }

    const SharedLinkInterfacePtr sharedLink = vehicle->vehicleLinkManager()->primaryLink().lock();
    if (!sharedLink) {
        return false;
    }

    mavlink_global_position_int_t gpi{};
    gpi.time_boot_ms = pos.timeMs;
    gpi.lat          = pos.latE7;            // degE7
    gpi.lon          = pos.lonE7;            // degE7
    gpi.alt          = pos.altMm;            // mm (MSL)
    gpi.relative_alt = pos.altMm;            // mm (approximation)
    gpi.hdg          = UINT16_MAX;           // unknown heading

    mavlink_message_t msg;
    (void) mavlink_msg_global_position_int_encode_chan(
        MAVLinkProtocol::instance()->getSystemId(),
        MAVLinkProtocol::getComponentId(),
        sharedLink->mavlinkChannel(),
        &msg,
        &gpi);

    if (!vehicle->sendMessageOnLinkThreadSafe(sharedLink.get(), msg)) {
        return false;
    }

    emit mavlinkMessageSent();
    return true;
}

bool RadarMavlinkBridge::sendRadarScanAsObstacleDistance(const RadarProtocol::RadarScan2DData &scan)
{
    if (!scan.valid) {
        return false;
    }

    Vehicle *const vehicle = _activeVehicle();
    if (!vehicle) {
        return false;
    }

    const SharedLinkInterfacePtr sharedLink = vehicle->vehicleLinkManager()->primaryLink().lock();
    if (!sharedLink) {
        return false;
    }

    // OBSTACLE_DISTANCE carries a fixed 72-element array.
    constexpr int kBins = 72; // MAVLINK_MSG_OBSTACLE_DISTANCE_FIELD_DISTANCES_LEN

    mavlink_obstacle_distance_t od{};
    od.time_usec    = static_cast<uint64_t>(QDateTime::currentMSecsSinceEpoch()) * 1000ULL;
    od.sensor_type  = MAV_DISTANCE_SENSOR_RADAR;
    od.min_distance = scan.minDistanceCm;
    od.max_distance = scan.maxDistanceCm;

    // Angular width per bin (deg). angle_inc_cd is in 0.01 deg units.
    od.increment_f  = static_cast<float>(scan.angleIncCd) * 0.01f;
    od.increment    = static_cast<uint8_t>(std::lround(od.increment_f));
    // Offset of the index-0 element (deg). angle_min_cd is in 0.01 deg.
    od.angle_offset = static_cast<float>(scan.angleMinCd) * 0.01f;
    od.frame        = MAV_FRAME_BODY_FRD; // body-front aligned; adjust if needed

    // Initialise all bins to "unknown/not used".
    for (int i = 0; i < kBins; ++i) {
        od.distances[i] = UINT16_MAX;
    }

    const int n = scan.distancesCm.size();
    if (n <= 0) {
        // Nothing to place, but still a valid (empty) scan.
    } else if (n <= kBins) {
        // Fewer than 72 points: copy directly, remaining bins stay UINT16_MAX.
        for (int i = 0; i < n; ++i) {
            od.distances[i] = scan.distancesCm[i];
        }
    } else {
        // More than 72 points: downsample. For each target bin pick the minimum
        // distance across the source points mapped to it (conservative for
        // obstacle avoidance -- keeps the nearest obstacle).
        for (int b = 0; b < kBins; ++b) {
            const int start = static_cast<int>(static_cast<int64_t>(b)     * n / kBins);
            int       end   = static_cast<int>(static_cast<int64_t>(b + 1) * n / kBins);
            if (end <= start) end = start + 1;
            uint16_t best = UINT16_MAX;
            for (int j = start; j < end && j < n; ++j) {
                best = std::min(best, scan.distancesCm[j]);
            }
            od.distances[b] = best;
        }
    }

    mavlink_message_t msg;
    (void) mavlink_msg_obstacle_distance_encode_chan(
        MAVLinkProtocol::instance()->getSystemId(),
        MAVLinkProtocol::getComponentId(),
        sharedLink->mavlinkChannel(),
        &msg,
        &od);

    if (!vehicle->sendMessageOnLinkThreadSafe(sharedLink.get(), msg)) {
        return false;
    }

    emit mavlinkMessageSent();
    return true;
}
