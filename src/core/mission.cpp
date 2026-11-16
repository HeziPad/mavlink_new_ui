#include "mission.hpp"

#include "geo.hpp"

#include <cmath>

namespace gcs {

int32_t toE7(double deg) {
    return static_cast<int32_t>(std::llround(deg * 1.0e7));
}

std::vector<MissionItem> buildVtolMission(double home_lat, double home_lon, double home_alt_amsl,
                                          const std::vector<UserWaypoint>& wps, float takeoff_alt) {
    std::vector<MissionItem> items;
    if (wps.empty()) {
        return items;
    }
    items.reserve(wps.size() + 2);

    MissionItem home;
    home.frame = mav::kFrameGlobal;
    home.command = mav::kCmdNavWaypoint;
    home.x = toE7(home_lat);
    home.y = toE7(home_lon);
    home.z = static_cast<float>(home_alt_amsl);
    items.push_back(home);

    MissionItem takeoff;
    takeoff.frame = mav::kFrameGlobalRelativeAlt;
    takeoff.command = mav::kCmdNavVtolTakeoff;
    takeoff.x = toE7(home_lat);
    takeoff.y = toE7(home_lon);
    takeoff.z = takeoff_alt;
    items.push_back(takeoff);

    // The final user point becomes the VTOL_LAND itself rather than a waypoint
    // followed by a land on the same spot: ArduPlane refuses to arm when the
    // leg into a VTOL_LAND is shorter than its back-transition distance
    // ("VTOL land too short"), and a duplicated point makes that leg 0 m.
    for (size_t i = 0; i + 1 < wps.size(); ++i) {
        const auto& wp = wps[i];
        MissionItem item;
        item.frame = mav::kFrameGlobalRelativeAlt;
        item.command = mav::kCmdNavWaypoint;
        item.x = toE7(wp.lat);
        item.y = toE7(wp.lon);
        item.z = wp.alt_rel;
        items.push_back(item);
    }

    MissionItem land;
    land.frame = mav::kFrameGlobalRelativeAlt;
    land.command = mav::kCmdNavVtolLand;
    land.x = toE7(wps.back().lat);
    land.y = toE7(wps.back().lon);
    land.z = 0.0f;
    items.push_back(land);

    for (size_t i = 0; i < items.size(); ++i) {
        items[i].seq = static_cast<uint16_t>(i);
    }
    return items;
}

std::vector<UserWaypoint> extractUserWaypoints(const std::vector<MissionItem>& items) {
    std::vector<UserWaypoint> out;
    for (size_t i = 1; i < items.size(); ++i) {
        const auto& it = items[i];
        if (it.x == 0 && it.y == 0) {
            continue;  // commands without a location
        }
        if (it.command == mav::kCmdNavWaypoint) {
            out.push_back({it.lat(), it.lon(), it.z});
        } else if (it.command == mav::kCmdNavVtolLand) {
            if (out.empty() ||
                geo::distance(out.back().lat, out.back().lon, it.lat(), it.lon()) > 1.0) {
                out.push_back({it.lat(), it.lon(), out.empty() ? 40.0f : out.back().alt_rel});
            }
        }
    }
    return out;
}

double finalApproachLength(double home_lat, double home_lon, const std::vector<UserWaypoint>& wps) {
    if (wps.empty()) {
        return 0.0;
    }
    const double plat = wps.size() > 1 ? wps[wps.size() - 2].lat : home_lat;
    const double plon = wps.size() > 1 ? wps[wps.size() - 2].lon : home_lon;
    return geo::distance(plat, plon, wps.back().lat, wps.back().lon);
}

double routeLength(double home_lat, double home_lon, const std::vector<UserWaypoint>& wps) {
    double total = 0.0;
    double plat = home_lat;
    double plon = home_lon;
    for (const auto& wp : wps) {
        total += geo::distance(plat, plon, wp.lat, wp.lon);
        plat = wp.lat;
        plon = wp.lon;
    }
    return total;
}

}  // namespace gcs
