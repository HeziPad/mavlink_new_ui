#pragma once

#include <cstdint>
#include <vector>

namespace gcs {

// Values mirror MAVLink so the link layer can copy them 1:1.
namespace mav {
constexpr uint8_t kFrameGlobal = 0;               // MAV_FRAME_GLOBAL (AMSL)
constexpr uint8_t kFrameGlobalRelativeAlt = 3;    // MAV_FRAME_GLOBAL_RELATIVE_ALT
constexpr uint8_t kFrameGlobalRelativeAltInt = 6; // MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
constexpr uint16_t kCmdNavWaypoint = 16;
constexpr uint16_t kCmdNavVtolTakeoff = 84;
constexpr uint16_t kCmdNavVtolLand = 85;
}  // namespace mav

// Matches MISSION_ITEM_INT field-for-field (lat/lon in degE7).
struct MissionItem {
    uint16_t seq = 0;
    uint8_t frame = mav::kFrameGlobalRelativeAlt;
    uint16_t command = mav::kCmdNavWaypoint;
    uint8_t current = 0;
    uint8_t autocontinue = 1;
    float param1 = 0.0f;
    float param2 = 0.0f;
    float param3 = 0.0f;
    float param4 = 0.0f;
    int32_t x = 0;  // lat * 1e7
    int32_t y = 0;  // lon * 1e7
    float z = 0.0f; // altitude (frame dependent)
    uint8_t mission_type = 0;

    double lat() const { return x / 1.0e7; }
    double lon() const { return y / 1.0e7; }
};

struct UserWaypoint {
    double lat = 0.0;
    double lon = 0.0;
    float alt_rel = 80.0f;
};

int32_t toE7(double deg);

// Layout:
//   0          HOME           (frame GLOBAL, home AMSL altitude)
//   1          VTOL_TAKEOFF   (takeoff_alt, relative)
//   2..N       WAYPOINT       (user points except the last)
//   N+1        VTOL_LAND      (the final user point)
// Returns an empty vector when `wps` is empty.
std::vector<MissionItem> buildVtolMission(double home_lat, double home_lon, double home_alt_amsl,
                                          const std::vector<UserWaypoint>& wps,
                                          float takeoff_alt = 40.0f);

// Inverse of buildVtolMission for display / editing: recovers the navigable
// points (waypoints, and a land point if it is not on top of the last one).
std::vector<UserWaypoint> extractUserWaypoints(const std::vector<MissionItem>& items);

// Length of the leg flown into the VTOL_LAND (previous point, or home for a
// single-point plan). ArduPlane rejects arming when this is shorter than its
// back-transition distance (~120 m with default params), so the UI warns.
double finalApproachLength(double home_lat, double home_lon, const std::vector<UserWaypoint>& wps);

// Total horizontal route length home -> wps (metres).
double routeLength(double home_lat, double home_lon, const std::vector<UserWaypoint>& wps);

}  // namespace gcs
