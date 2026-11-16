#pragma once

#include <cstdint>
#include <optional>
#include <vector>

namespace gcs {

// Values mirror MAVLink so the link layer can copy them 1:1.
namespace mav {
constexpr uint8_t kFrameGlobal = 0;               // MAV_FRAME_GLOBAL (AMSL)
constexpr uint8_t kFrameGlobalRelativeAlt = 3;    // MAV_FRAME_GLOBAL_RELATIVE_ALT
constexpr uint8_t kFrameGlobalRelativeAltInt = 6; // MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
constexpr uint16_t kCmdNavWaypoint = 16;
constexpr uint16_t kCmdNavLoiterUnlim = 17;
constexpr uint16_t kCmdNavLoiterTurns = 18;
constexpr uint16_t kCmdNavLoiterTime = 19;
constexpr uint16_t kCmdNavLand = 21;
constexpr uint16_t kCmdNavLoiterToAlt = 31;
constexpr uint16_t kCmdNavVtolTakeoff = 84;
constexpr uint16_t kCmdNavVtolLand = 85;
constexpr uint16_t kCmdDoJump = 177;
constexpr float kVtolLandFwSpiral = 1.0f;         // NAV_VTOL_LAND_OPTIONS_FW_SPIRAL_APPROACH
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

// What the operator plans: a loop flown forever, plus an optional landing
// point that is only flown to on request (Land button).
struct MissionPlan {
    std::vector<UserWaypoint> wps;      // loop points, each with its own altitude
    // Separate landing point. Its alt_rel is the approach altitude: the
    // aircraft spirals down to it on the wing, then back-transitions and
    // descends vertically from there.
    std::optional<UserWaypoint> land;

    bool empty() const { return wps.empty() && !land; }
};

int32_t toE7(double deg);

constexpr float kDefaultLandApproachAlt = 30.0f;

// Layout:
//   0          HOME           (frame GLOBAL, home AMSL altitude)
//   1          VTOL_TAKEOFF   (takeoff_alt, relative)
//   2..n+1     WAYPOINT x n   (per-point relative altitude)
//   n+2        DO_JUMP -> 2, repeat forever   (n >= 2)
//              LOITER_UNLIM at the point is used instead when n == 1
//   n+3        VTOL_LAND at plan.land, fixed-wing spiral approach (optional);
//              z = approach altitude. ArduPlane reads a relative z of 0 as
//              "current altitude", so 0 would mean no descent on the wing.
//
// The land item sits behind an endless jump, so it is never reached by
// sequence; the GCS jumps to it with DO_SET_MISSION_CURRENT. Being preceded
// by a DO_JUMP (not a NAV_WAYPOINT) it also stays clear of ArduPlane's
// "VTOL land too short" arming check, which the spiral approach makes moot.
// Returns an empty vector when the plan has no waypoints.
std::vector<MissionItem> buildLoopMission(double home_lat, double home_lon, double home_alt_amsl,
                                          const MissionPlan& plan, float takeoff_alt = 40.0f);

// Inverse of buildLoopMission for display / editing. Also accepts missions
// from other tools: every located WAYPOINT / LOITER_* becomes a loop point
// and the first VTOL_LAND / LAND becomes the landing point.
MissionPlan extractPlan(const std::vector<MissionItem>& items);

// seq of the first VTOL_LAND (or LAND) item, -1 if none.
int landSeq(const std::vector<MissionItem>& items);

// Maps a mission seq to an index into extractPlan(items).wps; -1 when the
// item is not a loop point (home, takeoff, jump, land).
int seqToWaypointIndex(const std::vector<MissionItem>& items, int seq);

// Horizontal length of one lap (wp1 -> ... -> wpN -> wp1), metres.
double lapLength(const std::vector<UserWaypoint>& wps);

}  // namespace gcs
