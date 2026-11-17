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
constexpr uint16_t kCmdDoLandStart = 189;     // marks where a landing sequence starts (RTL_AUTOLAND)
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

// ---- landing approach ---------------------------------------------------------
//
// The aircraft circles down on the wing on a circle BESIDE the landing point,
// leaves it on the tangent at the approach altitude heading into the wind,
// flies a straight leg long enough for ArduPlane to stay on the wing until its
// own back-transition point, then lands vertically. ArduPlane only flies the
// wing toward a VTOL_LAND when it is further than
// transition_threshold = 1.5 * v^2 / (2 * Q_TRANS_DECEL); closer than that it
// goes straight to rotor position control (slow and rotor-hungry).
struct ApproachInputs {
    double wind_from_deg = 0.0;   // direction the wind blows from
    double wind_speed = 0.0;      // m/s
    double from_lat = 0.0;        // where the aircraft comes from (side choice,
    double from_lon = 0.0;        //   and the approach direction without wind)
    double airspeed = 22.0;       // cruise airspeed, m/s
    double trans_decel = 2.0;     // Q_TRANS_DECEL, m/s^2
    double radius = 80.0;         // circle radius, m
};

struct LandingApproach {
    double c_lat = 0.0, c_lon = 0.0;  // circle centre
    double e_lat = 0.0, e_lon = 0.0;  // exit point on the circle, start of the straight leg
    double l_lat = 0.0, l_lon = 0.0;  // landing point
    double heading_deg = 0.0;         // straight-leg course (into the wind when there is wind)
    double leg_m = 0.0;               // exit -> landing point
    double radius = 0.0;
    bool ccw = false;
    float alt = kDefaultLandApproachAlt;  // approach altitude (relative)
};

constexpr double kMinWindForApproach = 2.0;  // m/s; below this, approach from where we are
// leg = transition_threshold * kLegMargin + kLegExtra, at least kMinLeg
constexpr double kLegMargin = 1.3;
constexpr double kLegExtra = 40.0;
constexpr double kMinLeg = 150.0;

double transitionThreshold(double airspeed, double trans_decel);
LandingApproach computeLandingApproach(const UserWaypoint& land, const ApproachInputs& in);

// Layout:
//   0          HOME           (frame GLOBAL, home AMSL altitude)
//   1          VTOL_TAKEOFF   (takeoff_alt, relative)
//   2..n+1     WAYPOINT x n   (per-point relative altitude)
//   n+2        DO_JUMP -> 2, repeat forever   (n >= 2)
//              LOITER_UNLIM at the point is used instead when n == 1
//   n+3        DO_LAND_START at plan.land: with RTL_AUTOLAND = 2 every RTL
//              (failsafes included) jumps here and flies the same landing as
//              the Land button
//   n+4        LOITER_TO_ALT at the approach circle centre (+-radius = CW/CCW,
//              z = approach alt, exit on the tangent toward the next item)
//   n+5        VTOL_LAND at plan.land (no spiral option; z = 0 = ground at home
//              level: ArduPlane uses this as the ground for its descent)
//
// The approach sits behind an endless jump, so it is never reached by
// sequence; the GCS jumps to it with DO_SET_MISSION_CURRENT. Being preceded
// by a LOITER_TO_ALT (not a NAV_WAYPOINT) the land item also stays clear of
// ArduPlane's "VTOL land too short" arming check.
// `approach` is required when plan.land is set (otherwise the land is left out).
// Returns an empty vector when the plan has no waypoints.
std::vector<MissionItem> buildLoopMission(double home_lat, double home_lon, double home_alt_amsl,
                                          const MissionPlan& plan, float takeoff_alt = 40.0f,
                                          const LandingApproach* approach = nullptr);

// Inverse of buildLoopMission for display / editing. Also accepts missions
// from other tools: every located WAYPOINT / LOITER_* becomes a loop point,
// except a loiter directly before the land (the approach circle), and the
// first VTOL_LAND / LAND becomes the landing point.
MissionPlan extractPlan(const std::vector<MissionItem>& items);

// The approach as uploaded (circle + land), if the mission has one.
std::optional<LandingApproach> extractApproach(const std::vector<MissionItem>& items);

// seq where landing starts: the approach circle if there is one, else the
// first VTOL_LAND / LAND item; -1 if none. (The DO_LAND_START marker before
// the circle is not returned: jumping to the circle is the same thing.)
int landSeq(const std::vector<MissionItem>& items);

// Maps a mission seq to an index into extractPlan(items).wps; -1 when the
// item is not a loop point (home, takeoff, jump, approach, land).
int seqToWaypointIndex(const std::vector<MissionItem>& items, int seq);

// Horizontal length of one lap (wp1 -> ... -> wpN -> wp1), metres.
double lapLength(const std::vector<UserWaypoint>& wps);
// Where Resume Plan rejoins the loop: the seq of the loop waypoint nearest to
// (lat, lon). If the aircraft is already within `skip_within_m` of it, the
// next one around the loop instead (so it doesn't turn back over it).
// -1 when the mission has no loop.
int nearestLoopSeq(const std::vector<MissionItem>& items, double lat, double lon, double skip_within_m);

}  // namespace gcs
