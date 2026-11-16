#pragma once

#include "mission.hpp"

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

namespace gcs {

using Clock = std::chrono::steady_clock;

enum class VtolState { Unknown, Hover, TransitionToCruise, Cruise, TransitionToHover };
enum class LandedState { Unknown, OnGround, InAir, Takeoff, Landing };

struct TrailPoint {
    double lat = 0.0;
    double lon = 0.0;
    float alt_rel = 0.0f;
};

// Plain-value snapshot of everything the UI needs. Copied under a mutex by
// MavlinkClient::snapshot(); never shared by reference across threads.
struct VehicleState {
    // ---- link -------------------------------------------------------------
    bool socket_open = false;
    std::string endpoint;
    std::string error;
    bool peer_known = false;
    uint64_t datagrams = 0;
    uint64_t messages = 0;
    uint64_t bytes = 0;
    Clock::time_point last_rx{};
    Clock::time_point last_heartbeat{};

    // ---- heartbeat ----------------------------------------------------------
    bool has_heartbeat = false;
    uint8_t sysid = 0;
    uint8_t compid = 0;
    uint8_t mav_type = 0;
    uint8_t autopilot = 0;
    uint8_t base_mode = 0;
    uint8_t system_status = 0;
    uint32_t custom_mode = 0;
    bool armed = false;
    std::string mode_name;

    // ---- attitude -----------------------------------------------------------
    bool has_attitude = false;
    double roll_deg = 0.0;
    double pitch_deg = 0.0;
    double yaw_deg = 0.0;

    // ---- VFR HUD ------------------------------------------------------------
    bool has_vfr_hud = false;
    double airspeed = 0.0;
    double groundspeed = 0.0;
    int heading_deg = 0;
    int throttle_pct = 0;
    double climb = 0.0;

    // ---- position -----------------------------------------------------------
    bool has_position = false;
    double lat = 0.0;
    double lon = 0.0;
    double alt_amsl = 0.0;
    double alt_rel = 0.0;
    double vn = 0.0;
    double ve = 0.0;
    double vd = 0.0;

    // ---- home ---------------------------------------------------------------
    bool has_home = false;
    double home_lat = 0.0;
    double home_lon = 0.0;
    double home_alt_amsl = 0.0;

    // ---- power --------------------------------------------------------------
    bool has_battery = false;
    double battery_v = 0.0;
    double battery_a = -1.0;   // < 0 : unknown
    int battery_pct = -1;      // < 0 : unknown

    // ---- GPS ----------------------------------------------------------------
    bool has_gps = false;
    int gps_fix = 0;           // GPS_FIX_TYPE
    int gps_sats = 0;
    double gps_hdop = 99.0;

    // ---- health -------------------------------------------------------------
    bool has_sys_status = false;
    uint32_t sensors_present = 0;
    uint32_t sensors_enabled = 0;
    uint32_t sensors_health = 0;
    bool has_ekf = false;
    uint16_t ekf_flags = 0;

    // ---- VTOL ---------------------------------------------------------------
    bool has_ext_state = false;
    VtolState reported_vtol = VtolState::Unknown;
    LandedState landed = LandedState::Unknown;

    // ---- mission ------------------------------------------------------------
    int mission_current = -1;
    int last_reached = -1;
    bool mission_known = false;             // downloaded or uploaded this session
    std::vector<MissionItem> mission;       // what the vehicle holds (incl. home at 0)
    bool mission_transfer_active = false;

    // ---- trail --------------------------------------------------------------
    std::vector<TrailPoint> trail;
};

// ---- derived values (pure functions, unit-tested) ---------------------------
constexpr double kCruiseAirspeed = 12.0;   // m/s threshold between hover and wing-borne

double heartbeatAge(const VehicleState& s, Clock::time_point now);  // seconds, inf if never
bool linkAlive(const VehicleState& s, Clock::time_point now);       // heartbeat < 3 s
VtolState effectiveVtolState(const VehicleState& s);
bool isAirborne(const VehicleState& s);
bool hasFlyableMission(const VehicleState& s);
bool prearmHealthy(const VehicleState& s);
bool ekfHealthy(const VehicleState& s);
bool gpsHealthy(const VehicleState& s);
const char* vtolStateName(VtolState v);

}  // namespace gcs
