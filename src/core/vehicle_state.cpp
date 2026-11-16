#include "vehicle_state.hpp"

#include "plane_modes.hpp"

#include <limits>

namespace gcs {

namespace {
constexpr uint32_t kPrearmBit = 0x10000000u;  // MAV_SYS_STATUS_PREARM_CHECK
constexpr uint16_t kEkfAttitude = 1;
constexpr uint16_t kEkfVelHoriz = 2;
constexpr uint16_t kEkfVelVert = 4;
constexpr uint16_t kEkfPosHorizAbs = 16;
constexpr uint16_t kEkfPosVertAbs = 32;
constexpr uint16_t kEkfUninitialized = 1024;
constexpr uint16_t kEkfGpsGlitching = 32768;
}  // namespace

double heartbeatAge(const VehicleState& s, Clock::time_point now) {
    if (!s.has_heartbeat) {
        return std::numeric_limits<double>::infinity();
    }
    return std::chrono::duration<double>(now - s.last_heartbeat).count();
}

bool linkAlive(const VehicleState& s, Clock::time_point now) {
    return heartbeatAge(s, now) < 3.0;
}

VtolState effectiveVtolState(const VehicleState& s) {
    // ArduPlane reports FW whenever it is not in a Q-mode, even parked on the
    // ground; a grounded quadplane is in its VTOL configuration, so show HOVER.
    if (s.has_ext_state && s.landed == LandedState::OnGround) {
        return VtolState::Hover;
    }
    if (s.has_ext_state && s.reported_vtol != VtolState::Unknown) {
        return s.reported_vtol;
    }
    if (!s.has_heartbeat) {
        return VtolState::Unknown;
    }
    if (plane::isQMode(s.custom_mode)) {
        return VtolState::Hover;
    }
    if (s.has_vfr_hud && s.airspeed > kCruiseAirspeed && s.throttle_pct > 0) {
        return VtolState::Cruise;
    }
    return VtolState::Hover;
}

bool isAirborne(const VehicleState& s) {
    if (s.has_ext_state && s.landed != LandedState::Unknown) {
        return s.landed != LandedState::OnGround;
    }
    return s.armed && s.has_position && s.alt_rel > 2.0;
}

bool hasFlyableMission(const VehicleState& s) {
    // index 0 is home; anything beyond it is something to fly
    return s.mission_known && s.mission.size() > 1;
}

bool prearmHealthy(const VehicleState& s) {
    if (!s.has_sys_status) {
        return false;
    }
    if ((s.sensors_present & kPrearmBit) == 0) {
        return true;  // autopilot does not report it; do not block
    }
    return (s.sensors_health & kPrearmBit) != 0;
}

bool ekfHealthy(const VehicleState& s) {
    if (!s.has_ekf) {
        return false;
    }
    const uint16_t need = kEkfAttitude | kEkfVelHoriz | kEkfVelVert | kEkfPosHorizAbs | kEkfPosVertAbs;
    if ((s.ekf_flags & need) != need) {
        return false;
    }
    return (s.ekf_flags & (kEkfUninitialized | kEkfGpsGlitching)) == 0;
}

bool gpsHealthy(const VehicleState& s) {
    return s.has_gps && s.gps_fix >= 3 && s.gps_sats >= 6;
}

const char* vtolStateName(VtolState v) {
    switch (v) {
    case VtolState::Hover: return "HOVER";
    case VtolState::TransitionToCruise: return "TRANSITION \xE2\x86\x92 CRUISE";
    case VtolState::Cruise: return "CRUISE";
    case VtolState::TransitionToHover: return "TRANSITION \xE2\x86\x92 HOVER";
    case VtolState::Unknown: break;
    }
    return "\xE2\x80\x94";
}

}  // namespace gcs
