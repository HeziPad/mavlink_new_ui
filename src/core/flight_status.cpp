#include "flight_status.hpp"

#include "geo.hpp"
#include "plane_modes.hpp"

#include <cmath>

namespace gcs {

namespace {

bool isFinalHold(const VehicleState& s, int seq) {
    if (seq < 1 || seq >= static_cast<int>(s.mission.size()) || s.mission[seq].command != mav::kCmdNavLoiterUnlim) {
        return false;
    }
    const int h = holdIndex(extractPlan(s.mission));
    return h >= 0 && seqToWaypointIndex(s.mission, seq) == h;
}

}  // namespace

FlightStatus flightStatus(const VehicleState& s, const FlightContext& ctx, Clock::time_point now) {
    FlightStatus f;
    if (!s.has_heartbeat) {
        f.label = "—";
        return f;
    }
    const LiftRotors lift = liftRotorState(s, now);
    f.rotors = lift == LiftRotors::Active || lift == LiftRotors::Assist;
    f.assist = lift == LiftRotors::Assist;
    const VtolState vtol = effectiveVtolState(s);
    const uint32_t m = s.custom_mode;

    if (!isAirborne(s)) {
        if (s.has_ext_state && s.landed == LandedState::Takeoff) {
            f.label = "TAKEOFF";
            f.tone = Tone::Busy;
        } else if (s.armed) {
            f.label = "ARMED · ON GROUND";
            f.tone = Tone::Warn;
        } else {
            f.label = "ON GROUND";
        }
        return f;
    }

    // landing first: it can be AUTO (Land button, failsafe RTL jump) or Q-modes
    if (isLandingInMission(s) || (s.has_ext_state && s.landed == LandedState::Landing)) {
        const bool vertical = vtol == VtolState::Hover || vtol == VtolState::TransitionToHover;
        f.label = vertical ? "LANDING · VERTICAL" : "LANDING · APPROACH";
        f.tone = Tone::Busy;
        return f;
    }
    if (m == plane::QLAND || m == plane::LOITER_ALT_QLAND) {
        f.label = "QLAND · LANDING HERE";
        f.tone = Tone::Warn;
        return f;
    }
    if (m == plane::RTL || m == plane::QRTL) {
        f.label = m == plane::RTL ? "RTL" : "QRTL";
        f.tone = Tone::Warn;
        return f;
    }
    if (s.has_ext_state && s.landed == LandedState::Takeoff) {
        f.label = "TAKEOFF";
        f.tone = Tone::Busy;
        return f;
    }
    if (vtol == VtolState::TransitionToCruise && !f.assist) {
        f.label = "TRANSITION";
        f.tone = Tone::Busy;
        return f;
    }
    if (vtol == VtolState::TransitionToHover) {
        f.label = "TRANSITION TO HOVER";
        f.tone = Tone::Busy;
        return f;
    }

    if (m == plane::AUTO) {
        if (s.mission_current == 1 && vtol == VtolState::Hover) {
            f.label = "TAKEOFF";
            f.tone = Tone::Busy;
        } else if (isFinalHold(s, s.mission_current)) {
            f.label = "END HOLD";
            f.tone = Tone::Good;
        } else if (const int wp = seqToWaypointIndex(s.mission, s.mission_current); wp >= 0) {
            f.label = "ON PLAN · WP" + std::to_string(wp + 1);
            f.tone = Tone::Good;
        } else {
            f.label = "AUTO";
            f.tone = Tone::Good;
        }
    } else if (m == plane::GUIDED) {
        if (ctx.transition_leg) {
            f.label = vtol == VtolState::Hover ? "TAKEOFF" : "CLIMB-OUT";
            f.tone = Tone::Busy;
        } else if (vtol == VtolState::Hover && s.has_vfr_hud && s.airspeed < kCruiseAirspeed) {
            f.label = "HOVER";  // a GUIDED target reached on the rotors (Q_GUIDED_MODE = 1?)
            f.tone = Tone::Warn;
        } else if (ctx.orbit) {
            f.label = ctx.orbit_hold ? "HOLD" : "FLY TO";
            f.tone = Tone::Good;
        } else {
            f.label = "GUIDED";
            f.tone = Tone::Good;
        }
    } else {
        // anything else (FBWA, MANUAL, QLOITER, ...): ArduPilot's own name
        f.label = s.mode_name.empty() ? plane::modeName(m) : s.mode_name;
        f.tone = plane::isQMode(m) ? Tone::Warn : Tone::Neutral;
    }
    if (f.assist) {
        f.label += " · ROTOR ASSIST";
        f.tone = Tone::Warn;
    }
    return f;
}

double etaSeconds(double dist_m, double groundspeed) {
    return groundspeed >= kMinEtaSpeed ? dist_m / groundspeed : -1.0;
}

NavTarget pointTarget(const VehicleState& s, const std::string& name, double lat, double lon, double radius_m) {
    NavTarget t;
    if (!s.has_position) {
        return t;
    }
    t.valid = true;
    t.name = name;
    t.lat = lat;
    t.lon = lon;
    t.dist_m = std::max(0.0, geo::distance(s.lat, s.lon, lat, lon) - std::max(0.0, radius_m));
    t.eta_s = etaSeconds(t.dist_m, std::hypot(s.vn, s.ve));
    return t;
}

NavTarget missionTarget(const VehicleState& s) {
    const int seq = s.mission_current;
    if (s.custom_mode != plane::AUTO || !s.mission_known || seq < 1 || seq >= static_cast<int>(s.mission.size())) {
        return {};
    }
    const MissionItem& it = s.mission[seq];
    if (it.x == 0 && it.y == 0) {
        return {};
    }
    std::string name;
    double radius = 0.0;
    if (it.command == mav::kCmdNavVtolLand || it.command == mav::kCmdNavLand) {
        name = "L";
    } else if (seq == landSeq(s.mission) && it.command == mav::kCmdNavLoiterToAlt) {
        name = "APPROACH";
        radius = std::abs(it.param2);
    } else if (it.command == mav::kCmdNavVtolTakeoff) {
        name = "TAKEOFF";
    } else if (isFinalHold(s, seq)) {
        name = "END HOLD";
        radius = std::abs(it.param3) > 1.0f ? std::abs(it.param3) : (s.loiter_radius > 1.0 ? s.loiter_radius : 0.0);
    } else if (const int wp = seqToWaypointIndex(s.mission, seq); wp >= 0) {
        name = "WP" + std::to_string(wp + 1);
    } else {
        name = "ITEM " + std::to_string(seq);
    }
    return pointTarget(s, name, it.lat(), it.lon(), radius);
}

std::optional<double> rangefinderAgl(const VehicleState& s, Clock::time_point now) {
    if (!s.has_rangefinder || s.rangefinder_orient != kRangefinderDown || s.rangefinder_quality == 1) {
        return std::nullopt;
    }
    if (std::chrono::duration<double>(now - s.rangefinder_rx).count() > kRangefinderTimeout) {
        return std::nullopt;
    }
    const double d = s.rangefinder_m;
    if (s.rangefinder_max <= s.rangefinder_min || d <= s.rangefinder_min ||
        d >= s.rangefinder_max * kRangefinderMaxFrac) {
        return std::nullopt;  // nothing in range (a max-range reading), or below its minimum
    }
    if (std::abs(s.roll_deg) > kRangefinderMaxTilt || std::abs(s.pitch_deg) > kRangefinderMaxTilt) {
        return std::nullopt;
    }
    return d * std::cos(s.roll_deg * geo::kDegToRad) * std::cos(s.pitch_deg * geo::kDegToRad);
}

}  // namespace gcs
