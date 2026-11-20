#include "vehicle_state.hpp"

#include <algorithm>
#include <cmath>

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
    // a route to fly: a landing-only mission (no plan) is not one
    return s.mission_known && s.mission.size() > 1 && !extractPlan(s.mission).wps.empty();
}

SafetyState safetyState(const VehicleState& s) {
    if (!s.has_sys_status || !(s.sensors_present & kSensorMotorOutputs)) {
        return SafetyState::Unknown;
    }
    return (s.sensors_enabled & kSensorMotorOutputs) ? SafetyState::Live : SafetyState::Safe;
}

bool resumeNeedsReset(const VehicleState& s) {
    return s.mission_known && hasFlyableMission(s) && seqToWaypointIndex(s.mission, s.mission_current) < 0;
}

bool isLandingInMission(const VehicleState& s) {
    if (s.custom_mode != plane::AUTO || !s.mission_known) {
        return false;
    }
    const int seq = landSeq(s.mission);  // the approach circle when there is one
    return seq >= 0 && s.mission_current >= seq;
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

namespace {

LiftRotors rawLiftRotorState(const VehicleState& s) {
    if (!s.has_servo_raw || !s.servo_map_final) {
        return LiftRotors::Unknown;
    }
    bool any_motor = false;
    bool spinning = false;
    for (size_t i = 0; i < s.servo_function.size(); ++i) {
        const int fn = s.servo_function[i];
        if (fn >= kServoFnMotor1 && fn <= kServoFnMotor8) {
            any_motor = true;
            spinning = spinning || s.servo_raw[i] >= kLiftSpinPwm;
        }
    }
    if (!any_motor) {
        return LiftRotors::Unknown;
    }
    if (!spinning) {
        return LiftRotors::Off;
    }
    // Hover, transitions and the ground are where the lift rotors are supposed
    // to run. Wing-borne flight, or a "transition" after the aircraft already
    // reached cruise (how ArduPlane reports Q_ASSIST), is assist.
    const VtolState v = effectiveVtolState(s);
    const bool wingborne = v == VtolState::Cruise || (v == VtolState::TransitionToCruise && s.cruised_since_hover);
    return wingborne ? LiftRotors::Assist : LiftRotors::Active;
}

}  // namespace

LiftRotors liftRotorState(const VehicleState& s, Clock::time_point now) {
    const LiftRotors raw = rawLiftRotorState(s);
    if (raw == LiftRotors::Assist &&
        (s.wingborne_spin_since == Clock::time_point{} || now - s.wingborne_spin_since < kAssistDebounce)) {
        return LiftRotors::Active;  // probably still spinning down after the transition
    }
    return raw;
}

void updateAssistTimer(VehicleState& s, Clock::time_point now) {
    if (rawLiftRotorState(s) != LiftRotors::Assist) {
        s.wingborne_spin_since = {};
    } else if (s.wingborne_spin_since == Clock::time_point{}) {
        s.wingborne_spin_since = now;
    }
}

void updateCruiseLatch(VehicleState& s) {
    if (s.landed == LandedState::OnGround || s.reported_vtol == VtolState::Hover ||
        s.reported_vtol == VtolState::TransitionToHover) {
        s.cruised_since_hover = false;
    } else if (s.reported_vtol == VtolState::Cruise) {
        s.cruised_since_hover = true;
    }
}

const char* liftRotorsName(LiftRotors r) {
    switch (r) {
    case LiftRotors::Off: return "OFF";
    case LiftRotors::Active: return "ACTIVE";
    case LiftRotors::Assist: return "ASSIST";
    default: return "--";
    }
}

namespace {
double secondsSince(Clock::time_point t, Clock::time_point now) {
    if (t == Clock::time_point{}) {
        return 0.0;
    }
    return std::chrono::duration<double>(now - t).count();
}
}  // namespace

void bodyRatesToEuler(double roll_rad, double pitch_rad, double p, double q, double r, double& roll_rate,
                      double& pitch_rate, double& yaw_rate) {
    const double sr = std::sin(roll_rad), cr = std::cos(roll_rad);
    const double cp = std::max(0.05, std::cos(pitch_rad));  // avoid the gimbal-lock blow-up
    const double tp = std::sin(pitch_rad) / cp;
    roll_rate = (p + (q * sr + r * cr) * tp) * geo::kRadToDeg;
    pitch_rate = (q * cr - r * sr) * geo::kRadToDeg;
    yaw_rate = ((q * sr + r * cr) / cp) * geo::kRadToDeg;
}

geo::Attitude predictAttitude(const VehicleState& s, Clock::time_point now) {
    const double age = std::clamp(secondsSince(s.att_rx, now), 0.0, kMaxPredictAttitude);
    geo::Attitude a;
    a.roll_deg = s.roll_deg + s.roll_rate * age;
    a.pitch_deg = s.pitch_deg + s.pitch_rate * age;
    a.yaw_deg = geo::wrap360(s.yaw_deg + s.yaw_rate * age);
    return a;
}

PredictedPosition predictPosition(const VehicleState& s, Clock::time_point now) {
    const double age = std::clamp(secondsSince(s.pos_rx, now), 0.0, kMaxPredictPosition);
    PredictedPosition p;
    geo::offset(s.lat, s.lon, s.vn * age, s.ve * age, p.lat, p.lon);
    p.alt_rel = s.alt_rel - s.vd * age;
    return p;
}

PredictedVelocity predictVelocity(const VehicleState& s, Clock::time_point now) {
    const double age = std::clamp(secondsSince(s.pos_rx, now), 0.0, kMaxPredictPosition);
    const double a = s.yaw_rate * age * geo::kDegToRad;
    PredictedVelocity v;
    v.vn = s.vn * std::cos(a) - s.ve * std::sin(a);
    v.ve = s.vn * std::sin(a) + s.ve * std::cos(a);
    v.vd = s.vd;
    return v;
}

void TrackPredictor::addSample(double vn, double ve, double vd, Clock::time_point t, double yaw_rate_dps) {
    const double track = std::atan2(ve, vn);
    const double speed = std::hypot(vn, ve);
    const double dt = have_ ? std::chrono::duration<double>(t - t_).count() : 0.0;
    if (have_ && dt > 0.02 && dt < 0.5 && speed > 2.0 && speed_ > 2.0) {
        const double d = std::remainder(track - track_, 2.0 * geo::kPi);
        // light filtering: GPS velocity has a little noise
        track_rate_ += 0.5 * (d / dt - track_rate_);
        speed_rate_ += 0.5 * ((speed - speed_) / dt - speed_rate_);
    } else {
        track_rate_ = yaw_rate_dps * geo::kDegToRad;  // first sample or a gap: assume a coordinated turn
        speed_rate_ = 0.0;
    }
    have_ = true;
    t_ = t;
    track_ = track;
    speed_ = speed;
    vd_ = vd;
}

PredictedVelocity TrackPredictor::at(Clock::time_point now) const {
    PredictedVelocity v;
    if (!have_) {
        return v;
    }
    const double age = std::clamp(std::chrono::duration<double>(now - t_).count(), 0.0, kMaxPredictPosition);
    const double tr = track_ + track_rate_ * age;
    const double sp = std::max(0.0, speed_ + speed_rate_ * age);
    v.vn = sp * std::cos(tr);
    v.ve = sp * std::sin(tr);
    v.vd = vd_;
    return v;
}

CameraLos cameraLos(const VehicleState& s, const geo::Attitude& aircraft, double fixed_tilt_deg,
                    Clock::time_point now, bool gimbal_body_frame) {
    CameraLos los;
    if (s.has_gimbal && secondsSince(s.gimbal_rx, now) < kGimbalTimeout && gimbal_body_frame) {
        los.gimbal = true;
        los.earth = geo::composeMount(aircraft, s.gimbal_yaw, s.gimbal_pitch);
        los.pan_rel = s.gimbal_yaw;
        los.tilt_rel = s.gimbal_pitch;
    } else if (s.has_gimbal && secondsSince(s.gimbal_rx, now) < kGimbalTimeout) {
        los.gimbal = true;
        los.earth.roll_deg = s.gimbal_roll;
        los.earth.pitch_deg = s.gimbal_pitch;
        los.earth.yaw_deg = geo::wrap360(s.gimbal_yaw_earth ? s.gimbal_yaw : aircraft.yaw_deg + s.gimbal_yaw);
        const geo::Attitude rel = geo::relativeTo(aircraft, los.earth);
        los.pan_rel = rel.yaw_deg;
        los.tilt_rel = rel.pitch_deg;
    } else {
        los.earth = geo::composeMount(aircraft, 0.0, fixed_tilt_deg);
        los.pan_rel = 0.0;
        los.tilt_rel = fixed_tilt_deg;
    }
    return los;
}

}  // namespace gcs
