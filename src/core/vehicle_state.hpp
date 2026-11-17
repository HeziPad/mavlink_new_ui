#pragma once

#include "geo.hpp"
#include "mission.hpp"

#include <array>
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
    std::chrono::steady_clock::time_point t{};  // when it was recorded
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
    // Euler angle rates (deg/s), converted from the body rates in ATTITUDE,
    // and when the sample arrived: lets the UI predict between samples.
    double roll_rate = 0.0;
    double pitch_rate = 0.0;
    double yaw_rate = 0.0;
    Clock::time_point att_rx{};

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
    Clock::time_point pos_rx{};

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
    // EKF_STATUS_REPORT innovation test ratios: < 0.5 good, >= 0.8 bad
    double ekf_vel_var = 0.0;
    double ekf_pos_var = 0.0;
    double ekf_hgt_var = 0.0;
    double ekf_mag_var = 0.0;

    // ---- VTOL ---------------------------------------------------------------
    bool has_ext_state = false;
    VtolState reported_vtol = VtolState::Unknown;
    // Wing-borne at least once since the last hover. ArduPlane reports Q_ASSIST
    // in cruise as TRANSITION_TO_FW, so this separates "assisting" from a real
    // forward transition after takeoff. Maintained by updateCruiseLatch().
    bool cruised_since_hover = false;
    // When wing-borne lift-motor spin started (zero = not spinning). Lets a
    // spin-down after a transition pass without being called assist.
    Clock::time_point wingborne_spin_since{};
    LandedState landed = LandedState::Unknown;

    // ---- mission ------------------------------------------------------------
    int mission_current = -1;
    int last_reached = -1;
    bool mission_known = false;             // downloaded or uploaded this session
    std::vector<MissionItem> mission;       // what the vehicle holds (incl. home at 0)
    bool mission_transfer_active = false;

    // ---- navigation / wind ------------------------------------------------------
    bool has_nav = false;
    double nav_alt_error = 0.0;             // NAV_CONTROLLER_OUTPUT: target - current (m)
    bool has_wind = false;
    double wind_from_deg = 0.0;             // direction the wind blows FROM
    double wind_speed = 0.0;

    // ---- gimbal ---------------------------------------------------------------
    // GIMBAL_DEVICE_ATTITUDE_STATUS. ArduPilot: roll / pitch are earth-frame,
    // yaw is relative to the vehicle (flag) or earth-frame.
    bool has_gimbal = false;
    double gimbal_roll = 0.0;
    double gimbal_pitch = 0.0;
    double gimbal_yaw = 0.0;
    bool gimbal_yaw_earth = false;
    Clock::time_point gimbal_rx{};

    // ---- measured message rates (Hz, 1 s window) ---------------------------------
    double attitude_hz = 0.0;
    double position_hz = 0.0;

    // ---- parameters (read once after connect) -----------------------------------
    int q_guided_mode = -1;                 // Q_GUIDED_MODE, -1 unknown
    double airspeed_cruise = -1.0;          // AIRSPEED_CRUISE (m/s), -1 unknown
    double trans_decel = -1.0;              // Q_TRANS_DECEL (m/s^2)
    double loiter_radius = -1.0;            // WP_LOITER_RAD (m)
    std::array<int, 16> servo_function{};   // SERVOn_FUNCTION, 0 until read
    int servo_functions_known = 0;          // how many of the 16 have arrived
    bool servo_map_final = false;           // all arrived, or gave up waiting

    // ---- outputs ------------------------------------------------------------
    bool has_servo_raw = false;
    std::array<uint16_t, 16> servo_raw{};   // SERVO_OUTPUT_RAW, PWM us

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
// AUTO and on the approach circle or the land item (not on the loop)
bool isLandingInMission(const VehicleState& s);
// Resuming AUTO from here would not continue the loop: the current item is
// home, the VTOL takeoff (it would hover and climb on the rotors again) or the
// landing. Set the current item to a loop waypoint first.
bool resumeNeedsReset(const VehicleState& s);
bool prearmHealthy(const VehicleState& s);
bool ekfHealthy(const VehicleState& s);
bool gpsHealthy(const VehicleState& s);
const char* vtolStateName(VtolState v);

// Lift (VTOL) rotor activity, from the motor outputs.
//   Off     lift motors idle (wing-borne or disarmed)
//   Active  expected use: hover, transition, or armed on the ground
//   Assist  wing-borne but a lift motor is spinning (Q_ASSIST stall help)
enum class LiftRotors { Unknown, Off, Active, Assist };
// SERVOn_FUNCTION 33..40 are Motor1..8, the quadplane lift motors.
constexpr int kServoFnMotor1 = 33;
constexpr int kServoFnMotor8 = 40;
// Above this PWM a lift motor is considered spinning (idle/disarmed output is
// ~1000 us; Q_M_SPIN_ARM sits around 1100).
constexpr uint16_t kLiftSpinPwm = 1150;
// Lift motors still spinning down after a forward transition look like
// assist for a moment; only call it assist once it lasts this long.
constexpr auto kAssistDebounce = std::chrono::seconds(2);
LiftRotors liftRotorState(const VehicleState& s, Clock::time_point now);

// ---- prediction between samples ----------------------------------------------
// Telemetry arrives at 10-25 Hz; drawing only the latest sample makes motion
// step. These extrapolate from the last sample with its rates, for at most
// kMaxPredict, so the UI can draw a continuous estimate every frame.
constexpr double kMaxPredictAttitude = 0.25;  // s
constexpr double kMaxPredictPosition = 0.5;   // s
geo::Attitude predictAttitude(const VehicleState& s, Clock::time_point now);
struct PredictedPosition {
    double lat = 0.0;
    double lon = 0.0;
    double alt_rel = 0.0;
};
PredictedPosition predictPosition(const VehicleState& s, Clock::time_point now);
// Euler angle rates from body rates (rad/s in, deg/s out).
void bodyRatesToEuler(double roll_rad, double pitch_rad, double p, double q, double r, double& roll_rate,
                      double& pitch_rate, double& yaw_rate);

// ---- camera line of sight -------------------------------------------------------
struct CameraLos {
    geo::Attitude earth;   // camera attitude in the earth frame (yaw = LOS azimuth, pitch = elevation)
    double pan_rel = 0.0;  // relative to the platform
    double tilt_rel = 0.0;
    bool gimbal = false;   // false: fixed mount from settings
};
// gimbal angles fresher than this are used; otherwise the fixed mount tilt
constexpr double kGimbalTimeout = 2.0;  // s
// gimbal_body_frame: the gimbal reports body-frame angles even though the
// message says earth-frame roll/pitch. ArduPilot's servo mount (no feedback)
// does this; gimbals with attitude feedback follow the spec (earth frame).
CameraLos cameraLos(const VehicleState& s, const geo::Attitude& aircraft, double fixed_tilt_deg,
                    Clock::time_point now, bool gimbal_body_frame = false);
// Call after reported_vtol / landed change.
void updateCruiseLatch(VehicleState& s);
// Call after servo outputs or VTOL state change.
void updateAssistTimer(VehicleState& s, Clock::time_point now);
const char* liftRotorsName(LiftRotors r);

// Hardware safety switch, from SYS_STATUS: ArduPilot always reports
// MOTOR_OUTPUTS present, and *enabled* unless the safety switch is engaged.
//   Safe  outputs cut (motors can't spin, arming is refused)
//   Live  outputs flow (no switch fitted, or safety turned off)
enum class SafetyState { Unknown, Safe, Live };
constexpr uint32_t kSensorMotorOutputs = 0x8000;
SafetyState safetyState(const VehicleState& s);

}  // namespace gcs
