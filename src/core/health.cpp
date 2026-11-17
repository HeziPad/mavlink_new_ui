#include "health.hpp"

#include <algorithm>
#include <cctype>
#include <cstdio>

namespace gcs {

namespace {

// ---- EKF_STATUS_FLAGS -------------------------------------------------------------
constexpr uint16_t kEkfAttitude = 1;
constexpr uint16_t kEkfVelHoriz = 2;
constexpr uint16_t kEkfVelVert = 4;
constexpr uint16_t kEkfPosHorizAbs = 16;
constexpr uint16_t kEkfPosVertAbs = 32;
constexpr uint16_t kEkfConstPosMode = 128;
constexpr uint16_t kEkfUninitialized = 1024;
constexpr uint16_t kEkfGpsGlitching = 32768;

// ---- MAV_SYS_STATUS_SENSOR ----------------------------------------------------------
struct SensorInfo {
    uint32_t bit;
    const char* id;
    Severity severity;
    const char* title;
    const char* cause;
    std::vector<const char*> suggestions;
};

const std::vector<SensorInfo>& sensorTable() {
    static const std::vector<SensorInfo> t = {
        {0x1, "sensor.gyro", Severity::Warning, "Gyroscope unhealthy",
         "The rotation sensor (gyro) is reporting errors. The autopilot needs it to stay level.",
         {"Keep the aircraft still and reboot the autopilot",
          "If it repeats, re-run the IMU calibration in Mission Planner / QGroundControl",
          "Persistent errors usually mean a hardware fault: don't fly"}},
        {0x2, "sensor.accel", Severity::Warning, "Accelerometer unhealthy",
         "The accelerometer is reporting errors or needs calibration.",
         {"Reboot with the aircraft level and still",
          "Re-run the accelerometer calibration (Mission Planner: Setup > Mandatory > Accel Calibration)"}},
        {0x4, "sensor.mag", Severity::Warning, "Compass unhealthy",
         "The compass is not giving usable readings, so heading may be wrong.",
         {"Move away from metal, vehicles and power cables",
          "Recalibrate the compass",
          "Check the compass/GPS cable and the mounting orientation (COMPASS_ORIENT)"}},
        {0x8, "sensor.baro", Severity::Warning, "Barometer unhealthy",
         "The pressure sensor that measures altitude is reporting errors.",
         {"Shield the flight controller from wind and direct sunlight (foam over the baro)",
          "Reboot the autopilot; if it persists, the sensor may be faulty"}},
        {0x10, "sensor.airspeed", Severity::Warning, "Airspeed sensor unhealthy",
         "The pitot/airspeed sensor isn't working. Without it the plane flies on throttle and pitch estimates and "
         "stall protection is weaker.",
         {"Check the pitot tube is clear (remove the cover!) and the tubing isn't pinched",
          "Check the sensor cable",
          "Re-run the airspeed calibration on the ground with the pitot shielded from wind"}},
        {0x20, "sensor.gps", Severity::Warning, "GPS unhealthy",
         "The GPS receiver isn't delivering a usable position.",
         {"Give it a clear view of the sky, away from buildings and trees",
          "Check the GPS cable and that the video transmitter isn't next to the GPS"}},
        {0x8000, "sensor.motors", Severity::Warning, "Motor outputs problem",
         "The autopilot reports a problem with its motor/servo outputs.",
         {"Check ESC power and signal wiring", "Check the SERVOx_FUNCTION and Q_FRAME settings"}},
        {0x10000, "sensor.rc", Severity::Warning, "No RC receiver signal",
         "The autopilot isn't receiving your radio transmitter. That is your manual backup.",
         {"Turn on the transmitter and check it is bound to the receiver",
          "Check the receiver wiring and the RC failsafe settings"}},
        {0x20000, "sensor.gyro2", Severity::Caution, "Second gyroscope unhealthy",
         "A backup gyro is reporting errors.", {"Reboot; if it persists, re-run the IMU calibration"}},
        {0x40000, "sensor.accel2", Severity::Caution, "Second accelerometer unhealthy",
         "A backup accelerometer is reporting errors.", {"Re-run the accelerometer calibration"}},
        {0x80000, "sensor.mag2", Severity::Caution, "Second compass unhealthy",
         "A backup compass is reporting errors or disagrees with the first.",
         {"Recalibrate all compasses", "Disable an internal compass that sits near power wiring"}},
        {0x100000, "sensor.fence", Severity::Warning, "Geofence breached",
         "The aircraft is outside the configured fence, or the fence is not loaded correctly.",
         {"Check the fence in your ground station", "Expect the fence action (e.g. RTL) to take over"}},
        {0x200000, "sensor.ahrs", Severity::Warning, "Attitude/heading system not ready",
         "The autopilot's attitude and heading reference (AHRS/EKF) isn't healthy yet.",
         {"Wait 30-60 s after power-on with the aircraft still", "See the navigation filter card for details"}},
        {0x400000, "sensor.terrain", Severity::Info, "Terrain data not loaded",
         "Terrain following is enabled but the autopilot has no terrain data for this area. This GCS doesn't send "
         "terrain tiles, so it only clears if the SD card already holds them. Missions here use heights above home, "
         "so this doesn't affect them.",
         {"Ignore it if you don't fly terrain-relative missions, or set TERRAIN_ENABLE = 0 to hide it",
          "To use terrain: pre-load tiles with Mission Planner or QGroundControl once"}},
        {0x1000000, "sensor.logging", Severity::Caution, "Logging not working",
         "The flight log isn't being written (no SD card or it's full/faulty).",
         {"Check the SD card is inserted; format it if needed", "Flying works, but there will be no log"}},
        {0x2000000, "sensor.battery", Severity::Warning, "Battery monitor reports a problem",
         "The battery is below its configured failsafe level or the monitor is faulty.",
         {"Charge or swap the battery", "Check BATT_LOW_VOLT / BATT_CRT_VOLT match your pack"}},
    };
    return t;
}

constexpr uint32_t kPrearmBit = 0x10000000u;

Advisory make(const char* id, Severity sev, std::string title, std::string cause,
              std::vector<std::string> suggestions, std::string detail = {}) {
    Advisory a;
    a.id = id;
    a.severity = sev;
    a.title = std::move(title);
    a.cause = std::move(cause);
    a.suggestions = std::move(suggestions);
    a.detail = std::move(detail);
    return a;
}

std::string fmt(const char* f, double v) {
    char buf[512];
    std::snprintf(buf, sizeof(buf), f, v);
    return buf;
}

std::string ekfDetail(const VehicleState& s) {
    char buf[160];
    std::snprintf(buf, sizeof(buf), "EKF flags 0x%04x · vel %.2f pos %.2f hgt %.2f mag %.2f", s.ekf_flags,
                  s.ekf_vel_var, s.ekf_pos_var, s.ekf_hgt_var, s.ekf_mag_var);
    return buf;
}

void assessEkf(const VehicleState& s, bool airborne, std::vector<Advisory>& out) {
    if (!s.has_ekf) {
        out.push_back(make("ekf.none", Severity::Caution, "No navigation filter report yet",
                           "The autopilot hasn't sent its estimator (EKF) status. It normally arrives within a few "
                           "seconds of connecting.",
                           {"Wait a few seconds", "If it never arrives, check the telemetry stream rates"}));
        return;
    }
    const std::string detail = ekfDetail(s);
    const uint16_t f = s.ekf_flags;
    if (f & kEkfUninitialized) {
        out.push_back(make("ekf.init", Severity::Warning, "Navigation filter starting up",
                           "The estimator that works out attitude and position (EKF) hasn't finished starting. "
                           "Nothing that needs a position will work yet.",
                           {"Normal for about 30 s after power-on: keep the aircraft still",
                            "If it lasts more than 2 minutes, reboot the autopilot"},
                           detail));
        return;  // the other flags are meaningless until it's running
    }
    if (!(f & kEkfAttitude)) {
        out.push_back(make("ekf.attitude", Severity::Critical, "Attitude estimate unhealthy",
                           "The autopilot can't trust which way is up. It can't stabilise the aircraft reliably.",
                           airborne ? std::vector<std::string>{"Land as soon as possible (QLAND/Land)",
                                                               "Be ready to take over on the RC transmitter"}
                                    : std::vector<std::string>{"Don't take off",
                                                               "Keep it still on level ground and wait",
                                                               "Check the IMU calibration and vibration damping"},
                           detail));
    }
    const bool pos_ok = (f & kEkfVelHoriz) && (f & kEkfPosHorizAbs) && !(f & kEkfConstPosMode);
    if (!pos_ok) {
        if (airborne) {
            out.push_back(make("ekf.position", Severity::Critical, "Position estimate lost",
                               "The autopilot no longer trusts where it is. AUTO, Guide, Hold and QRTL depend on it "
                               "and may drift or fail.",
                               {"Be ready to take manual control on the RC transmitter",
                                "Bring it back by line of sight and land",
                                "Afterwards: check GPS placement and interference from the video transmitter"},
                               detail));
        } else {
            out.push_back(make("ekf.position", Severity::Warning, "Position not ready",
                               "The autopilot doesn't trust its GPS position yet, so it can't arm for AUTO, Guide "
                               "or QRTL.",
                               {"Wait for GPS lock: clear view of the sky, usually 30-60 s",
                                "Keep the video transmitter and metal away from the GPS antenna",
                                "Check the GPS card for satellite count"},
                               detail));
        }
    }
    if (!(f & kEkfPosVertAbs) || !(f & kEkfVelVert)) {
        out.push_back(make("ekf.height", airborne ? Severity::Critical : Severity::Warning,
                           "Height estimate not ready",
                           "The autopilot doesn't trust its altitude (barometer and GPS height disagree or aren't "
                           "ready).",
                           {"Wait a minute after power-on", "Shield the barometer from wind and sunlight",
                            "Don't fly if it doesn't clear"},
                           detail));
    }
    if (f & kEkfGpsGlitching) {
        out.push_back(make("ekf.glitch", airborne ? Severity::Critical : Severity::Warning, "GPS glitch",
                           "The GPS position jumped in a way that doesn't match the aircraft's motion, so the "
                           "autopilot is ignoring it for now.",
                           {"Usually reflections (buildings, metal) or interference: move to open ground",
                            airborne ? "Watch the aircraft: navigation may drift until GPS recovers"
                                     : "Wait for it to clear before taking off"},
                           detail));
    }
    // innovation ratios: how well each sensor agrees with the filter
    struct Var {
        const char* id;
        double v;
        const char* title;
        const char* cause;
        std::vector<std::string> todo;
    };
    const Var vars[] = {
        {"ekf.var.mag", s.ekf_mag_var, "Compass disagrees with motion",
         "The compass heading doesn't match how the aircraft is actually moving: usually magnetic interference.",
         {"On the ground: move away from metal, cars and power cables", "Recalibrate the compass",
          "In flight: high current near the compass can cause this; keep an eye on heading"}},
        {"ekf.var.pos", s.ekf_pos_var, "GPS position inconsistent",
         "GPS position and the inertial sensors disagree more than usual.",
         {"Check satellite count and HDOP", "Move away from buildings or tall trees"}},
        {"ekf.var.vel", s.ekf_vel_var, "GPS velocity inconsistent",
         "GPS speed and the inertial sensors disagree more than usual.",
         {"Usually poor GPS reception or high vibration", "Check vibration damping of the flight controller"}},
        {"ekf.var.hgt", s.ekf_hgt_var, "Altitude sources disagree",
         "Barometer and GPS altitude disagree more than usual.",
         {"Shield the barometer from airflow and sunlight", "Can happen briefly in fast climbs or descents"}},
    };
    for (const auto& v : vars) {
        if (v.v >= kEkfVarCaution) {
            out.push_back(make(v.id, v.v >= kEkfVarBad ? Severity::Warning : Severity::Caution, v.title,
                               std::string(v.cause) + fmt(" (ratio %.2f; under 0.5 is good)", v.v), v.todo, detail));
        }
    }
}

void assessGps(const VehicleState& s, bool airborne, std::vector<Advisory>& out) {
    if (!s.has_gps) {
        out.push_back(make("gps.none", Severity::Warning, "No GPS data",
                           "The autopilot isn't reporting any GPS. Missions, Guide, Hold and QRTL need it.",
                           {"Check the GPS is plugged in and powered", "Check GPS_TYPE is set"}));
        return;
    }
    char sats[96];
    std::snprintf(sats, sizeof(sats), "fix type %d · %d satellites · HDOP %.1f", s.gps_fix, s.gps_sats, s.gps_hdop);
    if (s.gps_fix < 3) {
        out.push_back(make("gps.fix", airborne ? Severity::Critical : Severity::Warning, "No 3D GPS fix",
                           airborne ? "GPS lock was lost in flight. Position-based modes may stop working."
                                    : "The GPS hasn't locked on yet. The vehicle won't arm for missions without it.",
                           {"Give the antenna a clear view of the sky", "Wait: a cold start takes up to a few minutes",
                            "Keep the video transmitter away from the GPS"},
                           sats));
    } else if (s.gps_sats < 6) {
        out.push_back(make("gps.sats", Severity::Caution, "Few GPS satellites",
                           "Few satellites are in use, so the position is less accurate.",
                           {"Wait a little longer before taking off", "Move away from obstructions"}, sats));
    }
    if (s.gps_fix >= 3 && s.gps_hdop > 2.0) {
        out.push_back(make("gps.hdop", Severity::Caution, "Poor GPS geometry",
                           fmt("Satellites are bunched up in the sky (HDOP %.1f; under 1.5 is good), so position is "
                               "less precise.",
                               s.gps_hdop),
                           {"Usually improves within a minute or two", "Avoid flying close to obstacles until it does"},
                           sats));
    }
}

}  // namespace

const char* severityName(Severity s) {
    switch (s) {
    case Severity::Info: return "INFO";
    case Severity::Caution: return "CHECK";
    case Severity::Warning: return "FAULT";
    case Severity::Critical: return "CRITICAL";
    }
    return "";
}

std::vector<Advisory> assess(const VehicleState& s, Clock::time_point now, bool ever_connected) {
    std::vector<Advisory> out;
    if (!s.error.empty()) {
        out.push_back(make("link.error", Severity::Critical, "Ground station network error",
                           "The GCS can't use its UDP socket, so no telemetry can arrive.",
                           {"Check no other program (Mission Planner, QGC, mavproxy) is bound to the same port",
                            "Check the --udp host:port setting"},
                           s.error));
        return out;
    }
    const bool alive = linkAlive(s, now);
    if (!alive) {
        if (ever_connected) {
            out.push_back(make("link.lost", Severity::Critical, "Link lost",
                               fmt("No heartbeat from the vehicle for %.0f s. It keeps flying its current mode; its "
                                   "own failsafe takes over after the configured timeout.",
                                   heartbeatAge(s, now)),
                               {"Check the telemetry radio and antenna orientation",
                                "Get line of sight to the aircraft (move the ground antenna up)",
                                "If it doesn't recover, the aircraft follows its failsafe settings (FS_LONG_ACTN)"}));
        } else {
            out.push_back(make("link.wait", Severity::Info, "Waiting for the vehicle",
                               "No heartbeat yet.",
                               {"Power the vehicle and its telemetry radio",
                                "Check the vehicle sends MAVLink to this GCS's UDP port"}));
        }
        return out;  // everything else is stale
    }

    const bool airborne = isAirborne(s);
    assessEkf(s, airborne, out);
    assessGps(s, airborne, out);

    if (s.has_sys_status) {
        for (const auto& si : sensorTable()) {
            const bool present = s.sensors_present & si.bit;
            const bool enabled = s.sensors_enabled & si.bit;
            const bool healthy = s.sensors_health & si.bit;
            if (present && enabled && !healthy) {
                std::vector<std::string> todo(si.suggestions.begin(), si.suggestions.end());
                char d[64];
                std::snprintf(d, sizeof(d), "SYS_STATUS sensor bit 0x%x", si.bit);
                Severity sev = si.severity;
                if (airborne && sev == Severity::Warning &&
                    (si.bit == 0x1 || si.bit == 0x2 || si.bit == 0x10000 || si.bit == 0x8000)) {
                    sev = Severity::Critical;  // gyro / accel / RC / motors in the air
                }
                out.push_back(make(si.id, sev, si.title, si.cause, std::move(todo), d));
            }
        }
        if (!s.armed && (s.sensors_present & kPrearmBit) && !(s.sensors_health & kPrearmBit)) {
            out.push_back(make("prearm", Severity::Warning, "Pre-arm checks failing",
                               "The autopilot will refuse to arm. It sends the reason as a \"PreArm\" message "
                               "every 30 s; those appear as their own cards here.",
                               {"Fix the PreArm card(s) below",
                                "Some checks depend on the current mode (e.g. \"QRTL mode not armable\"): Launch "
                                "switches mode first, so those don't block it"}));
        }
    }

    if (s.battery_pct >= 0) {
        if (s.battery_pct < 15) {
            out.push_back(make("battery.critical", Severity::Critical, "Battery critical",
                               fmt("%.0f%% left. The pack may sag below what the lift rotors need for a vertical "
                                   "landing.",
                                   s.battery_pct),
                               {"Land now: Land (if a landing point is set) or QRTL",
                                "Avoid long hovers: the lift rotors draw the most current"}));
        } else if (s.battery_pct < 30) {
            out.push_back(make("battery.low", Severity::Warning, "Battery low",
                               fmt("%.0f%% left. Keep enough for the vertical landing.", s.battery_pct),
                               {"Head back and land soon", "Plan for about 1-2 minutes of rotor time to land"}));
        }
    }

    if (!s.has_home) {
        out.push_back(make("home.none", Severity::Caution, "No home position yet",
                           "Home is set once GPS has a fix. Launch, Land and QRTL need it.",
                           {"Wait for GPS lock"}));
    }
    if (s.q_guided_mode == 1) {
        out.push_back(make("param.q_guided_mode", Severity::Warning, "Guide and Hold would hover on the rotors",
                           "Q_GUIDED_MODE is 1, so GUIDED targets end in a rotor hover instead of a wing-borne "
                           "circle. That uses lots of battery and breaks the \"rotors only for takeoff and landing\" "
                           "rule.",
                           {"Set Q_GUIDED_MODE = 0 in your parameter editor (Mission Planner / QGroundControl)",
                            "No reboot needed"},
                           "Q_GUIDED_MODE = 1"));
    }
    if (liftRotorState(s, now) == LiftRotors::Assist) {
        out.push_back(make("rotors.assist", Severity::Caution, "Lift rotors assisting in cruise",
                           "Q_ASSIST is spinning the lift rotors to help the wing: airspeed is low, the bank is "
                           "steep, or altitude is low. Normal for a moment in gusts or tight turns.",
                           {"If it lasts: fly faster (raise AIRSPEED_CRUISE) or use a larger orbit radius",
                            "Check the airspeed sensor reads sensibly"}));
    }

    std::stable_sort(out.begin(), out.end(),
                     [](const Advisory& a, const Advisory& b) { return a.severity > b.severity; });
    return out;
}

// ---- STATUSTEXT -----------------------------------------------------------------------

namespace {

std::string lower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return s;
}

bool has(const std::string& l, const char* needle) {
    return l.find(needle) != std::string::npos;
}

struct Pattern {
    std::vector<const char*> any;  // lower-case substrings; any one matches
    const char* id;
    Severity severity;
    const char* title;
    const char* cause;
    std::vector<const char*> todo;
    bool prearm_only = false;  // the words also appear in normal status lines
};

const std::vector<Pattern>& patterns() {
    // order matters: the first match wins, so specific before generic
    static const std::vector<Pattern> t = {
        {{"vtol land too short"}, "txt.vtol_land_short", Severity::Warning, "Landing leg too short",
         "A waypoint right before the VTOL landing is too close to it for the transition.",
         {"Re-sync the plan from this GCS: it puts an approach circle before the landing",
          "Or move the last waypoint further from the landing point"}},
        {{"not armable"}, "txt.mode_not_armable", Severity::Caution, "Can't arm in this mode",
         "The autopilot doesn't allow arming in the current mode.",
         {"Launch switches to AUTO/GUIDED first, so this usually doesn't matter",
          "To arm manually, switch to QSTABILIZE, QHOVER or QLOITER"}, true},
        {{"hardware safety switch", "safety switch"}, "txt.safety", Severity::Warning, "Safety switch not pressed",
         "The hardware safety switch on the GPS/flight controller is still engaged.",
         {"Press and hold the safety button until it lights solid",
          "Or disable it with BRD_SAFETY_DEFLT = 0 if your setup has no switch"}, true},
        {{"compass not calibrated", "compasses not calibrated", "compass calibration", "compass offsets"},
         "txt.compass_cal", Severity::Warning, "Compass needs calibration",
         "The compass hasn't been calibrated (or its offsets are out of range), so heading can't be trusted.",
         {"Run the compass calibration (Mission Planner: Setup > Mandatory > Compass)",
          "Calibrate outdoors, away from cars and metal"}},
        {{"compasses inconsistent", "inconsistent compasses"}, "txt.compass_inconsistent", Severity::Warning,
         "Compasses disagree",
         "Two compasses point in different directions: one is affected by interference or mis-oriented.",
         {"Recalibrate all compasses", "Check COMPASS_ORIENT for an external compass",
          "Disable the internal compass if it sits near power wiring"}},
        {{"check mag field", "mag field"}, "txt.mag_field", Severity::Warning, "Magnetic field abnormal",
         "The measured magnetic field strength is unusual: nearby metal or magnets.",
         {"Move away from cars, steel structures and power cables", "Recalibrate the compass on site"}},
        {{"accels inconsistent", "accels not healthy", "accel calibration", "accels calibrated", "3d accel"},
         "txt.accel", Severity::Warning, "Accelerometer problem",
         "The accelerometers disagree or need calibration.",
         {"Let the board warm up a minute and reboot", "Re-run the accelerometer calibration",
          "Check the flight controller is firmly mounted"}},
        {{"gyros inconsistent", "gyros not healthy", "gyro cal", "gyros not calibrated"}, "txt.gyro",
         Severity::Warning, "Gyroscope problem", "The gyros disagree or failed to calibrate at start-up.",
         {"Reboot with the aircraft completely still (gyros calibrate at boot)"}},
        {{"throttle"}, "txt.throttle", Severity::Warning, "Throttle stick not low",
         "The autopilot won't arm while the RC throttle is above minimum.",
         {"Put the throttle stick fully down", "Check the throttle channel isn't reversed (RC3_REVERSED)"}, true},
        {{"rc not calibrated", "rc not found", "rc3_min", "rc3_max", "radio failsafe", "rc failsafe",
          "rc not healthy"},
         "txt.rc", Severity::Warning, "RC transmitter problem",
         "The radio transmitter isn't detected or isn't calibrated.",
         {"Turn on the transmitter and check it's bound", "Run the radio calibration",
          "Check the receiver wiring / protocol (RC_PROTOCOLS)"}},
        {{"airspeed"}, "txt.airspeed", Severity::Warning, "Airspeed sensor problem",
         "The airspeed sensor isn't healthy or reads implausibly.",
         {"Remove the pitot cover and check the tubing", "Re-run the airspeed calibration",
          "Check the sensor cable"}},
        {{"need 3d fix", "bad fix", "gps 1: ", "gps: ", "gps and ahrs differ", "gps blending", "gps glitch",
          "gps not healthy", "need position estimate", "gps positions differ"},
         "txt.gps", Severity::Warning, "GPS not ready",
         "The GPS doesn't have a good enough fix, or it disagrees with the autopilot's estimate.",
         {"Wait for more satellites with a clear view of the sky",
          "Keep the video transmitter and metal away from the GPS"}},
        {{"ekf variance", "ekf failsafe", "ekf primary changed", "ekf3 lane switch"}, "txt.ekf_failsafe",
         Severity::Critical, "Navigation filter trouble",
         "The estimator's sensors disagree badly (or it switched to a backup), and the autopilot may take its EKF "
         "failsafe action.",
         {"Watch the aircraft: position-based modes may misbehave", "Be ready to take manual control",
          "Afterwards: check compass calibration, GPS and vibration in the log"}},
        {{"waiting for home", "ahrs: ", "ekf", "ahrs not healthy", "yaw inconsistent", "attitude is bad"},
         "txt.ekf", Severity::Warning, "Navigation filter not ready",
         "The autopilot's estimator (EKF/AHRS) isn't ready or isn't confident yet.",
         {"Wait 30-60 s after power-on with the aircraft still",
          "If it mentions yaw: recalibrate the compass and keep away from metal"}},
        {{"baro", "barometer"}, "txt.baro", Severity::Warning, "Barometer problem",
         "The barometer (altitude sensor) isn't healthy or disagrees with the others.",
         {"Shield it from wind and sunlight", "Reboot the autopilot"}},
        {{"battery", "low voltage", "batt"}, "txt.battery", Severity::Warning, "Battery problem",
         "The battery is below its configured minimum or the battery monitor is faulty.",
         {"Charge or swap the pack", "Check BATT_ARM_VOLT / BATT_LOW_VOLT suit your pack"}},
        {{"fence"}, "txt.fence", Severity::Warning, "Geofence problem",
         "The geofence is breached or isn't set up correctly.",
         {"Check the fence in your ground station", "Make sure home is inside the fence"}},
        {{"logging", "no sd card", "sd card"}, "txt.logging", Severity::Caution, "Logging problem",
         "The flight log can't be written.",
         {"Insert or reformat the SD card", "Disable the check (ARMING_CHECK) only if you accept no logs"}, true},
        {{"q_frame", "frame class", "motors: "}, "txt.frame", Severity::Warning, "VTOL motor setup problem",
         "The lift motor configuration isn't valid.",
         {"Check Q_ENABLE, Q_FRAME_CLASS and Q_FRAME_TYPE", "Check the SERVOx_FUNCTION motor assignments"}, true},
        {{"board voltage", "board ("}, "txt.board_voltage", Severity::Warning, "Flight controller voltage",
         "The flight controller's supply voltage is out of range.",
         {"Check the power module / BEC", "Don't fly on an unstable supply"}},
        {{"internal error"}, "txt.internal", Severity::Warning, "Autopilot internal error",
         "The firmware reported an internal error.",
         {"Reboot the autopilot", "If it repeats, download the log and report it"}},
        {{"failsafe", "short event", "long event"}, "txt.failsafe", Severity::Critical, "Failsafe triggered",
         "The autopilot triggered a failsafe (lost RC or GCS link, or low battery) and is taking its configured "
         "action.",
         {"Watch what mode it switched to", "Restore the link / RC if you can",
          "Take manual control on the RC transmitter if needed"}},
    };
    return t;
}

}  // namespace

std::optional<Advisory> explainStatusText(const std::string& text, int severity) {
    const std::string l = lower(text);
    const bool prearm = l.rfind("prearm", 0) == 0 || l.rfind("arm:", 0) == 0;
    // only problems: severity <= WARNING (4), or any PreArm line
    if (!prearm && severity > 4) {
        return std::nullopt;
    }
    // "Throttle failsafe off", "GPS glitch cleared", "EKF ... recovered": good news
    if (!prearm) {
        for (const char* ok : {" off", "cleared", "restored", "resolved", "recovered", " ok"}) {
            const auto at = l.rfind(ok);
            if (at != std::string::npos && at + std::char_traits<char>::length(ok) == l.size()) {
                return std::nullopt;
            }
        }
    }
    for (const auto& p : patterns()) {
        if (p.prearm_only && !prearm) {
            continue;
        }
        for (const char* n : p.any) {
            if (has(l, n)) {
                Advisory a;
                a.id = p.id;
                a.severity = p.severity;
                a.title = p.title;
                a.cause = p.cause;
                for (const char* s : p.todo) {
                    a.suggestions.emplace_back(s);
                }
                a.detail = text;
                if (prearm) {
                    a.id = "prearm." + a.id;
                    a.title = "PreArm: " + a.title;
                    if (a.severity == Severity::Critical) {
                        a.severity = Severity::Warning;
                    }
                }
                return a;
            }
        }
    }
    if (prearm) {
        // unknown PreArm reason: still a card, with the autopilot's words
        Advisory a;
        a.id = "prearm.other:" + l;
        a.severity = Severity::Warning;
        a.title = "PreArm check failing";
        a.cause = "The autopilot refuses to arm: \"" + text + "\"";
        a.suggestions = {"Search the message in the ArduPilot docs (\"pre-arm safety checks\")"};
        a.detail = text;
        return a;
    }
    return std::nullopt;
}

std::optional<Advisory> AdvisoryBook::onStatusText(const std::string& text, int severity, Clock::time_point now) {
    auto adv = explainStatusText(text, severity);
    if (!adv) {
        return std::nullopt;
    }
    for (auto& e : entries_) {
        if (e.adv.id == adv->id) {
            e.last = now;
            e.adv.detail = adv->detail;
            return std::nullopt;  // a repeat
        }
    }
    const bool prearm = adv->id.rfind("prearm.", 0) == 0;
    entries_.push_back({*adv, now, prearm});
    return adv;
}

void AdvisoryBook::update(const VehicleState& s, Clock::time_point now) {
    const bool armed = s.armed;
    entries_.erase(std::remove_if(entries_.begin(), entries_.end(),
                                  [&](const Entry& e) {
                                      if (e.prearm && armed && !was_armed_) {
                                          return true;  // it armed: pre-arm reasons are moot
                                      }
                                      return std::chrono::duration<double>(now - e.last).count() > kStickyLife;
                                  }),
                   entries_.end());
    was_armed_ = armed;
}

std::vector<Advisory> AdvisoryBook::current() const {
    std::vector<Advisory> out;
    out.reserve(entries_.size());
    for (const auto& e : entries_) {
        out.push_back(e.adv);
    }
    return out;
}

std::vector<Advisory> mergeAdvisories(std::vector<Advisory> a, const std::vector<Advisory>& b) {
    for (const auto& x : b) {
        const bool dup = std::any_of(a.begin(), a.end(), [&](const Advisory& y) { return y.id == x.id; });
        if (!dup) {
            a.push_back(x);
        }
    }
    std::stable_sort(a.begin(), a.end(), [](const Advisory& x, const Advisory& y) { return x.severity > y.severity; });
    return a;
}

}  // namespace gcs
