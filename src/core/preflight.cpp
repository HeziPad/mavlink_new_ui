#include "preflight.hpp"

#include "geo.hpp"
#include "mission.hpp"
#include "plane_modes.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <sstream>

namespace gcs::preflight {

const char* stageName(Stage s) {
    switch (s) {
    case Stage::Look: return "Look at the aircraft";
    case Stage::Connection: return "Connection";
    case Stage::Battery: return "Battery";
    case Stage::Sensors: return "Sensors and position";
    case Stage::Setup: return "Aircraft settings";
    case Stage::Camera: return "Camera";
    case Stage::Plan: return "Plan";
    case Stage::Moving: return "Moving parts";
    case Stage::Final: return "Ready to arm";
    case Stage::Count: break;
    }
    return "";
}

const char* kindName(Kind k) {
    switch (k) {
    case Kind::Auto: return "AUTO";
    case Kind::Test: return "TEST";
    case Kind::You: return "CHECK";
    }
    return "";
}

const char* statusName(Status s) {
    switch (s) {
    case Status::Pending: return "pending";
    case Status::Waiting: return "waiting";
    case Status::Pass: return "pass";
    case Status::Warn: return "warn";
    case Status::Fail: return "fail";
    case Status::Skipped: return "skipped";
    case Status::NotApplicable: return "n/a";
    }
    return "";
}

bool isDone(Status s) {
    return s == Status::Pass || s == Status::Warn || s == Status::Skipped || s == Status::NotApplicable;
}

// ---- catalogue -------------------------------------------------------------------------

const std::vector<StepDef>& steps() {
    // id, stage, kind, title, sentence (while it runs), how (when it isn't right), critical, turnaround, wait
    static const std::vector<StepDef> list = {
        // ---- the look card: only a person can see these
        {"air.props", Stage::Look, Kind::You, "Propellers undamaged and tight", "",
         "Replace a cracked or chipped propeller and tighten any loose one.", true, true, 0},
        {"air.pitot", Stage::Look, Kind::You, "Pitot cover off", "", "Take the cover off the pitot tube.", true, true, 0},
        {"air.battery", Stage::Look, Kind::You, "Battery strapped in and plugged in", "",
         "Strap the battery down and push the connector fully in.", true, true, 0},
        {"site.area", Stage::Look, Kind::You, "10 m clear around it, nobody close", "",
         "Move people and loose things at least 10 m away.", true, true, 0},
        {"sens.pitot", Stage::Look, Kind::Test, "Blow gently into the pitot tube", "",
         "The airspeed didn't rise: check the pitot tube and its hoses for a blockage or a leak.", true, true, 0},
        // ---- connection
        {"profile.vehicle", Stage::Connection, Kind::Auto, "Aircraft type and wiring",
         "Reading the aircraft's type and how its motors and servos are wired",
         "The aircraft's outputs aren't set up for a QuadPlane: needs a technician.", true, true, 30},
        {"link.telemetry", Stage::Connection, Kind::Auto, "Telemetry link", "Checking the radio link",
         "Check the telemetry radio and its antennas.", true, true, 10},
        // ---- battery
        {"power.voltage", Stage::Battery, Kind::Auto, "Battery charged", "Checking the battery is charged",
         "Fit a fully charged battery.", true, true, 10},
        {"power.balance", Stage::Battery, Kind::Auto, "Battery cells even", "Checking the battery's cells are even",
         "One cell is lower than the others: balance-charge the battery, or use another.", false, true, 10},
        {"power.current", Stage::Battery, Kind::Auto, "Current sensor", "Checking the battery current reading",
         "The battery current isn't measured right, so battery time left is a guess: needs a technician.", false,
         false, 10},
        {"power.board", Stage::Battery, Kind::Auto, "Autopilot power", "Checking the autopilot's power supply",
         "The autopilot's supply voltage is out of range: needs a technician.", true, false, 10},
        // ---- sensors
        {"sens.health", Stage::Sensors, Kind::Auto, "Sensors healthy", "Checking every sensor reports healthy",
         "A sensor isn't working.", true, true, 20},
        {"sens.ekf", Stage::Sensors, Kind::Auto, "Position estimate", "Waiting for the autopilot to know where it is",
         "The autopilot isn't sure where it is. Keep it still, away from metal and power lines, and wait.", true, true,
         90},
        {"sens.gps", Stage::Sensors, Kind::Auto, "GPS", "Waiting for a good GPS fix",
         "Not enough satellites yet: give it a clear view of the sky and wait.", true, true, 90},
        {"sens.home", Stage::Sensors, Kind::Auto, "Home position", "Checking home is set where the aircraft is",
         "Home isn't where the aircraft is: restart the autopilot here.", true, true, 60},
        {"sens.vibe", Stage::Sensors, Kind::Auto, "Vibration", "Checking for vibration at rest",
         "High vibration at rest: check the autopilot's mounting.", false, false, 10},
        {"sens.logging", Stage::Sensors, Kind::Auto, "Flight log", "Checking the flight log is recording",
         "The flight log isn't recording: check the autopilot's SD card.", false, false, 8},
        // ---- settings
        {"setup.app", Stage::Setup, Kind::Auto, "Aircraft settings", "Checking the aircraft is set up for this app",
         "Some of the aircraft's settings aren't right for this app.", true, false, 30},
        {"setup.battery", Stage::Setup, Kind::Auto, "Battery warning levels",
         "Checking the low-battery warnings match this battery",
         "The battery warning levels on the aircraft don't match this battery: ask a technician to set them.", false,
         false, 30},
        // ---- camera
        {"pay.video", Stage::Camera, Kind::Auto, "Video", "Checking the video from the camera",
         "No video: check the camera and the video link.", false, true, 8},
        {"pay.gimbal", Stage::Camera, Kind::Test, "Gimbal", "Moving the gimbal and watching it follow",
         "The gimbal didn't follow: check it's powered and nothing blocks it.", false, false, 20},
        // ---- plan
        {"mis.synced", Stage::Plan, Kind::Auto, "Plan on the aircraft", "Reading the plan on the aircraft",
         "Open the plan and Sync it.", true, true, 30},
        {"mis.landing", Stage::Plan, Kind::Auto, "Landing", "Checking where it lands",
         "Open the plan and Sync it: that adds the landing.", true, true, 30},
        {"mis.alts", Stage::Plan, Kind::Auto, "Plan heights", "Checking the plan's heights",
         "Check the waypoint heights in the plan.", false, true, 30},
        {"mis.range", Stage::Plan, Kind::Auto, "Plan distances", "Checking how far the plan goes",
         "Make a new plan here and Sync it.", true, true, 30},
        {"mis.endurance", Stage::Plan, Kind::Auto, "Battery for the plan", "Checking the battery lasts the plan",
         "Shorten the plan.", false, true, 30},
        // ---- moving parts
        {"surf.roll", Stage::Moving, Kind::Test, "Ailerons", "Moving the ailerons",
         "An aileron didn't move: check its servo and wiring.", true, false, 30},
        {"surf.pitch", Stage::Moving, Kind::Test, "Elevator", "Moving the elevator",
         "The elevator didn't move: check its servo and wiring.", true, false, 30},
        {"surf.yaw", Stage::Moving, Kind::Test, "Rudder", "Moving the rudder",
         "The rudder didn't move: check its servo and wiring.", false, false, 30},
        {"mot.lift", Stage::Moving, Kind::Test, "Lift motors", "Spinning each lift motor in turn",
         "Check the lift motors, their wiring and their speed controllers.", true, false, 90},
        {"mot.pusher", Stage::Moving, Kind::Test, "Pusher motor", "Spinning the pusher motor",
         "Check the pusher motor, its wiring and its speed controller.", true, false, 40},
        // ---- final
        {"fin.ready", Stage::Final, Kind::Auto, "Ready to arm", "Asking the autopilot whether it's ready to arm",
         "The autopilot isn't ready to arm.", true, true, 30},
    };
    return list;
}

const StepDef* findStep(const std::string& id) {
    for (const auto& s : steps()) {
        if (s.id == id) {
            return &s;
        }
    }
    return nullptr;
}

// ---- profile ---------------------------------------------------------------------------

double fullCellVolts(Chemistry c) {
    switch (c) {
    case Chemistry::LiHV: return 4.30;
    case Chemistry::LiIon: return 4.05;
    case Chemistry::LiPo: break;
    }
    return 4.10;
}
const char* chemistryName(Chemistry c) {
    switch (c) {
    case Chemistry::LiHV: return "LiHV";
    case Chemistry::LiIon: return "Li-ion";
    case Chemistry::LiPo: break;
    }
    return "LiPo";
}
double storageCellVolts(Chemistry c) { return c == Chemistry::LiIon ? 3.70 : 3.85; }
double lowCellVolts(Chemistry c) { return c == Chemistry::LiIon ? 3.30 : 3.50; }
double criticalCellVolts(Chemistry c) { return c == Chemistry::LiIon ? 3.10 : 3.30; }

int guessCells(double v, Chemistry c) {
    if (v < 2.5) {
        return 0;
    }
    // the cell count whose per-cell voltage is in the chemistry's range
    const double hi = c == Chemistry::LiHV ? 4.40 : (c == Chemistry::LiIon ? 4.25 : 4.25);
    for (int n = 1; n <= 24; ++n) {
        if (v / n <= hi) {
            return n;
        }
    }
    return 0;
}

// ---- helpers ------------------------------------------------------------------------------

namespace {

std::string f1(double v) {
    char b[32];
    std::snprintf(b, sizeof b, "%.1f", v);
    return b;
}
std::string f2(double v) {
    char b[32];
    std::snprintf(b, sizeof b, "%.2f", v);
    return b;
}

std::optional<float> param(const VehicleState& s, const std::string& n) {
    const auto it = s.params.find(n);
    if (it == s.params.end()) {
        return std::nullopt;
    }
    return it->second;
}

bool sensorOk(const VehicleState& s, uint32_t bit) {
    return !(s.sensors_present & bit) || !(s.sensors_enabled & bit) || (s.sensors_health & bit);
}

Eval ev(Status st, std::string d) {
    return {st, std::move(d)};
}

}  // namespace

OutputMap outputMap(const VehicleState& s, int lift_motors) {
    OutputMap m;
    m.lift.assign(static_cast<size_t>(std::max(0, lift_motors)), 0);
    for (int ch = 1; ch <= 16; ++ch) {
        const int f = s.servo_function[ch - 1];
        if (f >= 33 && f <= 40 && f - 32 <= lift_motors) {
            m.lift[f - 33] = ch;
        }
        switch (f) {
        case 70: m.throttle = ch; break;
        case 4:
            if (!m.aileron) m.aileron = ch;
            m.ailerons.push_back(ch);
            break;
        case 19: m.elevator = ch; break;
        case 21: m.rudder = ch; break;
        case 77: m.elevon_l = ch; break;
        case 78: m.elevon_r = ch; break;
        case 79: m.vtail_l = ch; break;
        case 80: m.vtail_r = ch; break;
        default: break;
        }
    }
    return m;
}

// ---- evaluation -------------------------------------------------------------------------------

std::optional<Eval> evaluate(const std::string& id, const VehicleState& s, const Profile& p, Clock::time_point now) {
    const bool alive = linkAlive(s, now);
    if (!alive && findStep(id) && findStep(id)->kind == Kind::Auto) {
        return ev(Status::Waiting, "no link to the aircraft");
    }

    if (id == "profile.vehicle") {
        const auto fc = param(s, "Q_FRAME_CLASS"), ft = param(s, "Q_FRAME_TYPE"), qe = param(s, "Q_ENABLE");
        if (!s.has_version || !fc || !ft || !s.servo_map_final) {
            return ev(Status::Waiting, "reading firmware, frame and outputs…");
        }
        if (qe && *qe < 0.5f) {
            return ev(Status::Fail, "Q_ENABLE is 0: this vehicle isn't set up as a QuadPlane");
        }
        const FrameLayout f = frameLayout(static_cast<int>(*fc), static_cast<int>(*ft));
        const OutputMap m = outputMap(s, static_cast<int>(f.motors.size()));
        const uint32_t v = s.fw_version;
        std::string d = "ArduPlane " + std::to_string((v >> 24) & 0xff) + "." + std::to_string((v >> 16) & 0xff) +
                        "." + std::to_string((v >> 8) & 0xff) + " · " + f.name;
        const int missing = static_cast<int>(std::count(m.lift.begin(), m.lift.end(), 0));
        if (missing > 0) {
            return ev(Status::Fail, d + " · " + std::to_string(missing) + " lift motor output(s) not assigned");
        }
        if (!m.throttle) {
            return ev(Status::Fail, d + " · no pusher (throttle) output");
        }
        const bool roll = m.aileron || (m.elevon_l && m.elevon_r);
        const bool pitch = m.elevator || (m.elevon_l && m.elevon_r) || (m.vtail_l && m.vtail_r);
        if (!roll || !pitch) {
            return ev(Status::Fail, d + " · a roll or pitch surface output is missing");
        }
        if (!f.known) {
            return ev(Status::Warn, d + " · unusual frame: check the motor pictures carefully");
        }
        return ev(Status::Pass, d + " · outputs complete");
    }

    if (id == "power.voltage") {
        if (!s.has_battery || s.battery_v <= 0.0) {
            return ev(Status::Waiting, "no battery voltage");
        }
        if (p.cells <= 0) {
            return ev(Status::Waiting, "reading the battery (" + f2(s.battery_v) + " V)");
        }
        const double per = s.battery_v / p.cells;
        const std::string d = f1(s.battery_v) + " V, " + std::to_string(p.cells) + " cells " + chemistryName(p.chemistry) +
                              " (" + f2(per) + " V each, full is " + f2(fullCellVolts(p.chemistry)) + ")";
        if (per >= fullCellVolts(p.chemistry)) {
            return ev(Status::Pass, d);
        }
        if (per >= storageCellVolts(p.chemistry)) {
            return ev(Status::Warn, "not fully charged: " + d);
        }
        return ev(Status::Fail, "not charged: " + d);
    }
    if (id == "power.balance") {
        if (s.cell_v.size() < 2) {
            return ev(Status::NotApplicable, "the battery monitor doesn't report cells");
        }
        const auto [lo, hi] = std::minmax_element(s.cell_v.begin(), s.cell_v.end());
        const double spread = *hi - *lo;
        const std::string d = std::to_string(s.cell_v.size()) + " cells, " + f2(*lo) + "-" + f2(*hi) + " V";
        return ev(spread <= 0.05 ? Status::Pass : (spread <= 0.10 ? Status::Warn : Status::Fail),
                  d + " (spread " + f2(spread) + " V)");
    }
    if (id == "power.current") {
        if (s.battery_a < 0.0) {
            return ev(Status::Warn, "no current reading: capacity used can't be tracked");
        }
        if (s.battery_a > 8.0 && !s.armed) {
            return ev(Status::Warn, f1(s.battery_a) + " A at rest: high, check the current sensor calibration");
        }
        return ev(Status::Pass, f1(s.battery_a) + " A at rest");
    }
    if (id == "power.board") {
        if (!s.has_power) {
            return ev(Status::NotApplicable, "the board doesn't report its supply");
        }
        const std::string d = f2(s.board_vcc) + " V";
        if (s.board_vcc >= 4.8 && s.board_vcc <= 5.4) {
            return ev(Status::Pass, d);
        }
        return ev(s.board_vcc >= 4.6 && s.board_vcc <= 5.6 ? Status::Warn : Status::Fail, d + " (want 4.8-5.4 V)");
    }

    if (id == "link.telemetry") {
        std::string d = f1(s.attitude_hz) + " Hz attitude";
        if (s.attitude_hz < 4.0) {
            return ev(Status::Warn, d + ": slow link");
        }
        if (s.has_radio) {
            const int margin = std::min(s.radio_rssi - s.radio_noise, s.radio_remrssi - s.radio_remnoise);
            d += " · radio margin " + std::to_string(margin) + " (local " + std::to_string(s.radio_rssi) + "/" +
                 std::to_string(s.radio_noise) + ", remote " + std::to_string(s.radio_remrssi) + "/" +
                 std::to_string(s.radio_remnoise) + ")";
            if (margin < 20) {
                return ev(Status::Warn, d + ": weak at this short range");
            }
        }
        return ev(Status::Pass, d);
    }

    if (id == "sens.health") {
        struct B {
            uint32_t bit;
            const char* name;
        };
        const B bits[] = {{0x1, "gyro"}, {0x2, "accel"}, {0x4, "compass"}, {0x8, "baro"}, {0x10, "airspeed"},
                          {0x20000, "gyro 2"}, {0x40000, "accel 2"}, {0x80000, "compass 2"}, {0x200000, "AHRS"}};
        if (!s.has_sys_status) {
            return ev(Status::Waiting, "no sensor status yet");
        }
        std::string bad;
        for (const auto& b : bits) {
            if (!sensorOk(s, b.bit)) {
                bad += (bad.empty() ? "" : ", ") + std::string(b.name);
            }
        }
        return bad.empty() ? ev(Status::Pass, "all present sensors healthy") : ev(Status::Fail, "unhealthy: " + bad);
    }
    if (id == "sens.ekf") {
        if (!s.has_ekf) {
            return ev(Status::Waiting, "no EKF report yet");
        }
        const double worst = std::max({s.ekf_vel_var, s.ekf_pos_var, s.ekf_hgt_var, s.ekf_mag_var});
        const std::string d = "ratios vel " + f2(s.ekf_vel_var) + " pos " + f2(s.ekf_pos_var) + " hgt " +
                              f2(s.ekf_hgt_var) + " mag " + f2(s.ekf_mag_var);
        if (!ekfHealthy(s)) {
            return ev(Status::Waiting, "EKF not ready · " + d);
        }
        return ev(worst < kEkfVarCaution ? Status::Pass : (worst < kEkfVarBad ? Status::Warn : Status::Fail), d);
    }
    if (id == "sens.gps") {
        if (!s.has_gps) {
            return ev(Status::Waiting, "no GPS data");
        }
        const std::string d = std::to_string(s.gps_sats) + " sats · HDOP " + f1(s.gps_hdop) + " · fix " +
                              std::to_string(s.gps_fix);
        if (s.gps_fix >= 3 && s.gps_sats >= 10 && s.gps_hdop <= 1.2) {
            return ev(Status::Pass, d);
        }
        if (s.gps_fix >= 3 && s.gps_sats >= 6 && s.gps_hdop <= 2.0) {
            return ev(Status::Warn, d + ": usable, better to wait");
        }
        return ev(Status::Waiting, d + ": waiting for a better fix");
    }
    if (id == "sens.home") {
        if (!s.has_home) {
            return ev(Status::Waiting, "home not set yet (needs a GPS fix)");
        }
        if (!s.has_position) {
            return ev(Status::Waiting, "no position");
        }
        const double d = geo::distance(s.lat, s.lon, s.home_lat, s.home_lon);
        const std::string t = "home " + std::to_string(static_cast<int>(std::lround(d))) + " m from the aircraft";
        return ev(d <= 20.0 ? Status::Pass : Status::Fail, d <= 20.0 ? t : t + ": re-set home here (re-power)");
    }
    if (id == "sens.logging") {
        constexpr uint32_t kLog = 0x1000000;
        if (!s.has_sys_status || !(s.sensors_present & kLog)) {
            return ev(Status::NotApplicable, "logging not reported");
        }
        return ev(sensorOk(s, kLog) ? Status::Pass : Status::Fail,
                  sensorOk(s, kLog) ? "logging OK" : "logging failed: check the SD card");
    }
    if (id == "sens.vibe") {
        if (!s.has_vibe) {
            return ev(Status::Waiting, "no vibration data");
        }
        const double m = std::max({s.vibe_x, s.vibe_y, s.vibe_z});
        const std::string d = "x " + f1(s.vibe_x) + " y " + f1(s.vibe_y) + " z " + f1(s.vibe_z) + " m/s²";
        return ev(m < 15.0 ? Status::Pass : (m < 30.0 ? Status::Warn : Status::Fail), d);
    }

    if (id == "pay.video") {
        return std::nullopt;  // the UI knows the video
    }

    if (id.rfind("mis.", 0) == 0) {
        if (!s.mission_known) {
            return ev(Status::Waiting, "mission not read yet");
        }
        const MissionPlan plan = extractPlan(s.mission);
        if (id == "mis.synced") {
            if (plan.wps.empty()) {
                return ev(Status::Pass, plan.land ? "no plan: flown by hand (Fly To / Hold), lands at home"
                                                  : "no plan: flown by hand (Fly To / Hold); Launch puts a "
                                                    "landing at home on board");
            }
            return ev(Status::Pass, std::to_string(plan.wps.size()) + " waypoints on the vehicle");
        }
        if (id == "mis.landing") {
            if (!plan.land) {
                if (s.mission.size() <= 1) {
                    return ev(Status::Pass, "no plan: Launch puts a landing at home on board");
                }
                return ev(Status::Fail, "no landing in the mission: Plan → Sync adds one (at home if there's no L)");
            }
            const bool marker = std::any_of(s.mission.begin(), s.mission.end(),
                                            [](const MissionItem& it) { return it.command == mav::kCmdDoLandStart; });
            if (!marker) {
                return ev(Status::Fail, "no DO_LAND_START: failsafes wouldn't fly the landing. Plan → Sync");
            }
            const bool at_home = s.has_home && geo::distance(plan.land->lat, plan.land->lon, s.home_lat, s.home_lon) < 5.0;
            return ev(Status::Pass, at_home ? "Land and failsafes land at home (no L planned)"
                                            : "Land and failsafes land at L");
        }
        if (id == "mis.alts") {
            if (plan.wps.empty()) {
                return ev(Status::NotApplicable, "no waypoints");
            }
            float lo = 1e9f, hi = -1e9f;
            for (const auto& w : plan.wps) {
                lo = std::min(lo, w.alt_rel);
                hi = std::max(hi, w.alt_rel);
            }
            const std::string d = std::to_string(static_cast<int>(lo)) + "-" + std::to_string(static_cast<int>(hi)) + " m";
            if (lo < 25.0f) {
                return ev(Status::Warn, d + ": a waypoint below 25 m");
            }
            if (hi > 120.0f) {
                return ev(Status::Warn, d + ": above 120 m (check local rules)");
            }
            return ev(Status::Pass, d);
        }
        if (id == "mis.range") {
            if (plan.wps.empty()) {
                return ev(Status::NotApplicable, "no waypoints");
            }
            if (!s.has_home) {
                return ev(Status::Waiting, "needs home");
            }
            double far = 0.0;
            for (const auto& w : plan.wps) {
                far = std::max(far, geo::distance(s.home_lat, s.home_lon, w.lat, w.lon));
            }
            if (plan.land) {
                far = std::max(far, geo::distance(s.home_lat, s.home_lon, plan.land->lat, plan.land->lon));
            }
            if (far > 50000.0) {
                // not a long mission: a mission left over from another place (e.g. SITL restarted elsewhere)
                return ev(Status::Fail, "the mission is " + std::to_string(static_cast<int>(far / 1000.0)) +
                                            " km from home: it was planned somewhere else. Plan a new one here and Sync");
            }
            const std::string d = "farthest point " + std::to_string(static_cast<int>(far)) + " m from home";
            return ev(far <= 3000.0 ? Status::Pass : Status::Warn, far <= 3000.0 ? d : d + ": keep it in range of the link");
        }
        if (id == "mis.endurance") {
            if (plan.wps.empty()) {
                return ev(Status::NotApplicable, "no waypoints");
            }
            const double lap = routeLength(plan);
            const double speed = s.airspeed_cruise > 0.0 ? s.airspeed_cruise : 18.0;
            const double lap_min = lap / speed / 60.0;
            if (p.endurance_min <= 0.0) {
                return ev(Status::NotApplicable, "lap " + std::to_string(static_cast<int>(lap)) + " m ≈ " + f1(lap_min) +
                                                     " min; enter the flight time in Settings → Battery to check it");
            }
            // keep 25 % for the landing and margin
            const double usable = p.endurance_min * 0.75;
            const int laps = lap_min > 0.0 ? static_cast<int>(usable / lap_min) : 0;
            const std::string d = "lap " + f1(lap_min) + " min · ≈ " + std::to_string(laps) + " laps with a 25 % reserve";
            return ev(laps >= 1 ? Status::Pass : Status::Fail, laps >= 1 ? d : d + ": the loop is too long for one pack");
        }
    }

    if (id == "fin.ready") {
        // Launch picks the mode and the mission item itself; this is the
        // autopilot's own verdict (its PreArm messages say why not)
        std::string missing;
        auto need = [&](bool ok, const char* what) {
            if (!ok) {
                missing += (missing.empty() ? "" : ", ") + std::string(what);
            }
        };
        need(!s.armed, "disarmed");
        need(gpsHealthy(s) && ekfHealthy(s), "GPS and position");
        need(prearmHealthy(s), "the pre-arm checks");
        return missing.empty() ? ev(Status::Pass, "the autopilot's pre-arm checks pass")
                               : ev(Status::Waiting, "waiting for " + missing);
    }

    if (id == "setup.app") {
        const SetupReport r = setupReport(s, p);
        if (!r.known) {
            return ev(Status::Waiting, "reading the aircraft's settings");
        }
        if (r.wrong.empty()) {
            return ev(Status::Pass, "set up for this app, failsafes included");
        }
        std::string d;
        for (const auto& w : r.wrong) {
            d += (d.empty() ? "" : " · ") + w.check->title;
        }
        return ev(Status::Fail, std::to_string(r.wrong.size()) + (r.wrong.size() == 1 ? " setting" : " settings") +
                                    " not right: " + d);
    }
    if (id == "setup.battery") {
        if (p.cells <= 0) {
            return ev(Status::Waiting, "reading the battery");
        }
        bool known = true;
        std::string d;
        for (const auto& c : paramChecks()) {
            if (c.group != ParamGroup::Battery) continue;
            if (c.name == "BATT_CAPACITY" && p.capacity_mah <= 0) continue;  // not entered: nothing to compare
            const ParamVerdict v = checkParam(c, s, p);
            known &= v.known;
            if (v.known && !v.ok) {
                d += (d.empty() ? "" : " · ") + v.detail;
            }
        }
        if (!known) {
            return ev(Status::Waiting, "reading the aircraft's settings");
        }
        return d.empty() ? ev(Status::Pass, "set for this " + std::to_string(p.cells) + "-cell battery")
                         : ev(Status::Warn, d);
    }
    return std::nullopt;
}

// ---- parameters ---------------------------------------------------------------------------------

const std::vector<ParamCheck>& paramChecks() {
    using G = ParamGroup;
    static const std::vector<ParamCheck> list = {
        {"param.Q_GUIDED_MODE", "Q_GUIDED_MODE", "Hold and Fly To circle on the wing",
         "1 makes Hold/Fly To hover on the rotors instead of circling on the wing.", G::Setup},
        {"param.Q_ASSIST_SPEED", "Q_ASSIST_SPEED", "Lift rotors catch a stall",
         "Below this airspeed the lift rotors catch a stalling wing; 0 turns that off.", G::Setup},
        {"param.AIRSPEED", "", "Airspeed limits in order",
         "AIRSPEED_MIN < AIRSPEED_CRUISE < AIRSPEED_MAX, or the speed controller fights itself.", G::Setup},
        {"param.ARMING", "", "All arming checks on", "Skipped arming checks are how broken sensors get airborne.",
         G::Setup},
        {"param.THR_FAILSAFE", "THR_FAILSAFE", "Notices a lost transmitter",
         "Without it, losing the transmitter isn't detected.", G::Setup},
        {"param.FS_LONG_ACTN", "FS_LONG_ACTN", "Comes back and lands after a long link loss",
         "After a long RC or GCS link loss: RTL, which flies the same landing as the Land button.", G::Setup},
        {"param.FS_GCS_ENABL", "FS_GCS_ENABL", "Notices a lost link to this app",
         "This GCS flies the aircraft; if its link is lost the aircraft should come back and land.", G::Setup},
        {"param.RTL_AUTOLAND", "RTL_AUTOLAND", "Return home flies the landing",
         "2 makes every RTL go straight to the mission's landing (circle beside L, into the wind, land "
         "vertically) instead of circling over home. With 0 the mission's landing marker also blocks arming.",
         G::Setup},
        {"param.Q_RTL_MODE", "Q_RTL_MODE", "Return home lands at L",
         "Any other value makes RTL do its own vertical landing at home and skip the landing at L.", G::Setup},
        {"param.BRD_SAFETY_DEFLT", "BRD_SAFETY_DEFLT", "Motors cut at power-up",
         "The motors stay cut after power-up until you turn safety off by hand. Applies after a restart.", G::Setup,
         true},
        {"param.BATT_FS_LOW_ACT", "BATT_FS_LOW_ACT", "Lands by itself at low battery",
         "At low battery it should fly the landing at L on its own.", G::Setup},
        {"param.BATT_FS_CRT_ACT", "BATT_FS_CRT_ACT", "Lands at once at critical battery",
         "At critical battery it should land right away (QLand).", G::Setup},
        {"param.BATT_LOW_VOLT", "BATT_LOW_VOLT", "Low-battery level", "The trigger for the low-battery landing.",
         G::Battery},
        {"param.BATT_CRT_VOLT", "BATT_CRT_VOLT", "Critical-battery level",
         "The trigger for the critical-battery landing.", G::Battery},
        {"param.BATT_CAPACITY", "BATT_CAPACITY", "Battery size", "Remaining % and capacity failsafes use it.",
         G::Battery},
        {"param.BATT_ARM_VOLT", "BATT_ARM_VOLT", "Won't arm on a flat battery",
         "Refuses to arm on a pack that isn't charged.", G::Battery},
    };
    return list;
}

bool SetupReport::fixable() const {
    return std::any_of(wrong.begin(), wrong.end(),
                       [](const SetupItem& w) { return w.verdict.recommended && !w.check->name.empty(); });
}

SetupReport setupReport(const VehicleState& s, const Profile& p) {
    SetupReport r;
    r.known = true;
    for (const auto& c : paramChecks()) {
        if (c.group != ParamGroup::Setup) continue;
        const ParamVerdict v = checkParam(c, s, p);
        r.known &= v.known;
        if (v.known && !v.ok) {
            r.wrong.push_back({&c, v});
        }
    }
    return r;
}

std::vector<std::string> paramsToFetch() {
    std::vector<std::string> v = {"Q_ENABLE",       "Q_FRAME_CLASS", "Q_FRAME_TYPE", "AIRSPEED_MIN",
                                  "AIRSPEED_CRUISE", "AIRSPEED_MAX", "ARMING_CHECK", "ARMING_SKIPCHK",
                                  "ARSPD_TYPE",     "MNT1_TYPE"};
    for (const auto& c : paramChecks()) {
        if (!c.name.empty()) {
            v.push_back(c.name);
        }
    }
    for (int i = 1; i <= 16; ++i) {
        const std::string n = "SERVO" + std::to_string(i);
        for (const char* suffix : {"_MIN", "_MAX", "_TRIM", "_REVERSED"}) {
            v.push_back(n + suffix);
        }
    }
    std::sort(v.begin(), v.end());
    v.erase(std::unique(v.begin(), v.end()), v.end());
    return v;
}

ParamVerdict checkParam(const ParamCheck& c, const VehicleState& s, const Profile& p) {
    ParamVerdict v;
    auto get = [&](const char* n) { return param(s, n); };
    auto num = [](float x) {
        char b[32];
        if (std::abs(x - std::round(x)) < 1e-4f) {
            std::snprintf(b, sizeof b, "%d", static_cast<int>(std::lround(x)));
        } else {
            std::snprintf(b, sizeof b, "%.2f", x);
        }
        return std::string(b);
    };
    auto simple = [&](const char* n, float want, const char* meaning) {
        const auto cur = get(n);
        if (!cur) {
            return v;
        }
        v.known = true;
        v.current = *cur;
        v.ok = std::abs(*cur - want) < 1e-4f;
        if (!v.ok) {
            v.recommended = want;
        }
        v.detail = std::string(n) + " = " + num(*cur) + (v.ok ? " ✓ " : " → recommended " + num(want) + ": ") + meaning;
        return v;
    };

    if (c.id == "param.Q_GUIDED_MODE") return simple("Q_GUIDED_MODE", 0.0f, "circle on the wing");
    if (c.id == "param.RTL_AUTOLAND") return simple("RTL_AUTOLAND", 2.0f, "RTL flies the landing at L");
    if (c.id == "param.Q_RTL_MODE") return simple("Q_RTL_MODE", 0.0f, "RTL hands over to the mission landing");
    if (c.id == "param.BRD_SAFETY_DEFLT") return simple("BRD_SAFETY_DEFLT", 1.0f, "safety starts ON");
    if (c.id == "param.THR_FAILSAFE") return simple("THR_FAILSAFE", 1.0f, "RC loss is detected");
    if (c.id == "param.FS_GCS_ENABL") {
        const auto cur = get("FS_GCS_ENABL");
        if (!cur) return v;
        v.known = true;
        v.current = *cur;
        v.ok = *cur >= 1.0f;
        if (!v.ok) v.recommended = 1.0f;
        v.detail = "FS_GCS_ENABL = " + num(*cur) + (v.ok ? " ✓ heartbeat loss triggers the failsafe" : " → recommended 1");
        return v;
    }
    if (c.id == "param.FS_LONG_ACTN") {
        const auto cur = get("FS_LONG_ACTN");
        if (!cur) return v;
        v.known = true;
        v.current = *cur;
        const int a = static_cast<int>(std::lround(*cur));
        v.ok = a == 1;
        if (!v.ok) v.recommended = 1.0f;
        const char* names[] = {"Continue", "RTL", "Glide", "Parachute", "Auto", "AUTOLAND"};
        v.detail = "FS_LONG_ACTN = " + std::string(a >= 0 && a <= 5 ? names[a] : "?") +
                   (v.ok ? " ✓ flies the landing at L" : " → recommended RTL (flies the landing at L)");
        return v;
    }
    if (c.id == "param.BATT_FS_LOW_ACT" || c.id == "param.BATT_FS_CRT_ACT") {
        const bool low = c.id == "param.BATT_FS_LOW_ACT";
        const auto cur = get(low ? "BATT_FS_LOW_ACT" : "BATT_FS_CRT_ACT");
        if (!cur) return v;
        v.known = true;
        v.current = *cur;
        const int a = static_cast<int>(std::lround(*cur));
        const char* names[] = {"Warn only", "RTL", "Land", "Terminate", "QLand", "Parachute", "Loiter to QLand",
                               "AUTOLAND or RTL"};
        // low: RTL or Land both jump to the mission's landing sequence
        // critical: land right where it is, there may not be enough left to reach L
        v.ok = low ? (a == 1 || a == 2) : (a == 4 || a == 6);
        if (!v.ok) v.recommended = low ? 1.0f : 4.0f;
        v.detail = std::string(low ? "BATT_FS_LOW_ACT" : "BATT_FS_CRT_ACT") + " = " +
                   (a >= 0 && a <= 7 ? names[a] : "?") +
                   (v.ok ? (low ? " ✓ flies the landing at L" : " ✓ lands where it is")
                         : low ? " → recommended RTL (flies the landing at L)" : " → recommended QLand (land where it is)");
        return v;
    }
    if (c.id == "param.BATT_LOW_VOLT" || c.id == "param.BATT_CRT_VOLT") {
        const bool low = c.id == "param.BATT_LOW_VOLT";
        const char* n = low ? "BATT_LOW_VOLT" : "BATT_CRT_VOLT";
        const auto cur = get(n);
        if (!cur || p.cells <= 0) return v;
        v.known = true;
        v.current = *cur;
        const float want = static_cast<float>(std::round(p.cells * (low ? lowCellVolts(p.chemistry) : criticalCellVolts(p.chemistry)) * 10.0) / 10.0);
        v.ok = *cur > 0.0f && std::abs(*cur - want) <= p.cells * 0.1f;
        if (!v.ok) v.recommended = want;
        v.detail = std::string(low ? "low-battery level " : "critical-battery level ") + num(*cur) + " V" +
                   (v.ok ? " ✓" : "; for this " + std::to_string(p.cells) + "-cell battery about " + num(want) + " V");
        return v;
    }
    if (c.id == "param.BATT_CAPACITY") {
        const auto cur = get("BATT_CAPACITY");
        if (!cur || p.capacity_mah <= 0) return v;
        v.known = true;
        v.current = *cur;
        v.ok = std::abs(*cur - p.capacity_mah) <= p.capacity_mah * 0.05f;
        if (!v.ok) v.recommended = static_cast<float>(p.capacity_mah);
        v.detail = "battery size set to " + num(*cur) + " mAh" +
                   (v.ok ? " ✓" : "; Settings → Battery says " + std::to_string(p.capacity_mah) + " mAh");
        return v;
    }
    if (c.id == "param.BATT_ARM_VOLT") {
        const auto cur = get("BATT_ARM_VOLT");
        if (!cur || p.cells <= 0) return v;
        v.known = true;
        v.current = *cur;
        const float want = static_cast<float>(std::round(p.cells * (storageCellVolts(p.chemistry) + 0.05) * 10.0) / 10.0);
        v.ok = *cur > 0.0f;
        if (!v.ok) v.recommended = want;
        v.detail = v.ok ? "won't arm below " + num(*cur) + " V ✓"
                        : "it would arm on a flat battery (about " + num(want) + " V is right for this one)";
        return v;
    }
    if (c.id == "param.Q_ASSIST_SPEED") {
        const auto cur = get("Q_ASSIST_SPEED"), mn = get("AIRSPEED_MIN");
        if (!cur || !mn) return v;
        v.known = true;
        v.current = *cur;
        v.ok = *cur > 0.0f && *cur < *mn;
        const float want = std::max(5.0f, std::round(*mn * 0.85f));
        if (!v.ok) v.recommended = want;
        v.detail = "Q_ASSIST_SPEED = " + num(*cur) + " m/s (AIRSPEED_MIN " + num(*mn) + ")" +
                   (v.ok ? " ✓" : " → recommended " + num(want) + " m/s");
        return v;
    }
    if (c.id == "param.AIRSPEED") {
        const auto mn = get("AIRSPEED_MIN"), cr = get("AIRSPEED_CRUISE"), mx = get("AIRSPEED_MAX");
        if (!mn || !cr || !mx) return v;
        v.known = true;
        v.ok = *mn < *cr && *cr < *mx;
        v.detail = "min " + num(*mn) + " · cruise " + num(*cr) + " · max " + num(*mx) + " m/s" +
                   (v.ok ? " ✓" : ": fix the order in your parameter editor");
        return v;
    }
    if (c.id == "param.ARMING") {
        // ArduPilot after 4.6: ARMING_SKIPCHK (0 = skip none); before: ARMING_CHECK (1 = all)
        if (const auto skip = get("ARMING_SKIPCHK")) {
            v.known = true;
            v.current = *skip;
            v.ok = *skip < 0.5f;
            if (!v.ok) v.recommended = 0.0f;
            v.detail = "ARMING_SKIPCHK = " + num(*skip) + (v.ok ? " ✓ nothing skipped" : " → recommended 0 (skip nothing)");
            return v;
        }
        const auto chk = get("ARMING_CHECK");
        if (!chk) return v;
        v.known = true;
        v.current = *chk;
        v.ok = std::abs(*chk - 1.0f) < 1e-4f;
        if (!v.ok) v.recommended = 1.0f;
        v.detail = "ARMING_CHECK = " + num(*chk) + (v.ok ? " ✓ all checks" : " → recommended 1 (all checks)");
        return v;
    }
    return v;
}

// ---- session ------------------------------------------------------------------------------------------

void Session::begin(const VehicleState& s, Clock::time_point now, bool turnaround) {
    active_ = true;
    turnaround_ = turnaround;
    passed_ = false;
    flown_ = false;
    start_ = now;
    results_.clear();
    writes_.clear();
    overrides_.clear();
    boot_count_ = s.boot_count;
    params_changed_ = s.params_changed;
    pack_volts_ = s.battery_v;
}

std::vector<const StepDef*> Session::activeSteps() const {
    std::vector<const StepDef*> v;
    for (const auto& st : steps()) {
        if (!turnaround_ || st.turnaround) {
            v.push_back(&st);
        }
    }
    return v;
}

bool Session::includes(const std::string& id) const {
    const StepDef* d = findStep(id);
    return d && (!turnaround_ || d->turnaround);
}

void Session::set(const std::string& id, Status st, const std::string& detail, bool by_user, Clock::time_point now,
                  const std::string& note) {
    Result& r = results_[id];
    r.status = st;
    r.detail = detail;
    r.by_user = by_user;
    r.note = note;
    r.at_s = std::chrono::duration<double>(now - start_).count();
}

Result Session::result(const std::string& id) const {
    const auto it = results_.find(id);
    return it == results_.end() ? Result{} : it->second;
}

int Session::doneCount() const {
    int n = 0;
    for (const auto* d : activeSteps()) {
        n += isDone(result(d->id).status);
    }
    return n;
}

int Session::totalCount() const {
    return static_cast<int>(activeSteps().size());
}

int Session::failCount() const {
    int n = 0;
    for (const auto* d : activeSteps()) {
        n += result(d->id).status == Status::Fail;
    }
    return n;
}

bool Session::complete() const {
    if (!active_) {
        return false;
    }
    for (const auto* d : activeSteps()) {
        const Status st = result(d->id).status;
        if (d->critical ? (!isDone(st) || st == Status::Skipped) : !(isDone(st) || st == Status::Fail)) {
            return false;  // a required step not passed, or an optional one not decided yet
        }
    }
    return true;
}

void Session::markPassed(const VehicleState& s, Clock::time_point now) {
    passed_ = true;
    passed_at_ = now;
    boot_count_ = s.boot_count;
    params_changed_ = s.params_changed;
    pack_volts_ = s.battery_v;
}

std::string Session::invalidReason(const VehicleState& s, Clock::time_point now) const {
    if (!passed_) {
        return "preflight not done";
    }
    if (flown_) {
        return "it has flown since: run the turnaround preflight";
    }
    if (s.boot_count != boot_count_) {
        return "the autopilot rebooted";
    }
    if (s.params_changed != params_changed_) {
        return "parameters were changed elsewhere";
    }
    if (pack_volts_ > 0.0 && s.battery_v > pack_volts_ + 0.5) {
        return "the battery was swapped";
    }
    if (std::chrono::duration<double>(now - passed_at_).count() > kValidMinutes * 60.0) {
        return "more than an hour ago";
    }
    return {};
}

void Session::noteOverride(const std::string& why, Clock::time_point) {
    overrides_.push_back(why);
}

void Session::noteParamWrite(const std::string& name, float from, float to, bool ok) {
    writes_.push_back({name, from, to, ok});
}

namespace {
std::string jsonEscape(const std::string& s) {
    std::string o;
    for (char c : s) {
        switch (c) {
        case '"': o += "\\\""; break;
        case '\\': o += "\\\\"; break;
        case '\n': o += "\\n"; break;
        default:
            if (static_cast<unsigned char>(c) < 0x20) {
                char b[8];
                std::snprintf(b, sizeof b, "\\u%04x", c);
                o += b;
            } else {
                o += c;
            }
        }
    }
    return o;
}
}  // namespace

std::string Session::reportJson(const std::string& started_iso, const VehicleState& s, const Profile& p) const {
    std::ostringstream o;
    const uint32_t v = s.fw_version;
    o << "{\n";
    o << "  \"started\": \"" << jsonEscape(started_iso) << "\",\n";
    o << "  \"kind\": \"" << (turnaround_ ? "turnaround" : "full") << "\",\n";
    o << "  \"complete\": " << (complete() ? "true" : "false") << ",\n";
    o << "  \"firmware\": \"" << ((v >> 24) & 0xff) << "." << ((v >> 16) & 0xff) << "." << ((v >> 8) & 0xff) << "\",\n";
    o << "  \"sysid\": " << static_cast<int>(s.sysid) << ",\n";
    o << "  \"battery\": {\"chemistry\": " << static_cast<int>(p.chemistry) << ", \"cells\": " << p.cells
      << ", \"capacity_mah\": " << p.capacity_mah << ", \"volts\": " << s.battery_v << "},\n";
    o << "  \"steps\": [\n";
    const auto list = activeSteps();
    for (size_t i = 0; i < list.size(); ++i) {
        const Result r = result(list[i]->id);
        o << "    {\"id\": \"" << jsonEscape(list[i]->id) << "\", \"title\": \"" << jsonEscape(list[i]->title)
          << "\", \"status\": \"" << statusName(r.status) << "\", \"detail\": \"" << jsonEscape(r.detail)
          << "\", \"note\": \"" << jsonEscape(r.note) << "\", \"by_user\": " << (r.by_user ? "true" : "false")
          << ", \"t\": " << r.at_s << "}" << (i + 1 < list.size() ? "," : "") << "\n";
    }
    o << "  ],\n  \"param_writes\": [";
    for (size_t i = 0; i < writes_.size(); ++i) {
        o << (i ? ", " : "") << "{\"name\": \"" << jsonEscape(writes_[i].name) << "\", \"from\": " << writes_[i].from
          << ", \"to\": " << writes_[i].to << ", \"ok\": " << (writes_[i].ok ? "true" : "false") << "}";
    }
    o << "],\n  \"overrides\": [";
    for (size_t i = 0; i < overrides_.size(); ++i) {
        o << (i ? ", " : "") << "\"" << jsonEscape(overrides_[i]) << "\"";
    }
    o << "]\n}\n";
    return o.str();
}

}  // namespace gcs::preflight
