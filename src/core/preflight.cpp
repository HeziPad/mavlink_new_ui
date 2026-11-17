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
    case Stage::Profile: return "Aircraft";
    case Stage::Site: return "Site";
    case Stage::Airframe: return "Airframe";
    case Stage::Power: return "Power";
    case Stage::Links: return "Links & RC";
    case Stage::Sensors: return "Sensors";
    case Stage::Surfaces: return "Surfaces";
    case Stage::Motors: return "Motors";
    case Stage::Payload: return "Payload";
    case Stage::Params: return "Settings";
    case Stage::Mission: return "Mission";
    case Stage::Final: return "Ready";
    case Stage::Count: break;
    }
    return "";
}

const char* kindName(Kind k) {
    switch (k) {
    case Kind::Auto: return "AUTO";
    case Kind::Test: return "TEST";
    case Kind::Cal: return "CALIBRATE";
    case Kind::You: return "CHECK";
    case Kind::Fix: return "SETTING";
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
    static const std::vector<StepDef> list = [] {
        std::vector<StepDef> v = {
            // ---- aircraft profile
            {"profile.vehicle", Stage::Profile, Kind::Auto, "Autopilot and frame",
             "Everything after this depends on knowing the frame and which output drives what.",
             "Wait for the vehicle's firmware, frame and output map to load.", true, false},
            {"profile.battery", Stage::Profile, Kind::You, "Battery profile",
             "Voltage thresholds and failsafes are only right for the right chemistry and cell count.",
             "Choose the chemistry, check the cell count and enter the pack capacity.", true, false},
            // ---- site
            {"site.airspace", Stage::Site, Kind::You, "Airspace and permission",
             "Flying where you're not allowed is the one failure no autopilot can fix.",
             "Confirm the area is permitted (no restrictions or NOTAMs) and you have any needed approval.", false,
             false},
            {"site.weather", Stage::Site, Kind::You, "Weather and visibility",
             "Gusts and rain hurt most in the hover, where a QuadPlane is weakest.",
             "Wind within the aircraft's limits, no rain or icing, the aircraft stays in sight.", false, false},
            {"site.area", Stage::Site, Kind::You, "Takeoff area clear",
             "The lift rotors need room: a gust in the hover can drift it several metres.",
             "Clear a 10 m circle around the aircraft: no people, wires, branches or loose objects.", true, true},
            {"site.landing", Stage::Site, Kind::You, "Landing point clear",
             "Land comes down vertically at L; anything there gets hit.",
             "Check the landing point (L on the map) is clear, flat and reachable.", false, true},
            {"site.people", Stage::Site, Kind::You, "People briefed",
             "Bystanders walking under a hovering aircraft is the classic accident.",
             "Everyone knows where to stand (behind you) and that it lifts off vertically.", false, false},
            {"site.gcs", Stage::Site, Kind::You, "Ground station ready",
             "A dead tablet mid-flight leaves only the transmitter.",
             "GCS device charged (or on power), telemetry radio connected, transmitter charged.", false, false},
            // ---- airframe
            {"air.structure", Stage::Airframe, Kind::You, "Wing, booms and mounts",
             "A loose wing joint or boom fails under the hover loads.",
             "Wing joiners locked, booms tight, every motor mount tight, no cracks.", true, false},
            {"air.props", Stage::Airframe, Kind::You, "Propellers",
             "A cracked or loose prop sheds a blade; a wrong-hand prop gives no lift.",
             "4 lift props and the pusher: undamaged, tight, and each the right hand for its position.", true, true},
            {"air.linkages", Stage::Airframe, Kind::You, "Control linkages",
             "Slop or a popped clevis means a surface stops responding.",
             "Every horn, clevis and pushrod secure; wiggle each surface: no play.", true, false},
            {"air.pitot", Stage::Airframe, Kind::You, "Pitot cover off",
             "A covered pitot reads zero airspeed: the autopilot thinks it's stalling.",
             "Remove the pitot cover; check the tube and port are clear.", true, true},
            {"air.cg", Stage::Airframe, Kind::You, "Centre of gravity",
             "A tail-heavy QuadPlane is unstable on the wing and fights the rotors in the hover.",
             "Balance it at the marked CG point (with this battery and payload).", false, false},
            {"air.battery", Stage::Airframe, Kind::You, "Battery secured",
             "A battery that shifts moves the CG; a loose connector browns out the autopilot.",
             "Battery strapped, connector fully seated, wires clear of props; hatches closed.", true, true},
            {"air.antennas", Stage::Airframe, Kind::You, "Antennas and payload",
             "A loose GPS mast or folded RC antenna costs you the link or the position.",
             "GPS, telemetry, RC and video antennas secure and oriented; gimbal/payload secured.", false, false},
            // ---- power
            {"power.voltage", Stage::Power, Kind::Auto, "Battery charged",
             "A half-charged pack may not have the power for the vertical landing.",
             "Fit a fully charged pack.", true, true},
            {"power.balance", Stage::Power, Kind::Auto, "Cell balance",
             "One weak cell sags under hover load long before the pack voltage looks low.",
             "Cells within 0.05 V of each other.", false, true},
            {"power.current", Stage::Power, Kind::Auto, "Current sensor",
             "Without current, capacity used and the battery failsafe are guesswork.",
             "The current reading should be small and positive at rest.", false, false},
            {"power.board", Stage::Power, Kind::Auto, "Autopilot supply",
             "A sagging 5 V rail resets the autopilot in flight.", "Board voltage 4.8-5.4 V.", true, false},
            // ---- links
            {"link.telemetry", Stage::Links, Kind::Auto, "Telemetry link",
             "This GCS flies the aircraft through it; a weak link means delayed commands.",
             "Steady message rate; radio signal well above noise.", true, false},
            {"link.sticks", Stage::Links, Kind::Test, "Transmitter sticks",
             "The transmitter is your manual backup; a reversed or dead axis is dangerous.",
             "Move each stick to both ends; every axis must reach both ends. Leave throttle low.", true, false},
            {"link.modes", Stage::Links, Kind::Test, "Mode switch",
             "In an emergency you need to know exactly what each switch position does.",
             "Flip the mode switch through every position and check the mode shown.", false, false},
            {"link.rcfailsafe", Stage::Links, Kind::Test, "RC failsafe",
             "If the transmitter link drops in flight, the failsafe must trigger.",
             "Turn the transmitter off: the vehicle must report RC failsafe. Then turn it back on.", true, false},
            // ---- sensors
            {"sens.gyro", Stage::Sensors, Kind::Cal, "Gyro calibration",
             "Gyro drift makes the attitude estimate wander.", "Keep the aircraft completely still.", false, false},
            {"sens.level", Stage::Sensors, Kind::Cal, "Level",
             "If 'level' is wrong the aircraft flies with a constant tilt and drifts in the hover.",
             "Place the aircraft level (wing chord at its flying angle), then calibrate.", false, false},
            {"sens.accel", Stage::Sensors, Kind::Cal, "Accelerometer",
             "Needed after mounting changes; a bad accel calibration corrupts the attitude.",
             "Only if it's uncalibrated or failing: the 6-position calibration.", false, false},
            {"sens.compass", Stage::Sensors, Kind::Cal, "Compass",
             "A bad compass gives a wrong heading: the aircraft circles or flies off.",
             "Calibrate from a known heading (best for a big airframe) or by rotating it.", true, false},
            {"sens.baro", Stage::Sensors, Kind::Cal, "Barometer and airspeed zero",
             "Zeroes altitude here and the airspeed offset.",
             "Shield the pitot from the wind, then calibrate.", false, false},
            {"sens.pitot", Stage::Sensors, Kind::Test, "Pitot response",
             "A blocked or leaking pitot fools stall protection and the transitions.",
             "Blow gently into the pitot (or cup a hand over it): the airspeed must rise.", true, true},
            {"sens.orientation", Stage::Sensors, Kind::Test, "Attitude follows the aircraft",
             "A wrong board orientation flips the aircraft on takeoff.",
             "Tilt right, nose up, and turn it: the ball and heading must follow.", true, false},
            {"sens.heading", Stage::Sensors, Kind::You, "Heading matches",
             "The final compass sanity check.",
             "The aircraft arrow on the map points where the real nose points.", true, false},
            {"sens.health", Stage::Sensors, Kind::Auto, "Sensors healthy",
             "Any unhealthy sensor is a reason not to fly.", "Gyro, accel, compass, baro, airspeed all healthy.",
             true, false},
            {"sens.ekf", Stage::Sensors, Kind::Auto, "Navigation filter",
             "The EKF decides where the aircraft is; it must be confident.",
             "EKF healthy, all innovation ratios under 0.5.", true, true},
            {"sens.gps", Stage::Sensors, Kind::Auto, "GPS",
             "Every automatic mode needs a good position.", "3D fix, 10 or more satellites, HDOP 1.2 or better.",
             true, true},
            {"sens.gps2", Stage::Sensors, Kind::Auto, "Second GPS",
             "The backup only helps if it's healthy too.", "3D fix on the second GPS.", false, false},
            {"sens.home", Stage::Sensors, Kind::Auto, "Home position",
             "QRTL and the failsafes fly back to home.", "Home set, within 20 m of the aircraft.", true, true},
            {"sens.logging", Stage::Sensors, Kind::Auto, "Flight log",
             "Without a log, nothing can be learned from a problem.", "SD card logging works.", false, false},
            {"sens.vibe", Stage::Sensors, Kind::Auto, "Vibration at rest",
             "High vibration at rest points at a loose board or mount.", "Vibration low, no clipping.", false,
             false},
            // ---- surfaces
            {"surf.roll", Stage::Surfaces, Kind::Test, "Ailerons",
             "A reversed aileron rolls the aircraft the wrong way at the first turn.",
             "Watch the ailerons while the GCS rolls right then left.", true, false},
            {"surf.pitch", Stage::Surfaces, Kind::Test, "Elevator",
             "A reversed elevator stalls or dives the aircraft after the transition.",
             "Watch the elevator while the GCS pitches up then down.", true, false},
            {"surf.yaw", Stage::Surfaces, Kind::Test, "Rudder",
             "A reversed rudder makes turns uncoordinated.", "Watch the rudder while the GCS yaws right then left.",
             false, false},
            {"surf.stabilize", Stage::Surfaces, Kind::Test, "Stabilisation",
             "The autopilot must push back against a disturbance, not with it.",
             "In FBWA, tilt the aircraft right by hand: the ailerons must roll it back left.", true, false},
            {"surf.trim", Stage::Surfaces, Kind::You, "Surfaces neutral",
             "Off-neutral surfaces at rest mean the autopilot fights a trim error all flight.",
             "With the sticks centred, every surface sits neutral.", false, false},
            // ---- motors
            {"mot.gate", Stage::Motors, Kind::You, "Ready to spin motors",
             "Spinning motors is the most dangerous part of a preflight.",
             "Choose props OFF (safest) or props ON with the aircraft held/tied down and everyone clear.", true,
             false},
            {"mot.lift", Stage::Motors, Kind::Test, "Lift motors",
             "A motor in the wrong position or spinning the wrong way flips the aircraft at liftoff.",
             "Each motor spins briefly: check it's the one shown, spinning the way shown.", true, false},
            {"mot.pusher", Stage::Motors, Kind::Test, "Pusher motor",
             "No pusher thrust means no transition: it hovers until the battery runs out.",
             "The pusher spins briefly: check it blows air backwards.", true, false},
            // ---- payload
            {"pay.gimbal", Stage::Payload, Kind::Test, "Gimbal",
             "A stuck gimbal means no useful picture and wrong camera targets.",
             "The GCS moves the gimbal: check it moves the way shown and re-centres.", false, false},
            {"pay.video", Stage::Payload, Kind::Auto, "Video feed", "No video, no mission.",
             "Frames arriving from the camera.", false, false},
            // ---- mission
            {"mis.synced", Stage::Mission, Kind::Auto, "Mission on the vehicle",
             "AUTO flies what the vehicle holds, not what the GCS shows.",
             "The vehicle has a mission and it matches the plan.", true, true},
            {"mis.landing", Stage::Mission, Kind::Auto, "Landing point",
             "Land (the wind-aligned landing) needs a landing point.", "The mission has a landing point L.", false,
             true},
            {"mis.alts", Stage::Mission, Kind::Auto, "Mission altitudes",
             "Too low hits obstacles; too high breaks rules or the fence.",
             "Every waypoint between the takeoff altitude and 120 m (or the fence).", false, true},
            {"mis.range", Stage::Mission, Kind::Auto, "Distances",
             "Far waypoints eat the reserve for the vertical landing.",
             "Waypoints and L within a sensible range of home.", false, true},
            {"mis.endurance", Stage::Mission, Kind::Auto, "Endurance",
             "Running out of battery on a loop that flies forever.",
             "Enough battery for the laps you plan, plus the landing reserve.", false, true},
            {"mis.review", Stage::Mission, Kind::You, "Route reviewed",
             "The last look at where it will go.", "Check the route, the takeoff altitude and L on the map.",
             false, true},
            // ---- final
            {"fin.ready", Stage::Final, Kind::Auto, "Vehicle set for launch",
             "Launch must start the mission from its takeoff, with every pre-arm check passing.",
             "The GCS selects AUTO, resets the mission to the takeoff and runs the pre-arm checks.", true, true},
            {"fin.safety", Stage::Final, Kind::Test, "Safety OFF",
             "With safety on, the motors can't spin and Launch can't arm.",
             "Hold SAFETY until it reads OFF. From now on the props are live.", true, true},
        };
        for (const auto& c : paramChecks()) {
            v.push_back({c.id, Stage::Params, Kind::Fix, c.title, c.why, "", c.critical, false});
        }
        // keep stage order (params were appended)
        std::stable_sort(v.begin(), v.end(),
                         [](const StepDef& a, const StepDef& b) { return a.stage < b.stage; });
        return v;
    }();
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
    if (!alive && id != "profile.battery") {
        if (findStep(id) && findStep(id)->kind == Kind::Auto) {
            return ev(Status::Waiting, "no vehicle link");
        }
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
        if (!p.set || p.cells <= 0) {
            return ev(Status::Waiting, "set the battery profile first (" + f2(s.battery_v) + " V)");
        }
        const double per = s.battery_v / p.cells;
        const std::string d = f2(s.battery_v) + " V = " + f2(per) + " V/cell (full ≥ " + f2(fullCellVolts(p.chemistry)) + ")";
        if (per >= fullCellVolts(p.chemistry)) {
            return ev(Status::Pass, d);
        }
        if (per >= storageCellVolts(p.chemistry)) {
            return ev(Status::Warn, d + ": not fully charged");
        }
        return ev(Status::Fail, d + ": not charged");
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
    if (id == "sens.gps2") {
        if (!s.has_gps2) {
            return ev(Status::NotApplicable, "no second GPS");
        }
        const std::string d = std::to_string(s.gps2_sats) + " sats · HDOP " + f1(s.gps2_hdop);
        return ev(s.gps2_fix >= 3 ? Status::Pass : Status::Waiting, d);
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

    if (id.rfind("mis.", 0) == 0 && id != "mis.review") {
        if (!s.mission_known) {
            return ev(Status::Waiting, "mission not read yet");
        }
        const MissionPlan plan = extractPlan(s.mission);
        if (id == "mis.synced") {
            return plan.wps.empty() ? ev(Status::Fail, "no waypoints on the vehicle: Plan → Sync")
                                    : ev(Status::Pass, std::to_string(plan.wps.size()) + " waypoints on the vehicle");
        }
        if (id == "mis.landing") {
            return plan.land ? ev(Status::Pass, "landing point set")
                             : ev(Status::Warn, "no landing point: only QRTL can land it");
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
            if (!s.has_home || plan.wps.empty()) {
                return ev(Status::Waiting, "needs home and waypoints");
            }
            double far = 0.0;
            for (const auto& w : plan.wps) {
                far = std::max(far, geo::distance(s.home_lat, s.home_lon, w.lat, w.lon));
            }
            if (plan.land) {
                far = std::max(far, geo::distance(s.home_lat, s.home_lon, plan.land->lat, plan.land->lon));
            }
            const std::string d = "farthest point " + std::to_string(static_cast<int>(far)) + " m from home";
            return ev(far <= 3000.0 ? Status::Pass : Status::Warn, far <= 3000.0 ? d : d + ": keep it in range of the link");
        }
        if (id == "mis.endurance") {
            if (plan.wps.empty()) {
                return ev(Status::NotApplicable, "no waypoints");
            }
            const double lap = lapLength(plan.wps);
            const double speed = s.airspeed_cruise > 0.0 ? s.airspeed_cruise : 18.0;
            const double lap_min = lap / speed / 60.0;
            if (p.endurance_min <= 0.0) {
                return ev(Status::Warn, "lap " + std::to_string(static_cast<int>(lap)) + " m ≈ " + f1(lap_min) +
                                            " min; enter the typical flight time in the profile to estimate laps");
            }
            // keep 25 % for the landing and margin
            const double usable = p.endurance_min * 0.75;
            const int laps = lap_min > 0.0 ? static_cast<int>(usable / lap_min) : 0;
            const std::string d = "lap " + f1(lap_min) + " min · ≈ " + std::to_string(laps) + " laps with a 25 % reserve";
            return ev(laps >= 1 ? Status::Pass : Status::Fail, laps >= 1 ? d : d + ": the loop is too long for one pack");
        }
    }

    if (id == "fin.ready") {
        std::string missing;
        auto need = [&](bool ok, const char* what) {
            if (!ok) {
                missing += (missing.empty() ? "" : ", ") + std::string(what);
            }
        };
        need(!s.armed, "disarmed");
        need(s.custom_mode == plane::AUTO, "AUTO selected");
        need(s.mission_known && s.mission_current == 1, "mission at its takeoff");
        need(prearmHealthy(s), "pre-arm checks passing");
        need(gpsHealthy(s) && ekfHealthy(s), "GPS and EKF good");
        return missing.empty() ? ev(Status::Pass, "AUTO · mission at the takeoff · pre-arm checks pass")
                               : ev(Status::Waiting, "needs: " + missing);
    }
    if (id == "fin.safety") {
        const SafetyState st = safetyState(s);
        if (st == SafetyState::Live) {
            return ev(Status::Pass, "safety OFF: motors live");
        }
        if (st == SafetyState::Unknown) {
            return ev(Status::NotApplicable, "no safety switch reported");
        }
        return ev(Status::Waiting, "safety ON: hold SAFETY to turn it off");
    }

    if (id.rfind("param.", 0) == 0) {
        for (const auto& c : paramChecks()) {
            if (c.id == id) {
                const ParamVerdict v = checkParam(c, s, p);
                if (!v.known) {
                    return ev(Status::Waiting, "reading…");
                }
                return ev(v.ok ? Status::Pass : (c.critical ? Status::Fail : Status::Warn), v.detail);
            }
        }
    }
    return std::nullopt;
}

// ---- parameters ---------------------------------------------------------------------------------

const std::vector<ParamCheck>& paramChecks() {
    static const std::vector<ParamCheck> list = {
        {"param.Q_GUIDED_MODE", "Q_GUIDED_MODE", "Guide stays on the wing",
         "1 makes Hold/Guide hover on the rotors instead of circling on the wing.", true},
        {"param.Q_ASSIST_SPEED", "Q_ASSIST_SPEED", "Stall protection (Q_ASSIST)",
         "Below this airspeed the lift rotors catch a stalling wing; 0 turns that off.", true},
        {"param.AIRSPEED", "", "Airspeed limits consistent",
         "AIRSPEED_MIN < AIRSPEED_CRUISE < AIRSPEED_MAX, or the speed controller fights itself.", true},
        {"param.ARMING", "", "All arming checks on",
         "Skipped arming checks are how broken sensors get airborne.", true},
        {"param.THR_FAILSAFE", "THR_FAILSAFE", "RC failsafe enabled",
         "Without it, losing the transmitter isn't detected.", true},
        {"param.FS_LONG_ACTN", "FS_LONG_ACTN", "Long failsafe returns home",
         "What it does after a long link loss on the wing; RTL brings it back.", true},
        {"param.FS_GCS_ENABL", "FS_GCS_ENABL", "GCS link failsafe",
         "This GCS flies the aircraft; if its link is lost the aircraft should react.", false},
        {"param.BATT_FS_LOW_ACT", "BATT_FS_LOW_ACT", "Low battery action",
         "At low battery it should come home on its own.", true},
        {"param.BATT_FS_CRT_ACT", "BATT_FS_CRT_ACT", "Critical battery action",
         "At critical battery it should land right away (QLand).", true},
        {"param.BATT_LOW_VOLT", "BATT_LOW_VOLT", "Low battery voltage",
         "The trigger for the low-battery action, per this pack.", true},
        {"param.BATT_CRT_VOLT", "BATT_CRT_VOLT", "Critical battery voltage",
         "The trigger for the critical-battery landing, per this pack.", true},
        {"param.BATT_CAPACITY", "BATT_CAPACITY", "Battery capacity",
         "Remaining % and capacity failsafes use it.", false},
        {"param.BATT_ARM_VOLT", "BATT_ARM_VOLT", "Minimum voltage to arm",
         "Refuses to arm on a pack that isn't charged.", false},
        {"param.FENCE_ENABLE", "FENCE_ENABLE", "Geofence",
         "A fence (with a maximum altitude) catches a flyaway.", false},
        {"param.WP_LOITER_RAD", "WP_LOITER_RAD", "Default circle radius",
         "Smaller than the aircraft can turn at its bank limit means it can never fly the circle.", false},
        {"param.ROLL_LIMIT_DEG", "ROLL_LIMIT_DEG", "Bank limit",
         "Very steep banks stall a loaded wing; very shallow ones make huge circles.", false},
        {"param.LOG_DISARMED", "LOG_DISARMED", "Logging while disarmed",
         "Logging on the ground fills the SD card with nothing.", false},
    };
    return list;
}

std::vector<std::string> paramsToFetch() {
    std::vector<std::string> v = {"Q_ENABLE", "Q_FRAME_CLASS", "Q_FRAME_TYPE", "AIRSPEED_MIN", "AIRSPEED_CRUISE",
                                  "AIRSPEED_MAX", "ARMING_CHECK", "ARMING_SKIPCHK", "ARSPD_TYPE", "FLTMODE_CH",
                                  "FLTMODE1", "FLTMODE2", "FLTMODE3", "FLTMODE4", "FLTMODE5", "FLTMODE6",
                                  "RCMAP_ROLL", "RCMAP_PITCH", "RCMAP_THROTTLE", "RCMAP_YAW", "RC_OVERRIDE_TIME"};
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
        v.ok = a == 1 || a == 5;
        if (!v.ok) v.recommended = 1.0f;
        const char* names[] = {"Continue", "RTL", "Glide", "Parachute", "Auto", "AUTOLAND"};
        v.detail = "FS_LONG_ACTN = " + std::string(a >= 0 && a <= 5 ? names[a] : "?") +
                   (v.ok ? " ✓" : " → recommended RTL (home, then lands vertically)");
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
        v.ok = low ? (a == 1 || a == 6 || a == 7) : (a == 4 || a == 6 || a == 2);
        if (!v.ok) v.recommended = low ? 1.0f : 4.0f;
        v.detail = std::string(low ? "BATT_FS_LOW_ACT" : "BATT_FS_CRT_ACT") + " = " +
                   (a >= 0 && a <= 7 ? names[a] : "?") +
                   (v.ok ? " ✓" : low ? " → recommended RTL" : " → recommended QLand (land where it is)");
        return v;
    }
    if (c.id == "param.BATT_LOW_VOLT" || c.id == "param.BATT_CRT_VOLT") {
        const bool low = c.id == "param.BATT_LOW_VOLT";
        const char* n = low ? "BATT_LOW_VOLT" : "BATT_CRT_VOLT";
        const auto cur = get(n);
        if (!cur || !p.set || p.cells <= 0) return v;
        v.known = true;
        v.current = *cur;
        const float want = static_cast<float>(std::round(p.cells * (low ? lowCellVolts(p.chemistry) : criticalCellVolts(p.chemistry)) * 10.0) / 10.0);
        v.ok = *cur > 0.0f && std::abs(*cur - want) <= p.cells * 0.1f;
        if (!v.ok) v.recommended = want;
        v.detail = std::string(n) + " = " + num(*cur) + " V" +
                   (v.ok ? " ✓" : " → recommended " + num(want) + " V for " + std::to_string(p.cells) + " cells");
        return v;
    }
    if (c.id == "param.BATT_CAPACITY") {
        const auto cur = get("BATT_CAPACITY");
        if (!cur || !p.set || p.capacity_mah <= 0) return v;
        v.known = true;
        v.current = *cur;
        v.ok = std::abs(*cur - p.capacity_mah) <= p.capacity_mah * 0.05f;
        if (!v.ok) v.recommended = static_cast<float>(p.capacity_mah);
        v.detail = "BATT_CAPACITY = " + num(*cur) + " mAh" + (v.ok ? " ✓" : " → this pack is " + std::to_string(p.capacity_mah) + " mAh");
        return v;
    }
    if (c.id == "param.BATT_ARM_VOLT") {
        const auto cur = get("BATT_ARM_VOLT");
        if (!cur || !p.set || p.cells <= 0) return v;
        v.known = true;
        v.current = *cur;
        const float want = static_cast<float>(std::round(p.cells * (storageCellVolts(p.chemistry) + 0.05) * 10.0) / 10.0);
        v.ok = *cur > 0.0f;
        if (!v.ok) v.recommended = want;
        v.detail = "BATT_ARM_VOLT = " + num(*cur) + " V" + (v.ok ? " ✓" : " → recommended " + num(want) + " V");
        return v;
    }
    if (c.id == "param.FENCE_ENABLE") return simple("FENCE_ENABLE", 1.0f, "set the fence (and its altitude) in your planner first");
    if (c.id == "param.LOG_DISARMED") return simple("LOG_DISARMED", 0.0f, "log only when armed");
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
    if (c.id == "param.WP_LOITER_RAD") {
        const auto rad = get("WP_LOITER_RAD"), roll = get("ROLL_LIMIT_DEG"), cr = get("AIRSPEED_CRUISE");
        if (!rad || !roll || !cr) return v;
        v.known = true;
        v.current = *rad;
        const double bank = std::clamp(static_cast<double>(*roll), 10.0, 80.0) * geo::kDegToRad;
        // the turn it can fly at 75 % of the bank limit
        const double min_r = (*cr) * (*cr) / (9.81 * std::tan(bank * 0.75));
        v.ok = std::abs(*rad) >= min_r;
        const float want = static_cast<float>(std::ceil(min_r / 10.0) * 10.0);
        if (!v.ok) v.recommended = want;
        v.detail = "WP_LOITER_RAD = " + num(std::abs(*rad)) + " m (tightest comfortable circle at " + num(*cr) +
                   " m/s ≈ " + num(static_cast<float>(std::round(min_r))) + " m)" + (v.ok ? " ✓" : " → recommended " + num(want) + " m");
        return v;
    }
    if (c.id == "param.ROLL_LIMIT_DEG") {
        const auto cur = get("ROLL_LIMIT_DEG");
        if (!cur) return v;
        v.known = true;
        v.current = *cur;
        v.ok = *cur >= 30.0f && *cur <= 65.0f;
        if (!v.ok) v.recommended = 45.0f;
        v.detail = "ROLL_LIMIT_DEG = " + num(*cur) + "°" + (v.ok ? " ✓" : " → recommended 45°");
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
        if (!isDone(st)) {
            return false;
        }
        if (d->critical && st == Status::Skipped) {
            return false;
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
