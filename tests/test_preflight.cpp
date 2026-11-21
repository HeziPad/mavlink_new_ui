#include "check.hpp"
#include "core/frame_layout.hpp"
#include "core/plane_modes.hpp"
#include "core/preflight.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <string>

using namespace gcs;
using namespace gcs::preflight;

namespace {

const Clock::time_point T0 = Clock::time_point{} + std::chrono::hours(1);

VehicleState live() {
    VehicleState s;
    s.has_heartbeat = true;
    s.last_heartbeat = T0;
    return s;
}

Profile lipo6s() {
    Profile p;
    p.chemistry = Chemistry::LiPo;
    p.cells = 6;
    p.capacity_mah = 16000;
    p.endurance_min = 40.0;
    return p;
}

Status st(const std::string& id, const VehicleState& s, const Profile& p = lipo6s()) {
    const auto e = evaluate(id, s, p, T0);
    return e ? e->status : Status::Pending;
}

void testCatalogue() {
    std::set<std::string> ids;
    Stage last = Stage::Look;
    for (const auto& d : steps()) {
        CHECK(ids.insert(d.id).second);  // unique
        CHECK(!d.title.empty() && !d.how.empty());
        CHECK(d.stage >= last);          // ordered by stage
        // every step that runs by itself says what it's doing
        CHECK(d.stage == Stage::Look || !d.sentence.empty());
        CHECK(d.stage == Stage::Look || d.wait_s > 0.0);
        last = d.stage;
    }
    CHECK(findStep("mot.lift") && findStep("mot.lift")->critical);
    CHECK(findStep("setup.app") && findStep("setup.app")->critical);
    CHECK(findStep("setup.battery") && !findStep("setup.battery")->critical);  // a note, never written
    // gone for the simple run: tuning parameters, transmitter tests, the safety step
    CHECK(!findStep("param.Q_WVANE_GAIN") && !findStep("link.sticks") && !findStep("fin.safety"));
    // every stage has steps
    for (int i = 0; i < static_cast<int>(Stage::Count); ++i) {
        bool any = false;
        for (const auto& d : steps()) any |= d.stage == static_cast<Stage>(i);
        CHECK(any);
    }
}

void testFrames() {
    const FrameLayout x = frameLayout(1, 1);
    CHECK(x.known && x.motors.size() == 4);
    // Quad X: motor 1 front-right CCW (A), motor 3 front-left CW (D),
    // motor 4 back-right CW (B), motor 2 back-left CCW (C): A..D go clockwise
    CHECK(x.motors[0].angle == 45 && x.motors[0].ccw && x.motors[0].letter() == 'A');
    CHECK(x.motors[2].angle == -45 && !x.motors[2].ccw && x.motors[2].letter() == 'D');
    CHECK(x.motors[3].angle == 135 && !x.motors[3].ccw && x.motors[3].letter() == 'B');
    CHECK(x.motors[1].angle == -135 && x.motors[1].ccw && x.motors[1].letter() == 'C');
    const FrameLayout h = frameLayout(1, 3);
    CHECK(h.motors[0].angle == 45 && !h.motors[0].ccw);  // H: reversed spin
    CHECK(!frameLayout(7, 9).known && frameLayout(7, 9).motors.size() == 4);
}

void testPower() {
    VehicleState s = live();
    s.has_battery = true;
    s.battery_v = 25.0;  // 4.17 V/cell
    CHECK(st("power.voltage", s) == Status::Pass);
    s.battery_v = 23.4;  // 3.9
    CHECK(st("power.voltage", s) == Status::Warn);
    s.battery_v = 22.0;  // 3.67
    CHECK(st("power.voltage", s) == Status::Fail);
    Profile none;
    CHECK(st("power.voltage", s, none) == Status::Waiting);
    CHECK(guessCells(25.0, Chemistry::LiPo) == 6);
    CHECK(guessCells(16.6, Chemistry::LiPo) == 4);
    s.cell_v = {4.18, 4.17, 4.19, 4.18};
    CHECK(st("power.balance", s) == Status::Pass);
    s.cell_v = {4.18, 4.02, 4.19, 4.18};
    CHECK(st("power.balance", s) == Status::Fail);
}

void testSensors() {
    VehicleState s = live();
    s.has_gps = true;
    s.gps_fix = 3;
    s.gps_sats = 12;
    s.gps_hdop = 0.9;
    CHECK(st("sens.gps", s) == Status::Pass);
    s.gps_sats = 7;
    CHECK(st("sens.gps", s) == Status::Warn);
    s.gps_fix = 2;
    CHECK(st("sens.gps", s) == Status::Waiting);
    s.has_home = true;
    s.has_position = true;
    s.lat = s.home_lat = -35.36;
    s.lon = s.home_lon = 149.16;
    CHECK(st("sens.home", s) == Status::Pass);
    s.lat += 0.001;  // ~110 m
    CHECK(st("sens.home", s) == Status::Fail);
    s.has_sys_status = true;
    s.sensors_present = s.sensors_enabled = 0x1 | 0x2 | 0x10;
    s.sensors_health = 0x1 | 0x2;
    CHECK(st("sens.health", s) == Status::Fail);  // airspeed unhealthy
    s.sensors_health |= 0x10;
    CHECK(st("sens.health", s) == Status::Pass);
    // no link: automatic steps wait
    VehicleState dead;
    CHECK(st("sens.gps", dead) == Status::Waiting);
}

void testParams() {
    VehicleState s = live();
    const Profile p = lipo6s();
    auto check = [&](const char* id) {
        for (const auto& c : paramChecks()) {
            if (c.id == id) return checkParam(c, s, p);
        }
        return ParamVerdict{};
    };
    CHECK(!check("param.Q_GUIDED_MODE").known);
    s.params["Q_GUIDED_MODE"] = 1.0f;
    auto v = check("param.Q_GUIDED_MODE");
    CHECK(v.known && !v.ok && v.recommended && *v.recommended == 0.0f);
    s.params["BATT_LOW_VOLT"] = 0.0f;
    v = check("param.BATT_LOW_VOLT");
    CHECK(v.known && !v.ok && v.recommended && std::abs(*v.recommended - 21.0f) < 0.01f);  // 6 x 3.5
    s.params["BATT_LOW_VOLT"] = 21.1f;
    CHECK(check("param.BATT_LOW_VOLT").ok);
    s.params["AIRSPEED_MIN"] = 16.0f;
    s.params["Q_ASSIST_SPEED"] = 0.0f;
    v = check("param.Q_ASSIST_SPEED");
    CHECK(!v.ok && v.recommended && *v.recommended < 16.0f && *v.recommended > 0.0f);
    // newer ArduPilot: ARMING_SKIPCHK replaces ARMING_CHECK
    s.params["ARMING_SKIPCHK"] = 4.0f;
    v = check("param.ARMING");
    CHECK(v.known && !v.ok && v.recommended && *v.recommended == 0.0f);
    s.params.erase("ARMING_SKIPCHK");
    s.params["ARMING_CHECK"] = 1.0f;
    CHECK(check("param.ARMING").ok);
    // failsafes fly the Land approach: RTL_AUTOLAND 2, Q_RTL_MODE 0
    s.params["RTL_AUTOLAND"] = 0.0f;
    v = check("param.RTL_AUTOLAND");
    CHECK(v.known && !v.ok && v.recommended && *v.recommended == 2.0f);
    s.params["Q_RTL_MODE"] = 2.0f;  // VTOL approach at home would skip the landing at L
    v = check("param.Q_RTL_MODE");
    CHECK(!v.ok && v.recommended && *v.recommended == 0.0f);
    s.params["FS_LONG_ACTN"] = 0.0f;
    CHECK(!check("param.FS_LONG_ACTN").ok);
    s.params["FS_LONG_ACTN"] = 1.0f;
    CHECK(check("param.FS_LONG_ACTN").ok);
    s.params["BATT_FS_LOW_ACT"] = 2.0f;  // Land: jumps to the landing sequence too
    CHECK(check("param.BATT_FS_LOW_ACT").ok);
    s.params["BATT_FS_CRT_ACT"] = 1.0f;  // critical must land in place, not fly to L
    v = check("param.BATT_FS_CRT_ACT");
    CHECK(!v.ok && v.recommended && *v.recommended == 4.0f);
    // safety starts ON at boot; applies after a reboot
    s.params["BRD_SAFETY_DEFLT"] = 0.0f;
    v = check("param.BRD_SAFETY_DEFLT");
    CHECK(!v.ok && v.recommended && *v.recommended == 1.0f);
    for (const auto& c : paramChecks()) {
        if (c.id == "param.BRD_SAFETY_DEFLT") {
            CHECK(c.reboot && c.group == ParamGroup::Setup);
        }
    }
    const auto names = paramsToFetch();
    CHECK(std::find(names.begin(), names.end(), "RTL_AUTOLAND") != names.end());
    CHECK(std::find(names.begin(), names.end(), "Q_FRAME_CLASS") != names.end());
}

// every setup parameter right for this app
void goodSetup(VehicleState& s) {
    s.params["Q_GUIDED_MODE"] = 0.0f;
    s.params["Q_ASSIST_SPEED"] = 14.0f;
    s.params["AIRSPEED_MIN"] = 16.0f;
    s.params["AIRSPEED_CRUISE"] = 22.0f;
    s.params["AIRSPEED_MAX"] = 30.0f;
    s.params["ARMING_CHECK"] = 1.0f;
    s.params["THR_FAILSAFE"] = 1.0f;
    s.params["FS_LONG_ACTN"] = 1.0f;
    s.params["FS_GCS_ENABL"] = 1.0f;
    s.params["RTL_AUTOLAND"] = 2.0f;
    s.params["Q_RTL_MODE"] = 0.0f;
    s.params["BRD_SAFETY_DEFLT"] = 1.0f;
    s.params["BATT_FS_LOW_ACT"] = 1.0f;
    s.params["BATT_FS_CRT_ACT"] = 4.0f;
}

void testSetup() {
    VehicleState s = live();
    const Profile p = lipo6s();
    CHECK(st("setup.app", s) == Status::Waiting);  // not read yet
    goodSetup(s);
    CHECK(setupReport(s, p).known && setupReport(s, p).wrong.empty());
    CHECK(st("setup.app", s) == Status::Pass);
    // two wrong: one row, a failure, both named, both fixable in one go
    s.params["Q_GUIDED_MODE"] = 1.0f;
    s.params["RTL_AUTOLAND"] = 0.0f;
    const SetupReport r = setupReport(s, p);
    CHECK(r.wrong.size() == 2 && r.fixable());
    const auto e = evaluate("setup.app", s, p, T0);
    CHECK(e && e->status == Status::Fail && e->detail.find("2 settings") != std::string::npos);
    CHECK(e && e->detail.find("Return home flies the landing") != std::string::npos);
    // airspeed out of order has no single right value: still listed, not fixable alone
    goodSetup(s);
    s.params["AIRSPEED_CRUISE"] = 40.0f;
    CHECK(setupReport(s, p).wrong.size() == 1 && !setupReport(s, p).fixable());
    // battery levels are never part of the setup fix, and only a note
    goodSetup(s);
    s.params["BATT_LOW_VOLT"] = 14.0f;  // set for a 4S pack, a 6S one is fitted
    s.params["BATT_CRT_VOLT"] = 19.8f;
    s.params["BATT_ARM_VOLT"] = 23.0f;
    s.params["BATT_CAPACITY"] = 16000.0f;
    CHECK(setupReport(s, p).wrong.empty());
    const auto b = evaluate("setup.battery", s, p, T0);
    CHECK(b && b->status == Status::Warn && b->detail.find("6-cell") != std::string::npos);
    s.params["BATT_LOW_VOLT"] = 21.0f;
    CHECK(st("setup.battery", s) == Status::Pass);
    Profile no_cap = p;
    no_cap.capacity_mah = 0;  // not entered: the size isn't compared
    s.params["BATT_CAPACITY"] = 5000.0f;
    CHECK(st("setup.battery", s, no_cap) == Status::Pass);
    CHECK(st("setup.battery", s) == Status::Warn);
}

void testOwnedParams() {
    // ArduPilot rewrites these itself: not "parameters changed elsewhere"
    CHECK(autopilotOwnedParam("BARO1_GND_PRESS") && autopilotOwnedParam("BARO2_GND_TEMP"));
    CHECK(autopilotOwnedParam("ARSPD_OFFSET") && autopilotOwnedParam("ARSPD2_OFFSET"));
    CHECK(autopilotOwnedParam("STAT_RUNTIME"));
    CHECK(!autopilotOwnedParam("Q_GUIDED_MODE") && !autopilotOwnedParam("BARO_ALT_OFFSET"));
}

void testMissionElsewhere() {
    // SITL restarted at another location keeps its old mission: 13,000 km away
    VehicleState s = live();
    s.has_home = true;
    s.home_lat = 32.33;
    s.home_lon = 34.86;
    MissionPlan plan;
    plan.wps = {{-35.36, 149.16, 80.0f}, {-35.35, 149.17, 80.0f}};
    s.mission = buildLoopMission(-35.363, 149.165, 584.0, plan, 40.0f);
    s.mission_known = true;
    CHECK(st("mis.range", s) == Status::Fail);
    for (const auto& d : steps()) {
        if (d.id == "mis.range") {
            CHECK(d.critical);
        }
    }
}

void testNoPlan() {
    // no plan loaded: the GCS shows an empty mission until Launch puts the
    // landing at home on board
    VehicleState s = live();
    s.has_home = true;
    s.home_lat = 32.33;
    s.home_lon = 34.86;
    s.mission_known = true;
    CHECK(st("mis.synced", s) == Status::Pass);
    CHECK(st("mis.landing", s) == Status::Pass);
    CHECK(st("mis.range", s) == Status::NotApplicable);
    CHECK(st("mis.alts", s) == Status::NotApplicable);

    MissionPlan plan;
    plan.land = UserWaypoint{32.33, 34.86, 30.0f};
    ApproachInputs in;
    in.from_lat = 32.33;
    in.from_lon = 34.86;
    const LandingApproach appr = computeLandingApproach(*plan.land, in);
    s.mission = buildLoopMission(32.33, 34.86, 50.0, plan, 40.0f, &appr);
    CHECK(st("mis.synced", s) == Status::Pass);
    CHECK(st("mis.landing", s) == Status::Pass);
    const auto e = evaluate("mis.landing", s, lipo6s(), T0);
    CHECK(e && e->detail.find("home") != std::string::npos);

    // a route without a landing still fails
    MissionPlan route;
    route.wps = {{32.331, 34.861, 80.0f}, {32.332, 34.862, 80.0f}};
    s.mission = buildLoopMission(32.33, 34.86, 50.0, route, 40.0f);
    CHECK(st("mis.landing", s) == Status::Fail);
}

void testFinal() {
    VehicleState s = live();
    s.custom_mode = plane::QLOITER;  // Launch picks the mode itself
    s.has_sys_status = true;
    s.sensors_present = s.sensors_health = 0x10000000;  // pre-arm passing
    s.has_gps = true;
    s.gps_fix = 3;
    s.gps_sats = 12;
    s.has_ekf = true;
    s.ekf_flags = 1 | 2 | 4 | 16 | 32;
    CHECK(st("fin.ready", s) == Status::Pass);
    s.sensors_health = 0;  // a PreArm problem
    CHECK(st("fin.ready", s) == Status::Waiting);
}

void testSession() {
    VehicleState s = live();
    s.battery_v = 25.0;
    Session ses;
    ses.begin(s, T0, false);
    CHECK(!ses.complete());
    CHECK(ses.totalCount() > 25);
    for (const auto* d : ses.activeSteps()) {
        ses.set(d->id, Status::Pass, "", true, T0);
    }
    CHECK(ses.complete());
    // a critical step skipped is not complete
    ses.set("mot.lift", Status::Skipped, "", true, T0, "props off");
    CHECK(!ses.complete());
    ses.set("mot.lift", Status::Pass, "", true, T0);
    // an optional step that failed is a note, not a block
    ses.set("power.balance", Status::Fail, "", false, T0);
    CHECK(ses.complete());
    ses.set("power.balance", Status::Waiting, "", false, T0);  // not decided yet
    CHECK(!ses.complete());
    ses.set("power.balance", Status::Skipped, "", true, T0, "checked earlier");
    CHECK(ses.complete());

    ses.markPassed(s, T0);
    CHECK(ses.invalidReason(s, T0 + std::chrono::minutes(10)).empty());
    CHECK(!ses.invalidReason(s, T0 + std::chrono::minutes(61)).empty());  // stale
    VehicleState r = s;
    r.boot_count = 1;
    CHECK(ses.invalidReason(r, T0).find("rebooted") != std::string::npos);
    r = s;
    r.params_changed = 2;
    CHECK(!ses.invalidReason(r, T0).empty());
    r = s;
    r.battery_v = 25.4 + 0.5;  // fresh pack
    CHECK(ses.invalidReason(r, T0).find("swapped") != std::string::npos);
    ses.noteFlight();
    CHECK(ses.invalidReason(s, T0).find("turnaround") != std::string::npos);

    // the turnaround list is a short subset
    Session t;
    t.begin(s, T0, true);
    CHECK(t.totalCount() < ses.totalCount());
    CHECK(t.includes("power.voltage") && t.includes("air.props") && !t.includes("mot.lift"));

    ses.noteParamWrite("Q_GUIDED_MODE", 1.0f, 0.0f, true);
    ses.noteOverride("skipped by the pilot", T0);
    Profile p = lipo6s();
    const std::string json = ses.reportJson("2026-11-17T12:00:00", s, p);
    CHECK(json.find("\"param_writes\": [{\"name\": \"Q_GUIDED_MODE\"") != std::string::npos);
    CHECK(json.find("checked earlier") != std::string::npos);
    CHECK(json.find("\"overrides\": [\"skipped by the pilot\"]") != std::string::npos);
}

}  // namespace

int main() {
    testSetup();
    testOwnedParams();
    testMissionElsewhere();
    testNoPlan();
    testCatalogue();
    testFrames();
    testPower();
    testSensors();
    testParams();
    testFinal();
    testSession();
    return TEST_MAIN_RESULT();
}
