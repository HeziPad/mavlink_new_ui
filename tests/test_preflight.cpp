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
    p.set = true;
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
    Stage last = Stage::Profile;
    for (const auto& d : steps()) {
        CHECK(ids.insert(d.id).second);  // unique
        CHECK(!d.title.empty() && !d.why.empty());
        CHECK(d.stage >= last);          // ordered by stage
        last = d.stage;
    }
    CHECK(findStep("mot.lift") && findStep("mot.lift")->critical);
    CHECK(findStep("param.Q_GUIDED_MODE") && findStep("param.Q_GUIDED_MODE")->stage == Stage::Params);
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
    // loiter radius vs bank limit: 25 m/s at 45 deg -> ~ 64 / tan(33.75) ~ 95 m
    s.params["WP_LOITER_RAD"] = 60.0f;
    s.params["ROLL_LIMIT_DEG"] = 45.0f;
    s.params["AIRSPEED_CRUISE"] = 25.0f;
    v = check("param.WP_LOITER_RAD");
    CHECK(!v.ok && v.recommended && *v.recommended >= 90.0f);
    // and the step evaluator maps critical failures to Fail
    s.params["Q_GUIDED_MODE"] = 1.0f;
    CHECK(st("param.Q_GUIDED_MODE", s) == Status::Fail);
    const auto names = paramsToFetch();
    CHECK(std::find(names.begin(), names.end(), "Q_FRAME_CLASS") != names.end());
}

void testFinal() {
    VehicleState s = live();
    s.custom_mode = plane::AUTO;
    s.mission_known = true;
    s.mission_current = 1;
    s.has_sys_status = true;
    s.sensors_present = s.sensors_health = 0x10000000;  // pre-arm passing
    s.has_gps = true;
    s.gps_fix = 3;
    s.gps_sats = 12;
    s.has_ekf = true;
    s.ekf_flags = 1 | 2 | 4 | 16 | 32;
    CHECK(st("fin.ready", s) == Status::Pass);
    s.mission_current = 7;
    CHECK(st("fin.ready", s) == Status::Waiting);
    s.has_sys_status = true;
    s.sensors_present = kSensorMotorOutputs;
    s.sensors_enabled = 0;
    CHECK(st("fin.safety", s) == Status::Waiting);
    s.sensors_enabled = kSensorMotorOutputs;
    CHECK(st("fin.safety", s) == Status::Pass);
}

void testSession() {
    VehicleState s = live();
    s.battery_v = 25.0;
    Session ses;
    ses.begin(s, T0, false);
    CHECK(!ses.complete());
    CHECK(ses.totalCount() > 40);
    for (const auto* d : ses.activeSteps()) {
        ses.set(d->id, Status::Pass, "", true, T0);
    }
    CHECK(ses.complete());
    // a critical step skipped is not complete
    ses.set("mot.lift", Status::Skipped, "", true, T0, "props off");
    CHECK(!ses.complete());
    ses.set("mot.lift", Status::Pass, "", true, T0);
    // a non-critical skip is fine
    ses.set("air.cg", Status::Skipped, "", true, T0, "checked earlier");
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
    CHECK(t.totalCount() < ses.totalCount() / 2);
    CHECK(t.includes("power.voltage") && !t.includes("mot.lift"));

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
    testCatalogue();
    testFrames();
    testPower();
    testSensors();
    testParams();
    testFinal();
    testSession();
    return TEST_MAIN_RESULT();
}
