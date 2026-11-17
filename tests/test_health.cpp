#include "check.hpp"
#include "core/health.hpp"

#include <algorithm>
#include <string>

using namespace gcs;

namespace {

const Clock::time_point T0 = Clock::time_point{} + std::chrono::hours(1);

VehicleState healthy() {
    VehicleState s;
    s.has_heartbeat = true;
    s.last_heartbeat = T0;
    s.has_ekf = true;
    s.ekf_flags = 1 | 2 | 4 | 8 | 16 | 32;
    s.has_gps = true;
    s.gps_fix = 3;
    s.gps_sats = 12;
    s.gps_hdop = 0.8;
    s.has_sys_status = true;
    s.sensors_present = s.sensors_enabled = s.sensors_health = 0x1 | 0x2 | 0x4 | 0x8 | 0x20 | 0x10000000;
    s.has_home = true;
    s.battery_pct = 90;
    s.q_guided_mode = 0;
    return s;
}

const Advisory* find(const std::vector<Advisory>& v, const std::string& id) {
    for (const auto& a : v) {
        if (a.id == id) {
            return &a;
        }
    }
    return nullptr;
}

void testHealthyIsQuiet() {
    const auto v = assess(healthy(), T0, true);
    CHECK(v.empty());
}

void testLink() {
    VehicleState s = healthy();
    s.last_heartbeat = T0 - std::chrono::seconds(10);
    auto v = assess(s, T0, true);
    CHECK(v.size() == 1);
    CHECK(find(v, "link.lost") && find(v, "link.lost")->severity == Severity::Critical);
    v = assess(s, T0, false);
    CHECK(find(v, "link.wait") && find(v, "link.wait")->severity == Severity::Info);
}

void testEkfFlags() {
    VehicleState s = healthy();
    s.ekf_flags = 1024;  // uninitialised: one card, not a flood
    auto v = assess(s, T0, true);
    CHECK(find(v, "ekf.init"));
    CHECK(!find(v, "ekf.position"));

    s.ekf_flags = 1 | 4 | 32;  // no horizontal position
    v = assess(s, T0, true);
    const Advisory* a = find(v, "ekf.position");
    CHECK(a && a->severity == Severity::Warning);  // on the ground
    CHECK(a && !a->suggestions.empty());
    CHECK(a && a->detail.find("0x0025") != std::string::npos);

    s.has_vfr_hud = true;  // airborne: the same thing is critical
    s.armed = true;
    s.airspeed = 20.0;
    s.has_ext_state = true;
    s.landed = LandedState::InAir;
    v = assess(s, T0, true);
    a = find(v, "ekf.position");
    CHECK(a && a->severity == Severity::Critical);
    CHECK(v.front().severity == Severity::Critical);  // sorted worst first

    s = healthy();
    s.ekf_flags |= 32768;
    CHECK(find(assess(s, T0, true), "ekf.glitch"));
}

void testEkfVariances() {
    VehicleState s = healthy();
    s.ekf_mag_var = 0.6;
    auto v = assess(s, T0, true);
    CHECK(find(v, "ekf.var.mag") && find(v, "ekf.var.mag")->severity == Severity::Caution);
    s.ekf_mag_var = 0.9;
    v = assess(s, T0, true);
    CHECK(find(v, "ekf.var.mag") && find(v, "ekf.var.mag")->severity == Severity::Warning);
    s.ekf_mag_var = 0.3;
    CHECK(!find(assess(s, T0, true), "ekf.var.mag"));
}

void testSensorsAndPrearm() {
    VehicleState s = healthy();
    s.sensors_present |= 0x10;  // airspeed present + enabled, unhealthy
    s.sensors_enabled |= 0x10;
    s.sensors_health &= ~0x10000000u;  // pre-arm failing
    auto v = assess(s, T0, true);
    CHECK(find(v, "sensor.airspeed"));
    CHECK(find(v, "prearm"));
    s.armed = true;  // pre-arm only matters while disarmed
    CHECK(!find(assess(s, T0, true), "prearm"));
    s = healthy();
    s.sensors_present |= 0x400000;  // terrain present but disabled: not reported
    CHECK(!find(assess(s, T0, true), "sensor.terrain"));
}

void testGpsBatteryParams() {
    VehicleState s = healthy();
    s.gps_fix = 2;
    CHECK(find(assess(s, T0, true), "gps.fix"));
    s = healthy();
    s.gps_sats = 4;
    CHECK(find(assess(s, T0, true), "gps.sats"));
    s = healthy();
    s.battery_pct = 10;
    CHECK(find(assess(s, T0, true), "battery.critical"));
    s.battery_pct = 25;
    CHECK(find(assess(s, T0, true), "battery.low"));
    s = healthy();
    s.q_guided_mode = 1;
    CHECK(find(assess(s, T0, true), "param.q_guided_mode"));
}

void testStatusText() {
    auto a = explainStatusText("PreArm: Compass not calibrated", 2);
    CHECK(a && a->id == "prearm.txt.compass_cal");
    CHECK(a && a->title.find("Compass") != std::string::npos);
    CHECK(a && a->detail == "PreArm: Compass not calibrated");

    a = explainStatusText("PreArm: Throttle (RC3) is not neutral", 2);
    CHECK(a && a->id == "prearm.txt.throttle");
    a = explainStatusText("PreArm: VTOL land too short, min 197m", 2);
    CHECK(a && a->id == "prearm.txt.vtol_land_short");
    a = explainStatusText("PreArm: QRTL mode not armable", 2);
    CHECK(a && a->id == "prearm.txt.mode_not_armable");
    a = explainStatusText("PreArm: Something brand new", 2);
    CHECK(a && a->id.rfind("prearm.other:", 0) == 0);
    CHECK(a && a->cause.find("Something brand new") != std::string::npos);

    a = explainStatusText("Failsafe. Short event on: type=1/reason=3", 2);
    CHECK(a && a->severity == Severity::Critical);
    a = explainStatusText("EKF variance", 2);
    CHECK(a && a->id == "txt.ekf_failsafe");

    CHECK(!explainStatusText("EKF3 IMU0 is using GPS", 6));   // info chatter
    CHECK(!explainStatusText("Throttle armed", 6));
    CHECK(!explainStatusText("Throttle failsafe off", 4));    // status, not a problem
    CHECK(!explainStatusText("GPS Glitch cleared", 2));
    CHECK(!explainStatusText("Mission: 3 WP", 6));
}

void testBook() {
    AdvisoryBook book;
    VehicleState s = healthy();
    CHECK(book.onStatusText("PreArm: Compass not calibrated", 2, T0).has_value());
    CHECK(!book.onStatusText("PreArm: Compass not calibrated", 2, T0 + std::chrono::seconds(30)).has_value());
    book.update(s, T0 + std::chrono::seconds(40));
    CHECK(book.current().size() == 1);  // refreshed by the repeat at +30 s
    book.update(s, T0 + std::chrono::seconds(70));
    CHECK(book.current().empty());      // expired

    book.onStatusText("PreArm: Compass not calibrated", 2, T0);
    book.onStatusText("Failsafe. Long event on: type=2", 2, T0);
    book.update(s, T0);
    s.armed = true;
    book.update(s, T0 + std::chrono::seconds(1));
    const auto cur = book.current();
    CHECK(cur.size() == 1 && cur[0].id == "txt.failsafe");  // arming clears pre-arm only

    const auto merged = mergeAdvisories(assess(healthy(), T0, true), cur);
    CHECK(merged.size() == 1);
}

}  // namespace

int main() {
    testHealthyIsQuiet();
    testLink();
    testEkfFlags();
    testEkfVariances();
    testSensorsAndPrearm();
    testGpsBatteryParams();
    testStatusText();
    testBook();
    return TEST_MAIN_RESULT();
}
