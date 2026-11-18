#include "check.hpp"
#include "core/flight_status.hpp"
#include "core/geo.hpp"
#include "core/plane_modes.hpp"

using namespace gcs;

static VehicleState flying(uint32_t mode) {
    VehicleState s;
    s.has_heartbeat = true;
    s.armed = true;
    s.custom_mode = mode;
    s.mode_name = plane::modeName(mode);
    s.has_ext_state = true;
    s.landed = LandedState::InAir;
    s.reported_vtol = VtolState::Cruise;
    s.has_position = true;
    s.lat = 47.0;
    s.lon = 8.0;
    s.alt_rel = 80.0;
    s.vn = 20.0;
    s.has_vfr_hud = true;
    s.airspeed = 20.0;
    return s;
}

static std::vector<MissionItem> mission(Pattern pattern) {
    MissionPlan p;
    p.pattern = pattern;
    p.wps = {{47.01, 8.0, 80.0f}, {47.02, 8.0, 80.0f}, {47.03, 8.0, 80.0f}};
    p.land = UserWaypoint{47.0, 8.0, 30.0f};
    ApproachInputs in;
    in.from_lat = 47.01;
    in.from_lon = 8.0;
    const LandingApproach a = computeLandingApproach(*p.land, in);
    return buildLoopMission(47.0, 8.0, 400.0, p, 40.0f, &a);
}

static void testStates() {
    const auto now = Clock::now();
    FlightContext ctx;
    VehicleState s;
    CHECK(flightStatus(s, ctx, now).label == "—");
    s.has_heartbeat = true;
    s.has_ext_state = true;
    s.landed = LandedState::OnGround;
    CHECK(flightStatus(s, ctx, now).label == "ON GROUND");
    s.armed = true;
    CHECK(flightStatus(s, ctx, now).label == "ARMED · ON GROUND");

    s = flying(plane::AUTO);
    s.mission = mission(Pattern::Loop);
    s.mission_known = true;
    s.mission_current = 3;
    CHECK(flightStatus(s, ctx, now).label == "ON PLAN · WP2");
    CHECK(flightStatus(s, ctx, now).tone == Tone::Good);
    s.mission_current = landSeq(s.mission);
    CHECK(flightStatus(s, ctx, now).label == "LANDING · APPROACH");
    s.reported_vtol = VtolState::Hover;
    CHECK(flightStatus(s, ctx, now).label == "LANDING · VERTICAL");

    s = flying(plane::AUTO);
    s.mission = mission(Pattern::OneWay);
    s.mission_known = true;
    s.mission_current = 4;  // the LOITER_UNLIM at WP3
    CHECK(flightStatus(s, ctx, now).label == "END HOLD");

    s = flying(plane::GUIDED);
    ctx.orbit = true;
    ctx.orbit_hold = true;
    CHECK(flightStatus(s, ctx, now).label == "HOLD");
    ctx.orbit_hold = false;
    CHECK(flightStatus(s, ctx, now).label == "FLY TO");
    ctx = {};
    ctx.transition_leg = true;
    s.reported_vtol = VtolState::TransitionToCruise;
    CHECK(flightStatus(s, ctx, now).label == "TRANSITION");
    s.reported_vtol = VtolState::Cruise;
    CHECK(flightStatus(s, ctx, now).label == "CLIMB-OUT");

    ctx = {};
    s = flying(plane::QLAND);
    CHECK(flightStatus(s, ctx, now).label == "QLAND · LANDING HERE");
    s = flying(plane::RTL);
    CHECK(flightStatus(s, ctx, now).label == "RTL");
    s = flying(plane::FBWA);
    CHECK(flightStatus(s, ctx, now).label == s.mode_name);
}

static void testTargets() {
    VehicleState s = flying(plane::AUTO);
    s.mission = mission(Pattern::Loop);
    s.mission_known = true;
    s.mission_current = 2;
    NavTarget t = missionTarget(s);
    CHECK(t.valid && t.name == "WP1");
    CHECK_NEAR(t.dist_m, 1112.0, 5.0);
    CHECK_NEAR(t.eta_s, t.dist_m / 20.0, 1e-6);
    {
        // a waypoint hold: on its circle it has arrived (no ETA counting on)
        VehicleState h = s;
        auto& it = h.mission[2];
        it.command = mav::kCmdNavLoiterTime;
        it.param1 = 60.0f;
        it.param3 = 0.0f;
        h.loiter_radius = 100.0;
        CHECK(!missionTarget(h).arrived);  // still 1.1 km away
        h.lat = it.lat() + 0.001;          // ~110 m from the centre: on the circle
        h.lon = it.lon();
        NavTarget ht = missionTarget(h);
        CHECK(ht.valid && ht.name == "WP1" && ht.arrived && ht.eta_s == 0.0);
        it.command = mav::kCmdNavLoiterTurns;  // own radius in param3
        it.param3 = -400.0f;
        h.lat = it.lat() + 0.0036;          // ~400 m out
        CHECK(missionTarget(h).arrived);
        it.param3 = -150.0f;
        CHECK(!missionTarget(h).arrived);   // 400 m from a 150 m circle
        CHECK(onCircle(120.0, 100.0) && !onCircle(170.0, 100.0));
    }
    s.mission_current = landSeq(s.mission);
    CHECK(missionTarget(s).name == "APPROACH");
    s.mission_current = landSeq(s.mission) + 1;
    CHECK(missionTarget(s).name == "L");
    s.custom_mode = plane::GUIDED;
    CHECK(!missionTarget(s).valid);
    // a circle: the distance to its edge
    t = pointTarget(s, "HOLD", 47.01, 8.0, 100.0);
    CHECK_NEAR(t.dist_m, 1012.0, 5.0);
    // too slow for an ETA
    s.vn = 1.0;
    CHECK(pointTarget(s, "X", 47.01, 8.0).eta_s < 0.0);
    CHECK(etaSeconds(100.0, 2.9) < 0.0);
    CHECK_NEAR(etaSeconds(100.0, 4.0), 25.0, 1e-9);
}

static void testRangefinder() {
    const auto now = Clock::now();
    VehicleState s;
    CHECK(!rangefinderAgl(s, now));
    s.has_rangefinder = true;
    s.rangefinder_orient = kRangefinderDown;
    s.rangefinder_min = 0.2;
    s.rangefinder_max = 40.0;
    s.rangefinder_m = 12.0;
    s.rangefinder_quality = 0;
    s.rangefinder_rx = now;
    CHECK(rangefinderAgl(s, now) && std::abs(*rangefinderAgl(s, now) - 12.0) < 1e-9);
    // tilt correction
    s.roll_deg = 20.0;
    CHECK_NEAR(*rangefinderAgl(s, now), 12.0 * std::cos(20.0 * geo::kDegToRad), 1e-9);
    s.roll_deg = 35.0;
    CHECK(!rangefinderAgl(s, now));   // banked too steeply
    s.roll_deg = 0.0;
    s.rangefinder_m = 40.0;
    CHECK(!rangefinderAgl(s, now));   // max range: nothing seen
    s.rangefinder_m = 38.5;
    CHECK(!rangefinderAgl(s, now));   // within 5 % of max
    s.rangefinder_m = 0.1;
    CHECK(!rangefinderAgl(s, now));   // below min
    s.rangefinder_m = 12.0;
    s.rangefinder_quality = 1;
    CHECK(!rangefinderAgl(s, now));   // flagged invalid
    s.rangefinder_quality = 80;
    CHECK(rangefinderAgl(s, now).has_value());
    CHECK(!rangefinderAgl(s, now + std::chrono::seconds(2)));  // stale
    s.rangefinder_orient = 0;
    CHECK(!rangefinderAgl(s, now));   // pointing forward
}

int main() {
    testStates();
    testTargets();
    testRangefinder();
    return TEST_MAIN_RESULT();
}
