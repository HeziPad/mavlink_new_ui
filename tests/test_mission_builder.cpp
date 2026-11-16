#include "check.hpp"
#include "core/mission.hpp"
#include "core/plane_modes.hpp"
#include "core/vehicle_state.hpp"

using namespace gcs;

static void testLayout() {
    MissionPlan plan;
    plan.wps = {{-35.36, 149.16, 80.0f}, {-35.35, 149.17, 95.0f}, {-35.34, 149.18, 120.0f}};
    plan.land = UserWaypoint{-35.362, 149.166, 35.0f};
    const auto items = buildLoopMission(-35.363, 149.165, 584.0, plan, 40.0f);
    const size_t n = plan.wps.size();
    CHECK(items.size() == n + 4);  // home, takeoff, n wps, jump, land

    for (size_t i = 0; i < items.size(); ++i) {
        CHECK(items[i].seq == i);
        CHECK(items[i].autocontinue == 1);
    }

    CHECK(items[0].command == mav::kCmdNavWaypoint);
    CHECK(items[0].frame == mav::kFrameGlobal);
    CHECK(items[0].x == toE7(-35.363));
    CHECK_NEAR(items[0].z, 584.0, 1e-3);

    CHECK(items[1].command == mav::kCmdNavVtolTakeoff);
    CHECK_NEAR(items[1].z, 40.0, 1e-6);
    CHECK(items[1].frame == mav::kFrameGlobalRelativeAlt);

    for (size_t i = 0; i < n; ++i) {
        const auto& it = items[i + 2];
        CHECK(it.command == mav::kCmdNavWaypoint);
        CHECK(it.x == toE7(plan.wps[i].lat));
        CHECK(it.y == toE7(plan.wps[i].lon));
        CHECK_NEAR(it.z, plan.wps[i].alt_rel, 1e-6);  // per-point altitude kept
    }

    const auto& jump = items[n + 2];
    CHECK(jump.command == mav::kCmdDoJump);
    CHECK_NEAR(jump.param1, 2.0, 1e-6);
    CHECK_NEAR(jump.param2, -1.0, 1e-6);  // forever

    const auto& land = items[n + 3];
    CHECK(land.command == mav::kCmdNavVtolLand);
    CHECK_NEAR(land.param1, mav::kVtolLandFwSpiral, 1e-6);
    CHECK(land.x == toE7(-35.362));
    CHECK_NEAR(land.z, 35.0, 1e-6);  // approach altitude, never 0 (= "current alt")
    CHECK(landSeq(items) == static_cast<int>(n + 3));

    // no land point: ends with the jump
    plan.land.reset();
    const auto noland = buildLoopMission(-35.363, 149.165, 584.0, plan, 40.0f);
    CHECK(noland.size() == n + 3);
    CHECK(noland.back().command == mav::kCmdDoJump);
    CHECK(landSeq(noland) == -1);

    // single point: circle it instead of jumping
    MissionPlan one;
    one.wps = {plan.wps[1]};
    const auto o = buildLoopMission(-35.363, 149.165, 584.0, one, 40.0f);
    CHECK(o.size() == 3);
    CHECK(o[2].command == mav::kCmdNavLoiterUnlim);
    CHECK_NEAR(o[2].z, 95.0, 1e-6);

    CHECK(buildLoopMission(0, 0, 0, MissionPlan{}, 40.0f).empty());
}

static void testExtractRoundTrip() {
    MissionPlan plan;
    plan.wps = {{47.1, 8.1, 70.0f}, {47.2, 8.2, 75.0f}, {47.15, 8.3, 90.0f}};
    plan.land = UserWaypoint{47.05, 8.05, 0.0f};
    const auto items = buildLoopMission(47.0, 8.0, 400.0, plan);
    const auto back = extractPlan(items);
    CHECK(back.wps.size() == 3);
    CHECK_NEAR(back.wps[1].lat, 47.2, 1e-7);
    CHECK_NEAR(back.wps[2].alt_rel, 90.0, 1e-6);
    CHECK(back.land.has_value());
    CHECK_NEAR(back.land->lon, 8.05, 1e-7);
    CHECK_NEAR(back.land->alt_rel, kDefaultLandApproachAlt, 1e-6);  // 0 in -> default

    CHECK(seqToWaypointIndex(items, 0) == -1);  // home
    CHECK(seqToWaypointIndex(items, 1) == -1);  // takeoff
    CHECK(seqToWaypointIndex(items, 2) == 0);
    CHECK(seqToWaypointIndex(items, 4) == 2);
    CHECK(seqToWaypointIndex(items, 5) == -1);  // jump
    CHECK(seqToWaypointIndex(items, 6) == -1);  // land

    // single point round-trips through LOITER_UNLIM
    MissionPlan one;
    one.wps = {{47.1, 8.1, 70.0f}};
    CHECK(extractPlan(buildLoopMission(47.0, 8.0, 400.0, one)).wps.size() == 1);

    // legacy layout (land at the last point) loads as waypoints + land point
    std::vector<MissionItem> legacy(4);
    legacy[1].command = mav::kCmdNavVtolTakeoff;
    legacy[2].command = mav::kCmdNavWaypoint;
    legacy[2].x = toE7(47.1);
    legacy[2].y = toE7(8.1);
    legacy[3].command = mav::kCmdNavVtolLand;
    legacy[3].x = toE7(47.2);
    legacy[3].y = toE7(8.2);
    const auto lp = extractPlan(legacy);
    CHECK(lp.wps.size() == 1);
    CHECK(lp.land.has_value());
    CHECK_NEAR(lp.land->alt_rel, kDefaultLandApproachAlt, 1e-6);

    CHECK_NEAR(lapLength({{47.0, 8.0, 0.0f}, {47.001, 8.0, 0.0f}}), 2 * 111.2, 1.0);
    CHECK_NEAR(lapLength({{47.0, 8.0, 0.0f}}), 0.0, 1e-9);
}

static void testDerivedState() {
    VehicleState s;
    CHECK(effectiveVtolState(s) == VtolState::Unknown);
    s.has_heartbeat = true;
    s.custom_mode = plane::QLOITER;
    CHECK(effectiveVtolState(s) == VtolState::Hover);
    s.custom_mode = plane::AUTO;
    s.has_vfr_hud = true;
    s.airspeed = 20.0;
    s.throttle_pct = 50;
    CHECK(effectiveVtolState(s) == VtolState::Cruise);
    s.has_ext_state = true;
    s.reported_vtol = VtolState::TransitionToHover;
    CHECK(effectiveVtolState(s) == VtolState::TransitionToHover);

    s.landed = LandedState::OnGround;
    CHECK(!isAirborne(s));
    s.reported_vtol = VtolState::Cruise;  // ArduPlane says FW while parked
    CHECK(effectiveVtolState(s) == VtolState::Hover);
    s.landed = LandedState::Takeoff;
    CHECK(isAirborne(s));

    CHECK(!hasFlyableMission(s));
    s.mission_known = true;
    s.mission.resize(1);
    CHECK(!hasFlyableMission(s));
    s.mission.resize(4);
    CHECK(hasFlyableMission(s));
}

int main() {
    testLayout();
    testExtractRoundTrip();
    testDerivedState();
    return TEST_MAIN_RESULT();
}
