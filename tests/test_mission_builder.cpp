#include "check.hpp"
#include "core/mission.hpp"
#include "core/plane_modes.hpp"
#include "core/vehicle_state.hpp"

using namespace gcs;

static void testLayout() {
    const std::vector<UserWaypoint> wps = {
        {-35.36, 149.16, 80.0f}, {-35.35, 149.17, 90.0f}, {-35.34, 149.18, 100.0f}};
    const auto items = buildVtolMission(-35.363, 149.165, 584.0, wps, 40.0f);
    CHECK(items.size() == wps.size() + 2);

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

    for (size_t i = 0; i + 1 < wps.size(); ++i) {
        CHECK(items[i + 2].command == mav::kCmdNavWaypoint);
        CHECK(items[i + 2].x == toE7(wps[i].lat));
        CHECK(items[i + 2].y == toE7(wps[i].lon));
        CHECK_NEAR(items[i + 2].z, wps[i].alt_rel, 1e-6);
    }

    const auto& land = items.back();
    CHECK(land.command == mav::kCmdNavVtolLand);
    CHECK(land.x == toE7(wps.back().lat));
    CHECK(land.y == toE7(wps.back().lon));
    // no zero-length approach: the item before the land is a different point
    CHECK(items[items.size() - 2].x != land.x || items[items.size() - 2].y != land.y);

    // single point: home, takeoff, land at that point
    const auto one = buildVtolMission(-35.363, 149.165, 584.0, {wps[0]}, 40.0f);
    CHECK(one.size() == 3);
    CHECK(one[2].command == mav::kCmdNavVtolLand);
    CHECK(one[2].x == toE7(wps[0].lat));
    CHECK_NEAR(finalApproachLength(47.0, 8.0, {{47.001, 8.0, 80.0f}}), 111.3, 0.5);
    CHECK_NEAR(finalApproachLength(0, 0, {{47.0, 8.0, 80.0f}, {47.001, 8.0, 80.0f}}), 111.3, 0.5);

    CHECK(buildVtolMission(0, 0, 0, {}, 40.0f).empty());
}

static void testExtractRoundTrip() {
    const std::vector<UserWaypoint> wps = {{47.1, 8.1, 70.0f}, {47.2, 8.2, 75.0f}};
    const auto back = extractUserWaypoints(buildVtolMission(47.0, 8.0, 400.0, wps));
    CHECK(back.size() == 2);  // the land item is the last user point
    CHECK_NEAR(back[1].lat, 47.2, 1e-7);
    CHECK_NEAR(back[0].alt_rel, 70.0, 1e-6);
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
