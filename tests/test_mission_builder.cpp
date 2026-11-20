#include "check.hpp"
#include "core/geo.hpp"
#include "core/mission.hpp"
#include "core/plane_modes.hpp"
#include "core/vehicle_state.hpp"

#include <cmath>

using namespace gcs;

static void testLayout() {
    MissionPlan plan;
    plan.wps = {{-35.36, 149.16, 80.0f}, {-35.35, 149.17, 95.0f}, {-35.34, 149.18, 120.0f}};
    plan.land = UserWaypoint{-35.362, 149.166, 35.0f};
    ApproachInputs in;
    in.wind_from_deg = 270.0;
    in.wind_speed = 6.0;
    in.from_lat = -35.36;
    in.from_lon = 149.16;
    const LandingApproach appr = computeLandingApproach(*plan.land, in);
    const auto items = buildLoopMission(-35.363, 149.165, 584.0, plan, 40.0f, &appr);
    const size_t n = plan.wps.size();
    CHECK(items.size() == n + 6);  // home, takeoff, n wps, jump, land start, approach circle, land

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

    const auto& start = items[n + 3];
    CHECK(start.command == mav::kCmdDoLandStart);
    CHECK(start.x == toE7(-35.362));
    CHECK(seqToWaypointIndex(items, static_cast<int>(n + 3)) == -1);  // not a loop point
    const auto& circle = items[n + 4];
    CHECK(circle.command == mav::kCmdNavLoiterToAlt);
    CHECK(circle.x == toE7(appr.c_lat));
    CHECK_NEAR(std::abs(circle.param2), appr.radius, 1e-3);
    CHECK((circle.param2 < 0) == appr.ccw);
    CHECK_NEAR(circle.param4, 1.0, 1e-6);  // exit on the tangent
    CHECK_NEAR(circle.z, 35.0, 1e-6);

    const auto& land = items[n + 5];
    CHECK(land.command == mav::kCmdNavVtolLand);
    CHECK_NEAR(land.param1, 0.0, 1e-6);  // straight in, not the spiral option
    CHECK(land.x == toE7(-35.362));
    CHECK_NEAR(land.z, 0.0, 1e-6);  // ground: ArduPlane's descent profile reference
    CHECK_NEAR(extractPlan(items).land->alt_rel, 35.0, 1e-6);  // approach alt kept via the circle
    CHECK(landSeq(items) == static_cast<int>(n + 4));  // landing starts at the circle

    // the uploaded approach can be recovered for drawing
    const auto back = extractApproach(items);
    CHECK(back.has_value());
    CHECK_NEAR(back->heading_deg, appr.heading_deg, 0.5);
    CHECK_NEAR(back->leg_m, appr.leg_m, 2.0);
    CHECK(geo::distance(back->e_lat, back->e_lon, appr.e_lat, appr.e_lon) < 2.0);

    // a land point but no approach given: left out
    const auto noappr = buildLoopMission(-35.363, 149.165, 584.0, plan, 40.0f);
    CHECK(noappr.size() == n + 3);

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
    ApproachInputs in;
    in.from_lat = 47.1;
    in.from_lon = 8.1;
    const LandingApproach appr = computeLandingApproach(*plan.land, in);
    const auto items = buildLoopMission(47.0, 8.0, 400.0, plan, 40.0f, &appr);
    const auto back = extractPlan(items);
    CHECK(back.wps.size() == 3);  // the approach circle is not a loop point
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
    CHECK(seqToWaypointIndex(items, 6) == -1);  // approach circle
    CHECK(seqToWaypointIndex(items, 7) == -1);  // land

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
    CHECK(!hasFlyableMission(s));  // no located route point
    MissionPlan route;
    route.wps = {{47.0, 8.0, 80.0f}, {47.001, 8.0, 80.0f}};
    s.mission = buildLoopMission(47.0, 8.0, 400.0, route);
    CHECK(hasFlyableMission(s));
}

static void testApproachGeometry() {
    const UserWaypoint l{47.0, 8.0, 30.0f};
    ApproachInputs in;
    in.wind_from_deg = 270.0;  // west wind: land heading west
    in.wind_speed = 6.0;
    in.airspeed = 25.0;
    in.trans_decel = 2.0;
    in.radius = 80.0;
    in.from_lat = 47.01;  // aircraft north of the approach line
    in.from_lon = 8.01;
    const auto a = computeLandingApproach(l, in);

    CHECK_NEAR(a.heading_deg, 270.0, 1e-6);
    // leg: 1.5 * 25^2 / 4 = 234.4; *1.3 + 40
    CHECK_NEAR(transitionThreshold(25.0, 2.0), 234.375, 1e-3);
    CHECK_NEAR(a.leg_m, 234.375 * kLegMargin + kLegExtra, 1e-6);
    // exit is downwind (east) of L by the leg length
    CHECK_NEAR(geo::distance(a.e_lat, a.e_lon, l.lat, l.lon), a.leg_m, 0.5);
    CHECK_NEAR(geo::bearing(a.e_lat, a.e_lon, l.lat, l.lon), 270.0, 0.5);
    // centre is one radius from the exit, beside the line (not on L)
    CHECK_NEAR(geo::distance(a.c_lat, a.c_lon, a.e_lat, a.e_lon), 80.0, 0.5);
    CHECK(geo::distance(a.c_lat, a.c_lon, l.lat, l.lon) > a.leg_m);
    // heading west, the aircraft (north) is on the right -> centre north, CW
    CHECK(!a.ccw);
    CHECK(a.c_lat > a.e_lat);
    // tangent at the exit points at L: centre is perpendicular to the leg
    const double ce = geo::bearing(a.c_lat, a.c_lon, a.e_lat, a.e_lon);
    CHECK_NEAR(geo::wrap180(ce - (a.heading_deg - 90.0)), 0.0, 0.5);

    // aircraft south of the line: centre south, CCW
    in.from_lat = 46.99;
    const auto b = computeLandingApproach(l, in);
    CHECK(b.ccw);
    CHECK(b.c_lat < b.e_lat);

    // no wind: approach from where the aircraft is
    in.wind_speed = 0.5;
    in.from_lat = 47.0;
    in.from_lon = 8.02;  // east of L
    const auto c = computeLandingApproach(l, in);
    CHECK_NEAR(c.heading_deg, 270.0, 0.5);

    // short leg never below the minimum
    in.airspeed = 5.0;
    CHECK_NEAR(computeLandingApproach(l, in).leg_m, kMinLeg, 1e-6);
}

static void testLandingInMission() {
    MissionPlan plan;
    plan.wps = {{-35.36, 149.16, 80.0f}, {-35.35, 149.17, 95.0f}};
    plan.land = UserWaypoint{-35.362, 149.166, 35.0f};
    ApproachInputs in;
    in.from_lat = -35.36;
    in.from_lon = 149.16;
    const LandingApproach appr = computeLandingApproach(*plan.land, in);
    VehicleState s;
    s.mission = buildLoopMission(-35.363, 149.165, 584.0, plan, 40.0f, &appr);
    s.mission_known = true;
    s.custom_mode = plane::AUTO;
    const int lseq = landSeq(s.mission);
    CHECK(lseq > 2);
    s.mission_current = 2;  // on the loop
    CHECK(!isLandingInMission(s));
    s.mission_current = lseq;  // approach circle
    CHECK(isLandingInMission(s));
    s.mission_current = lseq + 1;  // VTOL land
    CHECK(isLandingInMission(s));
    s.custom_mode = plane::GUIDED;  // aborted to Hold
    CHECK(!isLandingInMission(s));
    s.custom_mode = plane::AUTO;
    s.mission_known = false;
    CHECK(!isLandingInMission(s));

    // resuming AUTO must start on a loop waypoint
    s.mission_known = true;
    s.mission_current = 1;  // VTOL takeoff: AUTO would hover again
    CHECK(resumeNeedsReset(s));
    s.mission_current = 0;
    CHECK(resumeNeedsReset(s));
    s.mission_current = lseq;  // approach circle
    CHECK(resumeNeedsReset(s));
    s.mission_current = 3;  // second loop waypoint
    CHECK(!resumeNeedsReset(s));
}

static void testNearestLoopSeq() {
    MissionPlan plan;
    plan.wps = {{-35.360, 149.160, 80.0f}, {-35.355, 149.170, 80.0f}, {-35.365, 149.175, 80.0f}};
    plan.land = UserWaypoint{-35.362, 149.166, 35.0f};
    ApproachInputs in;
    in.from_lat = -35.36;
    in.from_lon = 149.16;
    const LandingApproach appr = computeLandingApproach(*plan.land, in);
    const auto items = buildLoopMission(-35.363, 149.165, 584.0, plan, 40.0f, &appr);
    const int wp1 = 2, wp2 = 3, wp3 = 4;  // after HOME and the VTOL takeoff
    CHECK(seqToWaypointIndex(items, wp1) == 0 && seqToWaypointIndex(items, wp3) == 2);
    // 300 m from WP2: go to WP2
    double lat, lon;
    geo::offset(-35.355, 149.170, 300.0, 0.0, lat, lon);
    CHECK(nearestLoopSeq(items, lat, lon, 80.0) == wp2);
    // right on top of WP2: the next one
    CHECK(nearestLoopSeq(items, -35.355, 149.1701, 80.0) == wp3);
    // on top of the last: wraps to the first
    CHECK(nearestLoopSeq(items, -35.365, 149.175, 80.0) == wp1);
    // never the takeoff, approach or landing items
    const int n = nearestLoopSeq(items, plan.land->lat, plan.land->lon, 0.0);
    CHECK(n == wp1 || n == wp2 || n == wp3);
    CHECK(nearestLoopSeq({}, 0, 0, 0) == -1);
}

static std::vector<MissionItem> patternMission(Pattern pattern, size_t n, bool with_land) {
    MissionPlan plan;
    plan.pattern = pattern;
    for (size_t i = 0; i < n; ++i) {
        plan.wps.push_back({47.0 + 0.01 * static_cast<double>(i), 8.0 + 0.003 * static_cast<double>(i * i),
                            80.0f + 5.0f * static_cast<float>(i)});
    }
    LandingApproach appr;
    if (with_land) {
        plan.land = UserWaypoint{46.99, 7.99, 30.0f};
        ApproachInputs in;
        in.from_lat = 47.0;
        in.from_lon = 8.0;
        appr = computeLandingApproach(*plan.land, in);
    }
    return buildLoopMission(46.995, 7.995, 400.0, plan, 40.0f, with_land ? &appr : nullptr);
}

static void testPatterns() {
    // back and forth, 4 points: 1 2 3 4 3 2, jump -> 2
    {
        const auto items = patternMission(Pattern::BackForth, 4, true);
        CHECK(items.size() == 2 + 6 + 1 + 3);
        const int order[] = {0, 1, 2, 3, 2, 1};
        for (int k = 0; k < 6; ++k) {
            CHECK(items[2 + k].command == mav::kCmdNavWaypoint);
            CHECK(seqToWaypointIndex(items, 2 + k) == order[k]);
        }
        CHECK(items[8].command == mav::kCmdDoJump);
        CHECK_NEAR(items[8].param1, 2.0, 1e-6);
        CHECK(items[3].x == items[7].x && items[4].x == items[6].x);
        const MissionPlan back = extractPlan(items);
        CHECK(back.pattern == Pattern::BackForth);
        CHECK(back.wps.size() == 4);
        CHECK(back.land.has_value());
        CHECK_NEAR(back.wps[3].alt_rel, 95.0, 1e-4);
        CHECK(landSeq(items) == 10);
        // rebuilt from what came back: identical (wind refresh relies on the same count)
        CHECK(patternMission(Pattern::BackForth, 4, true).size() == items.size());
        // at WP3 heading back toward WP2: carry on to WP2 (seq 7); heading out: WP4 (seq 5)
        const auto& w3 = items[4];
        const double to_w2 = geo::bearing(w3.lat(), w3.lon(), items[3].lat(), items[3].lon());
        const double to_w4 = geo::bearing(w3.lat(), w3.lon(), items[5].lat(), items[5].lon());
        CHECK(nearestLoopSeq(items, w3.lat(), w3.lon(), 50.0, to_w2) == 7);
        CHECK(nearestLoopSeq(items, w3.lat(), w3.lon(), 50.0, to_w4) == 5);
        // far from WP3, flying back: rejoin at its way-back visit
        double lat = 0.0, lon = 0.0;
        geo::destination(w3.lat(), w3.lon(), to_w4, 300.0, lat, lon);
        CHECK(nearestLoopSeq(items, lat, lon, 50.0, to_w2) == 6);
    }
    // back and forth with 2 points is the same flight as the loop
    {
        const auto items = patternMission(Pattern::BackForth, 2, false);
        CHECK(items.size() == 2 + 2 + 1);
        CHECK(extractPlan(items).pattern == Pattern::Loop);
    }
    // 3 points
    {
        const auto items = patternMission(Pattern::BackForth, 3, false);
        CHECK(items.size() == 2 + 4 + 1);
        CHECK(extractPlan(items).pattern == Pattern::BackForth);
        CHECK(extractPlan(items).wps.size() == 3);
        CHECK(seqToWaypointIndex(items, 5) == 1);
    }
    // one way, then hold at the last point
    {
        const auto items = patternMission(Pattern::OneWay, 3, true);
        CHECK(items.size() == 2 + 3 + 3);
        CHECK(items[2].command == mav::kCmdNavWaypoint);
        CHECK(items[3].command == mav::kCmdNavWaypoint);
        CHECK(items[4].command == mav::kCmdNavLoiterUnlim);
        CHECK(items[5].command == mav::kCmdDoLandStart);
        const MissionPlan back = extractPlan(items);
        CHECK(back.pattern == Pattern::OneWay);
        CHECK(back.wps.size() == 3);
        CHECK(seqToWaypointIndex(items, 4) == 2);
        // at the last point: stay on the hold, don't wrap to WP1
        CHECK(nearestLoopSeq(items, items[4].lat(), items[4].lon(), 50.0) == 4);
        CHECK(nearestLoopSeq(items, items[2].lat(), items[2].lon(), 50.0) == 3);
    }
    // one point: a loiter whatever the pattern, read back as a loop
    for (Pattern pt : {Pattern::Loop, Pattern::BackForth, Pattern::OneWay}) {
        const auto items = patternMission(pt, 1, false);
        CHECK(items.size() == 3);
        CHECK(items[2].command == mav::kCmdNavLoiterUnlim);
        CHECK(extractPlan(items).pattern == Pattern::Loop);
    }
    // the hold radius and direction ride on the LOITER_UNLIM's param3
    {
        MissionPlan p;
        p.pattern = Pattern::OneWay;
        p.wps = {{47.0, 8.0, 80.0f}, {47.01, 8.0, 90.0f}};
        p.hold_radius = 150.0f;
        p.hold_ccw = true;
        const auto items = buildLoopMission(47.0, 8.0, 400.0, p);
        CHECK(items[3].command == mav::kCmdNavLoiterUnlim);
        CHECK_NEAR(items[3].param3, -150.0, 1e-4);
        const MissionPlan back = extractPlan(items);
        CHECK_NEAR(back.hold_radius, 150.0, 1e-4);
        CHECK(back.hold_ccw);
        CHECK(holdIndex(back) == 1);
        p.hold_radius = 0.0f;  // default radius, CCW: -1
        CHECK_NEAR(buildLoopMission(47.0, 8.0, 400.0, p)[3].param3, -1.0, 1e-6);
        CHECK(extractPlan(buildLoopMission(47.0, 8.0, 400.0, p)).hold_radius == 0.0f);
        CHECK(extractPlan(buildLoopMission(47.0, 8.0, 400.0, p)).hold_ccw);
        p.hold_ccw = false;
        CHECK_NEAR(buildLoopMission(47.0, 8.0, 400.0, p)[3].param3, 0.0, 1e-6);
        p.pattern = Pattern::Loop;
        CHECK(holdIndex(p) == -1);
        p.wps.resize(1);
        CHECK(holdIndex(p) == 0);
    }
    // a timed hold at a waypoint: LOITER_TIME with the time and the direction
    {
        MissionPlan p;
        p.pattern = Pattern::BackForth;
        p.wps = {{47.0, 8.0, 80.0f}, {47.01, 8.0, 90.0f}, {47.02, 8.0, 100.0f}};
        p.wps[1].hold_s = 30.0f;
        p.wps[1].loiter_ccw = true;
        const auto items = buildLoopMission(47.0, 8.0, 400.0, p);
        CHECK(items[2].command == mav::kCmdNavWaypoint);
        CHECK(items[3].command == mav::kCmdNavLoiterTime);
        CHECK_NEAR(items[3].param1, 30.0, 1e-6);
        CHECK_NEAR(items[3].param3, -1.0, 1e-6);
        CHECK_NEAR(items[3].param4, 0.0, 1e-6);
        CHECK(items[5].command == mav::kCmdNavLoiterTime);  // the way back holds there too
        CHECK(seqToWaypointIndex(items, 5) == 1);
        const MissionPlan back = extractPlan(items);
        CHECK(back.pattern == Pattern::BackForth);
        CHECK(back.wps.size() == 3);
        CHECK_NEAR(back.wps[1].hold_s, 30.0, 1e-6);
        CHECK(back.wps[1].loiter_ccw);
        CHECK(back.wps[0].hold_s == 0.0f);
        p.wps[1].loiter_ccw = false;
        CHECK_NEAR(buildLoopMission(47.0, 8.0, 400.0, p)[3].param3, 0.0, 1e-6);
        CHECK(!extractPlan(buildLoopMission(47.0, 8.0, 400.0, p)).wps[1].loiter_ccw);
    }
    // a hold with its own radius: LOITER_TURNS (turns, +-radius)
    {
        MissionPlan p;
        p.wps = {{47.0, 8.0, 80.0f}, {47.01, 8.0, 90.0f}, {47.02, 8.0, 100.0f}};
        p.wps[1].loiter_radius = 150.0f;
        p.wps[1].hold_turns = 2;
        p.wps[1].loiter_ccw = true;
        p.wps[2].loiter_radius = 404.0f;  // above 255 m: 10 m steps
        p.wps[2].hold_turns = 300;        // clamped to 255
        auto items = buildLoopMission(47.0, 8.0, 400.0, p);
        CHECK(items[3].command == mav::kCmdNavLoiterTurns);
        CHECK_NEAR(items[3].param1, 2.0, 1e-6);
        CHECK_NEAR(items[3].param3, -150.0, 1e-6);
        CHECK(items[4].command == mav::kCmdNavLoiterTurns);
        CHECK_NEAR(items[4].param3, 400.0, 1e-6);
        CHECK_NEAR(items[4].param1, 255.0, 1e-6);
        const MissionPlan back = extractPlan(items);
        CHECK(back.wps[1].hold_turns == 2 && back.wps[1].loiter_radius == 150.0f && back.wps[1].loiter_ccw);
        CHECK(back.wps[1].holds());
        CHECK(back.wps[2].loiter_radius == 400.0f && !back.wps[2].loiter_ccw);
        // a radius but no time and no turns: flies through
        p.wps[1].hold_turns = 0;
        items = buildLoopMission(47.0, 8.0, 400.0, p);
        CHECK(items[3].command == mav::kCmdNavWaypoint);
        // the time is what the operator sets: turns derived from it
        p.wps[1].hold_s = 90.0f;
        p.wps[1].hold_turns = holdTurns(90.0, 41.0);
        CHECK(p.wps[1].hold_turns == 4);       // covers 90 s (3 x 41) plus a spare
        items = buildLoopMission(47.0, 8.0, 400.0, p);
        CHECK(items[3].command == mav::kCmdNavLoiterTurns && items[3].param1 == 4.0f);
        CHECK(holdTurns(0.0, 40.0) == 0);
        CHECK(holdTurns(10.0, 40.0) == 2);
        CHECK(holdTurns(80.0, 40.0) == 3);
        CHECK(holdTurns(1e6, 10.0) == kMaxTurns);
        CHECK(storedTurnsRadius(256.0f) == 260.0f);
        CHECK(storedTurnsRadius(9000.0f) == 2550.0f);
    }
    // loop round trip keeps Loop
    CHECK(extractPlan(patternMission(Pattern::Loop, 4, true)).pattern == Pattern::Loop);

    // route length by pattern
    MissionPlan p;
    p.wps = {{47.0, 8.0, 0.0f}, {47.001, 8.0, 0.0f}, {47.002, 8.0, 0.0f}};
    p.pattern = Pattern::OneWay;
    CHECK_NEAR(routeLength(p), 2 * 111.2, 1.0);
    p.pattern = Pattern::BackForth;
    CHECK_NEAR(routeLength(p), 4 * 111.2, 1.5);
    p.pattern = Pattern::Loop;
    CHECK_NEAR(routeLength(p), 4 * 111.2, 1.5);
}

// No plan: a landing-only mission (Land and failsafes land at home)
static void testHomeLandingOnly() {
    MissionPlan plan;
    CHECK(buildLoopMission(-35.363, 149.165, 584.0, plan, 40.0f, nullptr).empty());
    plan.land = UserWaypoint{-35.363, 149.165, 30.0f};
    ApproachInputs in;
    in.from_lat = -35.363;
    in.from_lon = 149.165;
    const LandingApproach appr = computeLandingApproach(*plan.land, in);
    CHECK(buildLoopMission(-35.363, 149.165, 584.0, plan, 40.0f, nullptr).empty());  // no approach: nothing
    const auto items = buildLoopMission(-35.363, 149.165, 584.0, plan, 40.0f, &appr);
    CHECK(items.size() == 5);
    if (items.size() == 5) {
        CHECK(items[1].command == mav::kCmdNavVtolTakeoff);
        CHECK(items[2].command == mav::kCmdDoLandStart);
        CHECK(items[3].command == mav::kCmdNavLoiterToAlt);
        CHECK(items[4].command == mav::kCmdNavVtolLand);
        CHECK(items[4].seq == 4);
    }
    CHECK(landSeq(items) == 3);
    const MissionPlan back = extractPlan(items);
    CHECK(back.wps.empty());
    CHECK(back.land.has_value());
    if (back.land) {
        CHECK_NEAR(back.land->lat, -35.363, 1e-6);
    }
    CHECK(nearestLoopSeq(items, -35.363, 149.165, 50.0) == -1);

    VehicleState s;
    s.mission_known = true;
    s.mission = items;
    CHECK(!hasFlyableMission(s));  // nothing to fly: Takeoff, not Launch; no Resume Plan
}

static void testSameMission() {
    MissionPlan plan;
    plan.wps = {{-35.36, 149.16, 80.0f}, {-35.35, 149.17, 95.0f}};
    const auto a = buildLoopMission(-35.363, 149.165, 584.0, plan);
    auto b = a;
    CHECK(sameMission(a, b));
    b[0].z = 590.0f;  // the vehicle rewrites home
    b[0].x += 10;
    CHECK(sameMission(a, b));
    b[2].param1 = 30.0f;
    CHECK(!sameMission(a, b));
    b = a;
    b.pop_back();
    CHECK(!sameMission(a, b));
    CHECK(sameMission({}, {}));

    MissionPlan q = plan;
    CHECK(q == plan);
    q.wps[1].hold_turns = 5;  // derived: not an edit
    CHECK(q == plan);
    q.wps[1].hold_s = 30.0f;
    CHECK(q != plan);
    q = plan;
    q.land = UserWaypoint{-35.36, 149.16, 30.0f};
    CHECK(q != plan);
}

int main() {
    testHomeLandingOnly();
    testSameMission();
    testPatterns();
    testNearestLoopSeq();
    testLandingInMission();
    testApproachGeometry();
    testLayout();
    testExtractRoundTrip();
    testDerivedState();
    return TEST_MAIN_RESULT();
}
