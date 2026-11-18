#include "check.hpp"
#include "core/endurance.hpp"
#include "core/geo.hpp"

using namespace gcs;
using namespace gcs::endurance;

static Inputs baseInputs() {
    Inputs in;
    in.capacity_mah = 10000.0;
    in.consumed_mah = 2000.0;
    in.battery_pct = 80;
    in.current_a = 12.0;
    in.cruise_a = 10.0;
    in.hover_a = 60.0;
    in.airspeed = 20.0;
    in.land_lat = 47.0;
    in.land_lon = 8.0;
    in.approach_alt = 30.0;
    in.approach_leg_m = 200.0;
    in.approach_radius = 80.0;
    geo::destination(47.0, 8.0, 90.0, 2000.0, in.here.lat, in.here.lon);
    in.here.alt = 80.0;
    return in;
}

static void testGroundSpeed() {
    CHECK_NEAR(groundSpeed(20.0, 0.0, 0.0, 0.0), 20.0, 1e-9);
    CHECK_NEAR(groundSpeed(20.0, 0.0, 0.0, 5.0), 15.0, 1e-9);     // flying north into a north wind
    CHECK_NEAR(groundSpeed(20.0, 180.0, 0.0, 5.0), 25.0, 1e-9);   // tailwind
    CHECK_NEAR(groundSpeed(20.0, 90.0, 0.0, 12.0), 16.0, 1e-9);   // crosswind: sqrt(400-144)
    CHECK_NEAR(groundSpeed(20.0, 0.0, 0.0, 30.0), kMinGroundSpeed, 1e-9);
}

static void testReturnCost() {
    Inputs in = baseInputs();
    const ReturnCost calm = returnCost(in, in.here, 10.0, 60.0);
    // rotors: 15 + 24/1.5 + 6/0.5 + 30 = 73 s
    CHECK_NEAR(calm.rotor_s, 73.0, 1e-9);
    CHECK(calm.cruise_s > 2000.0 / 20.0);
    const double expect = (10.0 * calm.cruise_s + 60.0 * calm.rotor_s) / 3.6 * kReturnMargin;
    CHECK_NEAR(calm.mah, expect, 1e-6);

    // the aircraft is east of L, so it flies back west: a wind from the west is the headwind
    in.wind_from_deg = 270.0;
    in.wind_speed = 6.0;
    const ReturnCost head = returnCost(in, in.here, 10.0, 60.0);
    in.wind_from_deg = 90.0;
    const ReturnCost tail = returnCost(in, in.here, 10.0, 60.0);
    CHECK(head.mah > calm.mah);
    CHECK(head.mah > tail.mah);

    // high up: extra circling to come down on the wing
    in = baseInputs();
    RoutePoint high = in.here;
    high.alt = 600.0;
    CHECK(returnCost(in, high, 10.0, 60.0).cruise_s > calm.cruise_s);
}

static void testEstimate() {
    Inputs in = baseInputs();
    Estimate e = estimate(in);
    CHECK(e.valid);
    // the lower of the gauges: 8000 by mAh, 8000 by % -> 8000; with 70 % it is 7000
    CHECK_NEAR(e.remaining_mah, 8000.0, 1e-6);
    in.battery_pct = 70;
    e = estimate(in);
    CHECK_NEAR(e.remaining_mah, 7000.0, 1e-6);
    CHECK_NEAR(e.reserve_mah, 2500.0, 1e-6);
    const double expect_left = (7000.0 - 2500.0 - e.worst.mah) / (10.0 / 3.6);
    CHECK_NEAR(e.time_left_s, expect_left, 1e-6);
    CHECK(e.time_left_s > kCautionS);
    CHECK(e.level == Level::None);

    // a far waypoint dominates the return cost and shortens the time left
    RoutePoint far;
    geo::destination(47.0, 8.0, 0.0, 8000.0, far.lat, far.lon);
    far.alt = 80.0;
    in.route = {far};
    const Estimate with_far = estimate(in);
    CHECK(with_far.worst.mah > with_far.here.mah);
    CHECK(with_far.time_left_s < e.time_left_s);

    // nearly empty: warning, then critical
    in.route.clear();
    in.battery_pct = -1;
    // 50 mAh (18 s of cruise) above what the return from here plus the reserve need
    in.consumed_mah = 10000.0 - 2500.0 - e.here.mah - 50.0;
    Estimate low = estimate(in);
    CHECK(low.level == Level::Caution);
    in.consumed_mah += 100.0;
    low = estimate(in);
    CHECK(low.level == Level::Critical);  // here cost no longer fits above the reserve
    CHECK(low.time_left_s < 0.0);

    // not learned yet: instantaneous current x 1.3, hover 4 x cruise
    in = baseInputs();
    in.cruise_a = -1.0;
    in.hover_a = -1.0;
    e = estimate(in);
    CHECK(!e.valid);  // still in the takeoff hover: no estimate, no card
    CHECK(advisories(e, Level::None).empty());
    in.in_cruise = true;
    e = estimate(in);
    CHECK(e.valid);
    CHECK_NEAR(e.cruise_a, 12.0 * kUnlearnedFactor, 1e-9);
    CHECK_NEAR(e.hover_a, 12.0 * kUnlearnedFactor * kHoverFactor, 1e-9);

    // unknown capacity / current: no estimate, and a card saying why
    in = baseInputs();
    in.capacity_mah = 0.0;
    e = estimate(in);
    CHECK(!e.valid);
    CHECK(advisories(e, Level::None).size() == 1);
    in = baseInputs();
    in.cruise_a = -1.0;
    in.current_a = -1.0;
    CHECK(!estimate(in).valid);
}

static void testTracker() {
    CurrentTracker t;
    CHECK(t.cruiseAmps() < 0.0);
    double time = 0.0;
    for (int i = 0; i < 5; ++i, time += 2.0) {
        t.addSample(time, 50.0, Phase::Hover);
    }
    CHECK_NEAR(t.hoverAmps(), 50.0, 1e-9);
    for (int i = 0; i < 60; ++i, time += 2.0) {
        t.addSample(time, 10.0, Phase::Cruise);
    }
    CHECK_NEAR(t.cruiseAmps(), 10.0, 1e-6);
    // a climb: the fast average shows it at once
    for (int i = 0; i < 5; ++i, time += 2.0) {
        t.addSample(time, 20.0, Phase::Cruise);
    }
    CHECK(t.cruiseAmps() > 14.0);
    // ... and a lull after it doesn't drop it below the slow average
    for (int i = 0; i < 5; ++i, time += 2.0) {
        t.addSample(time, 5.0, Phase::Cruise);
    }
    CHECK(t.cruiseAmps() > 9.0);
    // other phases don't count
    const double before = t.cruiseAmps();
    t.addSample(time, 100.0, Phase::Other);
    CHECK_NEAR(t.cruiseAmps(), before, 1e-9);
}

static void testLatch() {
    LevelLatch l;
    CHECK(l.update(Level::Caution, 0.0) == Level::Caution);
    CHECK(l.update(Level::None, 1.0) == Level::Caution);
    CHECK(l.update(Level::None, 20.0) == Level::Caution);
    CHECK(l.update(Level::None, 31.5) == Level::None);
    CHECK(l.update(Level::Critical, 32.0) == Level::Critical);  // rises at once
    CHECK(l.update(Level::Caution, 33.0) == Level::Critical);
    CHECK(l.update(Level::Critical, 40.0) == Level::Critical);  // back up: the clear timer restarts
    CHECK(l.update(Level::Caution, 41.0) == Level::Critical);
    CHECK(l.update(Level::Caution, 70.0) == Level::Critical);
    CHECK(l.update(Level::Caution, 71.5) == Level::Caution);
}

static void testText() {
    CHECK(clockText(860.0) == "14:20");
    CHECK(clockText(-5.0) == "0:00");
    CHECK(clockText(3725.0) == "1:02:05");
    Estimate e;
    e.valid = true;
    e.time_left_s = 200.0;
    CHECK(advisories(e, Level::None).empty());
    CHECK(advisories(e, Level::Caution).size() == 1);
    CHECK(advisories(e, Level::Critical)[0].severity == Severity::Critical);
}

int main() {
    testGroundSpeed();
    testReturnCost();
    testEstimate();
    testTracker();
    testLatch();
    testText();
    return TEST_MAIN_RESULT();
}
