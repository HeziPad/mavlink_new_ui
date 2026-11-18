#include "check.hpp"
#include "core/altitude.hpp"
#include "core/speed_mode.hpp"

#include <cmath>

using namespace gcs;
using namespace gcs::altitude;

static void testConversions() {
    Grounds g;
    g.home_amsl = 500.0;
    g.point = 560.0;   // a hill 60 m above home
    g.land = 490.0;    // L 10 m below home
    CHECK_NEAR(*toRelHome(700.0, Ref::Asl, g), 200.0, 1e-9);
    CHECK_NEAR(*toRelHome(100.0, Ref::Agl, g), 160.0, 1e-9);
    CHECK_NEAR(*toRelHome(100.0, Ref::Afl, g), 90.0, 1e-9);
    for (Ref r : {Ref::Asl, Ref::Agl, Ref::Afl}) {
        CHECK_NEAR(*fromRelHome(*toRelHome(123.0, r, g), r, g), 123.0, 1e-9);
    }
    Grounds none = g;
    none.point.reset();
    none.land.reset();
    CHECK(!toRelHome(100.0, Ref::Agl, none));
    CHECK_NEAR(*toRelHome(100.0, Ref::Afl, none), 100.0, 1e-9);  // L unknown: home level
}

static void testFloor() {
    Grounds g;
    g.home_amsl = 500.0;
    g.point = 560.0;
    CHECK_NEAR(minRelHome(g), 90.0, 1e-9);         // 60 m hill + 30 m
    CHECK_NEAR(minRelHome(g, kMinApproachAgl), 80.0, 1e-9);
    Check c = check(25.0, Ref::Agl, g);             // 25 m above the hill: too low
    CHECK(!c.ok && c.min_rel == 90.0);
    c = check(30.0, Ref::Agl, g);
    CHECK(c.ok && std::abs(c.rel_home - 90.0) < 1e-9);
    c = check(80.0, Ref::Asl, g);                   // below the ground there
    CHECK(!c.ok);
    c = check(6000.0, Ref::Asl, g);
    CHECK(!c.ok);
    // no terrain at the point: home level is the floor, AGL can't be used
    g.point.reset();
    CHECK_NEAR(minRelHome(g), 30.0, 1e-9);
    CHECK(!check(100.0, Ref::Agl, g).ok);
    CHECK(check(100.0, Ref::Afl, g).ok);
}

static void testSpeed() {
    using namespace gcs::speed;
    CHECK(target(Mode::Eco, Situation::Circling) == Target::Loiter);
    CHECK(target(Mode::Eco, Situation::EnRoute) == Target::Cruise);
    CHECK(target(Mode::Eco, Situation::Descending) == Target::Max);
    CHECK(target(Mode::Normal, Situation::Circling) == Target::Cruise);
    CHECK(target(Mode::Normal, Situation::Descending) == Target::Cruise);
    CHECK(target(Mode::Fast, Situation::EnRoute) == Target::Max);
    CHECK(target(Mode::Fast, Situation::Circling) == Target::Cruise);
    CHECK(loiterSpeed(13.0, 25.0, 18.0) == 20.0);  // SITL's quadplane: assist above min
    CHECK(loiterSpeed(16.0, 25.0, 12.0) == 16.0);
    CHECK(loiterSpeed(16.0, 25.0, 0.0) == 16.0);   // Q_ASSIST off
    CHECK(loiterSpeed(13.0, 18.0, 18.0) == 18.0);  // never faster than cruise
    for (Mode m : {Mode::Eco, Mode::Normal, Mode::Fast}) {
        CHECK(target(m, Situation::Landing) == Target::Cruise);
        CHECK(target(m, Situation::None) == Target::Default);
    }
}

int main() {
    testConversions();
    testFloor();
    testSpeed();
    return TEST_MAIN_RESULT();
}
