#include "check.hpp"
#include "core/altitude.hpp"

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

int main() {
    testConversions();
    testFloor();
    return TEST_MAIN_RESULT();
}
