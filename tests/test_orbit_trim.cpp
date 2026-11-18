#include "check.hpp"
#include "core/orbit_trim.hpp"

#include <cmath>

using namespace gcs;

namespace {

constexpr double kPi = 3.14159265358979323846;

// The "autopilot": flies cmd * scale (+ extra metres), centre offset by (on, oe)
// as wind would, sampled every `step` radians. Returns how many re-sends.
int fly(OrbitTrim& t, double scale, double extra, double laps, double on = 0.0, double oe = 0.0,
        double step = 0.05) {
    int resends = 0;
    double a = 0.0;
    for (double turned = 0.0; turned < laps * 2.0 * kPi; turned += step) {
        const double r = t.commandRadius() * scale + extra;
        a += step;
        if (t.addSample(on + r * std::cos(a), oe + r * std::sin(a))) {
            ++resends;
        }
    }
    return resends;
}

void testConvergesToRequested() {
    OrbitTrim t;
    t.setRequested(300.0);
    CHECK_NEAR(t.commandRadius(), 300.0, 1e-9);
    const int n = fly(t, 1.07, 0.0, 6.0);
    CHECK(n >= 1);
    CHECK_NEAR(t.commandRadius() * 1.07, 300.0, 3.0);  // flown == requested
    CHECK(t.converged());
    CHECK_NEAR(t.ratio(), 1.07, 0.01);
}

void testSmallCircleOvershoot() {
    // 80 m flies as 91 m: a ratio plus a fixed L1 overshoot
    OrbitTrim t;
    t.setRequested(80.0);
    fly(t, 1.035, 8.2, 10.0);
    CHECK_NEAR(t.commandRadius() * 1.035 + 8.2, 80.0, 3.0);
    CHECK(t.converged());
}

void testRatioCarriesOverToNewRadius() {
    OrbitTrim t;
    t.setRequested(300.0);
    fly(t, 1.07, 0.0, 6.0);
    t.setRequested(600.0);
    CHECK_NEAR(t.commandRadius(), 600.0 / 1.07, 6.0);  // right from the first send
    CHECK(fly(t, 1.07, 0.0, 3.0) == 0);                  // and nothing to fix
}

void testWindOffsetAverages() {
    // centre blown 25 m downwind: lap-mean distance still ~ the radius
    OrbitTrim t;
    t.setRequested(200.0);
    fly(t, 1.0, 0.0, 4.0, 25.0, 0.0);
    CHECK_NEAR(t.ratio(), 1.0, 0.02);
}

void testFlyInIsIgnoredAndBounds() {
    OrbitTrim t;
    t.setRequested(150.0);
    // far away (flying in): nothing measured
    for (int i = 0; i < 500; ++i) {
        CHECK(!t.addSample(2000.0 - i, 0.0));
    }
    CHECK(t.flownRadius() < 0.0);
    // absurd overshoot is clamped
    fly(t, 1.9, 0.0, 4.0);
    CHECK(t.ratio() <= OrbitTrim::kMaxRatio + 1e-9);
    t.setRequested(35.0);
    CHECK(t.commandRadius() >= OrbitTrim::kMinCommand);
}

void testLiveEstimate() {
    OrbitTrim t;
    t.setRequested(300.0);
    const double r = 321.0;  // flown, before any correction
    double a = 0.0;
    // nothing to show before a quarter lap
    while (a < OrbitTrim::kLiveMinTurn - 0.1) {
        a += 0.05;
        t.addSample(r * std::cos(a), r * std::sin(a));
        CHECK(t.liveRadius() < 0.0 || a >= OrbitTrim::kLiveMinTurn);
    }
    for (int i = 0; i < 6; ++i) {
        a += 0.05;
        t.addSample(r * std::cos(a), r * std::sin(a));
    }
    CHECK_NEAR(t.liveRadius(), r, 0.5);  // available after ~90 deg, long before a full lap
    CHECK(!t.liveMatched());
    CHECK(t.flownRadius() < 0.0);        // the trim's own lap isn't done yet
    t.setRequested(150.0);
    CHECK(t.liveRadius() < 0.0);         // a new request starts over
}

}  // namespace

int main() {
    testLiveEstimate();
    testConvergesToRequested();
    testSmallCircleOvershoot();
    testRatioCarriesOverToNewRadius();
    testWindOffsetAverages();
    testFlyInIsIgnoredAndBounds();
    return TEST_MAIN_RESULT();
}
