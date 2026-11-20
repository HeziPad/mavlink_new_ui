#include "check.hpp"
#include "core/descent_speed.hpp"

using namespace gcs::speed;

// steady samples for `secs` at 2 Hz; returns how many times the speed changed
static int run(DescentTuner& t, double secs, double sink, double thr, double airspeed_off = 0.0) {
    int changes = 0;
    for (double s = 0.0; s < secs - 1e-9; s += 0.5) {
        const double cmd = t.speed();
        changes += t.sample(0.5, cmd, cmd + airspeed_off, sink, thr, 5.0, 0.0) ? 1 : 0;
    }
    return changes;
}

static void testSteps() {
    DescentTuner t;
    t.reset(25.0, 20.0, 35.0);
    CHECK(!t.learned());
    // gliding at idle, sinking only 3 m/s (limit 5): faster, but only after 8 s
    CHECK(run(t, 7.5, 3.0, 0.0) == 0);
    CHECK(t.speed() == 25.0);
    CHECK(run(t, 0.5, 3.0, 0.0) == 1);
    CHECK(t.speed() == 26.0);
    CHECK(t.learned());
    // at the limit but paying throttle: slower
    CHECK(run(t, 8.0, 4.8, 30.0) == 1);
    CHECK(t.speed() == 25.0);
    // at the limit at idle: found, stays
    CHECK(run(t, 24.0, 4.7, 2.0) == 0);
    CHECK(t.speed() == 25.0);
    // sinking steeply but under the limit with throttle on: still slower
    CHECK(run(t, 8.0, 3.5, 30.0) == 1);
    CHECK(t.speed() == 24.0);
    // a gentle glide (throttle on, sinking 0.6 m/s): nothing learned
    DescentTuner g;
    g.reset(25.0, 20.0, 35.0);
    CHECK(run(g, 60.0, 0.6, 55.0) == 0);
    CHECK(!g.learned() && g.speed() == 25.0);
}

static void testClampsAndSteadiness() {
    DescentTuner t;
    t.reset(40.0, 20.0, 35.0);
    CHECK(t.speed() == 35.0);  // clamped at MAX
    run(t, 40.0, 3.0, 0.0);    // wants faster: stays at MAX
    CHECK(t.speed() == 35.0);
    t.reset(21.0, 20.0, 35.0);
    run(t, 40.0, 5.0, 50.0);   // wants slower: stops at LOITER
    CHECK(t.speed() == 20.0);
    // airspeed not on the command: never counts
    t.reset(25.0, 20.0, 35.0);
    CHECK(run(t, 30.0, 3.0, 0.0, 3.0) == 0);
    CHECK(t.speed() == 25.0);
    // an interruption restarts the 8 s
    CHECK(run(t, 6.0, 3.0, 0.0) == 0);
    t.interrupt();
    CHECK(run(t, 6.0, 3.0, 0.0) == 0);
    CHECK(run(t, 2.0, 3.0, 0.0) == 1);
    // a command other than the tuner's speed (e.g. FAST at MAX) doesn't count
    CHECK(!t.sample(10.0, 35.0, 35.0, 2.0, 0.0, 5.0, 0.0));
    // limits moved (parameters read): re-clamped
    t.setLimits(27.0, 35.0);
    CHECK(t.speed() == 27.0);
}

int main() {
    testSteps();
    testClampsAndSteadiness();
    return TEST_MAIN_RESULT();
}
