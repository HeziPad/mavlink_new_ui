#include "check.hpp"
#include "core/display_round.hpp"

using namespace gcs::display;

int main() {
    CHECK_NEAR(roundTo(323, 10), 320, 1e-9);
    CHECK_NEAR(roundTo(328, 10), 330, 1e-9);
    CHECK_NEAR(roundTo(-14, 10), -10, 1e-9);
    CHECK_NEAR(roundTo(7, 0), 7, 1e-9);

    // hovering at the boundary: it stays
    StickyRound r(10, 2);
    CHECK_NEAR(r.update(324), 320, 1e-9);
    CHECK_NEAR(r.update(326), 320, 1e-9);
    CHECK_NEAR(r.update(327), 320, 1e-9);
    CHECK_NEAR(r.update(325), 320, 1e-9);
    // clearly past it: it moves on, and stays there coming back a little
    CHECK_NEAR(r.update(328), 330, 1e-9);
    CHECK_NEAR(r.update(323.5), 330, 1e-9);
    CHECK_NEAR(r.update(322), 320, 1e-9);
    // a jump goes straight to the new value
    CHECK_NEAR(r.update(401), 400, 1e-9);
    r.reset();
    CHECK_NEAR(r.update(326), 330, 1e-9);
    return TEST_MAIN_RESULT();
}
