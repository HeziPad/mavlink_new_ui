#include "check.hpp"
#include "core/preflight_flow.hpp"

#include <string>

using namespace gcs;
using namespace gcs::preflight;

namespace {

const Clock::time_point T0 = Clock::time_point{} + std::chrono::hours(1);

Clock::time_point at(double s) {
    return T0 + std::chrono::milliseconds(static_cast<long long>(s * 1000.0));
}

int indexOf(const Flow& f, const std::string& id) {
    for (size_t i = 0; i < f.rows().size(); ++i) {
        if (f.rows()[i] == id) return static_cast<int>(i);
    }
    return -1;
}

// mark every row before `id` passed
void passUpTo(Session& ses, const Flow& f, const std::string& id) {
    for (const auto& r : f.rows()) {
        if (r == id) break;
        ses.set(r, Status::Pass, "", false, T0);
    }
}

void testOrderAndPace() {
    VehicleState s;
    Session ses;
    ses.begin(s, T0, false);
    Flow f;
    f.start(ses, T0);
    CHECK(f.phase() == Phase::Look);
    // the look card's steps aren't rows: they're on the card
    CHECK(indexOf(f, "air.props") < 0 && indexOf(f, "sens.pitot") < 0);
    CHECK(indexOf(f, "profile.vehicle") == 0);
    CHECK(!lookComplete(ses));
    for (const auto* d : ses.activeSteps()) {
        if (d->stage == Stage::Look) ses.set(d->id, Status::Pass, "", true, T0);
    }
    CHECK(lookComplete(ses));
    f.lookDone(T0);
    CHECK(f.phase() == Phase::Scan && f.currentId() == "profile.vehicle");

    // a decided row still stays up kMinShow
    ses.set("profile.vehicle", Status::Pass, "", false, T0);
    f.tick(ses, at(0.2));
    CHECK(f.currentId() == "profile.vehicle");
    f.tick(ses, at(0.6));
    CHECK(f.currentId() == "link.telemetry");
    CHECK(f.reached(0) && !f.reached(1));

    // a failure is decided too: it moves on
    ses.set("link.telemetry", Status::Fail, "", false, T0);
    f.tick(ses, at(1.2));
    CHECK(f.currentId() == "power.voltage");

    // still waiting: given its wait_s, then timed out
    ses.set("power.voltage", Status::Waiting, "", false, T0);
    f.tick(ses, at(5.0));
    CHECK(f.currentId() == "power.voltage" && !f.timedOut("power.voltage"));
    f.tick(ses, at(1.2 + findStep("power.voltage")->wait_s + 0.1));
    CHECK(f.timedOut("power.voltage"));
    CHECK(f.currentId() == "power.balance");
}

void testGateAndResult() {
    VehicleState s;
    Session ses;
    ses.begin(s, T0, false);
    Flow f;
    f.start(ses, T0);
    f.lookDone(T0);
    passUpTo(ses, f, "surf.roll");
    double t = 0.0;
    for (int i = 0; i < 100 && f.phase() == Phase::Scan; ++i) {
        t += 0.6;
        f.tick(ses, at(t));
        // the tick that reaches the first moving part already stops there:
        // the runner starts the row's test only while scanning
        CHECK(!(f.phase() == Phase::Scan && f.currentId() == "surf.roll"));
    }
    // stops before the first moving part until the hold
    CHECK(f.phase() == Phase::Gate && f.currentId() == "surf.roll");
    f.tick(ses, at(t + 30));
    CHECK(f.phase() == Phase::Gate);  // the gate never times out
    f.gateGo(at(t));
    CHECK(f.phase() == Phase::Scan);
    for (const auto* d : ses.activeSteps()) {
        ses.set(d->id, Status::Pass, "", false, T0);
    }
    for (int i = 0; i < 100 && f.phase() == Phase::Scan; ++i) {
        t += 0.6;
        f.tick(ses, at(t));
    }
    CHECK(f.phase() == Phase::Result);
    CHECK(problems(ses).empty() && notes(ses).empty());

    // a re-run with every moving part passed doesn't stop at the gate
    f.rerun(at(t));
    for (int i = 0; i < 100 && f.phase() == Phase::Scan; ++i) {
        t += 0.6;
        f.tick(ses, at(t));
    }
    CHECK(f.phase() == Phase::Result);
}

void testTurnaround() {
    VehicleState s;
    Session ses;
    ses.begin(s, T0, true);
    Flow f;
    f.start(ses, T0);
    f.lookDone(T0);
    for (const auto& r : f.rows()) {
        CHECK(findStep(r)->stage != Stage::Moving);  // nothing moves after a landing
        ses.set(r, Status::Pass, "", false, T0);
    }
    double t = 0.0;
    for (int i = 0; i < 100 && f.phase() == Phase::Scan; ++i) {
        t += 0.6;
        f.tick(ses, at(t));
    }
    CHECK(f.phase() == Phase::Result);
}

void testProblemsAndNotes() {
    VehicleState s;
    Session ses;
    ses.begin(s, T0, false);
    for (const auto* d : ses.activeSteps()) {
        ses.set(d->id, Status::Pass, "", false, T0);
    }
    ses.set("sens.gps", Status::Fail, "6 sats", false, T0);       // required: a problem
    ses.set("power.balance", Status::Fail, "spread", false, T0);   // optional: a note
    ses.set("pay.video", Status::Warn, "no frames", false, T0);    // a warning: a note
    const auto p = problems(ses);
    const auto n = notes(ses);
    CHECK(p.size() == 1 && p[0].def->id == "sens.gps");
    CHECK(n.size() == 2);
    CHECK(!ses.complete());
    ses.set("sens.gps", Status::Pass, "", false, T0);
    CHECK(problems(ses).empty() && ses.complete());
}

}  // namespace

int main() {
    testOrderAndPace();
    testGateAndResult();
    testTurnaround();
    testProblemsAndNotes();
    return TEST_MAIN_RESULT();
}
