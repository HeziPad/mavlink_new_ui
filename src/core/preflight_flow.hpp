#pragma once

// The order and pace of a preflight run, for an operator who just watches:
//   Look    the "look at the aircraft" card (ticked by hand)
//   Scan    every other step in catalogue order, one row at a time: a row
//           stays on screen at least kMinShow so it can be read, and a row
//           still waiting is given its StepDef::wait_s before it ends as
//           "timed out" (the runner then records it as a failure or a note)
//   Gate    before the first moving part: waits for the hold that says
//           everyone is clear (skipped when they all passed already)
//   Result  the problems and notes
// Qt-free; fed each frame with the session the runner keeps.

#include "preflight.hpp"

#include <set>
#include <string>
#include <vector>

namespace gcs::preflight {

enum class Phase { Idle, Look, Scan, Gate, Result };

class Flow {
public:
    static constexpr double kMinShow = 0.5;  // s each row is shown running

    void start(const Session& ses, Clock::time_point now);  // a new run (Look)
    void lookDone(Clock::time_point now);                    // CONTINUE on the look card
    void gateGo(Clock::time_point now);                      // the hold before the moving parts
    void rerun(Clock::time_point now);                       // TRY AGAIN: the scan from the top
    void showResult() { phase_ = Phase::Result; }  // reopened after a run: straight to the result
    void tick(const Session& ses, Clock::time_point now);

    Phase phase() const { return phase_; }
    const std::vector<std::string>& rows() const { return rows_; }  // the scan's steps, in order
    int current() const { return current_; }  // index into rows() of the running row
    std::string currentId() const;
    bool reached(int i) const { return phase_ == Phase::Result || i < current_; }  // finished rows
    bool timedOut(const std::string& id) const { return timed_out_.count(id) > 0; }

private:
    bool atGate(const Session& ses);  // the current row is a moving part not yet allowed: Gate

    Phase phase_ = Phase::Idle;
    std::vector<std::string> rows_;
    int current_ = 0;
    Clock::time_point row_start_{};
    bool gate_passed_ = false;
    std::set<std::string> timed_out_;
};

// Every look-card step done (CONTINUE allowed).
bool lookComplete(const Session& ses);

// What the result lists. Problems block Launch: a required step that isn't
// done. Notes don't: a warning, or an optional step that failed.
struct Item {
    const StepDef* def = nullptr;
    Result result;
};
std::vector<Item> problems(const Session& ses);
std::vector<Item> notes(const Session& ses);

}  // namespace gcs::preflight
