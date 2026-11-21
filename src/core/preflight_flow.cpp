#include "preflight_flow.hpp"

#include <algorithm>

namespace gcs::preflight {

namespace {
double secs(Clock::time_point a, Clock::time_point b) {
    return std::chrono::duration<double>(b - a).count();
}
}  // namespace

void Flow::start(const Session& ses, Clock::time_point now) {
    rows_.clear();
    for (const auto* d : ses.activeSteps()) {
        if (d->stage != Stage::Look) {
            rows_.push_back(d->id);
        }
    }
    phase_ = Phase::Look;
    current_ = 0;
    row_start_ = now;
    gate_passed_ = false;
    timed_out_.clear();
}

void Flow::lookDone(Clock::time_point now) {
    if (phase_ == Phase::Look) {
        phase_ = Phase::Scan;
        current_ = 0;
        row_start_ = now;
    }
}

void Flow::gateGo(Clock::time_point now) {
    if (phase_ == Phase::Gate) {
        gate_passed_ = true;
        phase_ = Phase::Scan;
        row_start_ = now;
    }
}

void Flow::rerun(Clock::time_point now) {
    phase_ = Phase::Scan;
    current_ = 0;
    row_start_ = now;
    gate_passed_ = false;
    timed_out_.clear();
}

std::string Flow::currentId() const {
    return (phase_ == Phase::Scan || phase_ == Phase::Gate) && current_ < static_cast<int>(rows_.size())
               ? rows_[static_cast<size_t>(current_)]
               : std::string();
}

void Flow::tick(const Session& ses, Clock::time_point now) {
    if (phase_ != Phase::Scan) {
        return;
    }
    if (current_ >= static_cast<int>(rows_.size())) {
        phase_ = Phase::Result;
        return;
    }
    if (atGate(ses)) {
        return;
    }
    const std::string& id = rows_[static_cast<size_t>(current_)];
    const StepDef* d = findStep(id);
    if (!d) {
        ++current_;
        return;
    }
    const Status st = ses.result(id).status;
    const bool decided = isDone(st) || st == Status::Fail;
    const double age = secs(row_start_, now);
    if (decided && age >= kMinShow) {
        ++current_;
        row_start_ = now;
    } else if (!decided && age >= std::max(kMinShow, d->wait_s)) {
        timed_out_.insert(id);
        ++current_;
        row_start_ = now;
    }
    if (current_ >= static_cast<int>(rows_.size())) {
        phase_ = Phase::Result;
    } else {
        atGate(ses);  // the same tick: nothing may start before the hold
    }
}

bool Flow::atGate(const Session& ses) {
    const StepDef* d = findStep(rows_[static_cast<size_t>(current_)]);
    if (!d || d->stage != Stage::Moving || gate_passed_) {
        return false;
    }
    // the hold first, unless every moving part already passed (a re-run)
    for (const auto& r : rows_) {
        const StepDef* m = findStep(r);
        if (m && m->stage == Stage::Moving && !isDone(ses.result(r).status)) {
            phase_ = Phase::Gate;
            return true;
        }
    }
    gate_passed_ = true;
    return false;
}

bool lookComplete(const Session& ses) {
    for (const auto* d : ses.activeSteps()) {
        if (d->stage == Stage::Look && !isDone(ses.result(d->id).status)) {
            return false;
        }
    }
    return true;
}

std::vector<Item> problems(const Session& ses) {
    std::vector<Item> v;
    for (const auto* d : ses.activeSteps()) {
        const Result r = ses.result(d->id);
        if (d->critical && (!isDone(r.status) || r.status == Status::Skipped)) {
            v.push_back({d, r});
        }
    }
    return v;
}

std::vector<Item> notes(const Session& ses) {
    std::vector<Item> v;
    for (const auto* d : ses.activeSteps()) {
        const Result r = ses.result(d->id);
        if (r.status == Status::Warn || (!d->critical && !isDone(r.status))) {
            v.push_back({d, r});
        }
    }
    return v;
}

}  // namespace gcs::preflight
