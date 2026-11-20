#include "descent_speed.hpp"

#include <algorithm>
#include <cmath>

namespace gcs::speed {

void DescentTuner::reset(double initial, double lo, double hi) {
    lo_ = lo;
    hi_ = std::max(lo, hi);
    speed_ = std::clamp(initial, lo_, hi_);
    steady_ = sink_sum_ = thr_sum_ = 0.0;
    learned_ = false;
}

void DescentTuner::setLimits(double lo, double hi) {
    lo_ = lo;
    hi_ = std::max(lo, hi);
    speed_ = std::clamp(speed_, lo_, hi_);
}

bool DescentTuner::sample(double dt, double commanded, double airspeed, double sink, double throttle_pct,
                          double sink_max, double idle_pct) {
    if (dt <= 0.0 || sink_max <= 0.0 || std::abs(commanded - speed_) > 0.05 ||
        std::abs(airspeed - commanded) > kSteadyBand) {
        interrupt();  // flying something else, or still settling on the speed
        return false;
    }
    if (steady_ == 0.0) {
        sink_sum_ = thr_sum_ = 0.0;
    }
    steady_ += dt;
    sink_sum_ += sink * dt;
    thr_sum_ += throttle_pct * dt;
    if (steady_ < kSettle) {
        return false;
    }
    const double sink_avg = sink_sum_ / steady_;
    const double thr_avg = thr_sum_ / steady_;
    const double idle = std::max(0.0, idle_pct);
    const bool at_limit = sink_avg >= kAtLimit * sink_max;
    interrupt();  // the next decision needs a fresh steady stretch
    if (sink_avg < kSteep * sink_max) {
        return false;  // a gentle descent (a glide slope along a leg): nothing to learn
    }
    double next = speed_;
    if (thr_avg > idle + kPayingBand) {
        next = speed_ - kStep;  // putting energy in on the way down: less drag would save it
    } else if (thr_avg <= idle + kIdleBand && !at_limit) {
        next = speed_ + kStep;  // gliding at idle, still slower than allowed: more drag
    }
    next = std::clamp(next, lo_, hi_);
    learned_ = true;
    if (std::abs(next - speed_) < 1e-9) {
        return false;
    }
    speed_ = next;
    return true;
}

}  // namespace gcs::speed
