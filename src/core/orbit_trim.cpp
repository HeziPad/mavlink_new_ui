#include "orbit_trim.hpp"

#include <algorithm>
#include <cmath>

namespace gcs {

namespace {
constexpr double kTwoPi = 6.283185307179586;
constexpr double kSettleTurn = kTwoPi / 2.0;  // half a lap after any change
constexpr double kMaxStep = 0.5;              // rad between samples; more = a gap, don't integrate
}  // namespace

void OrbitTrim::setRequested(double radius_m) {
    requested_ = radius_m;
    sent_ = commandRadius();
    restart();
}

double OrbitTrim::commandRadius() const {
    return std::max(kMinCommand, requested_ / ratio_);
}

void OrbitTrim::restart() {
    phase_ = Phase::Settling;
    have_last_ = false;
    turned_ = 0.0;
    wdist_ = 0.0;
}

bool OrbitTrim::converged() const {
    return flown_ > 0.0 && std::abs(flown_ - requested_) <= std::max(3.0, 0.03 * requested_);
}

bool OrbitTrim::addSample(double north, double east) {
    if (requested_ <= 0.0) {
        return false;
    }
    const double dist = std::hypot(north, east);
    if (dist < 0.5 * sent_ || dist > 2.0 * sent_) {
        restart();  // still flying in (or thrown off): not on the circle
        return false;
    }
    const double angle = std::atan2(east, north);
    if (!have_last_) {
        have_last_ = true;
        last_angle_ = angle;
        return false;
    }
    double d = angle - last_angle_;
    d = std::remainder(d, kTwoPi);
    last_angle_ = angle;
    d = std::abs(d);
    if (d > kMaxStep) {
        return false;
    }
    turned_ += d;
    if (phase_ == Phase::Settling) {
        if (turned_ >= kSettleTurn) {
            phase_ = Phase::Measuring;
            turned_ = 0.0;
            wdist_ = 0.0;
        }
        return false;
    }
    wdist_ += dist * d;
    if (turned_ < kTwoPi) {
        return false;
    }
    // a full lap: the angle-weighted mean distance is the flown radius
    flown_ = wdist_ / turned_;
    turned_ = 0.0;
    wdist_ = 0.0;
    ratio_ = std::clamp(flown_ / sent_, kMinRatio, kMaxRatio);
    const double next = commandRadius();
    if (std::abs(next - sent_) > std::max(1.5, 0.01 * requested_)) {
        sent_ = next;
        phase_ = Phase::Settling;  // let it settle on the new circle first
        return true;
    }
    return false;  // close enough: keep watching lap by lap
}

}  // namespace gcs
