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
    clearLive();
}

void OrbitTrim::clearLive() {
    live_.clear();
    live_turn_ = 0.0;
    live_wdist_ = 0.0;
}

double OrbitTrim::liveRadius() const {
    return live_turn_ >= kLiveMinTurn ? live_wdist_ / live_turn_ : -1.0;
}

bool OrbitTrim::liveMatched() const {
    const double r = liveRadius();
    return r > 0.0 && std::abs(r - requested_) <= std::max(3.0, 0.03 * requested_);
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
        clearLive();
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
    live_.push_back({d, dist});
    live_turn_ += d;
    live_wdist_ += d * dist;
    while (!live_.empty() && live_turn_ - live_.front().dtheta >= kTwoPi) {
        live_turn_ -= live_.front().dtheta;
        live_wdist_ -= live_.front().dtheta * live_.front().dist;
        live_.pop_front();
    }
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
        clearLive();               // the old circle no longer says anything
        return true;
    }
    return false;  // close enough: keep watching lap by lap
}

}  // namespace gcs
