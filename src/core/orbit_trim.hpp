#pragma once

#include <deque>

namespace gcs {

// Makes the circle ArduPlane flies match the one the GCS asked for.
//
// ArduPlane flies a GUIDED loiter bigger than commanded: AP_L1_Control scales
// the radius by EAS2TAS^2 (about +7 % at 700 m AMSL, more at higher fields,
// with NAVL1_LIM_BANK = 0) and the L1 loop adds overshoot on small circles
// (SITL: 80 m -> 91 m, 600 m -> 642 m). This measures the flown radius over
// whole laps (angle-weighted, so wind and uneven sampling average out) and
// commands requested / ratio. The learned ratio is kept across radius
// changes, so a new radius starts close to right.
class OrbitTrim {
public:
    static constexpr double kMinCommand = 30.0;  // m
    static constexpr double kMinRatio = 0.7;
    static constexpr double kMaxRatio = 1.5;

    // New or edited circle: restarts the measurement, keeps the ratio.
    void setRequested(double radius_m);
    // What to send in DO_REPOSITION param3.
    double commandRadius() const;
    // Position relative to the circle centre (m). Returns true when the
    // command radius changed and should be re-sent.
    bool addSample(double north, double east);

    double requested() const { return requested_; }
    double ratio() const { return ratio_; }
    double flownRadius() const { return flown_; }  // last full lap, < 0 unknown
    bool measuring() const { return phase_ == Phase::Measuring; }
    bool converged() const;

    // For display: updated every sample, angle-weighted over the last lap
    // (at least kLiveMinTurn of arc since the last command change). < 0 until then.
    static constexpr double kLiveMinTurn = 1.5707963267948966;  // 90 deg
    double liveRadius() const;
    bool liveMatched() const;  // the live flown radius matches the request

private:
    enum class Phase { Settling, Measuring };
    void restart();

    double requested_ = 0.0;
    double ratio_ = 1.0;
    double sent_ = 0.0;  // the command radius in effect
    double flown_ = -1.0;
    Phase phase_ = Phase::Settling;
    bool have_last_ = false;
    double last_angle_ = 0.0;
    double turned_ = 0.0;  // rad since the phase started
    double wdist_ = 0.0;   // sum of dist * dtheta over the measuring lap

    struct Arc {
        double dtheta;
        double dist;
    };
    std::deque<Arc> live_;  // sliding window, at most one lap
    double live_turn_ = 0.0;
    double live_wdist_ = 0.0;
    void clearLive();
};

}  // namespace gcs
