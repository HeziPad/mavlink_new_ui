#pragma once

// The best descent airspeed, learned in flight.
//
// ArduPlane never sinks faster than TECS_SINK_MAX, so a faster airspeed does
// not bring the aircraft down sooner. The cheapest way down at that rate is
// the slowest airspeed at which it already sinks that fast with the throttle
// at idle: slower and it sinks less than allowed, faster and TECS has to add
// throttle to hold the sink back (SITL: 7 mAh per 100 m at 20 m/s, 110 at
// 35 m/s, both at ~4.6 m/s). That speed depends on the airframe's drag, so it
// is found by watching each steady descent and stepping the speed 1 m/s:
//   throttle above idle (energy put in on the way down)  -> slower
//   throttle at idle and sinking slower than the limit   -> faster
// A gentle descent (neither at the limit nor at idle, e.g. ArduPlane's glide
// slope along a long leg) teaches nothing and is ignored.

namespace gcs::speed {

class DescentTuner {
public:
    static constexpr double kSettle = 8.0;        // s of steady descent before a decision
    static constexpr double kStep = 1.0;          // m/s per decision
    static constexpr double kSteadyBand = 1.5;    // m/s: airspeed this close to the command counts
    static constexpr double kAtLimit = 0.9;       // x TECS_SINK_MAX: "at the limit"
    static constexpr double kSteep = 0.5;         // x TECS_SINK_MAX: slower than this teaches nothing
    static constexpr double kIdleBand = 5.0;      // % above idle: still idle
    static constexpr double kPayingBand = 10.0;   // % above idle: paying throttle

    // start from a learned value (> 0) or `initial`, within [lo, hi]
    void reset(double initial, double lo, double hi);
    void setLimits(double lo, double hi);  // LOITER .. MAX (re-clamps)
    double speed() const { return speed_; }
    bool learned() const { return learned_; }  // changed or confirmed at least once

    // One sample during a descent: dt (s), the commanded and measured
    // airspeed (m/s), the sink rate (m/s, positive down), the throttle (%),
    // TECS_SINK_MAX (m/s) and the idle throttle (THR_MIN, %).
    // Returns true when speed() changed.
    bool sample(double dt, double commanded, double airspeed, double sink, double throttle_pct, double sink_max,
                double idle_pct);
    void interrupt() { steady_ = 0.0; }  // not descending (any more)

private:
    double speed_ = 0.0;
    double lo_ = 0.0, hi_ = 0.0;
    double steady_ = 0.0;
    double sink_sum_ = 0.0, thr_sum_ = 0.0;
    bool learned_ = false;
};

}  // namespace gcs::speed
