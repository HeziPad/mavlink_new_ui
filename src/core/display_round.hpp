#pragma once

// Calm numbers for the telemetry bar: a value shown to the nearest `step`
// that only moves on when the raw value is clearly past the boundary, so a
// reading hovering at 325 doesn't flick between 320 and 330.

#include <optional>

namespace gcs::display {

// Nearest multiple of `step` (step > 0).
double roundTo(double v, double step);

// The value to show now: the previous one while `raw` stays within
// `step / 2 + slack` of it, otherwise roundTo(raw, step). No previous value:
// roundTo(raw, step).
double sticky(double raw, std::optional<double> shown, double step, double slack);

// A remembered sticky value (one per readout).
class StickyRound {
public:
    StickyRound(double step, double slack) : step_(step), slack_(slack) {}
    double update(double raw);
    void reset() { shown_.reset(); }

private:
    double step_, slack_;
    std::optional<double> shown_;
};

}  // namespace gcs::display
