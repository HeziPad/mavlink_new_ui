#include "display_round.hpp"

#include <cmath>

namespace gcs::display {

double roundTo(double v, double step) {
    if (!(step > 0.0)) {
        return v;
    }
    return std::round(v / step) * step;
}

double sticky(double raw, std::optional<double> shown, double step, double slack) {
    if (shown && std::abs(raw - *shown) <= step / 2.0 + slack) {
        return *shown;
    }
    return roundTo(raw, step);
}

double StickyRound::update(double raw) {
    shown_ = sticky(raw, shown_, step_, slack_);
    return *shown_;
}

}  // namespace gcs::display
