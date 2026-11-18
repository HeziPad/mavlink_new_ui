#include "speed_mode.hpp"

#include <algorithm>

namespace gcs::speed {

Target target(Mode mode, Situation s) {
    switch (s) {
    case Situation::None:
        return Target::Default;
    case Situation::Landing:
        return Target::Cruise;  // the back-transition distance assumes cruise speed
    case Situation::Circling:
        return mode == Mode::Eco ? Target::Loiter : Target::Cruise;
    case Situation::EnRoute:
        return mode == Mode::Fast ? Target::Max : Target::Cruise;
    case Situation::Descending:
        return mode == Mode::Normal ? Target::Cruise : Target::Max;  // gravity pays for it
    }
    return Target::Default;
}

double loiterSpeed(double airspeed_min, double airspeed_cruise, double q_assist_speed) {
    double v = airspeed_min;
    if (q_assist_speed > 0.0) {
        v = std::max(v, q_assist_speed + kAssistMargin);
    }
    return airspeed_cruise > 0.0 ? std::min(v, airspeed_cruise) : v;
}

const char* targetName(Target t) {
    switch (t) {
    case Target::Loiter:
        return "LOITER";
    case Target::Cruise:
        return "CRUISE";
    case Target::Max:
        return "MAX";
    case Target::Default:
        break;
    }
    return "";
}

}  // namespace gcs::speed
