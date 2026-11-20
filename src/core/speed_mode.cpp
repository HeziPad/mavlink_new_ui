#include "speed_mode.hpp"

#include "geo.hpp"

#include <algorithm>
#include <cmath>

namespace gcs::speed {

Command command(Mode mode, Situation s, const Speeds& v, double headwind_ms, double wind) {
    Command c;
    switch (s) {
    case Situation::None:
        return c;  // ArduPilot's own
    case Situation::Landing:
        c = {Target::Cruise, v.cruise};
        break;
    case Situation::Climbing:
        c = {Target::Climb, v.climb};
        break;
    case Situation::Descending:
    case Situation::DescendingCircle:
        // ArduPlane caps the sink rate (TECS_SINK_MAX): speed doesn't bring it
        // down sooner. FAST takes MAX en route for the ground it covers.
        if (v.descent <= 0.0) {
            c = s == Situation::Descending ? Command{Target::Max, v.max} : Command{Target::Cruise, v.cruise};
        } else if (mode == Mode::Fast && s == Situation::Descending) {
            c = {Target::Max, v.max};
        } else {
            c = {Target::Descent, std::min(v.descent, v.max)};
        }
        break;
    case Situation::Circling:
        c = mode == Mode::Eco ? Command{Target::Loiter, v.loiter} : Command{Target::Cruise, v.cruise};
        break;
    case Situation::EnRoute:
        if (mode == Mode::Fast) {
            c = {Target::Max, v.max};
        } else if (mode == Mode::Eco) {
            // speed-to-fly: faster into a headwind, slower with a tailwind
            const double want = std::clamp(v.cruise + kHeadwindGain * headwind_ms, v.loiter, std::max(v.loiter, v.max));
            const double d = want - v.cruise;
            c = {std::abs(d) < 0.5 ? Target::Cruise : (d > 0 ? Target::CruiseUp : Target::CruiseDown), want};
        } else {
            c = {Target::Cruise, v.cruise};
        }
        break;
    }
    // never slower than the wind + a margin (capped at MAX): only ever raises it
    const double floor = std::min(wind + kWindMargin, v.max);
    if (wind > 0.0 && c.airspeed < floor) {
        c = {Target::Wind, floor};
    }
    return c;
}

double headwind(double wind_from_deg, double wind_speed, double course_deg) {
    return wind_speed * std::cos((wind_from_deg - course_deg) * geo::kDegToRad);
}

double loiterSpeed(double airspeed_min, double airspeed_cruise, double q_assist_speed) {
    double v = airspeed_min;
    if (q_assist_speed > 0.0) {
        v = std::max(v, q_assist_speed + kAssistMargin);
    }
    return airspeed_cruise > 0.0 ? std::min(v, airspeed_cruise) : v;
}

double climbSpeed(double loiter, double /*cruise*/) {
    return loiter;
}

Target chipTarget(const Command& c, const Speeds& v) {
    if (c.target == Target::Default) {
        return Target::Default;
    }
    const double dl = std::abs(c.airspeed - v.loiter), dc = std::abs(c.airspeed - v.cruise),
                 dm = std::abs(c.airspeed - v.max);
    if (dc <= dl && dc <= dm) {
        return Target::Cruise;  // ties go to CRUISE
    }
    return dl <= dm ? Target::Loiter : Target::Max;
}

const char* targetName(Target t) {
    switch (t) {
    case Target::Loiter:
        return "LOITER";
    case Target::Cruise:
        return "CRUISE";
    case Target::Max:
        return "MAX";
    case Target::Climb:
        return "CLIMB";
    case Target::CruiseUp:
        return "CRUISE+";
    case Target::CruiseDown:
        return "CRUISE−";
    case Target::Wind:
        return "WIND";
    case Target::Descent:
        return "DESCENT";
    case Target::Default:
        break;
    }
    return "";
}

}  // namespace gcs::speed
