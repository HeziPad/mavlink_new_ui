#include "endurance.hpp"

#include "geo.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace gcs::endurance {

namespace {

constexpr double kSlowTau = 60.0;  // s
constexpr double kFastTau = 15.0;  // s
constexpr double kMaxGap = 10.0;   // s; a longer gap between samples restarts the step

double mahOf(double amps, double seconds) {
    return amps * seconds / 3.6;  // A*s -> mAh
}

std::string printf1(const char* f, double a) {
    char buf[160];
    std::snprintf(buf, sizeof(buf), f, a);
    return buf;
}

std::string printf2(const char* f, double a, double b) {
    char buf[160];
    std::snprintf(buf, sizeof(buf), f, a, b);
    return buf;
}

}  // namespace

// ---- current learning ------------------------------------------------------------------

void CurrentTracker::addSample(double t, double amps, Phase phase) {
    const double dt = last_t_ < 0.0 || t <= last_t_ || t - last_t_ > kMaxGap ? 0.0 : t - last_t_;
    last_t_ = t;
    if (amps < 0.0) {
        return;
    }
    if (phase == Phase::Cruise) {
        if (cruise_s_ <= 0.0 && slow_ <= 0.0) {
            slow_ = fast_ = amps;
        } else {
            slow_ += (amps - slow_) * (1.0 - std::exp(-dt / kSlowTau));
            fast_ += (amps - fast_) * (1.0 - std::exp(-dt / kFastTau));
        }
        cruise_s_ += dt;
    } else if (phase == Phase::Hover) {
        hover_as_ += amps * dt;
        hover_s_ += dt;
    }
}

double CurrentTracker::cruiseAmps() const {
    return cruise_s_ >= kLearnS ? std::max(slow_, fast_) : -1.0;
}

double CurrentTracker::hoverAmps() const {
    return hover_s_ >= 5.0 ? hover_as_ / hover_s_ : -1.0;
}

// ---- the return ------------------------------------------------------------------------

double groundSpeed(double airspeed, double track_deg, double wind_from_deg, double wind_speed) {
    // wind blowing toward (from + 180): split into along-track and cross-track
    const double rel = (wind_from_deg + 180.0 - track_deg) * geo::kDegToRad;
    const double along = wind_speed * std::cos(rel);
    const double cross = wind_speed * std::sin(rel);
    if (cross * cross >= airspeed * airspeed) {
        return kMinGroundSpeed;  // can't hold the track at all
    }
    return std::max(kMinGroundSpeed, std::sqrt(airspeed * airspeed - cross * cross) + along);
}

ReturnCost returnCost(const Inputs& in, const RoutePoint& from, double cruise_a, double hover_a) {
    const double airspeed = std::max(5.0, in.airspeed);
    const double wind = std::max(0.0, in.wind_speed) * kWindFactor;
    ReturnCost c;

    // the leg to the landing point, against the (strengthened) wind
    const double d = geo::distance(from.lat, from.lon, in.land_lat, in.land_lon);
    const double travel_s =
        d < 1.0 ? 0.0 : d / groundSpeed(airspeed, geo::bearing(from.lat, from.lon, in.land_lat, in.land_lon),
                                          in.wind_from_deg, wind);
    // high above the approach altitude: extra circling to get down on the wing
    const double descent_s = std::max(0.0, from.alt - in.approach_alt) / kWingSinkRate;
    const double extra_circling_s = std::max(0.0, descent_s - travel_s);
    // the approach circle + one go-around circle, and the straight leg twice;
    // the leg points into the wind, so it is flown at headwind ground speed
    const double circle_s = 2.0 * geo::kPi * std::max(20.0, in.approach_radius) / airspeed * kCircleSlack;
    const double leg_gs = std::max(kMinGroundSpeed, airspeed - wind);
    const double leg_s = std::max(0.0, in.approach_leg_m) / leg_gs;
    c.cruise_s = travel_s + extra_circling_s + 2.0 * circle_s + 2.0 * leg_s;

    // on the rotors: back-transition, descent, final descent, hover margin
    const double alt = std::max(0.0, in.approach_alt);
    c.rotor_s = kTransitionS + std::max(0.0, alt - kFinalAlt) / kDescentRate + std::min(alt, kFinalAlt) / kFinalRate +
                kHoverMarginS;

    c.mah = (mahOf(cruise_a, c.cruise_s) + mahOf(hover_a, c.rotor_s)) * kReturnMargin;
    return c;
}

Estimate estimate(const Inputs& in) {
    Estimate e;
    if (in.capacity_mah <= 0.0) {
        e.why = "the battery capacity (BATT_CAPACITY) isn't known";
        return e;
    }
    if (in.consumed_mah < 0.0 && in.battery_pct < 0) {
        e.why = "the battery monitor reports no charge";
        return e;
    }
    // the lower of the two gauges: used-mAh counting and the autopilot's percentage
    double remaining = -1.0;
    if (in.consumed_mah >= 0.0) {
        remaining = in.capacity_mah - in.consumed_mah;
    }
    if (in.battery_pct >= 0) {
        const double by_pct = in.capacity_mah * in.battery_pct / 100.0;
        remaining = remaining < 0.0 ? by_pct : std::min(remaining, by_pct);
    }
    e.remaining_mah = std::max(0.0, remaining);

    e.cruise_learned = in.cruise_a > 0.0;
    if (e.cruise_learned) {
        e.cruise_a = in.cruise_a;
    } else if (in.current_a > 0.0 && in.in_cruise) {
        e.cruise_a = in.current_a * kUnlearnedFactor;
    } else if (in.current_a < 0.0) {
        e.why = "the battery monitor reports no current";
        return e;
    } else {
        return e;  // not wing-borne yet: nothing to estimate from (and nothing to report)
    }
    e.hover_measured = in.hover_a > 0.0;
    e.hover_a = e.hover_measured ? in.hover_a : e.cruise_a * kHoverFactor;

    e.reserve_mah = in.capacity_mah * std::clamp(in.reserve, 0.0, 0.9);
    e.here = returnCost(in, in.here, e.cruise_a, e.hover_a);
    e.worst = e.here;
    for (const auto& p : in.route) {
        const ReturnCost c = returnCost(in, p, e.cruise_a, e.hover_a);
        if (c.mah > e.worst.mah) {
            e.worst = c;
        }
    }
    const double spare = e.remaining_mah - e.reserve_mah - e.worst.mah;
    e.time_left_s = spare / (e.cruise_a / 3.6);
    e.valid = true;

    if (e.remaining_mah - e.here.mah < e.reserve_mah) {
        e.level = Level::Critical;
    } else if (e.time_left_s <= 0.0) {
        e.level = Level::Warning;
    } else if (e.time_left_s < kCautionS) {
        e.level = Level::Caution;
    }
    return e;
}

// ---- hysteresis ------------------------------------------------------------------------

Level LevelLatch::update(Level now_level, double t) {
    if (now_level >= level_) {
        level_ = now_level;
        lower_since_ = -1.0;
    } else if (lower_since_ < 0.0) {
        lower_since_ = t;
    } else if (t - lower_since_ >= kClearHoldS) {
        level_ = now_level;
        lower_since_ = -1.0;
    }
    return level_;
}

// ---- presentation ----------------------------------------------------------------------

std::string clockText(double seconds) {
    const long s = seconds <= 0.0 ? 0 : static_cast<long>(seconds);
    char buf[32];
    if (s >= 3600) {
        std::snprintf(buf, sizeof(buf), "%ld:%02ld:%02ld", s / 3600, (s / 60) % 60, s % 60);
    } else {
        std::snprintf(buf, sizeof(buf), "%ld:%02ld", s / 60, s % 60);
    }
    return buf;
}

std::vector<Advisory> advisories(const Estimate& e, Level level) {
    std::vector<Advisory> out;
    if (!e.valid) {
        if (!e.why.empty()) {
            Advisory a;
            a.id = "battery.endurance.unknown";
            a.severity = Severity::Caution;
            a.title = "Battery time left unavailable";
            a.cause = "The GCS can't estimate how long you can keep flying: " + e.why + ".";
            a.suggestions = {"Set BATT_CAPACITY to the pack's mAh and check the current sensor (BATT_MONITOR)",
                             "Until then, watch the battery % and land with plenty to spare"};
            out.push_back(std::move(a));
        }
        return out;
    }
    if (level == Level::None) {
        return out;
    }

    const std::string detail =
        printf2("left %.0f mAh, reserve %.0f mAh", e.remaining_mah, e.reserve_mah) + " · " +
        printf2("return+land from here %.0f mAh, from the farthest point %.0f mAh", e.here.mah, e.worst.mah) +
        " · " + printf2("cruise %.1f A, hover %.1f A", e.cruise_a, e.hover_a) +
        (e.cruise_learned ? "" : " (cruise not learned yet: current x 1.3)") +
        (e.hover_measured ? "" : " (hover not measured: 4 x cruise)");
    const std::string what =
        "It keeps enough for the flight back, the approach circle, one go-around and the vertical landing (" +
        printf1("about %.0f s on the lift rotors", e.here.rotor_s) +
        "), all x 1.3, with the wind taken 50 % stronger, and never touches the reserve.";

    Advisory a;
    a.detail = detail;
    if (level == Level::Critical) {
        a.id = "battery.endurance.critical";
        a.severity = Severity::Critical;
        a.title = "Below landing reserve: land now";
        a.cause = "Flying back and landing from here would eat into the battery reserve. " + what;
        a.suggestions = {"Tap Land now", "Avoid hovering: the lift rotors draw the most current"};
    } else if (level == Level::Warning) {
        a.id = "battery.endurance.land";
        a.severity = Severity::Warning;
        a.title = "Battery: time to land";
        a.cause = "The battery is down to what the return and landing need, plus the reserve. " + what;
        a.suggestions = {"Tap Land", "Don't start a new loop or a long Guide"};
    } else {
        a.id = "battery.endurance.soon";
        a.severity = Severity::Caution;
        a.title = "Battery: " + clockText(e.time_left_s) + " of flying left";
        a.cause = "Less than 5 minutes until you should head back and land. " + what;
        a.suggestions = {"Finish what you're doing and tap Land"};
    }
    out.push_back(std::move(a));
    return out;
}

}  // namespace gcs::endurance
