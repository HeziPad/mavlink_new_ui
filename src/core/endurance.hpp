#pragma once

// Battery time left: how long the aircraft can keep flying its mission and
// still have enough charge to fly back, circle down the approach, go around
// once, land vertically on the lift rotors, and touch down with the reserve
// untouched. Advice only; nothing here changes the vehicle.
//
// Deliberately pessimistic. It takes the lower of the two charge gauges, a
// stronger wind than measured, extra time for every rotor phase, a margin on
// the whole return and a reserve that is never planned into. The return is
// costed from the worst point of the plan, not just from here.

#include "health.hpp"

#include <string>
#include <vector>

namespace gcs::endurance {

// ---- assumptions (all on the cautious side) ---------------------------------------
constexpr double kWindFactor = 1.5;        // the return wind is taken 50 % stronger than measured
constexpr double kMinGroundSpeed = 5.0;    // m/s, floor for a return into a strong headwind
constexpr double kReturnMargin = 1.3;      // the whole return-and-land cost x 1.3
constexpr double kCircleSlack = 1.1;       // a circle in wind takes ~10 % longer than in still air
constexpr double kWingSinkRate = 2.0;      // m/s, circling down on the wing to the approach altitude
constexpr double kTransitionS = 15.0;      // back-transition on the rotors
constexpr double kDescentRate = 1.5;       // m/s, rotor descent (Q_WP_SPEED_DN default)
constexpr double kFinalAlt = 6.0;          // m, final descent below this (Q_LAND_FINAL_ALT default)
constexpr double kFinalRate = 0.5;         // m/s (Q_LAND_FINAL_SPD default)
constexpr double kHoverMarginS = 30.0;     // extra hover: repositioning, a gusty touchdown
constexpr double kHoverFactor = 4.0;       // hover current = 4 x cruise until a takeoff was measured
constexpr double kUnlearnedFactor = 1.3;   // before kLearnS of cruise: instantaneous cruise current x 1.3
                                           // (no estimate before the first cruise: a takeoff hover
                                           // current is no guide to what cruise costs)
constexpr double kLearnS = 20.0;           // s of cruise before the learned current is trusted
constexpr double kDefaultReserve = 0.25;   // never plan into the last 25 % of the pack
constexpr double kCautionS = 300.0;        // "return soon" below 5 min
constexpr double kClearHoldS = 30.0;       // a level clears only after 30 s below it

// What the lift and wing were doing over a current sample.
enum class Phase { Other, Cruise, Hover };

// Learns the cruise and hover current from samples (every ~2 s is plenty).
class CurrentTracker {
public:
    // t: seconds on any monotonic clock; amps < 0 (unknown) is ignored
    void addSample(double t, double amps, Phase phase);
    // Cruise: the higher of a ~60 s and a ~15 s average, so a climb or a
    // headwind shows at once but a short lull doesn't. < 0 until kLearnS.
    double cruiseAmps() const;
    // Mean current while hovering on the rotors (the takeoff); < 0 if never.
    double hoverAmps() const;
    double cruiseSeconds() const { return cruise_s_; }
    void reset() { *this = CurrentTracker{}; }

private:
    double last_t_ = -1.0;
    double slow_ = 0.0, fast_ = 0.0;
    double cruise_s_ = 0.0;
    double hover_as_ = 0.0, hover_s_ = 0.0;  // amp-seconds, seconds
};

struct RoutePoint {
    double lat = 0.0;
    double lon = 0.0;
    double alt = 0.0;  // relative, m
};

struct Inputs {
    // charge
    double capacity_mah = -1.0;  // BATT_CAPACITY; <= 0 unknown
    double consumed_mah = -1.0;  // < 0 unknown
    int battery_pct = -1;        // < 0 unknown
    double reserve = kDefaultReserve;  // fraction of capacity never planned into
    // current
    double current_a = -1.0;     // now; < 0 unknown
    double cruise_a = -1.0;      // learned (CurrentTracker); < 0 not yet
    double hover_a = -1.0;       // learned; < 0 not measured
    bool in_cruise = false;      // wing-borne now: current_a is a cruise current
    // flight
    double airspeed = 18.0;      // cruise airspeed, m/s
    double wind_from_deg = 0.0;
    double wind_speed = 0.0;     // m/s, as measured (the factor is applied here)
    RoutePoint here;
    std::vector<RoutePoint> route;  // plan waypoints: the return is costed from the worst
    // landing
    double land_lat = 0.0, land_lon = 0.0;
    double approach_alt = 30.0;  // m, relative
    double approach_leg_m = 0.0; // straight leg from the circle to L
    double approach_radius = 80.0;
};

// What the return-and-land from one point costs (before the margin).
struct ReturnCost {
    double cruise_s = 0.0;  // on the wing: the leg home, the circles, the straight approach
    double rotor_s = 0.0;   // transition, descent, hover margin
    double mah = 0.0;       // x kReturnMargin included
};

enum class Level { None, Caution, Warning, Critical };

struct Estimate {
    bool valid = false;
    std::string why;              // why it isn't valid ("set BATT_CAPACITY", ...)
    double remaining_mah = 0.0;   // the lower of the two gauges
    double reserve_mah = 0.0;
    double cruise_a = 0.0;        // used
    double hover_a = 0.0;
    bool cruise_learned = false;
    bool hover_measured = false;
    ReturnCost here;              // return-and-land from where it is now
    ReturnCost worst;             // ... from the worst plan point (or here)
    double time_left_s = 0.0;     // flying time before it must head back (< 0: past it)
    Level level = Level::None;    // before hysteresis
};

double groundSpeed(double airspeed, double track_deg, double wind_from_deg, double wind_speed);
ReturnCost returnCost(const Inputs& in, const RoutePoint& from, double cruise_a, double hover_a);
Estimate estimate(const Inputs& in);

// A level rises at once and only falls after it has stayed lower for
// kClearHoldS, so the card doesn't flicker on a gust.
class LevelLatch {
public:
    Level update(Level now_level, double t);
    Level level() const { return level_; }
    void reset() { *this = LevelLatch{}; }

private:
    Level level_ = Level::None;
    double lower_since_ = -1.0;
};

// The alert card for a level (empty for None), explaining what the number is made of.
std::vector<Advisory> advisories(const Estimate& e, Level level);
// "14:20"; negative times read "0:00"
std::string clockText(double seconds);

}  // namespace gcs::endurance
