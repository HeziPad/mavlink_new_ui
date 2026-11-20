#pragma once

// Which airspeed to ask for, from what the aircraft is doing. The speeds come
// from the aircraft's own parameters (never written here):
// LOITER = AIRSPEED_MIN (kept above Q_ASSIST_SPEED), CRUISE = AIRSPEED_CRUISE,
// MAX = AIRSPEED_MAX.
//
// Two layers. Physics rules hold in every mode:
//   landing     CRUISE  (the back-transition distance assumes cruise speed)
//   climbing    CLIMB   (the best-climb speed: slow, the energy goes into height)
//   descending  DESCENT in ECO / NORMAL: the slowest speed that sinks at
//               TECS_SINK_MAX at idle throttle (learned, core/descent_speed);
//               MAX in FAST en route (the same sink rate, more ground
//               covered); on a circle DESCENT in every mode (MAX needs too
//               much bank). Only a steep descent counts: along a long leg
//               ArduPlane glides down gently (a glide slope), which is level
//               flight as far as speed goes.
//   and never slower than the wind + kWindMargin (or it crawls / drifts back).
// The speed mode only chooses level flight:
//            circling   en route
//   ECO      LOITER     CRUISE + headwind / 2 (best range in wind), LOITER..MAX
//   NORMAL   CRUISE     CRUISE
//   FAST     CRUISE     MAX    (a circle at MAX needs more bank than allowed)

namespace gcs::speed {

enum class Mode { Eco = 0, Normal = 1, Fast = 2 };
enum class Situation { None, Circling, EnRoute, Climbing, Descending, DescendingCircle, Landing };
// what is asked for, as the chip names it. Default: ArduPilot's own (no command)
enum class Target { Default, Loiter, Cruise, Max, Climb, CruiseUp, CruiseDown, Wind, Descent };

constexpr double kClimbForSlow = 15.0;   // m below the goal altitude: climbing
constexpr double kDescentForMax = 15.0;  // m above the goal altitude: descending
constexpr double kWindMargin = 5.0;      // m/s of airspeed kept above the wind speed
constexpr double kHeadwindGain = 0.5;    // ECO en route: + this x headwind component
constexpr double kResendDelta = 1.0;     // m/s: a smaller change isn't re-sent

struct Speeds {
    double loiter = 0.0;  // loiterSpeed()
    double climb = 0.0;   // best-climb speed (the LOITER speed, see climbSpeed())
    double descent = 0.0; // best descent speed (learned: DescentTuner); <= 0: MAX
    double cruise = 0.0;
    double max = 0.0;
};

struct Command {
    Target target = Target::Default;
    double airspeed = 0.0;  // m/s to command; 0 with Target::Default
};

// headwind: wind component against the course to where it is going (m/s,
// negative = tailwind); wind: wind speed (m/s), 0 when unknown
Command command(Mode mode, Situation s, const Speeds& v, double headwind = 0.0, double wind = 0.0);

// wind component against a course (m/s): wind_from / course in degrees
double headwind(double wind_from_deg, double wind_speed, double course_deg);

// The LOITER speed actually commanded: AIRSPEED_MIN, but kept kAssistMargin
// above Q_ASSIST_SPEED (below it the lift rotors spin to help the wing) and
// no faster than cruise. assist <= 0: Q_ASSIST off / unknown.
constexpr double kAssistMargin = 2.0;  // m/s
double loiterSpeed(double airspeed_min, double airspeed_cruise, double q_assist_speed);
// The best-climb speed. Measured in SITL (quadplane, r20speed): see climbSpeed() in the .cpp.
double climbSpeed(double loiter, double cruise);
const char* targetName(Target t);
// For the SPEED chip: only LOITER / CRUISE / MAX, the one nearest to what is
// commanded (CLIMB reads LOITER, CRUISE+ reads CRUISE, ...); Default stays Default.
Target chipTarget(const Command& c, const Speeds& v);  // "LOITER" / "CLIMB" / "CRUISE" / "CRUISE+" / ... / "" (Default)

}  // namespace gcs::speed
