#pragma once

// Which airspeed to ask for, from what the aircraft is doing. The three
// speeds are the aircraft's own parameters (never written here):
// LOITER = AIRSPEED_MIN, CRUISE = AIRSPEED_CRUISE, MAX = AIRSPEED_MAX.

namespace gcs::speed {

enum class Mode { Eco = 0, Normal = 1, Fast = 2 };
enum class Situation { None, Circling, EnRoute, Descending, Landing };
enum class Target { Default, Loiter, Cruise, Max };  // Default: ArduPilot's own (no command)

constexpr double kDescentForMax = 15.0;  // m below the target altitude before ECO speeds up

Target target(Mode mode, Situation s);

// The LOITER speed actually commanded: AIRSPEED_MIN, but kept kAssistMargin
// above Q_ASSIST_SPEED (below it the lift rotors spin to help the wing) and
// no faster than cruise. assist <= 0: Q_ASSIST off / unknown.
constexpr double kAssistMargin = 2.0;  // m/s
double loiterSpeed(double airspeed_min, double airspeed_cruise, double q_assist_speed);
const char* targetName(Target t);  // "LOITER" / "CRUISE" / "MAX" / ""

}  // namespace gcs::speed
