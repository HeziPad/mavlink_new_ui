#pragma once

// What the pilot reads at a glance: one plain flight state instead of mode +
// VTOL state + lift rotors, where the aircraft is heading and when it gets
// there, and a height above ground it can trust. Pure functions, unit-tested.

#include "vehicle_state.hpp"

#include <optional>
#include <string>

namespace gcs {

// ---- flight state ------------------------------------------------------------------

// What only the GCS knows (it isn't in the telemetry).
struct FlightContext {
    bool orbit = false;           // a GUIDED circle this GCS commanded
    bool orbit_hold = false;      //   ... from Hold (else Fly To)
    bool transition_leg = false;  // the straight climb-out leg after a Launch
};

enum class Tone { Neutral, Good, Busy, Warn, Bad };

struct FlightStatus {
    std::string label;      // "ON PLAN · WP3", "TRANSITION", "FLY TO", ...
    Tone tone = Tone::Neutral;
    bool rotors = false;    // lift rotors spinning
    bool assist = false;    // wing-borne but Q_ASSIST is spinning them
};

FlightStatus flightStatus(const VehicleState& s, const FlightContext& ctx, Clock::time_point now);

// ---- where it is going -----------------------------------------------------------------

struct NavTarget {
    bool valid = false;
    std::string name;      // "WP3", "L", "APPROACH", "HOLD", "FLY TO", "END HOLD", "CLIMB-OUT"
    double lat = 0.0;
    double lon = 0.0;
    double dist_m = 0.0;   // to the point, or to the edge of a circle around it
    double eta_s = -1.0;   // < 0: unknown (too slow to say)
    bool arrived = false;  // circling it (a hold / orbit): eta 0
};

// On a hold circle of radius r: close enough to its centre to count as there
// (ArduPlane flies circles a little wider than asked, and L1 overshoots).
bool onCircle(double dist_to_centre, double radius_m);

constexpr double kMinEtaSpeed = 3.0;  // m/s ground speed below which no ETA is given

double etaSeconds(double dist_m, double groundspeed);
// The current mission item in AUTO (named for the pilot); invalid otherwise.
NavTarget missionTarget(const VehicleState& s);
// A point, or a circle of `radius_m` around it (distance to its edge).
NavTarget pointTarget(const VehicleState& s, const std::string& name, double lat, double lon, double radius_m = 0.0);

// ---- height above ground ------------------------------------------------------------------

// DISTANCE_SENSOR orientation "down" (MAV_SENSOR_ROTATION_PITCH_270)
constexpr int kRangefinderDown = 25;
constexpr double kRangefinderTimeout = 1.0;   // s
constexpr double kRangefinderMaxFrac = 0.95;  // readings at the top of the range mean "nothing in range"
constexpr double kRangefinderMaxTilt = 30.0;  // deg roll / pitch

// The rangefinder's height above ground when it is trustworthy: pointing
// down, fresh, inside its range (a max-range reading means nothing was
// seen), not flagged invalid, aircraft not banked or pitched steeply.
// Corrected for the tilt.
std::optional<double> rangefinderAgl(const VehicleState& s, Clock::time_point now);

}  // namespace gcs
