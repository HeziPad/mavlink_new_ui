#pragma once

// Typed altitudes: the operator gives a height in one of three references,
// the aircraft is commanded in height above home (missions and
// DO_REPOSITION use the relative frame). Pure, unit-tested.

#include <optional>

namespace gcs::altitude {

enum class Ref { Asl = 0, Agl = 1, Afl = 2 };  // sea level / ground at the point / ground at the landing point

constexpr double kMinAgl = 30.0;          // m above the ground at the point (orbits, waypoints)
constexpr double kMinApproachAgl = 20.0;  // m, the landing approach circle
constexpr double kMaxRelHome = 5000.0;

struct Grounds {
    double home_amsl = 0.0;
    std::optional<double> point;  // terrain AMSL at the point (nullopt: unknown)
    std::optional<double> land;   // terrain AMSL at the landing point
};

// nullopt when the reference can't be used (AGL / AFL without that ground)
std::optional<double> toRelHome(double value, Ref ref, const Grounds& g);
std::optional<double> fromRelHome(double rel_home, Ref ref, const Grounds& g);

// The lowest height above home allowed at the point: its ground (or home
// level when unknown) plus the minimum clearance.
double minRelHome(const Grounds& g, double min_agl = kMinAgl);

struct Check {
    bool ok = false;
    double rel_home = 0.0;   // what would be commanded
    double min_rel = 0.0;    // the floor at this point
    const char* reason = "";
};
Check check(double value, Ref ref, const Grounds& g, double min_agl = kMinAgl);

}  // namespace gcs::altitude
