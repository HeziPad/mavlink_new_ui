#include "altitude.hpp"

namespace gcs::altitude {

std::optional<double> toRelHome(double value, Ref ref, const Grounds& g) {
    switch (ref) {
    case Ref::Asl:
        return value - g.home_amsl;
    case Ref::Agl:
        if (!g.point) {
            return std::nullopt;
        }
        return *g.point + value - g.home_amsl;
    case Ref::Afl:
        return (g.land ? *g.land : g.home_amsl) + value - g.home_amsl;
    }
    return std::nullopt;
}

std::optional<double> fromRelHome(double rel_home, Ref ref, const Grounds& g) {
    switch (ref) {
    case Ref::Asl:
        return rel_home + g.home_amsl;
    case Ref::Agl:
        if (!g.point) {
            return std::nullopt;
        }
        return rel_home + g.home_amsl - *g.point;
    case Ref::Afl:
        return rel_home + g.home_amsl - (g.land ? *g.land : g.home_amsl);
    }
    return std::nullopt;
}

double minRelHome(const Grounds& g, double min_agl) {
    const double ground = g.point ? *g.point : g.home_amsl;
    return ground + min_agl - g.home_amsl;
}

Check check(double value, Ref ref, const Grounds& g, double min_agl) {
    Check c;
    c.min_rel = minRelHome(g, min_agl);
    const auto rel = toRelHome(value, ref, g);
    if (!rel) {
        c.reason = "no terrain data at this point: use ASL or AFL";
        return c;
    }
    c.rel_home = *rel;
    if (*rel < c.min_rel - 1e-6) {
        c.reason = "too low: below the minimum height above the ground here";
        return c;
    }
    if (*rel > kMaxRelHome) {
        c.reason = "too high";
        return c;
    }
    c.ok = true;
    return c;
}

}  // namespace gcs::altitude
