#include "mission.hpp"

#include "geo.hpp"

#include <algorithm>
#include <cmath>

namespace gcs {

namespace {

MissionItem located(uint16_t command, double lat, double lon, float alt) {
    MissionItem item;
    item.frame = mav::kFrameGlobalRelativeAlt;
    item.command = command;
    item.x = toE7(lat);
    item.y = toE7(lon);
    item.z = alt;
    return item;
}

bool isLoopPoint(const MissionItem& it) {
    switch (it.command) {
    case mav::kCmdNavWaypoint:
    case mav::kCmdNavLoiterUnlim:
    case mav::kCmdNavLoiterTurns:
    case mav::kCmdNavLoiterTime:
    case mav::kCmdNavLoiterToAlt:
        return it.x != 0 || it.y != 0;
    default:
        return false;
    }
}

bool isLand(const MissionItem& it) {
    return it.command == mav::kCmdNavVtolLand || it.command == mav::kCmdNavLand;
}

// a LOITER_TO_ALT immediately followed by the land item is the approach circle
bool isApproachCircle(const std::vector<MissionItem>& items, size_t i) {
    return items[i].command == mav::kCmdNavLoiterToAlt && i + 1 < items.size() && isLand(items[i + 1]);
}

}  // namespace

int32_t toE7(double deg) {
    return static_cast<int32_t>(std::llround(deg * 1.0e7));
}

double transitionThreshold(double airspeed, double trans_decel) {
    const double decel = trans_decel > 0.1 ? trans_decel : 2.0;
    return 1.5 * airspeed * airspeed / (2.0 * decel);
}

LandingApproach computeLandingApproach(const UserWaypoint& land, const ApproachInputs& in) {
    LandingApproach a;
    a.l_lat = land.lat;
    a.l_lon = land.lon;
    a.alt = land.alt_rel > 0.0f ? land.alt_rel : kDefaultLandApproachAlt;
    a.radius = std::max(20.0, in.radius);

    if (in.wind_speed >= kMinWindForApproach) {
        a.heading_deg = geo::wrap360(in.wind_from_deg);  // fly toward where the wind comes from
    } else if (geo::distance(in.from_lat, in.from_lon, land.lat, land.lon) > 1.0) {
        a.heading_deg = geo::bearing(in.from_lat, in.from_lon, land.lat, land.lon);
    } else {
        a.heading_deg = 0.0;
    }

    a.leg_m = std::max(kMinLeg, transitionThreshold(std::max(1.0, in.airspeed), in.trans_decel) * kLegMargin + kLegExtra);
    geo::destination(land.lat, land.lon, a.heading_deg + 180.0, a.leg_m, a.e_lat, a.e_lon);

    // circle on the aircraft's side of the approach line, so it doesn't have
    // to cross the line to get onto it
    const geo::NE rel = geo::delta(a.e_lat, a.e_lon, in.from_lat, in.from_lon);
    const double right = (a.heading_deg + 90.0) * geo::kDegToRad;
    const bool aircraft_right = rel.north * std::cos(right) + rel.east * std::sin(right) >= 0.0;
    // centre to the right of the direction of travel = clockwise
    a.ccw = !aircraft_right;
    geo::destination(a.e_lat, a.e_lon, a.heading_deg + (a.ccw ? -90.0 : 90.0), a.radius, a.c_lat, a.c_lon);
    return a;
}

std::vector<MissionItem> buildLoopMission(double home_lat, double home_lon, double home_alt_amsl,
                                          const MissionPlan& plan, float takeoff_alt,
                                          const LandingApproach* approach) {
    std::vector<MissionItem> items;
    const auto& wps = plan.wps;
    if (wps.empty()) {
        return items;
    }
    items.reserve(wps.size() + 4);

    MissionItem home = located(mav::kCmdNavWaypoint, home_lat, home_lon, static_cast<float>(home_alt_amsl));
    home.frame = mav::kFrameGlobal;
    items.push_back(home);

    items.push_back(located(mav::kCmdNavVtolTakeoff, home_lat, home_lon, takeoff_alt));

    if (wps.size() == 1) {
        // a one-point "loop" would re-reach the same waypoint every cycle;
        // circle it on the wing instead
        items.push_back(located(mav::kCmdNavLoiterUnlim, wps[0].lat, wps[0].lon, wps[0].alt_rel));
    } else {
        for (const auto& wp : wps) {
            items.push_back(located(mav::kCmdNavWaypoint, wp.lat, wp.lon, wp.alt_rel));
        }
        MissionItem jump;
        jump.frame = mav::kFrameGlobalRelativeAlt;
        jump.command = mav::kCmdDoJump;
        jump.param1 = 2.0f;   // first loop waypoint
        jump.param2 = -1.0f;  // repeat forever
        items.push_back(jump);
    }

    if (plan.land && approach) {
        MissionItem circle = located(mav::kCmdNavLoiterToAlt, approach->c_lat, approach->c_lon, approach->alt);
        circle.param2 = static_cast<float>(approach->ccw ? -approach->radius : approach->radius);
        circle.param4 = 1.0f;  // leave on the tangent toward the land point
        items.push_back(circle);
        // Straight-in: ArduPlane holds the current (approach) altitude on the
        // leg and starts its back-transition at its stopping distance. z = 0:
        // without a rangefinder ArduPlane takes the land item's altitude as
        // the ground for its descent profile, so a non-zero z makes it think
        // it is already at the ground and creep down at Q_LAND_FINAL_SPD.
        items.push_back(located(mav::kCmdNavVtolLand, plan.land->lat, plan.land->lon, 0.0f));
    }

    for (size_t i = 0; i < items.size(); ++i) {
        items[i].seq = static_cast<uint16_t>(i);
    }
    return items;
}

MissionPlan extractPlan(const std::vector<MissionItem>& items) {
    MissionPlan plan;
    for (size_t i = 1; i < items.size(); ++i) {
        const auto& it = items[i];
        if (isApproachCircle(items, i)) {
            continue;
        }
        if (isLoopPoint(it)) {
            plan.wps.push_back({it.lat(), it.lon(), it.z});
        } else if (isLand(it) && !plan.land && (it.x != 0 || it.y != 0)) {
            // approach altitude: from the approach circle when there is one
            const float z = i >= 1 && isApproachCircle(items, i - 1) ? items[i - 1].z : it.z;
            plan.land = UserWaypoint{it.lat(), it.lon(), z > 0.0f ? z : kDefaultLandApproachAlt};
        }
    }
    return plan;
}

int landSeq(const std::vector<MissionItem>& items) {
    for (size_t i = 1; i < items.size(); ++i) {
        if (isLand(items[i])) {
            return isApproachCircle(items, i - 1) && i - 1 >= 1 ? static_cast<int>(i - 1) : static_cast<int>(i);
        }
    }
    return -1;
}

std::optional<LandingApproach> extractApproach(const std::vector<MissionItem>& items) {
    for (size_t i = 2; i < items.size(); ++i) {
        if (!isLand(items[i]) || !isApproachCircle(items, i - 1)) {
            continue;
        }
        const auto& c = items[i - 1];
        const auto& l = items[i];
        LandingApproach a;
        a.c_lat = c.lat();
        a.c_lon = c.lon();
        a.l_lat = l.lat();
        a.l_lon = l.lon();
        a.radius = std::abs(c.param2) > 1.0f ? std::abs(c.param2) : 80.0;
        a.ccw = c.param2 < 0.0f;
        a.alt = c.z;
        // exit = tangent point from which the straight line reaches L
        const double d = geo::distance(a.c_lat, a.c_lon, a.l_lat, a.l_lon);
        const double b = geo::bearing(a.c_lat, a.c_lon, a.l_lat, a.l_lon);
        if (d <= a.radius + 1.0) {
            a.e_lat = a.l_lat;
            a.e_lon = a.l_lon;
            a.heading_deg = b;
            return a;
        }
        const double alpha = std::acos(a.radius / d) * geo::kRadToDeg;
        const double ce = a.ccw ? b + alpha : b - alpha;  // bearing centre -> exit
        geo::destination(a.c_lat, a.c_lon, ce, a.radius, a.e_lat, a.e_lon);
        a.heading_deg = geo::wrap360(a.ccw ? ce - 90.0 : ce + 90.0);
        a.leg_m = std::sqrt(d * d - a.radius * a.radius);
        return a;
    }
    return std::nullopt;
}

int seqToWaypointIndex(const std::vector<MissionItem>& items, int seq) {
    if (seq < 1 || seq >= static_cast<int>(items.size()) || !isLoopPoint(items[seq]) ||
        isApproachCircle(items, static_cast<size_t>(seq))) {
        return -1;
    }
    int idx = 0;
    for (int i = 1; i < seq; ++i) {
        if (isLoopPoint(items[i]) && !isApproachCircle(items, static_cast<size_t>(i))) {
            ++idx;
        }
    }
    return idx;
}

double lapLength(const std::vector<UserWaypoint>& wps) {
    if (wps.size() < 2) {
        return 0.0;
    }
    double total = 0.0;
    for (size_t i = 0; i < wps.size(); ++i) {
        const auto& a = wps[i];
        const auto& b = wps[(i + 1) % wps.size()];
        total += geo::distance(a.lat, a.lon, b.lat, b.lon);
    }
    return total;
}

}  // namespace gcs
