#include "mission.hpp"

#include "geo.hpp"

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

}  // namespace

int32_t toE7(double deg) {
    return static_cast<int32_t>(std::llround(deg * 1.0e7));
}

std::vector<MissionItem> buildLoopMission(double home_lat, double home_lon, double home_alt_amsl,
                                          const MissionPlan& plan, float takeoff_alt) {
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

    if (plan.land) {
        const float approach = plan.land->alt_rel > 0.0f ? plan.land->alt_rel : kDefaultLandApproachAlt;
        MissionItem land = located(mav::kCmdNavVtolLand, plan.land->lat, plan.land->lon, approach);
        land.param1 = mav::kVtolLandFwSpiral;
        items.push_back(land);
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
        if (isLoopPoint(it)) {
            plan.wps.push_back({it.lat(), it.lon(), it.z});
        } else if (isLand(it) && !plan.land && (it.x != 0 || it.y != 0)) {
            plan.land = UserWaypoint{it.lat(), it.lon(), it.z > 0.0f ? it.z : kDefaultLandApproachAlt};
        }
    }
    return plan;
}

int landSeq(const std::vector<MissionItem>& items) {
    for (size_t i = 1; i < items.size(); ++i) {
        if (isLand(items[i])) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

int seqToWaypointIndex(const std::vector<MissionItem>& items, int seq) {
    if (seq < 1 || seq >= static_cast<int>(items.size()) || !isLoopPoint(items[seq])) {
        return -1;
    }
    int idx = 0;
    for (int i = 1; i < seq; ++i) {
        if (isLoopPoint(items[i])) {
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
