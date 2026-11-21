#include "tiles.hpp"

#include "geo.hpp"

#include <algorithm>
#include <cmath>

namespace gcs::tiles {

namespace {
void world(double lat, double lon, int z, double& wx, double& wy) {
    const double n = kTilePx * std::exp2(z);
    wx = (lon + 180.0) / 360.0 * n;
    const double s = std::sin(std::clamp(lat, -85.05, 85.05) * geo::kDegToRad);
    wy = (0.5 - std::log((1.0 + s) / (1.0 - s)) / (4.0 * geo::kPi)) * n;
}
}  // namespace

TilePos locate(double lat, double lon, int z) {
    double wx = 0, wy = 0;
    world(lat, lon, z, wx, wy);
    const int last = (1 << z) - 1;
    TilePos t;
    t.z = z;
    t.x = std::clamp(static_cast<int>(std::floor(wx / kTilePx)), 0, last);
    t.y = std::clamp(static_cast<int>(std::floor(wy / kTilePx)), 0, last);
    t.px = wx - t.x * kTilePx;
    t.py = wy - t.y * kTilePx;
    return t;
}

Range around(double lat, double lon, double radius_m, int z) {
    double n_lat = 0, n_lon = 0, s_lat = 0, s_lon = 0;
    geo::offset(lat, lon, radius_m, -radius_m, n_lat, n_lon);  // north-west corner
    geo::offset(lat, lon, -radius_m, radius_m, s_lat, s_lon);  // south-east corner
    const TilePos a = locate(n_lat, n_lon, z);
    const TilePos b = locate(s_lat, s_lon, z);
    Range r;
    r.z = z;
    r.x0 = std::min(a.x, b.x);
    r.x1 = std::max(a.x, b.x);
    r.y0 = std::min(a.y, b.y);
    r.y1 = std::max(a.y, b.y);
    return r;
}

Range box(double min_lon, double min_lat, double max_lon, double max_lat, int z) {
    const TilePos a = locate(max_lat, min_lon, z);  // north-west
    const TilePos b = locate(min_lat, max_lon, z);  // south-east
    Range r;
    r.z = z;
    r.x0 = a.x;
    r.x1 = b.x;
    r.y0 = a.y;
    r.y1 = b.y;
    return r;
}

double tileMeters(double lat, int z) {
    return std::cos(lat * geo::kDegToRad) * 2.0 * geo::kPi * geo::kEarthRadius / std::exp2(z);
}

}  // namespace gcs::tiles
