#pragma once

#include <cstddef>
#include <cstdint>

namespace gcs::tiles {

// Slippy-map (Web Mercator, 256 px) tile maths, shared by the map, the
// elevation lookup and the offline downloader.
constexpr double kTilePx = 256.0;

struct TilePos {
    int z = 0;
    int x = 0, y = 0;       // tile
    double px = 0, py = 0;  // pixel inside the tile, 0..256
};
TilePos locate(double lat, double lon, int z);

// AWS Terrain Tiles "terrarium" encoding: metres above sea level.
inline double terrarium(uint8_t r, uint8_t g, uint8_t b) {
    return r * 256.0 + g + b / 256.0 - 32768.0;
}

// Every tile at zoom z that touches the square around (lat, lon) +- radius.
struct Range {
    int z = 0;
    int x0 = 0, x1 = -1, y0 = 0, y1 = -1;  // inclusive
    size_t count() const {
        return x1 < x0 || y1 < y0 ? 0 : static_cast<size_t>(x1 - x0 + 1) * static_cast<size_t>(y1 - y0 + 1);
    }
};
Range around(double lat, double lon, double radius_m, int z);

// Ground size of one tile edge at this latitude, metres.
double tileMeters(double lat, int z);

}  // namespace gcs::tiles
