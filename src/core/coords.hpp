#pragma once

// Coordinate formats for display and (future) input: decimal degrees,
// degrees-minutes-seconds and UTM on WGS84. Pure functions, unit-tested.

#include <optional>
#include <string>

namespace gcs::coords {

enum class Fmt { Decimal = 0, Dms = 1, Utm = 2 };

struct LatLon {
    double lat = 0.0;
    double lon = 0.0;
};

struct Utm {
    int zone = 0;         // 1..60
    char band = 'N';      // latitude band letter C..X (I and O skipped)
    bool north = true;
    double easting = 0.0;   // m
    double northing = 0.0;  // m (southern hemisphere: false northing 10 000 km)
};

// Standard zones (no Norway / Svalbard exceptions), valid for |lat| <= 84.
Utm toUtm(double lat, double lon);
// The zone and hemisphere fix the projection; the band letter is ignored.
LatLon fromUtm(int zone, bool north, double easting, double northing);
char utmBand(double lat);

// "32.08530° N  34.78180° E" / "32°05'07.1\" N  34°46'54.5\" E" / "36R 668123 3551234"
std::string format(double lat, double lon, Fmt fmt);

// Reads what format() writes, plus common variants: signed decimal pairs
// ("32.0853, 34.7818"), N/S/E/W before or after, ° ' " or spaces as
// separators, and UTM "36R 668123 3551234" / "36 N 668123 3551234".
std::optional<LatLon> parse(const std::string& text, Fmt fmt);

}  // namespace gcs::coords
