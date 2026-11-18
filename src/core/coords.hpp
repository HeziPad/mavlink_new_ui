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

// ---- typed in separate boxes (the entry pad) ----------------------------------------

// Degrees / minutes / seconds as typed: whole degrees and minutes, minutes and
// seconds below 60, the total within max_deg (90 latitude, 180 longitude).
std::optional<double> angleFromDms(double deg, double min, double sec, bool positive, double max_deg);
// Split for pre-filling, rounded to 0.1" like format() (59.95" carries).
struct DmsParts {
    long deg = 0;
    long min = 0;
    double sec = 0.0;
    bool positive = true;
};
DmsParts toDms(double v);
// A position from decimal degrees (|lat| <= 90, |lon| <= 180).
std::optional<LatLon> checkedLatLon(double lat, double lon);
// UTM with the same range checks as parse(): zone 1..60, easting 100..900 km,
// northing 0..10 000 km.
std::optional<LatLon> fromUtmChecked(int zone, bool north, double easting, double northing);

}  // namespace gcs::coords
