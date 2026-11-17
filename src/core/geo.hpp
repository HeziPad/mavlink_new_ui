#pragma once

#include <vector>

namespace gcs::geo {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kRadToDeg = 180.0 / kPi;
constexpr double kEarthRadius = 6378137.0;

double wrap360(double deg);
double wrap180(double deg);

// Great-circle distance (m) and initial bearing (deg, 0..360).
double distance(double lat1, double lon1, double lat2, double lon2);
double bearing(double lat1, double lon1, double lat2, double lon2);

// Flat-earth offsets, accurate to well under a metre for a few km.
void offset(double lat, double lon, double north_m, double east_m, double& out_lat, double& out_lon);
void destination(double lat, double lon, double bearing_deg, double dist_m, double& out_lat,
                 double& out_lon);

struct NE {
    double north = 0.0;
    double east = 0.0;
};
NE delta(double lat0, double lon0, double lat, double lon);

// ---- camera model ----------------------------------------------------------
// Frames: camera x = optical axis, y = right, z = down (image-down).
// Body is FRD; the camera is pitched about body Y by mount_pitch (negative = down).
struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct CameraModel {
    double hfov_deg = 70.0;
    double mount_pitch_deg = -30.0;
    double width = 1280.0;
    double height = 720.0;
};

struct Attitude {
    double roll_deg = 0.0;
    double pitch_deg = 0.0;
    double yaw_deg = 0.0;
};

// Earth-frame attitude of a camera mounted on `body` at (mount_yaw, mount_pitch)
// relative to the body (yaw first, then pitch; no mount roll). With an earth
// frame camera attitude, use a CameraModel with mount_pitch_deg = 0.
Attitude composeMount(const Attitude& body, double mount_yaw_deg, double mount_pitch_deg);

// Attitude of `cam` (earth frame) expressed relative to `body`: yaw = pan,
// pitch = tilt, roll = camera roll, all relative to the platform.
Attitude relativeTo(const Attitude& body, const Attitude& cam);

// Direction (not normalised) of the ray through pixel (px, py) in NED.
Vec3 pixelRayNed(const CameraModel& cam, const Attitude& att, double px, double py);

// Project a point given relative to the camera in NED metres into pixels.
// Returns false if the point is behind the camera.
bool projectNed(const CameraModel& cam, const Attitude& att, const Vec3& rel_ned, double& px,
                double& py);

struct GroundTarget {
    bool hit_ground = false;  // false -> fallback along bearing
    double north = 0.0;
    double east = 0.0;
    double range = 0.0;
    double bearing_deg = 0.0;
};

// Intersect the pixel ray with a flat ground plane `alt_agl` metres below the
// camera. If the ray does not reach the ground within max_range (pointing at
// or above the horizon, or the vehicle is on the ground) the target is placed
// fallback_range metres along the ray's horizontal bearing instead.
GroundTarget cameraRayToGround(const CameraModel& cam, const Attitude& att, double px, double py,
                               double alt_agl, double max_range = 2000.0,
                               double fallback_range = 300.0);

// Ground area the camera sees: its image border cast onto flat ground
// `alt_agl` below. Border points whose rays miss the ground within max_range
// (at or above the horizon) are placed max_range out along their bearing and
// flagged `open`, so the far edge can be drawn as "continues beyond".
struct Footprint {
    bool valid = false;           // at least part of the image sees the ground
    std::vector<NE> outline;      // clockwise around the image border
    std::vector<bool> open;       // per outline point
    GroundTarget center;          // where the boresight hits
};
Footprint cameraFootprint(const CameraModel& cam, const Attitude& att, double alt_agl, double max_range,
                          int per_edge = 8);

}  // namespace gcs::geo
