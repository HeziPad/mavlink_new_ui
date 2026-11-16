#include "geo.hpp"

#include <algorithm>
#include <cmath>

namespace gcs::geo {

double wrap360(double deg) {
    double w = std::fmod(deg, 360.0);
    if (w < 0.0) {
        w += 360.0;
    }
    return w;
}

double wrap180(double deg) {
    double w = wrap360(deg + 180.0) - 180.0;
    return w;
}

double distance(double lat1, double lon1, double lat2, double lon2) {
    const double p1 = lat1 * kDegToRad;
    const double p2 = lat2 * kDegToRad;
    const double dp = (lat2 - lat1) * kDegToRad;
    const double dl = (lon2 - lon1) * kDegToRad;
    const double a = std::sin(dp / 2) * std::sin(dp / 2) +
                     std::cos(p1) * std::cos(p2) * std::sin(dl / 2) * std::sin(dl / 2);
    return 2.0 * kEarthRadius * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
}

double bearing(double lat1, double lon1, double lat2, double lon2) {
    const double p1 = lat1 * kDegToRad;
    const double p2 = lat2 * kDegToRad;
    const double dl = (lon2 - lon1) * kDegToRad;
    const double y = std::sin(dl) * std::cos(p2);
    const double x = std::cos(p1) * std::sin(p2) - std::sin(p1) * std::cos(p2) * std::cos(dl);
    return wrap360(std::atan2(y, x) * kRadToDeg);
}

void offset(double lat, double lon, double north_m, double east_m, double& out_lat,
            double& out_lon) {
    out_lat = lat + (north_m / kEarthRadius) * kRadToDeg;
    const double c = std::cos(lat * kDegToRad);
    out_lon = lon + (east_m / (kEarthRadius * (std::fabs(c) < 1e-9 ? 1e-9 : c))) * kRadToDeg;
}

void destination(double lat, double lon, double bearing_deg, double dist_m, double& out_lat,
                 double& out_lon) {
    const double b = bearing_deg * kDegToRad;
    offset(lat, lon, dist_m * std::cos(b), dist_m * std::sin(b), out_lat, out_lon);
}

NE delta(double lat0, double lon0, double lat, double lon) {
    NE d;
    d.north = (lat - lat0) * kDegToRad * kEarthRadius;
    d.east = (lon - lon0) * kDegToRad * kEarthRadius * std::cos(lat0 * kDegToRad);
    return d;
}

// ---- rotations ---------------------------------------------------------------
namespace {

struct Mat3 {
    double m[3][3];
};

Mat3 mul(const Mat3& a, const Mat3& b) {
    Mat3 r{};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            r.m[i][j] = a.m[i][0] * b.m[0][j] + a.m[i][1] * b.m[1][j] + a.m[i][2] * b.m[2][j];
        }
    }
    return r;
}

Vec3 apply(const Mat3& a, const Vec3& v) {
    return {a.m[0][0] * v.x + a.m[0][1] * v.y + a.m[0][2] * v.z,
            a.m[1][0] * v.x + a.m[1][1] * v.y + a.m[1][2] * v.z,
            a.m[2][0] * v.x + a.m[2][1] * v.y + a.m[2][2] * v.z};
}

Vec3 applyT(const Mat3& a, const Vec3& v) {
    return {a.m[0][0] * v.x + a.m[1][0] * v.y + a.m[2][0] * v.z,
            a.m[0][1] * v.x + a.m[1][1] * v.y + a.m[2][1] * v.z,
            a.m[0][2] * v.x + a.m[1][2] * v.y + a.m[2][2] * v.z};
}

Mat3 rotX(double r) {
    const double c = std::cos(r), s = std::sin(r);
    return {{{1, 0, 0}, {0, c, -s}, {0, s, c}}};
}
Mat3 rotY(double r) {
    const double c = std::cos(r), s = std::sin(r);
    return {{{c, 0, s}, {0, 1, 0}, {-s, 0, c}}};
}
Mat3 rotZ(double r) {
    const double c = std::cos(r), s = std::sin(r);
    return {{{c, -s, 0}, {s, c, 0}, {0, 0, 1}}};
}

// camera -> NED
Mat3 cameraToNed(const CameraModel& cam, const Attitude& att) {
    const Mat3 body_to_ned = mul(mul(rotZ(att.yaw_deg * kDegToRad), rotY(att.pitch_deg * kDegToRad)),
                                 rotX(att.roll_deg * kDegToRad));
    const Mat3 cam_to_body = rotY(cam.mount_pitch_deg * kDegToRad);
    return mul(body_to_ned, cam_to_body);
}

Mat3 eulerToMat(const Attitude& a) {
    return mul(mul(rotZ(a.yaw_deg * kDegToRad), rotY(a.pitch_deg * kDegToRad)), rotX(a.roll_deg * kDegToRad));
}

Mat3 transpose(const Mat3& a) {
    Mat3 r{};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            r.m[i][j] = a.m[j][i];
        }
    }
    return r;
}

Attitude matToEuler(const Mat3& r) {
    Attitude a;
    a.pitch_deg = std::asin(std::clamp(-r.m[2][0], -1.0, 1.0)) * kRadToDeg;
    a.roll_deg = std::atan2(r.m[2][1], r.m[2][2]) * kRadToDeg;
    a.yaw_deg = std::atan2(r.m[1][0], r.m[0][0]) * kRadToDeg;
    return a;
}

double halfTanH(const CameraModel& cam) {
    return std::tan(cam.hfov_deg * 0.5 * kDegToRad);
}

}  // namespace

Attitude composeMount(const Attitude& body, double mount_yaw_deg, double mount_pitch_deg) {
    const Mat3 mount = mul(rotZ(mount_yaw_deg * kDegToRad), rotY(mount_pitch_deg * kDegToRad));
    Attitude a = matToEuler(mul(eulerToMat(body), mount));
    a.yaw_deg = wrap360(a.yaw_deg);
    return a;
}

Attitude relativeTo(const Attitude& body, const Attitude& cam) {
    return matToEuler(mul(transpose(eulerToMat(body)), eulerToMat(cam)));
}

Vec3 pixelRayNed(const CameraModel& cam, const Attitude& att, double px, double py) {
    const double th = halfTanH(cam);
    const double tv = th * cam.height / cam.width;
    const double nx = (px - cam.width * 0.5) / (cam.width * 0.5);
    const double ny = (py - cam.height * 0.5) / (cam.height * 0.5);
    const Vec3 ray_cam{1.0, nx * th, ny * tv};
    return apply(cameraToNed(cam, att), ray_cam);
}

bool projectNed(const CameraModel& cam, const Attitude& att, const Vec3& rel_ned, double& px,
                double& py) {
    const Vec3 v = applyT(cameraToNed(cam, att), rel_ned);
    if (v.x <= 1e-6) {
        return false;
    }
    const double th = halfTanH(cam);
    const double tv = th * cam.height / cam.width;
    px = cam.width * 0.5 + (v.y / v.x) / th * (cam.width * 0.5);
    py = cam.height * 0.5 + (v.z / v.x) / tv * (cam.height * 0.5);
    return true;
}

GroundTarget cameraRayToGround(const CameraModel& cam, const Attitude& att, double px, double py,
                               double alt_agl, double max_range, double fallback_range) {
    const Vec3 ray = pixelRayNed(cam, att, px, py);
    GroundTarget t;

    const double horiz = std::hypot(ray.x, ray.y);
    t.bearing_deg = horiz > 1e-9 ? wrap360(std::atan2(ray.y, ray.x) * kRadToDeg)
                                 : wrap360(att.yaw_deg);

    if (alt_agl > 1.0 && ray.z > 1e-6) {
        const double s = alt_agl / ray.z;
        const double n = ray.x * s;
        const double e = ray.y * s;
        const double range = std::hypot(n, e);
        if (range <= max_range) {
            t.hit_ground = true;
            t.north = n;
            t.east = e;
            t.range = range;
            return t;
        }
    }

    const double b = t.bearing_deg * kDegToRad;
    t.hit_ground = false;
    t.range = fallback_range;
    t.north = fallback_range * std::cos(b);
    t.east = fallback_range * std::sin(b);
    return t;
}

}  // namespace gcs::geo
