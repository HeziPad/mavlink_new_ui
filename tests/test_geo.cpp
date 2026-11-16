#include "check.hpp"
#include "core/geo.hpp"

using namespace gcs::geo;

static void testDistanceBearing() {
    // 0.001 deg latitude ~ 111.3 m due north
    CHECK_NEAR(distance(47.0, 8.0, 47.001, 8.0), 111.3, 0.5);
    CHECK_NEAR(bearing(47.0, 8.0, 47.001, 8.0), 0.0, 1e-6);
    CHECK_NEAR(bearing(47.0, 8.0, 47.0, 8.001), 90.0, 0.01);
    CHECK_NEAR(bearing(47.0, 8.0, 46.999, 8.0), 180.0, 1e-6);
    CHECK_NEAR(wrap180(270.0), -90.0, 1e-9);
    CHECK_NEAR(wrap360(-30.0), 330.0, 1e-9);
}

static void testOffsetRoundTrip() {
    double lat = 0, lon = 0;
    offset(-35.363, 149.165, 250.0, -120.0, lat, lon);
    const NE d = delta(-35.363, 149.165, lat, lon);
    CHECK_NEAR(d.north, 250.0, 0.05);
    CHECK_NEAR(d.east, -120.0, 0.05);
    CHECK_NEAR(distance(-35.363, 149.165, lat, lon), 277.3, 0.5);

    destination(-35.363, 149.165, 45.0, 1000.0, lat, lon);
    CHECK_NEAR(distance(-35.363, 149.165, lat, lon), 1000.0, 1.0);
    CHECK_NEAR(bearing(-35.363, 149.165, lat, lon), 45.0, 0.1);
}

static void testCameraNadir() {
    CameraModel cam;
    cam.mount_pitch_deg = -90.0;
    Attitude att;
    const GroundTarget t = cameraRayToGround(cam, att, cam.width / 2, cam.height / 2, 100.0);
    CHECK(t.hit_ground);
    CHECK_NEAR(t.north, 0.0, 1e-6);
    CHECK_NEAR(t.east, 0.0, 1e-6);
}

static void testCameraTilted() {
    CameraModel cam;
    cam.mount_pitch_deg = -45.0;
    Attitude att;
    GroundTarget t = cameraRayToGround(cam, att, cam.width / 2, cam.height / 2, 100.0);
    CHECK(t.hit_ground);
    CHECK_NEAR(t.north, 100.0, 1e-6);
    CHECK_NEAR(t.east, 0.0, 1e-6);

    // heading east: target moves east
    att.yaw_deg = 90.0;
    t = cameraRayToGround(cam, att, cam.width / 2, cam.height / 2, 100.0);
    CHECK_NEAR(t.north, 0.0, 1e-6);
    CHECK_NEAR(t.east, 100.0, 1e-6);
    CHECK_NEAR(t.bearing_deg, 90.0, 1e-6);

    // nose pitched up 10 deg -> camera looks 35 deg down -> farther
    att = {};
    att.pitch_deg = 10.0;
    t = cameraRayToGround(cam, att, cam.width / 2, cam.height / 2, 100.0);
    CHECK_NEAR(t.north, 100.0 / std::tan(35.0 * kDegToRad), 1e-6);

    // right half of the image -> target to the east of track
    att = {};
    t = cameraRayToGround(cam, att, cam.width * 0.9, cam.height / 2, 100.0);
    CHECK(t.east > 10.0);
    // roll right (right wing down) -> the belly, and a down-looking camera, swing left
    att.roll_deg = 20.0;
    t = cameraRayToGround(cam, att, cam.width / 2, cam.height / 2, 100.0);
    CHECK(t.east < -10.0);
}

static void testCameraHorizonFallback() {
    CameraModel cam;
    cam.mount_pitch_deg = 0.0;
    Attitude att;
    att.yaw_deg = 30.0;
    // top of the image looks above the horizon
    const GroundTarget t = cameraRayToGround(cam, att, cam.width / 2, 0.0, 100.0, 2000.0, 300.0);
    CHECK(!t.hit_ground);
    CHECK_NEAR(t.range, 300.0, 1e-9);
    CHECK_NEAR(t.bearing_deg, 30.0, 1e-6);

    // on the ground: always fallback
    cam.mount_pitch_deg = -45.0;
    const GroundTarget g = cameraRayToGround(cam, att, cam.width / 2, cam.height / 2, 0.0);
    CHECK(!g.hit_ground);
}

static void testProjectRoundTrip() {
    CameraModel cam;
    cam.mount_pitch_deg = -30.0;
    Attitude att{5.0, -3.0, 123.0};
    const double px = 900.0, py = 500.0, alt = 80.0;
    const GroundTarget t = cameraRayToGround(cam, att, px, py, alt);
    CHECK(t.hit_ground);
    double rx = 0, ry = 0;
    CHECK(projectNed(cam, att, Vec3{t.north, t.east, alt}, rx, ry));
    CHECK_NEAR(rx, px, 1e-6);
    CHECK_NEAR(ry, py, 1e-6);

    // a point behind the camera does not project
    CHECK(!projectNed(cam, Attitude{}, Vec3{-100.0, 0.0, 0.0}, rx, ry));
}

int main() {
    testDistanceBearing();
    testOffsetRoundTrip();
    testCameraNadir();
    testCameraTilted();
    testCameraHorizonFallback();
    testProjectRoundTrip();
    return TEST_MAIN_RESULT();
}
