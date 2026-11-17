#include "check.hpp"
#include "core/geo.hpp"
#include "core/vehicle_state.hpp"

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

static void testMountComposition() {
    // level aircraft heading 90, camera tilted down 30 and panned right 40
    const Attitude body{0.0, 0.0, 90.0};
    const Attitude cam = composeMount(body, 40.0, -30.0);
    CHECK_NEAR(cam.yaw_deg, 130.0, 1e-6);
    CHECK_NEAR(cam.pitch_deg, -30.0, 1e-6);
    const Attitude rel = relativeTo(body, cam);
    CHECK_NEAR(rel.yaw_deg, 40.0, 1e-6);
    CHECK_NEAR(rel.pitch_deg, -30.0, 1e-6);
    // pitched-up aircraft: earth elevation = body pitch + mount tilt (no pan)
    const Attitude up{0.0, 10.0, 0.0};
    CHECK_NEAR(composeMount(up, 0.0, -30.0).pitch_deg, -20.0, 1e-6);
}

static void testPrediction() {
    using namespace gcs;
    VehicleState s;
    const auto t0 = Clock::now();
    s.roll_deg = 10.0;
    s.roll_rate = 20.0;  // deg/s
    s.yaw_deg = 359.0;
    s.yaw_rate = 10.0;
    s.att_rx = t0;
    auto a = predictAttitude(s, t0 + std::chrono::milliseconds(100));
    CHECK_NEAR(a.roll_deg, 12.0, 1e-6);
    CHECK_NEAR(a.yaw_deg, 0.0, 1e-6);  // wraps
    // capped: a stale sample does not run away
    a = predictAttitude(s, t0 + std::chrono::seconds(5));
    CHECK_NEAR(a.roll_deg, 10.0 + 20.0 * kMaxPredictAttitude, 1e-6);

    s.lat = 47.0;
    s.lon = 8.0;
    s.alt_rel = 50.0;
    s.vn = 20.0;
    s.vd = -2.0;  // climbing
    s.pos_rx = t0;
    const auto p = predictPosition(s, t0 + std::chrono::milliseconds(250));
    CHECK_NEAR(distance(47.0, 8.0, p.lat, p.lon), 5.0, 0.01);
    CHECK_NEAR(p.alt_rel, 50.5, 1e-6);

    // body rates -> euler rates: level, pure yaw rate
    double rr = 0, pr = 0, yr = 0;
    bodyRatesToEuler(0.0, 0.0, 0.0, 0.0, 0.1, rr, pr, yr);
    CHECK_NEAR(yr, 0.1 * kRadToDeg, 1e-9);
    // banked 60 deg in a level turn: yaw rate comes from q and r
    bodyRatesToEuler(60.0 * kDegToRad, 0.0, 0.0, 0.1 * std::sin(60.0 * kDegToRad), 0.1 * std::cos(60.0 * kDegToRad),
                     rr, pr, yr);
    CHECK_NEAR(yr, 0.1 * kRadToDeg, 1e-9);
    CHECK_NEAR(pr, 0.0, 1e-9);
}

static void testCameraLos() {
    using namespace gcs;
    VehicleState s;
    const auto now = Clock::now();
    const Attitude ac{0.0, 5.0, 100.0};
    // no gimbal: fixed tilt
    auto los = cameraLos(s, ac, -30.0, now);
    CHECK(!los.gimbal);
    CHECK_NEAR(los.tilt_rel, -30.0, 1e-9);
    CHECK_NEAR(los.earth.pitch_deg, -25.0, 1e-6);
    // ArduPilot gimbal: earth-frame tilt, vehicle-frame pan
    s.has_gimbal = true;
    s.gimbal_rx = now;
    s.gimbal_pitch = -40.0;
    s.gimbal_yaw = 30.0;
    los = cameraLos(s, ac, -30.0, now);
    CHECK(los.gimbal);
    CHECK_NEAR(los.earth.yaw_deg, 130.0, 1e-6);
    CHECK_NEAR(los.earth.pitch_deg, -40.0, 1e-6);
    // relative to the platform (body axes): close to the reported pan, and
    // composing it back onto the aircraft gives the same earth LOS
    CHECK_NEAR(los.pan_rel, 30.0, 3.0);
    CHECK_NEAR(los.tilt_rel, -45.0, 3.0);
    const Attitude again = composeMount(ac, los.pan_rel, los.tilt_rel);
    CHECK_NEAR(wrap180(again.yaw_deg - los.earth.yaw_deg), 0.0, 1e-6);
    CHECK_NEAR(again.pitch_deg, los.earth.pitch_deg, 1e-6);
    // body-frame reporting (ArduPilot servo mount): angles are relative to the airframe
    los = cameraLos(s, ac, -30.0, now, true);
    CHECK_NEAR(los.pan_rel, 30.0, 1e-9);
    CHECK_NEAR(los.tilt_rel, -40.0, 1e-9);
    CHECK_NEAR(los.earth.pitch_deg, composeMount(ac, 30.0, -40.0).pitch_deg, 1e-9);
    // stale gimbal falls back
    los = cameraLos(s, ac, -30.0, now + std::chrono::seconds(5));
    CHECK(!los.gimbal);
}

static void testFootprint() {
    // straight down, square image, 90 deg FOV, 100 m up: a 200 m square
    CameraModel cam;
    cam.hfov_deg = 90.0;
    cam.mount_pitch_deg = 0.0;
    cam.width = cam.height = 1000.0;
    Attitude down{0.0, -90.0, 0.0};
    Footprint fp = cameraFootprint(cam, down, 100.0, 3000.0);
    CHECK(fp.valid);
    CHECK(fp.outline.size() == 32);
    double maxn = 0, maxe = 0;
    for (size_t i = 0; i < fp.outline.size(); ++i) {
        CHECK(!fp.open[i]);
        maxn = std::max(maxn, std::abs(fp.outline[i].north));
        maxe = std::max(maxe, std::abs(fp.outline[i].east));
    }
    CHECK_NEAR(maxn, 100.0, 1.0);
    CHECK_NEAR(maxe, 100.0, 1.0);
    CHECK_NEAR(fp.center.range, 0.0, 0.5);

    // 30 deg down looking north, 16:9: a trapezoid, wider far than near
    cam.hfov_deg = 60.0;
    cam.width = 1600.0;
    cam.height = 900.0;
    Attitude tilted{0.0, -30.0, 0.0};
    fp = cameraFootprint(cam, tilted, 100.0, 3000.0, 4);
    CHECK(fp.valid);
    CHECK_NEAR(fp.center.range, 100.0 / std::tan(30.0 * kDegToRad), 1.0);
    // corners: 0 = top-left (far), 4 = top-right, 8 = bottom-right (near), 12 = bottom-left
    const double far_w = fp.outline[4].east - fp.outline[0].east;
    const double near_w = fp.outline[8].east - fp.outline[12].east;
    CHECK(far_w > near_w * 1.5);
    CHECK(fp.outline[0].north > fp.outline[12].north);
    for (bool o : fp.open) {
        CHECK(!o);
    }

    // looking just above the horizon: the top edge is open at max range
    Attitude level{0.0, 0.0, 90.0};
    fp = cameraFootprint(cam, level, 100.0, 1500.0, 4);
    CHECK(fp.valid);  // the lower half still sees the ground
    CHECK(fp.open[0] && fp.open[2]);
    CHECK(!fp.open[8]);
    CHECK_NEAR(std::hypot(fp.outline[0].north, fp.outline[0].east), 1500.0, 1.0);
}

int main() {
    testFootprint();
    testMountComposition();
    testPrediction();
    testCameraLos();
    testDistanceBearing();
    testOffsetRoundTrip();
    testCameraNadir();
    testCameraTilted();
    testCameraHorizonFallback();
    testProjectRoundTrip();
    return TEST_MAIN_RESULT();
}
