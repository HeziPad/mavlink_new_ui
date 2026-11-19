// NextVision TRIP (icds/nextvision_trip.json): camera LOS, footprint and
// tracking for the map / video, and the gestures the operator uses.

#include "payload/adapter.hpp"

#include <cmath>

namespace gcs::payload {

namespace {

constexpr double kInvalid = 400.0;

bool validLatLon(double lat, double lon) {
    return std::fabs(lat - kInvalid) > 1e-3 && std::fabs(lon - kInvalid) > 1e-3 && std::fabs(lat) <= 90.0 &&
           std::fabs(lon) <= 180.0 && !(lat == 0.0 && lon == 0.0);
}

// LOS matrix M = Ly(Y) Lx(X) Lz(Z) (ICD LOS report, note 1): NED -> LOS frame,
// row 0 = image vertical (up), row 1 = image right, row 2 = the line of sight.
// The GCS camera attitude is a body looking along the LOS: forward = row 2,
// right = row 1, down = -row 0; its Z-Y-X Euler angles.
geo::Attitude losAttitude(double X, double Y, double Z) {
    const double cx = std::cos(X), sx = std::sin(X), cy = std::cos(Y), sy = std::sin(Y), cz = std::cos(Z),
                 sz = std::sin(Z);
    const double r0[3] = {cy * cz - sy * sx * sz, cy * sz + sy * sx * cz, -sy * cx};
    const double r1[3] = {-cx * sz, cx * cz, sx};
    const double r2[3] = {sy * cz + cy * sx * sz, sy * sz - cy * sx * cz, cy * cx};
    geo::Attitude a;
    a.yaw_deg = geo::wrap360(std::atan2(r2[1], r2[0]) * geo::kRadToDeg);
    a.pitch_deg = -std::asin(std::fmax(-1.0, std::fmin(1.0, r2[2]))) * geo::kRadToDeg;
    a.roll_deg = std::atan2(r1[2], -r0[2]) * geo::kRadToDeg;
    return a;
}

class NextVisionAdapter : public PayloadAdapter {
public:
    std::string icdId() const override { return "nextvision_trip"; }

    bool view(const DeviceSnapshot& dev, PayloadView& v) const override {
        if (!dev.icd) {
            return false;
        }
        const Icd& icd = *dev.icd;
        const DecodedReport* sys = find(dev, 0);
        const DecodedReport* los = find(dev, 1);
        const DecodedReport* gc = find(dev, 2);
        if (!sys && !los) {
            return false;
        }
        if (los) {
            v.camera = true;
            v.earth = losAttitude(los->num("los_x"), los->num("los_y"), los->num("los_z"));
            static const char* names[4][2] = {{"ul_lat", "ul_lon"}, {"ur_lat", "ur_lon"}, {"lr_lat", "lr_lon"},
                                              {"ll_lat", "ll_lon"}};
            for (size_t k = 0; k < 4; ++k) {
                const double lat = los->num(names[k][0], kInvalid), lon = los->num(names[k][1], kInvalid);
                if (validLatLon(lat, lon)) {
                    v.corners[k] = GeoPoint{lat, lon};
                }
            }
        }
        if (gc) {
            const double lat = gc->num("lat", kInvalid), lon = gc->num("lon", kInvalid);
            if (validLatLon(lat, lon)) {
                v.ground = GeoPoint{lat, lon};
            }
            v.ground_alt = gc->num("alt");
            v.slant_range = gc->num("slant_range");
        }
        if (sys) {
            v.hfov_deg = sys->num("fov");
            const auto mode = static_cast<int64_t>(sys->num("system_mode"));
            v.mode = icd.enumLabel("SystemModeReport", mode);
            v.sensor = icd.enumLabel("Sensor", static_cast<int64_t>(sys->num("active_sensor")));
            v.tracking = mode == 10 || mode == 19 || mode == 20;
            const auto rec = static_cast<int>(sys->num("recording_status"));
            v.recording = (rec & 0xF) == 1 || ((rec >> 4) & 0xF) == 1;
            if ((rec & 0xF) == 1) {
                v.status.push_back("REC CH0");
            }
            if (((rec >> 4) & 0xF) == 1) {
                v.status.push_back("REC CH1");
            }
            if ((rec & 0xF) == 2) {
                v.status.push_back("NO SD");
            }
            if (sys->num("illuminator_status") != 0) {
                v.status.push_back("ILLUM");
            }
            if (static_cast<int>(sys->num("status_flags")) & 1) {
                v.status.push_back("BOOT");
            }
            if (static_cast<int>(sys->num("bit_status")) != 0) {
                v.status.push_back("BIT FAIL");
            }
        }
        // the tracked point, from whichever tracking report is flowing
        for (int rid : {12, 23, 24}) {
            const DecodedReport* t = find(dev, rid);
            if (!t || !v.tracking) {
                continue;
            }
            const double lat = t->num(rid == 12 ? "roi_lat" : "lat", kInvalid);
            const double lon = t->num(rid == 12 ? "roi_lon" : "lon", kInvalid);
            if (validLatLon(lat, lon)) {
                v.track_target = GeoPoint{lat, lon};
            }
        }
        return true;
    }

    std::optional<CommandCall> trackAt(double nx, double ny, double nw, double nh) const override {
        // Smart Tracker: track on a bounding box (top-left corner, normalized)
        const double w = std::fmin(nw, 1.0), h = std::fmin(nh, 1.0);
        const double x = std::fmax(0.0, std::fmin(1.0 - w, nx - w / 2)), y = std::fmax(0.0, std::fmin(1.0 - h, ny - h / 2));
        return CommandCall{"SmartTracker.TrackBBox", {0, x, y, w, h}};
    }

    std::optional<CommandCall> joystick(double pan, double tilt, int zoom) const override {
        const double z = zoom > 0 ? 1 : zoom < 0 ? 2 : 0;
        // ground altitude 0: the TRIP keeps its own (takeoff / Set Ground Crossing Alt)
        return CommandCall{"SetGimbal", {pan, tilt, z, 0}};
    }
    double joystickHz() const override { return 20.0; }

    std::vector<std::pair<std::string, CommandCall>> quickActions(const DeviceSnapshot& dev) const override {
        std::vector<std::pair<std::string, CommandCall>> a = {
            {"OBSERVE", {"SetSystemMode", {3}}},
            {"GRR", {"SetSystemMode", {6}}},
            {"HOLD", {"SetSystemMode", {2}}},
            {"NADIR", {"SetSystemMode", {9}}},
            {"PILOT", {"SetSystemMode", {1}}},
            {"STOW", {"SetSystemMode", {0}}},
            {"DAY", {"SetSensor", {0}}},
            {"IR", {"SetSensor", {1}}},
            {"SNAP", {"TakeSnapshot", {0}}},
        };
        const DecodedReport* sys = find(dev, 0);
        const bool rec = sys && (static_cast<int>(sys->num("recording_status")) & 0xF) == 1;
        a.push_back({rec ? "REC ■" : "REC ●", {"SetRecording", {rec ? 0.0 : 1.0, 0}}});
        a.push_back({"WIDE", {"SetFOV", {90}}});
        const bool locked = sys && static_cast<int>(sys->num("system_mode")) == 3;
        a.push_back(locked ? std::make_pair(std::string("UNLOCK"), CommandCall{"ClearRetractLock", {}})
                           : std::make_pair(std::string("RETRACT"), CommandCall{"MountControl", {0, 0, 0, 0, 0, 0, 0}}));
        return a;
    }

private:
    static const DecodedReport* find(const DeviceSnapshot& dev, int rid) {
        const auto it = dev.reports.find(rid);
        return it == dev.reports.end() || !it->second.report.def ? nullptr : &it->second.report;
    }
};

}  // namespace

std::unique_ptr<PayloadAdapter> makeNextVisionAdapter() { return std::make_unique<NextVisionAdapter>(); }

}  // namespace gcs::payload
