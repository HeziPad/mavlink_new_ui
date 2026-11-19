#pragma once

#include "core/geo.hpp"
#include "payload/payload_manager.hpp"

#include <array>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace gcs::payload {

struct GeoPoint {
    double lat = 0.0;
    double lon = 0.0;
};

// What a payload adapter tells the rest of the GCS, in GCS terms. Everything
// is optional: an ICD without an adapter still gets the generic panel.
struct PayloadView {
    bool camera = false;            // this payload is a camera with a known line of sight
    geo::Attitude earth;            // camera attitude, earth frame (yaw = LOS azimuth, pitch = elevation)
    double hfov_deg = 0.0;          // 0 = unknown
    std::optional<GeoPoint> ground;          // LOS ground crossing
    double ground_alt = 0.0;                    // m AMSL
    double slant_range = 0.0;                   // m
    std::array<std::optional<GeoPoint>, 4> corners;  // footprint UL, UR, LR, LL
    std::string mode;               // the payload's own mode name
    std::string sensor;
    bool tracking = false;
    std::optional<GeoPoint> track_target;
    bool recording = false;
    std::vector<std::string> status;  // short chips: "REC CH0", "ILLUM", "BOOT"...
};

// A command call (ICD command name + its args in params order).
struct CommandCall {
    std::string name;
    std::vector<double> args;
};

// The optional, payload-specific glue: turns decoded reports into a
// PayloadView, and GCS gestures (tap on video, stick, zoom) into commands.
class PayloadAdapter {
public:
    virtual ~PayloadAdapter() = default;
    virtual std::string icdId() const = 0;
    // false: nothing usable yet (no reports)
    virtual bool view(const DeviceSnapshot& dev, PayloadView& out) const = 0;
    // tap on the video at normalized (x, y) of a box w x h (0..1, from the top-left)
    virtual std::optional<CommandCall> trackAt(double nx, double ny, double nw, double nh) const {
        (void)nx, (void)ny, (void)nw, (void)nh;
        return std::nullopt;
    }
    // stick: rates -1..1 (right / up positive), zoom -1 out / 0 stop / +1 in; and its rate
    virtual std::optional<CommandCall> joystick(double pan, double tilt, int zoom) const {
        (void)pan, (void)tilt, (void)zoom;
        return std::nullopt;
    }
    virtual double joystickHz() const { return 0.0; }
    // one-tap actions for the panel (label, call)
    virtual std::vector<std::pair<std::string, CommandCall>> quickActions(const DeviceSnapshot& dev) const {
        (void)dev;
        return {};
    }
};

// Adapter for a loaded ICD, or nullptr when there is none (generic only).
std::unique_ptr<PayloadAdapter> makeAdapter(const std::string& icd_id);

}  // namespace gcs::payload
