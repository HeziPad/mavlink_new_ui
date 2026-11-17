#pragma once

#include "core/frame_layout.hpp"
#include "core/preflight.hpp"

#include <QRectF>
#include <QString>

#include <array>
#include <vector>

class QPainter;

// Depictions for the preflight cards, in the HUD palette. All pure painters.
namespace gcs::ui::art {

// part of the airframe to highlight on the top view
enum class Part { None, Structure, Props, Pusher, Linkages, Pitot, CG, Battery, Antennas, Motors };

struct TopView {
    FrameLayout frame;
    int lit_motor = -1;        // index into frame.motors to light up (spinning)
    bool show_dirs = false;    // rotation arrows on every lift prop
    bool show_letters = false; // motor-test letters A..D
    Part highlight = Part::None;
    bool pusher_lit = false;
};
void quadplaneTop(QPainter& p, const QRectF& r, const TopView& v, double time);

// accelerometer calibration pose (ACCELCAL_VEHICLE_POS 1..6), side/front view
void pose(QPainter& p, const QRectF& r, int pos, double time);
QString poseName(int pos);

// control surfaces: `cmd` is the stick command being swept (-1..1, + = roll
// right / pitch up / yaw right). Draws the expected deflection, the live
// output bar underneath (`out` -1..1, NaN = unknown).
enum class Axis { Roll, Pitch, Yaw };
void surface(QPainter& p, const QRectF& r, Axis axis, double cmd, double out, double time);

// transmitter sticks (mode 2): roll/pitch/yaw -1..1, throttle 0..1, and which
// ends each axis has reached (roll-, roll+, pitch-, pitch+, thr-, thr+, yaw-, yaw+)
void sticks(QPainter& p, const QRectF& r, double roll, double pitch, double thr, double yaw,
            const std::array<bool, 8>& reached);

// compass calibration coverage: 80 sections (MAG_CAL_PROGRESS completion_mask)
void compassSphere(QPainter& p, const QRectF& r, const std::array<uint8_t, 10>& mask, int pct, double time);

// battery: per-cell bars when known, else one pack bar
void battery(QPainter& p, const QRectF& r, const std::vector<double>& cells, double pack_v, int ncells,
             double full_v, double storage_v);

// a 10 m clear circle around the aircraft, people standing outside
void site(QPainter& p, const QRectF& r, double time);

// pitot: air flowing in, the live airspeed
void pitot(QPainter& p, const QRectF& r, double airspeed, double time);

// gimbal looking where it's commanded (pitch / yaw in degrees)
void gimbal(QPainter& p, const QRectF& r, double pitch, double yaw);

// a big status glyph (✓ ✗ ! … ·)
void statusGlyph(QPainter& p, const QPointF& c, double radius, preflight::Status s, double time);

// a horizontal value gauge with a good band
void gauge(QPainter& p, const QRectF& r, double value, double lo, double hi, double good_lo, double good_hi,
           const QString& label);

}  // namespace gcs::ui::art
