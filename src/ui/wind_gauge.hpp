#pragma once

#include <QRectF>

class QPainter;

namespace gcs::ui {

struct WindInfo {
    bool valid = false;
    double from_deg = 0.0;  // direction the wind blows FROM
    double speed = 0.0;     // m/s
};

constexpr double kCalmWind = 0.5;  // m/s

// Wind gauge in the HUD palette: a ring with an arrow showing where the wind
// blows to, streaks drifting along it, and FROM / speed underneath.
//   north_up: the ring is the map's compass (N at the top); otherwise it is
//             relative to the aircraft's nose (nose at the top).
//   compact:  ring only (collapsed puck / tight spots).
// `time` drives the streaks; `heading_deg` is the aircraft's heading.
// Draws inside `r` (square-ish ring on top, text rows below when not compact).
void paintWindGauge(QPainter& p, const QRectF& r, const WindInfo& w, double heading_deg, double time, bool north_up,
                    bool compact);

// Headwind (+) / tailwind (-) and crosswind (+ from the right) for a heading.
void windComponents(const WindInfo& w, double heading_deg, double& head, double& cross);

}  // namespace gcs::ui
