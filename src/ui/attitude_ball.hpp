#pragma once

#include <QPointF>

class QPainter;

namespace gcs::ui {

// A cockpit-style attitude ball in the HUD palette: a shaded sky/ground sphere
// whose pitch ladder wraps around it (lines bunch up and shorten toward the
// edge), a fixed bank scale with a pointer riding the horizon, a fixed
// aircraft symbol, and the heading underneath. `compact` drops the ladder
// numbers, bank-scale numbers and heading box (PiP / collapsed puck sizes).
void paintAttitudeBall(QPainter& p, const QPointF& centre, double radius, double roll_deg, double pitch_deg,
                       double yaw_deg, bool compact = false);

}  // namespace gcs::ui
