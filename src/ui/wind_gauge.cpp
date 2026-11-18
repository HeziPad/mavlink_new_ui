#include "wind_gauge.hpp"

#include "core/geo.hpp"
#include "theme.hpp"
#include "units.hpp"

#include <QPainter>
#include <QPainterPath>

#include <algorithm>
#include <cmath>

namespace gcs::ui {

using namespace theme;

void windComponents(const WindInfo& w, double heading_deg, double& head, double& cross) {
    const double rel = (w.from_deg - heading_deg) * geo::kDegToRad;
    head = w.speed * std::cos(rel);
    cross = w.speed * std::sin(rel);
}

void paintWindGauge(QPainter& p, const QRectF& r, const WindInfo& w, double heading_deg, double time, bool north_up,
                    bool compact) {
    p.save();
    p.setRenderHint(QPainter::Antialiasing);
    const double text_h = compact ? 0.0 : 34.0;
    const double ring_h = r.height() - text_h;
    const double rad = std::max(8.0, std::min(r.width(), ring_h) / 2.0 - 4.0);
    const QPointF c(r.center().x(), r.top() + ring_h / 2.0);
    const bool calm = !w.valid || w.speed < kCalmWind;
    const QColor col = !w.valid ? kMuted : kCyan;

    // ring + reference tick (N, or the nose)
    p.setPen(QPen(alpha(col, 0.55), 1.2));
    p.setBrush(alpha(kBg, 0.5));
    p.drawEllipse(c, rad, rad);
    for (int k = 0; k < 8; ++k) {
        p.save();
        p.translate(c);
        p.rotate(k * 45.0);
        p.setPen(QPen(alpha(col, k == 0 ? 0.95 : 0.35), k == 0 ? 2.0 : 1.0));
        p.drawLine(QPointF(0, -rad), QPointF(0, -rad + (k % 2 ? 3.0 : 5.0)));
        p.restore();
    }
    if (!compact) {
        drawText(p, QRectF(c.x() - 10, c.y() - rad - 14, 20, 12), Qt::AlignCenter,
                 north_up ? QStringLiteral("N") : QStringLiteral("▲"), labelFont(8.5), alpha(col, 0.9));
    }

    if (calm) {
        drawText(p, QRectF(c.x() - rad, c.y() - 8, 2 * rad, 16), Qt::AlignCenter,
                 w.valid ? QStringLiteral("CALM") : QStringLiteral("—"), labelFont(compact ? 7.5 : 9.5), col);
    } else {
        // blows TO from+180; screen angle from the top, clockwise
        const double to = w.from_deg + 180.0 - (north_up ? 0.0 : heading_deg);
        p.save();
        p.translate(c);
        p.rotate(to);
        // streaks drifting downwind (speed-scaled)
        if (!reducedMotion()) {
            const double rate = 0.35 + std::min(1.5, w.speed / 8.0);
            for (int i = 0; i < 3; ++i) {
                const double ph = std::fmod(time * rate + i / 3.0, 1.0);
                const double y = rad * 0.85 - ph * rad * 1.7;  // from the upwind edge (bottom) to downwind (top)
                const double x = (i - 1) * rad * 0.42;
                const double a = std::sin(ph * geo::kPi);
                p.setPen(QPen(alpha(col, 0.45 * a), 1.2, Qt::SolidLine, Qt::RoundCap));
                p.drawLine(QPointF(x, y), QPointF(x, y + rad * 0.28));
            }
        }
        // arrow
        const double len = rad * 0.8;
        QPainterPath arrow;
        arrow.moveTo(0, -len);
        arrow.lineTo(rad * 0.28, -len + rad * 0.42);
        arrow.lineTo(rad * 0.09, -len + rad * 0.36);
        arrow.lineTo(rad * 0.09, len);
        arrow.lineTo(-rad * 0.09, len);
        arrow.lineTo(-rad * 0.09, -len + rad * 0.36);
        arrow.lineTo(-rad * 0.28, -len + rad * 0.42);
        arrow.closeSubpath();
        p.setPen(QPen(alpha(kBg, 0.8), 2.5));
        p.setBrush(Qt::NoBrush);
        p.drawPath(arrow);
        p.setPen(Qt::NoPen);
        p.setBrush(col);
        p.drawPath(arrow);
        p.restore();
    }

    if (!compact) {
        const QRectF t1(r.left(), r.bottom() - text_h, r.width(), 17);
        const QRectF t2(r.left(), r.bottom() - text_h + 17, r.width(), 15);
        if (!w.valid) {
            drawText(p, t1, Qt::AlignCenter, QStringLiteral("WIND —"), labelFont(9.5), kMuted);
        } else {
            drawText(p, t1, Qt::AlignCenter,
                     QStringLiteral("%1° · %2")
                         .arg(static_cast<int>(std::lround(geo::wrap360(w.from_deg))) % 360, 3, 10, QLatin1Char('0'))
                         .arg(units::speedText(w.speed, 1)),
                     monoFont(11.5, true), kText);
            double head = 0.0, cross = 0.0;
            windComponents(w, heading_deg, head, cross);
            const QString hw = head >= 0 ? QStringLiteral("HEAD %1").arg(units::speedText(head, 1, false))
                                         : QStringLiteral("TAIL %1").arg(units::speedText(-head, 1, false));
            const QString xw = QStringLiteral("X%1 %2")
                                   .arg(cross >= 0 ? QLatin1Char('R') : QLatin1Char('L'))
                                   .arg(units::speedText(std::abs(cross), 1, false));
            drawText(p, t2, Qt::AlignCenter, calm ? QStringLiteral("calm") : hw + QStringLiteral("  ") + xw,
                     monoFont(9.5), alpha(kMuted, 0.95));
        }
    }
    p.restore();
}

}  // namespace gcs::ui
