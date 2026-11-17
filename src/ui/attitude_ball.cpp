#include "attitude_ball.hpp"

#include "core/geo.hpp"
#include "theme.hpp"

#include <QLinearGradient>
#include <QPainter>
#include <QPainterPath>
#include <QRadialGradient>

#include <algorithm>
#include <cmath>

namespace gcs::ui {

using namespace theme;

namespace {
// degrees of pitch from the centre to the rim of the ball
constexpr double kVisibleDeg = 60.0;

// Spherical projection: a pitch line `d` degrees from the centre sits at
// R*sin(d * 90/kVisible) and is foreshortened by cos of the same angle.
double ballY(double d, double r) {
    const double a = std::clamp(d, -kVisibleDeg, kVisibleDeg) * (geo::kPi / 2.0) / kVisibleDeg;
    return r * std::sin(a);
}
double ballScale(double d) {
    const double a = std::clamp(d, -kVisibleDeg, kVisibleDeg) * (geo::kPi / 2.0) / kVisibleDeg;
    return std::cos(a);
}
}  // namespace

void paintAttitudeBall(QPainter& p, const QPointF& c, double r, double roll, double pitch, double yaw,
                       bool compact) {
    p.save();
    p.setRenderHint(QPainter::Antialiasing);

    // ---- sphere -------------------------------------------------------------
    QPainterPath disc;
    disc.addEllipse(c, r, r);
    p.save();
    p.setClipPath(disc);
    p.translate(c);
    p.rotate(-roll);
    // the horizon is the pitch line at 0: `-pitch` degrees from the centre
    const double hy = -ballY(-pitch, r);
    const double big = r * 3.0;
    QLinearGradient sky(0, hy - r * 2.0, 0, hy);
    sky.setColorAt(0.0, QColor(6, 30, 48));
    sky.setColorAt(1.0, QColor(28, 104, 140));
    p.fillRect(QRectF(-big, hy - big * 2.0, big * 2.0, big * 2.0), sky);
    QLinearGradient ground(0, hy, 0, hy + r * 2.0);
    ground.setColorAt(0.0, QColor(96, 64, 30));
    ground.setColorAt(1.0, QColor(34, 22, 10));
    p.fillRect(QRectF(-big, hy, big * 2.0, big * 2.0), ground);

    // pitch ladder wrapping around the ball
    for (int e = -80; e <= 80; e += (compact ? 20 : 10)) {
        if (e == 0) {
            continue;
        }
        const double d = e - pitch;
        if (std::abs(d) >= kVisibleDeg) {
            continue;
        }
        const double y = -ballY(d, r);
        const double k = ballScale(d);
        const bool major = e % 20 == 0;
        const double half = r * (major ? 0.34 : 0.2) * (0.35 + 0.65 * k);
        p.setPen(QPen(QColor(255, 255, 255, static_cast<int>(200 * k)), major ? 1.3 : 1.0));
        p.drawLine(QPointF(-half, y), QPointF(half, y));
        if (major && !compact) {
            drawText(p, QRectF(half + 2, y - 7, 22, 14), Qt::AlignLeft | Qt::AlignVCenter,
                     QString::number(std::abs(e)), monoFont(std::max(8.0, 9.5 * k)),
                     QColor(255, 255, 255, static_cast<int>(210 * k)));
        }
    }
    // horizon
    p.setPen(QPen(QColor(255, 255, 255, 60), 4.0));
    p.drawLine(QPointF(-big, hy), QPointF(big, hy));
    p.setPen(QPen(QColor(255, 255, 255, 235), 1.6));
    p.drawLine(QPointF(-big, hy), QPointF(big, hy));
    p.restore();

    // 3D shading: dark rim, soft specular highlight
    QRadialGradient shade(c, r);
    shade.setColorAt(0.0, QColor(0, 0, 0, 0));
    shade.setColorAt(0.72, QColor(0, 0, 0, 30));
    shade.setColorAt(1.0, QColor(0, 0, 0, 170));
    p.setPen(Qt::NoPen);
    p.setBrush(shade);
    p.drawEllipse(c, r, r);
    QRadialGradient spec(c + QPointF(-r * 0.35, -r * 0.4), r * 0.7);
    spec.setColorAt(0.0, QColor(255, 255, 255, 46));
    spec.setColorAt(1.0, QColor(255, 255, 255, 0));
    p.setBrush(spec);
    p.drawEllipse(c, r, r);

    // bezel
    p.setBrush(Qt::NoBrush);
    p.setPen(QPen(alpha(kCyan, 0.25), 5.0));
    p.drawEllipse(c, r + 2.5, r + 2.5);
    p.setPen(QPen(alpha(kCyan, 0.85), 1.4));
    p.drawEllipse(c, r + 1.0, r + 1.0);

    // ---- bank scale (fixed) + pointer (rides the horizon) -------------------
    p.save();
    p.translate(c);
    for (int b : {-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60}) {
        p.save();
        p.rotate(b);
        const bool major = b % 30 == 0 || std::abs(b) == 45;
        p.setPen(QPen(alpha(kText, 0.9), major ? 1.6 : 1.1));
        p.drawLine(QPointF(0, -r - 2), QPointF(0, -r - (major ? 10 : 6)));
        if (!compact && major && b != 0) {
            drawText(p, QRectF(-12, -r - 24, 24, 13), Qt::AlignCenter, QString::number(std::abs(b)), monoFont(9),
                     alpha(kText, 0.75));
        }
        p.restore();
    }
    p.rotate(-std::clamp(roll, -75.0, 75.0));
    const QColor pc = std::abs(roll) > 45.0 ? kAmber : kGreen;
    p.setPen(Qt::NoPen);
    p.setBrush(pc);
    const double ps = compact ? 4.0 : 6.0;
    p.drawPolygon(QPolygonF({QPointF(0, -r + 1), QPointF(-ps, -r + 1 + ps * 1.8), QPointF(ps, -r + 1 + ps * 1.8)}));
    p.restore();

    // ---- fixed aircraft symbol ------------------------------------------------
    const double w = r * 0.55;
    p.setPen(QPen(QColor(0, 0, 0, 150), compact ? 4.0 : 5.0, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    QPolygonF left({c + QPointF(-w, 0), c + QPointF(-w * 0.32, 0), c + QPointF(-w * 0.32, r * 0.1)});
    QPolygonF right({c + QPointF(w, 0), c + QPointF(w * 0.32, 0), c + QPointF(w * 0.32, r * 0.1)});
    p.drawPolyline(left);
    p.drawPolyline(right);
    p.setPen(QPen(kAmber, compact ? 2.0 : 2.6, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    p.drawPolyline(left);
    p.drawPolyline(right);
    p.setBrush(kAmber);
    p.setPen(Qt::NoPen);
    p.drawEllipse(c, compact ? 2.0 : 3.0, compact ? 2.0 : 3.0);

    // ---- heading ------------------------------------------------------------------
    if (!compact) {
        const QRectF box(c.x() - 26, c.y() + r + 8, 52, 20);
        drawGlass(p, chamferPath(box, 5), kCyan, 0.8, kGlassDeep);
        drawText(p, box, Qt::AlignCenter,
                 QStringLiteral("%1°").arg(static_cast<int>(std::lround(geo::wrap360(yaw))) % 360, 3, 10, QLatin1Char('0')),
                 monoFont(12, true), kText);
    }
    p.restore();
}

}  // namespace gcs::ui
