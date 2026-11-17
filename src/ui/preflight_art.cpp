#include "preflight_art.hpp"

#include "core/geo.hpp"
#include "theme.hpp"

#include <QPainter>
#include <QPainterPath>
#include <QRadialGradient>

#include <algorithm>
#include <cmath>

namespace gcs::ui::art {

using namespace theme;
using preflight::Status;

namespace {

QColor partColor(Part want, Part have) {
    return want == have ? kAmber : alpha(kCyan, 0.75);
}

void arrowHead(QPainter& p, const QPointF& tip, double angle_rad, double size, const QColor& c) {
    const QPointF d(std::cos(angle_rad), std::sin(angle_rad));
    const QPointF n(-d.y(), d.x());
    p.setPen(Qt::NoPen);
    p.setBrush(c);
    p.drawPolygon(QPolygonF({tip, tip - d * size + n * size * 0.55, tip - d * size - n * size * 0.55}));
}

// rotation arrow around a prop: an arc with a head, ccw or cw (screen y down)
void spinArrow(QPainter& p, const QPointF& c, double r, bool ccw, const QColor& col, double phase) {
    const double start = phase * 360.0;
    const double span = 250.0;
    QPainterPath arc;
    const QRectF box(c.x() - r, c.y() - r, 2 * r, 2 * r);
    // Qt angles are counter-clockwise on screen
    arc.arcMoveTo(box, start);
    arc.arcTo(box, start, ccw ? span : -span);
    p.setPen(QPen(col, 2.0, Qt::SolidLine, Qt::RoundCap));
    p.setBrush(Qt::NoBrush);
    p.drawPath(arc);
    const double end = (start + (ccw ? span : -span)) * geo::kDegToRad;
    const QPointF tip(c.x() + r * std::cos(end), c.y() - r * std::sin(end));
    // tangent direction of travel
    const double tang = ccw ? -end - geo::kPi / 2.0 : -end + geo::kPi / 2.0;
    arrowHead(p, tip, tang, 8.0, col);
}

}  // namespace

void quadplaneTop(QPainter& p, const QRectF& r, const TopView& v, double time) {
    p.save();
    p.setRenderHint(QPainter::Antialiasing);
    const QPointF c = r.center();
    const double s = std::min(r.width(), r.height()) / 2.0 * 0.92;  // half-size
    auto pt = [&](double x, double y) { return QPointF(c.x() + x * s, c.y() + y * s); };  // y down = aft
    const QColor base = alpha(kCyan, 0.75);
    const QColor fill = alpha(kCyan, 0.07);

    // wing
    QPainterPath wing;
    wing.moveTo(pt(-0.98, -0.08));
    wing.lineTo(pt(0.98, -0.08));
    wing.lineTo(pt(0.95, 0.10));
    wing.lineTo(pt(-0.95, 0.10));
    wing.closeSubpath();
    p.setPen(QPen(partColor(Part::Structure, v.highlight), v.highlight == Part::Structure ? 2.4 : 1.4));
    p.setBrush(fill);
    p.drawPath(wing);
    // ailerons (outer trailing edge)
    p.setPen(QPen(partColor(Part::Linkages, v.highlight), v.highlight == Part::Linkages ? 2.4 : 1.2));
    p.drawRect(QRectF(pt(-0.92, 0.04), pt(-0.55, 0.10)));
    p.drawRect(QRectF(pt(0.55, 0.04), pt(0.92, 0.10)));
    // fuselage
    QPainterPath fus;
    fus.addRoundedRect(QRectF(pt(-0.07, -0.62), pt(0.07, 0.62)), s * 0.07, s * 0.07);
    p.setPen(QPen(partColor(Part::Structure, v.highlight), 1.4));
    p.setBrush(alpha(kBg, 0.9));
    p.drawPath(fus);
    // tail: horizontal stabiliser + elevator, fin
    p.setBrush(fill);
    p.drawRect(QRectF(pt(-0.30, 0.68), pt(0.30, 0.76)));
    p.setPen(QPen(partColor(Part::Linkages, v.highlight), v.highlight == Part::Linkages ? 2.4 : 1.2));
    p.drawRect(QRectF(pt(-0.28, 0.76), pt(0.28, 0.80)));
    p.drawLine(pt(0, 0.62), pt(0, 0.86));

    // booms: through the front and rear motors on each side
    const auto& ms = v.frame.motors;
    p.setPen(QPen(partColor(Part::Structure, v.highlight), v.highlight == Part::Structure ? 2.6 : 1.6));
    auto motorPos = [&](const LiftMotor& m) {
        const double a = m.angle * geo::kDegToRad;
        return QPointF(std::sin(a) * 0.62, -std::cos(a) * 0.62);
    };
    for (size_t i = 0; i < ms.size(); ++i) {
        for (size_t j = i + 1; j < ms.size(); ++j) {
            const QPointF a = motorPos(ms[i]), b = motorPos(ms[j]);
            if (std::abs(a.x() - b.x()) < 0.05 && std::abs(a.x()) > 0.1) {
                p.drawLine(pt(a.x(), a.y()), pt(b.x(), b.y()));
            }
        }
    }

    // battery and CG
    if (v.highlight == Part::Battery) {
        p.setPen(QPen(kAmber, 2));
        p.setBrush(alpha(kAmber, 0.2));
        p.drawRect(QRectF(pt(-0.05, -0.38), pt(0.05, -0.12)));
    }
    if (v.highlight == Part::CG) {
        const QPointF cg = pt(0, 0.0);
        p.setPen(QPen(kAmber, 2));
        p.setBrush(Qt::NoBrush);
        p.drawEllipse(cg, 9, 9);
        p.setBrush(kAmber);
        p.drawPie(QRectF(cg.x() - 9, cg.y() - 9, 18, 18), 0, 90 * 16);
        p.drawPie(QRectF(cg.x() - 9, cg.y() - 9, 18, 18), 180 * 16, 90 * 16);
    }
    // pitot on the wing
    {
        const bool hl = v.highlight == Part::Pitot;
        p.setPen(QPen(hl ? kAmber : base, hl ? 2.6 : 1.4));
        p.drawLine(pt(-0.70, -0.08), pt(-0.70, -0.22));
        if (hl) {
            p.setPen(QPen(kAmber, 1.5, Qt::DashLine));
            p.setBrush(Qt::NoBrush);
            p.drawEllipse(pt(-0.70, -0.22), 10, 10);
        }
    }
    // antennas
    if (v.highlight == Part::Antennas) {
        p.setPen(QPen(kAmber, 2));
        p.drawEllipse(pt(0, -0.30), 5, 5);  // GPS
        p.drawLine(pt(0.04, 0.35), pt(0.20, 0.45));
        p.drawLine(pt(-0.04, 0.35), pt(-0.20, 0.45));
    }

    // lift props
    const double pr = s * 0.24;
    for (size_t i = 0; i < ms.size(); ++i) {
        const QPointF m = motorPos(ms[i]);
        const QPointF mc = pt(m.x(), m.y());
        const bool lit = static_cast<int>(i) == v.lit_motor;
        const bool hl = v.highlight == Part::Props || v.highlight == Part::Motors || lit;
        QColor col = lit ? kGreen : (hl ? kAmber : base);
        p.setPen(QPen(alpha(col, lit ? 1.0 : 0.8), lit ? 2.2 : 1.3));
        p.setBrush(alpha(col, lit ? 0.18 : 0.05));
        p.drawEllipse(mc, pr, pr);
        // blur disc when spinning
        if (lit) {
            const double a = std::fmod(time * (ms[i].ccw ? -1300.0 : 1300.0), 360.0);
            p.save();
            p.translate(mc);
            p.rotate(a);
            p.setPen(QPen(alpha(kGreen, 0.9), 3, Qt::SolidLine, Qt::RoundCap));
            p.drawLine(QPointF(-pr * 0.9, 0), QPointF(pr * 0.9, 0));
            p.restore();
        }
        p.setBrush(col);
        p.setPen(Qt::NoPen);
        p.drawEllipse(mc, 3.5, 3.5);
        if (v.show_dirs || lit) {
            spinArrow(p, mc, pr * 0.72, ms[i].ccw, lit ? kGreen : alpha(kAmber, 0.9), lit ? time * 0.6 : 0.15);
            drawText(p, QRectF(mc.x() - 30, mc.y() + pr + 2, 60, 14), Qt::AlignCenter,
                     ms[i].ccw ? QStringLiteral("CCW") : QStringLiteral("CW"), labelFont(9), alpha(col, 0.95));
        }
        if (v.show_letters) {
            const QRectF tag(mc.x() - 13, mc.y() - pr - 26, 26, 22);
            p.setPen(QPen(col, 1.4));
            p.setBrush(alpha(kBg, 0.85));
            p.drawRect(tag);
            drawText(p, tag, Qt::AlignCenter, QString(QChar(ms[i].letter())), monoFont(13, true), col);
        }
    }

    // pusher
    {
        const bool hl = v.highlight == Part::Pusher || v.highlight == Part::Props || v.pusher_lit;
        const QColor col = v.pusher_lit ? kGreen : (hl ? kAmber : base);
        const QPointF pc = pt(0, 0.64);
        p.setPen(QPen(col, v.pusher_lit ? 3 : 2, Qt::SolidLine, Qt::RoundCap));
        p.drawLine(pc + QPointF(-s * 0.16, 0), pc + QPointF(s * 0.16, 0));
        if (v.pusher_lit) {
            // air blown backwards
            for (int k = 0; k < 3; ++k) {
                const double ph = std::fmod(time * 1.4 + k / 3.0, 1.0);
                const double y = pc.y() + 8 + ph * s * 0.35;
                p.setPen(QPen(alpha(kGreen, 0.8 * (1 - ph)), 1.6));
                p.drawLine(QPointF(pc.x() - 12, y), QPointF(pc.x() - 12, y + 10));
                p.drawLine(QPointF(pc.x() + 12, y), QPointF(pc.x() + 12, y + 10));
            }
        }
    }
    // nose marker
    drawText(p, QRectF(c.x() - 40, r.top() - 2, 80, 14), Qt::AlignCenter, QStringLiteral("▲ NOSE"), labelFont(9),
             alpha(kMuted, 0.9));
    p.restore();
}

QString poseName(int pos) {
    switch (pos) {
    case 1: return QStringLiteral("LEVEL");
    case 2: return QStringLiteral("ON ITS LEFT SIDE");
    case 3: return QStringLiteral("ON ITS RIGHT SIDE");
    case 4: return QStringLiteral("NOSE DOWN");
    case 5: return QStringLiteral("NOSE UP");
    case 6: return QStringLiteral("ON ITS BACK");
    default: return {};
    }
}

void pose(QPainter& p, const QRectF& r, int pos, double time) {
    p.save();
    p.setRenderHint(QPainter::Antialiasing);
    const QPointF c = r.center();
    const double s = std::min(r.width(), r.height()) * 0.36;
    // ground
    p.setPen(QPen(alpha(kMuted, 0.6), 1.5));
    p.drawLine(QPointF(r.left() + 10, c.y() + s * 0.95), QPointF(r.right() - 10, c.y() + s * 0.95));
    // nose-down/up: side view; left/right/back/level: front view
    const bool side = pos == 4 || pos == 5;
    double rot = 0.0;
    switch (pos) {
    case 2: rot = -90; break;   // left side down (front view, rolled left)
    case 3: rot = 90; break;
    case 4: rot = 90; break;    // side view, nose down
    case 5: rot = -90; break;
    case 6: rot = 180; break;
    default: break;
    }
    const double settle = std::min(1.0, std::fmod(time, 3.0) / 0.8);
    p.translate(c);
    p.rotate(rot * (0.85 + 0.15 * settle));
    p.setPen(QPen(kCyan, 2));
    p.setBrush(alpha(kCyan, 0.1));
    if (side) {
        // side view: fuselage, wing stub, tail fin, nose to the left
        QPainterPath f;
        f.addRoundedRect(QRectF(-s, -s * 0.08, 2 * s, s * 0.16), s * 0.08, s * 0.08);
        p.drawPath(f);
        p.drawLine(QPointF(s * 0.85, -s * 0.08), QPointF(s * 0.95, -s * 0.38));
        p.drawRect(QRectF(-s * 0.2, -s * 0.12, s * 0.4, s * 0.06));
        drawText(p, QRectF(-s - 40, -10, 36, 20), Qt::AlignRight | Qt::AlignVCenter, QStringLiteral("NOSE"),
                 labelFont(9), kAmber);
    } else {
        // front view: wing span, fuselage circle, booms with props, fin
        p.drawLine(QPointF(-s, 0), QPointF(s, 0));
        p.drawEllipse(QPointF(0, 0), s * 0.1, s * 0.1);
        p.drawLine(QPointF(0, -s * 0.1), QPointF(0, -s * 0.38));
        for (double x : {-s * 0.55, s * 0.55}) {
            p.drawLine(QPointF(x, 0), QPointF(x, -s * 0.12));
            p.drawLine(QPointF(x - s * 0.18, -s * 0.12), QPointF(x + s * 0.18, -s * 0.12));
        }
        drawText(p, QRectF(-s, s * 0.12, s * 0.5, 16), Qt::AlignLeft, QStringLiteral("L"), labelFont(10), kAmber);
        drawText(p, QRectF(s * 0.5, s * 0.12, s * 0.5, 16), Qt::AlignRight, QStringLiteral("R"), labelFont(10),
                 kAmber);
    }
    p.restore();
    drawText(p, QRectF(r.left(), r.top(), r.width(), 22), Qt::AlignCenter, poseName(pos), labelFont(14), kAmber);
}

void surface(QPainter& p, const QRectF& r, Axis axis, double cmd, double out, double time) {
    (void)time;
    p.save();
    p.setRenderHint(QPainter::Antialiasing);
    const QPointF c(r.center().x(), r.top() + r.height() * 0.42);
    const double s = std::min(r.width() * 0.42, r.height() * 0.38);
    const double defl = std::clamp(cmd, -1.0, 1.0) * 25.0;  // degrees shown
    const QColor col = kCyan;
    auto label = [&](const QString& t, const QPointF& at, const QColor& k) {
        drawText(p, QRectF(at.x() - 70, at.y() - 9, 140, 18), Qt::AlignCenter, t, labelFont(11), k);
    };
    p.setPen(QPen(col, 2));
    p.setBrush(alpha(col, 0.1));
    if (axis == Axis::Roll) {
        // rear view: wing with an aileron at each tip; roll right = right UP, left DOWN
        p.drawLine(c + QPointF(-s, 0), c + QPointF(s, 0));
        p.drawEllipse(c, 10, 10);
        for (int side : {-1, 1}) {
            const QPointF hinge = c + QPointF(side * s * 0.62, 0);
            // seen from behind, the aircraft's right is on the right
            const double up = (side > 0 ? 1.0 : -1.0) * defl;  // right aileron up for roll right
            p.save();
            p.translate(hinge);
            p.rotate(side > 0 ? -up : up);
            p.setPen(QPen(std::abs(defl) > 1 ? kGreen : col, 4, Qt::SolidLine, Qt::RoundCap));
            p.drawLine(QPointF(0, 0), QPointF(side * s * 0.36, 0));
            p.restore();
            if (std::abs(defl) > 3) {
                const bool goes_up = up > 0;
                label(goes_up ? QStringLiteral("▲ UP") : QStringLiteral("▼ DOWN"),
                      hinge + QPointF(side * s * 0.2, goes_up ? -s * 0.42 : s * 0.42), kGreen);
            }
        }
        label(QStringLiteral("SEEN FROM BEHIND"), c + QPointF(0, s * 0.75), alpha(kMuted, 0.9));
    } else if (axis == Axis::Pitch) {
        // side view, nose left: tailplane with elevator; pitch up = elevator UP
        p.drawLine(c + QPointF(-s, 0), c + QPointF(s * 0.2, 0));
        const QPointF hinge = c + QPointF(s * 0.2, 0);
        p.save();
        p.translate(hinge);
        p.rotate(-defl);
        p.setPen(QPen(std::abs(defl) > 1 ? kGreen : col, 4, Qt::SolidLine, Qt::RoundCap));
        p.drawLine(QPointF(0, 0), QPointF(s * 0.55, 0));
        p.restore();
        if (std::abs(defl) > 3) {
            label(defl > 0 ? QStringLiteral("▲ ELEVATOR UP") : QStringLiteral("▼ ELEVATOR DOWN"),
                  hinge + QPointF(s * 0.3, defl > 0 ? -s * 0.45 : s * 0.45), kGreen);
        }
        drawText(p, QRectF(c.x() - s - 50, c.y() - 9, 46, 18), Qt::AlignRight | Qt::AlignVCenter,
                 QStringLiteral("NOSE ◀"), labelFont(9), alpha(kMuted, 0.9));
        label(QStringLiteral("SEEN FROM THE LEFT"), c + QPointF(0, s * 0.75), alpha(kMuted, 0.9));
    } else {
        // top view, nose up: fin with rudder; yaw right = rudder trailing edge to the RIGHT
        p.drawLine(c + QPointF(0, -s), c + QPointF(0, s * 0.2));
        const QPointF hinge = c + QPointF(0, s * 0.2);
        p.save();
        p.translate(hinge);
        p.rotate(-defl);
        p.setPen(QPen(std::abs(defl) > 1 ? kGreen : col, 4, Qt::SolidLine, Qt::RoundCap));
        p.drawLine(QPointF(0, 0), QPointF(0, s * 0.55));
        p.restore();
        if (std::abs(defl) > 3) {
            label(defl > 0 ? QStringLiteral("RUDDER RIGHT ▶") : QStringLiteral("◀ RUDDER LEFT"),
                  hinge + QPointF(defl > 0 ? s * 0.55 : -s * 0.55, s * 0.35), kGreen);
        }
        label(QStringLiteral("SEEN FROM ABOVE · ▲ NOSE"), c + QPointF(0, s * 0.95), alpha(kMuted, 0.9));
    }
    // command and output bars
    const QRectF bar(r.left() + 20, r.bottom() - 40, r.width() - 40, 10);
    const QRectF obar(r.left() + 20, r.bottom() - 20, r.width() - 40, 10);
    auto drawBar = [&](const QRectF& b, double v, const QColor& k, const QString& t) {
        p.setPen(QPen(alpha(kMuted, 0.5), 1));
        p.setBrush(alpha(kBg, 0.6));
        p.drawRect(b);
        p.drawLine(QPointF(b.center().x(), b.top() - 2), QPointF(b.center().x(), b.bottom() + 2));
        if (std::isfinite(v)) {
            const double w = b.width() / 2.0 * std::clamp(v, -1.0, 1.0);
            p.setPen(Qt::NoPen);
            p.setBrush(k);
            p.drawRect(QRectF(b.center().x() + std::min(0.0, w), b.top() + 1, std::abs(w), b.height() - 2));
        }
        drawText(p, QRectF(b.left() - 2, b.top() - 13, 200, 12), Qt::AlignLeft | Qt::AlignVCenter, t, labelFont(8.5),
                 alpha(kMuted, 0.9));
    };
    drawBar(bar, cmd, kCyan, QStringLiteral("STICK COMMAND"));
    drawBar(obar, out, kGreen, QStringLiteral("SERVO OUTPUT (live)"));
    p.restore();
}

void sticks(QPainter& p, const QRectF& r, double roll, double pitch, double thr, double yaw,
            const std::array<bool, 8>& reached) {
    p.save();
    p.setRenderHint(QPainter::Antialiasing);
    const double g = std::min(r.width() / 2.3, r.height() * 0.8);
    const QRectF left(r.center().x() - g - 12, r.center().y() - g / 2, g, g);
    const QRectF right(r.center().x() + 12, r.center().y() - g / 2, g, g);
    auto gimbal = [&](const QRectF& b, double x, double y, int ix_minus, int ix_plus, int iy_minus, int iy_plus,
                      const QString& lx, const QString& ly) {
        drawGlass(p, chamferPath(b, 10), kCyan, 0.5, alpha(kBg, 0.7));
        // end ticks
        auto tick = [&](bool ok, const QPointF& at) {
            p.setPen(Qt::NoPen);
            p.setBrush(ok ? kGreen : alpha(kMuted, 0.5));
            p.drawEllipse(at, 5, 5);
        };
        tick(reached[ix_minus], QPointF(b.left() + 10, b.center().y()));
        tick(reached[ix_plus], QPointF(b.right() - 10, b.center().y()));
        tick(reached[iy_minus], QPointF(b.center().x(), b.bottom() - 10));
        tick(reached[iy_plus], QPointF(b.center().x(), b.top() + 10));
        p.setPen(QPen(alpha(kCyan, 0.3), 1));
        p.drawLine(QPointF(b.left() + 18, b.center().y()), QPointF(b.right() - 18, b.center().y()));
        p.drawLine(QPointF(b.center().x(), b.top() + 18), QPointF(b.center().x(), b.bottom() - 18));
        const QPointF knob(b.center().x() + std::clamp(x, -1.0, 1.0) * (b.width() / 2 - 22),
                           b.center().y() - std::clamp(y, -1.0, 1.0) * (b.height() / 2 - 22));
        p.setPen(QPen(kCyan, 2));
        p.setBrush(alpha(kCyan, 0.35));
        p.drawEllipse(knob, 12, 12);
        drawText(p, QRectF(b.left(), b.bottom() + 4, b.width(), 14), Qt::AlignCenter, lx + QStringLiteral(" · ") + ly,
                 labelFont(9), alpha(kMuted, 0.95));
    };
    gimbal(left, yaw, thr * 2.0 - 1.0, 6, 7, 4, 5, QStringLiteral("YAW"), QStringLiteral("THROTTLE"));
    gimbal(right, roll, pitch, 0, 1, 2, 3, QStringLiteral("ROLL"), QStringLiteral("PITCH"));
    p.restore();
}

void compassSphere(QPainter& p, const QRectF& r, const std::array<uint8_t, 10>& mask, int pct, double time) {
    p.save();
    p.setRenderHint(QPainter::Antialiasing);
    const QPointF c = r.center();
    const double rad = std::min(r.width(), r.height()) * 0.42;
    QRadialGradient g(c - QPointF(rad * 0.3, rad * 0.3), rad * 1.4);
    g.setColorAt(0.0, alpha(kCyan, 0.18));
    g.setColorAt(1.0, alpha(kBg, 0.9));
    p.setPen(QPen(alpha(kCyan, 0.6), 1.5));
    p.setBrush(g);
    p.drawEllipse(c, rad, rad);
    // 80 sections: 8 latitude bands x 10 longitudes, drawn on the visible
    // hemisphere, slowly turning so the back half shows too
    const double spin = time * 0.4;
    for (int i = 0; i < 80; ++i) {
        const bool on = (mask[i / 8] >> (i % 8)) & 1;
        const int band = i / 10, lon = i % 10;
        const double lat = (-78.75 + band * 22.5) * geo::kDegToRad;
        const double lo = lon * 36.0 * geo::kDegToRad + spin;
        const double x = std::cos(lat) * std::sin(lo), y = -std::sin(lat), z = std::cos(lat) * std::cos(lo);
        if (z < -0.15) {
            continue;
        }
        const QPointF at = c + QPointF(x * rad, y * rad);
        const double sz = 4.0 + 4.0 * std::max(0.0, z);
        p.setPen(Qt::NoPen);
        p.setBrush(on ? alpha(kGreen, 0.5 + 0.5 * z) : alpha(kMuted, 0.25 + 0.2 * z));
        p.drawEllipse(at, sz, sz);
    }
    drawText(p, QRectF(c.x() - 50, c.y() - 14, 100, 28), Qt::AlignCenter, QStringLiteral("%1 %").arg(pct),
             monoFont(20, true), kText);
    p.restore();
}

void battery(QPainter& p, const QRectF& r, const std::vector<double>& cells, double pack_v, int ncells,
             double full_v, double storage_v) {
    p.save();
    p.setRenderHint(QPainter::Antialiasing);
    const int n = !cells.empty() ? static_cast<int>(cells.size()) : std::max(1, ncells);
    const double per = !cells.empty() || ncells <= 0 ? 0.0 : pack_v / ncells;
    const QRectF body(r.left() + 10, r.top() + 24, r.width() - 34, r.height() - 60);
    p.setPen(QPen(kCyan, 2));
    p.setBrush(alpha(kBg, 0.6));
    p.drawRoundedRect(body, 8, 8);
    p.drawRect(QRectF(body.right(), body.center().y() - 14, 10, 28));
    const double gap = 6, w = (body.width() - gap * (n + 1)) / n;
    for (int i = 0; i < n; ++i) {
        const double v = !cells.empty() ? cells[i] : per;
        const double frac = std::clamp((v - 3.3) / (4.2 - 3.3), 0.0, 1.0);
        const QColor k = v >= full_v ? kGreen : (v >= storage_v ? kAmber : kRed);
        const QRectF cell(body.left() + gap + i * (w + gap), body.top() + gap, w, body.height() - 2 * gap);
        p.setPen(QPen(alpha(k, 0.6), 1));
        p.setBrush(alpha(kBg, 0.4));
        p.drawRect(cell);
        p.setPen(Qt::NoPen);
        p.setBrush(alpha(k, 0.75));
        p.drawRect(QRectF(cell.left(), cell.bottom() - cell.height() * frac, cell.width(), cell.height() * frac));
        if (v > 0.0) {
            drawText(p, QRectF(cell.left() - 6, cell.top() + 4, cell.width() + 12, 16), Qt::AlignCenter,
                     QString::number(v, 'f', 2), monoFont(w > 40 ? 12 : 9.5, true), kText);
        }
    }
    drawText(p, QRectF(r.left(), r.bottom() - 30, r.width(), 22), Qt::AlignCenter,
             QStringLiteral("%1 V · full ≥ %2 V/cell").arg(pack_v, 0, 'f', 2).arg(full_v, 0, 'f', 2), monoFont(13, true),
             kText);
    p.restore();
}

void site(QPainter& p, const QRectF& r, double time) {
    p.save();
    p.setRenderHint(QPainter::Antialiasing);
    const QPointF c = r.center();
    const double rad = std::min(r.width(), r.height()) * 0.38;
    QPen ring(kAmber, 2, Qt::DashLine);
    ring.setDashOffset(-time * 6);
    p.setPen(ring);
    p.setBrush(alpha(kAmber, 0.05));
    p.drawEllipse(c, rad, rad);
    drawText(p, QRectF(c.x() + rad * 0.72 - 30, c.y() - rad * 0.72 - 20, 60, 16), Qt::AlignCenter,
             QStringLiteral("10 m"), labelFont(11), kAmber);
    // aircraft
    TopView v;
    v.frame = frameLayout(1, 1);
    quadplaneTop(p, QRectF(c.x() - rad * 0.4, c.y() - rad * 0.4, rad * 0.8, rad * 0.8), v, time);
    // people outside, behind the pilot line
    for (int i = 0; i < 3; ++i) {
        const double a = (200 + i * 25) * geo::kDegToRad;
        const QPointF at = c + QPointF(std::sin(a) * rad * 1.22, -std::cos(a) * rad * 1.22);
        p.setPen(QPen(kGreen, 2));
        p.setBrush(Qt::NoBrush);
        p.drawEllipse(at + QPointF(0, -8), 5, 5);
        p.drawLine(at + QPointF(0, -3), at + QPointF(0, 9));
    }
    p.restore();
}

void pitot(QPainter& p, const QRectF& r, double airspeed, double time) {
    p.save();
    p.setRenderHint(QPainter::Antialiasing);
    const QPointF c = r.center();
    const double s = std::min(r.width(), r.height()) * 0.4;
    p.setPen(QPen(kCyan, 3, Qt::SolidLine, Qt::RoundCap));
    p.drawLine(c + QPointF(-s, 0), c + QPointF(s * 0.6, 0));
    p.drawLine(c + QPointF(s * 0.6, 0), c + QPointF(s * 0.6, s * 0.5));
    // air flowing in (stronger with airspeed)
    const double strength = std::clamp(airspeed / 10.0, 0.15, 1.0);
    for (int k = 0; k < 4; ++k) {
        const double ph = std::fmod(time * (0.6 + strength) + k / 4.0, 1.0);
        const double x = c.x() - s * 1.9 + ph * s * 0.9;
        p.setPen(QPen(alpha(kGreen, strength * (1 - ph)), 2));
        p.drawLine(QPointF(x, c.y() - 6 + (k % 2) * 12), QPointF(x + 14, c.y() - 6 + (k % 2) * 12));
    }
    drawText(p, QRectF(c.x() - 80, c.y() + s * 0.6, 160, 28), Qt::AlignCenter,
             QStringLiteral("%1 m/s").arg(airspeed, 0, 'f', 1), monoFont(20, true), airspeed > 5 ? kGreen : kText);
    p.restore();
}

void gimbal(QPainter& p, const QRectF& r, double pitch, double yaw) {
    p.save();
    p.setRenderHint(QPainter::Antialiasing);
    const QPointF c = r.center();
    const double s = std::min(r.width(), r.height()) * 0.3;
    p.setPen(QPen(kCyan, 2));
    p.setBrush(alpha(kCyan, 0.1));
    p.drawEllipse(c, s * 0.5, s * 0.5);
    // look direction: yaw rotates in plane, pitch shortens it (top view)
    const double len = s * 1.5 * std::cos(pitch * geo::kDegToRad);
    const double a = yaw * geo::kDegToRad;
    const QPointF tip = c + QPointF(std::sin(a) * len, -std::cos(a) * len);
    p.setPen(QPen(kGreen, 3, Qt::SolidLine, Qt::RoundCap));
    p.drawLine(c, tip);
    arrowHead(p, tip, std::atan2(tip.y() - c.y(), tip.x() - c.x()), 12, kGreen);
    drawText(p, QRectF(c.x() - 100, r.bottom() - 26, 200, 20), Qt::AlignCenter,
             QStringLiteral("pitch %1° · yaw %2°").arg(pitch, 0, 'f', 0).arg(yaw, 0, 'f', 0), monoFont(12, true), kText);
    p.restore();
}

void statusGlyph(QPainter& p, const QPointF& c, double rad, Status s, double time) {
    p.save();
    p.setRenderHint(QPainter::Antialiasing);
    QColor col = kMuted;
    QString t;
    switch (s) {
    case Status::Pass: col = kGreen; t = QStringLiteral("✓"); break;
    case Status::Warn: col = kAmber; t = QStringLiteral("!"); break;
    case Status::Fail: col = kRed; t = QStringLiteral("✗"); break;
    case Status::Skipped: col = kAmber; t = QStringLiteral("»"); break;
    case Status::NotApplicable: col = kMuted; t = QStringLiteral("–"); break;
    case Status::Waiting: col = kCyan; break;
    case Status::Pending: col = alpha(kMuted, 0.6); break;
    }
    p.setPen(QPen(col, std::max(1.2, rad * 0.14)));
    p.setBrush(alpha(col, 0.12));
    p.drawEllipse(c, rad, rad);
    if (s == Status::Waiting) {
        const double a = std::fmod(time * 360.0, 360.0);
        p.setPen(QPen(kCyan, std::max(1.5, rad * 0.18), Qt::SolidLine, Qt::RoundCap));
        p.drawArc(QRectF(c.x() - rad * 0.6, c.y() - rad * 0.6, rad * 1.2, rad * 1.2), static_cast<int>(-a * 16), 100 * 16);
    } else if (!t.isEmpty()) {
        drawText(p, QRectF(c.x() - rad, c.y() - rad, 2 * rad, 2 * rad), Qt::AlignCenter, t,
                 monoFont(rad * 1.2, true), col);
    }
    p.restore();
}

void gauge(QPainter& p, const QRectF& r, double value, double lo, double hi, double good_lo, double good_hi,
           const QString& label) {
    p.save();
    p.setRenderHint(QPainter::Antialiasing);
    const QRectF bar(r.left(), r.top() + 16, r.width(), 8);
    auto xOf = [&](double v) { return bar.left() + bar.width() * std::clamp((v - lo) / (hi - lo), 0.0, 1.0); };
    p.setPen(Qt::NoPen);
    p.setBrush(alpha(kMuted, 0.25));
    p.drawRect(bar);
    p.setBrush(alpha(kGreen, 0.35));
    p.drawRect(QRectF(QPointF(xOf(good_lo), bar.top()), QPointF(xOf(good_hi), bar.bottom())));
    const bool good = value >= good_lo && value <= good_hi;
    p.setBrush(good ? kGreen : kAmber);
    p.drawPolygon(QPolygonF({QPointF(xOf(value), bar.top() - 1), QPointF(xOf(value) - 5, bar.top() - 8),
                             QPointF(xOf(value) + 5, bar.top() - 8)}));
    drawText(p, QRectF(r.left(), r.top(), r.width(), 14), Qt::AlignLeft | Qt::AlignVCenter, label, labelFont(9),
             alpha(kMuted, 0.95));
    p.restore();
}

}  // namespace gcs::ui::art
