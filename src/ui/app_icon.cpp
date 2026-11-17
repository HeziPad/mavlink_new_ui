#include "app_icon.hpp"

#include "theme.hpp"

#include <QPainter>
#include <QPainterPath>
#include <QRadialGradient>

#include <cmath>

namespace gcs::ui {

using namespace theme;

QImage appIconImage(int size) {
    QImage img(size, size, QImage::Format_ARGB32_Premultiplied);
    img.fill(Qt::transparent);
    QPainter p(&img);
    p.setRenderHint(QPainter::Antialiasing);
    p.scale(size / 512.0, size / 512.0);  // drawn on a 512 grid
    const bool tiny = size <= 24;

    // ---- tile -----------------------------------------------------------------
    const QRectF tile(16, 16, 480, 480);
    const QPainterPath body = chamferPath(tile, 84);
    QRadialGradient bg(QPointF(256, 230), 330);
    bg.setColorAt(0.0, QColor(0x10, 0x22, 0x30));
    bg.setColorAt(1.0, kBg);
    p.fillPath(body, bg);
    p.setPen(QPen(alpha(kCyan, 0.85), tiny ? 26 : 12));
    p.drawPath(body);

    if (!tiny) {
        // a faint HUD ring behind the aircraft
        p.setPen(QPen(alpha(kCyan, 0.18), 6));
        p.setBrush(Qt::NoBrush);
        p.drawEllipse(QPointF(256, 262), 176, 176);
        p.setPen(QPen(alpha(kCyan, 0.35), 8, Qt::SolidLine, Qt::FlatCap));
        for (int k = 0; k < 4; ++k) {
            const double a = k * M_PI / 2;
            p.drawLine(QPointF(256 + std::cos(a) * 176, 262 + std::sin(a) * 176),
                       QPointF(256 + std::cos(a) * 204, 262 + std::sin(a) * 204));
        }
    }

    // ---- aircraft (top view, nose up) -------------------------------------------
    const double ring = tiny ? 20 : 11;
    // booms first, under everything
    p.setPen(QPen(alpha(kCyan, 0.9), tiny ? 26 : 14, Qt::SolidLine, Qt::RoundCap));
    p.drawLine(QPointF(156, 140), QPointF(156, 332));
    p.drawLine(QPointF(356, 140), QPointF(356, 332));
    // four lift rotors at the boom ends, props drawn as one blade line
    const double rr = tiny ? 44 : 50;
    for (const QPointF c : {QPointF(156, 140), QPointF(356, 140), QPointF(156, 332), QPointF(356, 332)}) {
        p.setPen(QPen(kGreen, ring));
        p.setBrush(alpha(kGreen, 0.14));
        p.drawEllipse(c, rr, rr);
        if (!tiny) {
            p.setPen(QPen(alpha(kGreen, 0.9), 9, Qt::SolidLine, Qt::RoundCap));
            p.drawLine(c + QPointF(-rr * 0.62, rr * 0.2), c + QPointF(rr * 0.62, -rr * 0.2));
        }
    }
    // wing: a tapered planform, filled
    QPainterPath wing;
    wing.moveTo(60, 246);
    wing.lineTo(80, 214);
    wing.lineTo(432, 214);
    wing.lineTo(452, 246);
    wing.closeSubpath();
    // fuselage: pointed nose, slim tail
    QPainterPath fus;
    fus.moveTo(256, 92);
    fus.cubicTo(282, 120, 282, 170, 278, 220);
    fus.lineTo(266, 382);
    fus.lineTo(246, 382);
    fus.lineTo(234, 220);
    fus.cubicTo(230, 170, 230, 120, 256, 92);
    // tailplane
    QPainterPath tail;
    tail.moveTo(196, 392);
    tail.lineTo(210, 366);
    tail.lineTo(302, 366);
    tail.lineTo(316, 392);
    tail.closeSubpath();
    QPainterPath plane = wing.united(fus).united(tail);
    p.setPen(QPen(kText, tiny ? 10 : 6, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    p.setBrush(kCyan);
    p.drawPath(plane);
    // pusher prop behind the tail
    p.setPen(QPen(kAmber, tiny ? 28 : 16, Qt::SolidLine, Qt::RoundCap));
    p.drawLine(QPointF(214, 420), QPointF(298, 420));
    return img;
}

QIcon appIcon() {
    QIcon icon;
    for (int s : {16, 24, 32, 48, 64, 128, 256, 512}) {
        icon.addPixmap(QPixmap::fromImage(appIconImage(s)));
    }
    return icon;
}

}  // namespace gcs::ui
