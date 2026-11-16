#include "theme.hpp"

#include <QPainter>
#include <QPixmap>

#include <algorithm>
#include <cmath>

namespace gcs::ui::theme {

namespace {
bool g_reduced_motion = false;
}

bool reducedMotion() {
    return g_reduced_motion;
}

void setReducedMotion(bool on) {
    g_reduced_motion = on;
}

QColor alpha(QColor c, double a) {
    c.setAlphaF(static_cast<float>(std::clamp(a, 0.0, 1.0) * c.alphaF()));
    return c;
}

QColor mix(const QColor& a, const QColor& b, double t) {
    t = std::clamp(t, 0.0, 1.0);
    return QColor::fromRgbF(static_cast<float>(a.redF() + (b.redF() - a.redF()) * t),
                            static_cast<float>(a.greenF() + (b.greenF() - a.greenF()) * t),
                            static_cast<float>(a.blueF() + (b.blueF() - a.blueF()) * t),
                            static_cast<float>(a.alphaF() + (b.alphaF() - a.alphaF()) * t));
}

QFont monoFont(double px, bool bold) {
    QFont f;
    f.setFamilies({QStringLiteral("JetBrains Mono"), QStringLiteral("DejaVu Sans Mono"),
                   QStringLiteral("monospace")});
    f.setStyleHint(QFont::Monospace);
    f.setPixelSize(std::max(1, static_cast<int>(std::lround(px))));
    f.setWeight(bold ? QFont::DemiBold : QFont::Normal);
    return f;
}

QFont labelFont(double px, bool bold) {
    QFont f;
    f.setFamilies({QStringLiteral("Rajdhani"), QStringLiteral("Exo 2"), QStringLiteral("DejaVu Sans"),
                   QStringLiteral("sans-serif")});
    f.setPixelSize(std::max(1, static_cast<int>(std::lround(px))));
    f.setWeight(bold ? QFont::Bold : QFont::Normal);
    f.setCapitalization(QFont::AllUppercase);
    f.setLetterSpacing(QFont::AbsoluteSpacing, px * 0.12);
    return f;
}

QFont titleFont(double px) {
    QFont f = labelFont(px, true);
    f.setLetterSpacing(QFont::AbsoluteSpacing, px * 0.28);
    return f;
}

QPainterPath chamferPath(const QRectF& r, double cut) {
    cut = std::min({cut, r.width() / 2.0, r.height() / 2.0});
    QPainterPath path;
    path.moveTo(r.left() + cut, r.top());
    path.lineTo(r.right(), r.top());
    path.lineTo(r.right(), r.bottom() - cut);
    path.lineTo(r.right() - cut, r.bottom());
    path.lineTo(r.left(), r.bottom());
    path.lineTo(r.left(), r.top() + cut);
    path.closeSubpath();
    return path;
}

QPainterPath hexPath(const QPointF& c, double radius) {
    QPainterPath path;
    for (int i = 0; i < 6; ++i) {
        const double a = (60.0 * i - 90.0) * M_PI / 180.0;
        const QPointF pt(c.x() + radius * std::cos(a), c.y() + radius * std::sin(a));
        if (i == 0) {
            path.moveTo(pt);
        } else {
            path.lineTo(pt);
        }
    }
    path.closeSubpath();
    return path;
}

void drawGlowPath(QPainter& p, const QPainterPath& path, const QColor& c, double width,
                  double glow) {
    p.save();
    p.setBrush(Qt::NoBrush);
    p.setPen(QPen(alpha(c, 0.14), width + glow, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    p.drawPath(path);
    p.setPen(QPen(alpha(c, 0.32), width + glow * 0.4, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    p.drawPath(path);
    p.setPen(QPen(c, width, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    p.drawPath(path);
    p.restore();
}

void drawGlass(QPainter& p, const QPainterPath& path, const QColor& edge, double edge_alpha,
               const QColor& fill) {
    p.save();
    p.setPen(Qt::NoPen);
    p.setBrush(fill);
    p.drawPath(path);
    p.setBrush(Qt::NoBrush);
    p.setPen(QPen(alpha(edge, edge_alpha), 1.0));
    p.drawPath(path);
    p.restore();
}

void drawBrackets(QPainter& p, const QRectF& r, double len, const QColor& c, double width) {
    p.save();
    p.setPen(QPen(c, width, Qt::SolidLine, Qt::SquareCap, Qt::MiterJoin));
    p.setBrush(Qt::NoBrush);
    const double l = std::min({len, r.width() / 2.0, r.height() / 2.0});
    auto corner = [&](QPointF a, QPointF b, QPointF c2) {
        QPainterPath path;
        path.moveTo(a);
        path.lineTo(b);
        path.lineTo(c2);
        p.drawPath(path);
    };
    corner({r.left(), r.top() + l}, r.topLeft(), {r.left() + l, r.top()});
    corner({r.right() - l, r.top()}, r.topRight(), {r.right(), r.top() + l});
    corner({r.right(), r.bottom() - l}, r.bottomRight(), {r.right() - l, r.bottom()});
    corner({r.left() + l, r.bottom()}, r.bottomLeft(), {r.left(), r.bottom() - l});
    p.restore();
}

void drawScanlines(QPainter& p, const QRectF& r, double strength) {
    static const QPixmap pattern = [] {
        QPixmap pm(4, 3);
        pm.fill(Qt::transparent);
        QPainter pp(&pm);
        pp.fillRect(QRect(0, 0, 4, 1), QColor(0, 0, 0, 70));
        pp.fillRect(QRect(0, 1, 4, 1), QColor(63, 224, 255, 6));
        return pm;
    }();
    p.save();
    p.setOpacity(p.opacity() * std::clamp(strength, 0.0, 1.0));
    p.fillRect(r, QBrush(pattern));
    p.restore();
}

void drawText(QPainter& p, const QRectF& r, int flags, const QString& text, const QFont& f,
              const QColor& c) {
    p.setFont(f);
    p.setPen(c);
    p.drawText(r, flags, text);
}

}  // namespace gcs::ui::theme
