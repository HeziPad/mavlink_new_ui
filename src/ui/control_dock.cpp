#include "control_dock.hpp"

#include "anim.hpp"

#include <QFontMetricsF>
#include <QHelpEvent>
#include <QMouseEvent>
#include <QPointingDevice>
#include <QPainter>
#include <QToolTip>

#include <cmath>

namespace gcs::ui {

using namespace theme;

namespace {
constexpr double kSlotW = 92;
constexpr double kSlotH = 76;
constexpr double kGap = 10;
constexpr double kPad = 14;
constexpr double kLongPressShowAfter = 0.22;
}  // namespace

ControlDock::ControlDock(QWidget* parent) : QWidget(parent) {
    setMouseTracking(true);
    setFocusPolicy(Qt::NoFocus);
}

QSize ControlDock::sizeHint() const {
    return QSize(static_cast<int>(kSlots * kSlotW + (kSlots - 1) * kGap + 2 * kPad),
                 static_cast<int>(kSlotH + 2 * kPad + 6));
}

QRectF ControlDock::slotRect(int i) const {
    const QSize sh = sizeHint();
    const double x0 = (width() - sh.width()) / 2.0 + kPad;
    return {x0 + i * (kSlotW + kGap), kPad + 4, kSlotW, kSlotH};
}

int ControlDock::slotAt(const QPointF& pos) const {
    for (int i = 0; i < kSlots; ++i) {
        if (slotRect(i).contains(pos)) {
            return i;
        }
    }
    return -1;
}

void ControlDock::setSlot(int i, const DockSlotSpec& spec) {
    auto& s = slots_[i];
    const bool changed = s.spec.label != spec.label || s.spec.enabled != spec.enabled ||
                         s.spec.active != spec.active || s.spec.icon != spec.icon ||
                         s.spec.accent != spec.accent || s.spec.reason != spec.reason;
    // never change a slot's semantics under a finger that is mid-hold
    if (s.down && (spec.hold_to_confirm != s.spec.hold_to_confirm || spec.icon != s.spec.icon)) {
        s.down = false;
        s.charge = 0.0;
    }
    s.spec = spec;
    if (changed) {
        update();
    }
}

void ControlDock::setPending(int i, bool pending) {
    slots_[i].pending = pending;
    update();
}

void ControlDock::flash(int i, bool ok) {
    auto& s = slots_[i];
    s.flash = 1.0;
    s.flash_ok = ok;
    s.pending = false;
    if (!ok && !reducedMotion()) {
        s.shake = 1.0;
    }
    update();
}

void ControlDock::setProgress(int i, double fraction) {
    slots_[i].progress = fraction;
    update();
}

void ControlDock::pressSlot(int i) {
    auto& s = slots_[i];
    if (s.down) {
        return;
    }
    if (!s.spec.enabled) {
        if (!reducedMotion()) {
            s.shake = 0.6;
        }
        emit blocked(i, s.spec.reason);
        update();
        return;
    }
    s.down = true;
    s.consumed = false;
    s.hold_t = 0.0;
    s.pressed_at = std::chrono::steady_clock::now();
    s.press = 1.0;
    if (!s.spec.hold_to_confirm && !s.spec.long_press) {
        s.consumed = true;  // instant actions fire on press
        emit triggered(i);
    }
    update();
}

void ControlDock::releaseSlot(int i) {
    auto& s = slots_[i];
    if (!s.down) {
        return;
    }
    s.down = false;
    if (s.consumed) {
        return;
    }
    const double held = std::chrono::duration<double>(std::chrono::steady_clock::now() - s.pressed_at).count();
    if (s.spec.hold_to_confirm) {
        if (held >= kHoldSeconds) {  // released between ticks after a full hold
            s.consumed = true;
            s.flash = 0.6;
            s.flash_ok = true;
            emit triggered(i);
        } else {
            emit holdCancelled(i);
        }
        return;
    }
    if (s.spec.long_press && held >= kLongPressSeconds) {
        emit longPressed(i);
        return;
    }
    emit triggered(i);  // long-press slot released early = tap
}

void ControlDock::tick(double dt) {
    time_ += dt;
    bool animating = false;
    for (int i = 0; i < kSlots; ++i) {
        auto& s = slots_[i];
        const double hover_target = s.hovered && s.spec.enabled ? 1.0 : 0.0;
        if (s.hover != hover_target) {
            s.hover += (hover_target - s.hover) * std::min(1.0, dt * 14.0);
            if (std::abs(s.hover - hover_target) < 0.01) {
                s.hover = hover_target;
            }
            animating = true;
        }
        if (!s.down && s.press > 0.0) {
            s.press = std::max(0.0, s.press - dt * 6.0);
            animating = true;
        }
        if (s.down && !s.consumed) {
            // confirmation timing must not depend on frame rate
            s.hold_t = std::chrono::duration<double>(std::chrono::steady_clock::now() - s.pressed_at).count();
            animating = true;
            if (s.spec.hold_to_confirm) {
                s.charge = std::min(1.0, s.hold_t / kHoldSeconds);
                if (s.hold_t >= kHoldSeconds) {
                    s.consumed = true;
                    s.flash = 0.6;
                    s.flash_ok = true;
                    emit triggered(i);
                }
            } else if (s.spec.long_press) {
                s.charge = std::clamp((s.hold_t - kLongPressShowAfter) / (kLongPressSeconds - kLongPressShowAfter),
                                      0.0, 1.0);
                if (s.hold_t >= kLongPressSeconds) {
                    s.consumed = true;
                    emit longPressed(i);
                }
            }
        } else if (s.charge > 0.0) {
            s.charge = std::max(0.0, s.charge - dt * 3.5);
            animating = true;
        }
        if (s.flash > 0.0) {
            s.flash = std::max(0.0, s.flash - dt * 1.4);
            animating = true;
        }
        if (s.shake > 0.0) {
            s.shake = std::max(0.0, s.shake - dt * 2.5);
            animating = true;
        }
        animating |= s.pending || (s.spec.active && !reducedMotion());
    }
    if (animating) {
        update();
    }
}

void ControlDock::mousePressEvent(QMouseEvent* e) {
    if (e->button() != Qt::LeftButton) {
        return;
    }
    mouse_slot_ = slotAt(e->position());
    if (mouse_slot_ >= 0) {
        pressSlot(mouse_slot_);
    }
}

void ControlDock::mouseReleaseEvent(QMouseEvent* e) {
    if (e->button() != Qt::LeftButton || mouse_slot_ < 0) {
        return;
    }
    const int i = mouse_slot_;
    mouse_slot_ = -1;
    if (e->pointingDevice() && e->pointingDevice()->type() == QInputDevice::DeviceType::TouchScreen) {
        // a finger never "leaves", so don't let the hover glow stick after a tap
        for (auto& sl : slots_) {
            sl.hovered = false;
        }
    }
    auto& s = slots_[i];
    if (s.down && !s.consumed && !slotRect(i).adjusted(-12, -12, 12, 12).contains(e->position())) {
        // dragged off the slot: cancel rather than fire
        s.down = false;
        if (s.spec.hold_to_confirm) {
            emit holdCancelled(i);
        }
        return;
    }
    releaseSlot(i);
}

void ControlDock::mouseMoveEvent(QMouseEvent* e) {
    const int over = slotAt(e->position());
    for (int i = 0; i < kSlots; ++i) {
        slots_[i].hovered = i == over;
    }
}

void ControlDock::leaveEvent(QEvent* e) {
    for (auto& s : slots_) {
        s.hovered = false;
    }
    QWidget::leaveEvent(e);
}

bool ControlDock::event(QEvent* e) {
    if (e->type() == QEvent::ToolTip) {
        auto* he = static_cast<QHelpEvent*>(e);
        const int i = slotAt(he->pos());
        if (i >= 0) {
            const auto& spec = slots_[i].spec;
            QString tip = spec.enabled ? spec.hint : spec.reason;
            if (tip.isEmpty()) {
                tip = spec.label;
            }
            QToolTip::showText(he->globalPos(), QStringLiteral("[%1] %2").arg(i + 1).arg(tip), this);
        } else {
            QToolTip::hideText();
        }
        return true;
    }
    return QWidget::event(e);
}

void ControlDock::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);

    const QSize sh = sizeHint();
    const QRectF bar((width() - sh.width()) / 2.0, 4, sh.width(), sh.height() - 6);
    drawGlass(p, chamferPath(bar, 18), kCyan, 0.35, kGlassDeep);
    p.setPen(QPen(alpha(kCyan, 0.8), 2));
    p.drawLine(QPointF(bar.left() + 2, bar.top() + 18), QPointF(bar.left() + 18, bar.top() + 2));
    p.drawLine(QPointF(bar.right() - 18, bar.bottom() - 1), QPointF(bar.right() - 1, bar.bottom() - 18));

    for (int i = 0; i < kSlots; ++i) {
        paintSlot(p, i);
    }
}

void ControlDock::paintSlot(QPainter& p, int i) {
    const auto& s = slots_[i];
    const bool en = s.spec.enabled;
    QColor accent = en ? s.spec.accent : kMuted;

    double dx = 0.0;
    if (s.shake > 0.0) {
        dx = std::sin(time_ * 60.0) * 5.0 * s.shake;
    }
    const QRectF r = slotRect(i).translated(dx, -2.0 * s.hover + 1.5 * s.press);
    const QPainterPath path = chamferPath(r, 11);

    p.save();
    // body
    QColor fill = mix(kGlassDeep, QColor(18, 34, 46, 230), s.hover);
    if (s.spec.active) {
        fill = mix(fill, alpha(accent, 0.9), 0.14);
    }
    if (s.press > 0) {
        fill = mix(fill, QColor(0, 0, 0, 240), 0.3 * s.press);
    }
    p.setPen(Qt::NoPen);
    p.setBrush(fill);
    p.drawPath(path);

    // hold charge: radial sweep clipped to the slot ("ability charge")
    if (s.charge > 0.0) {
        p.save();
        p.setClipPath(path);
        const QPointF c = r.center();
        const double rad = std::hypot(r.width(), r.height());
        QPainterPath pie;
        pie.moveTo(c);
        pie.arcTo(QRectF(c.x() - rad, c.y() - rad, 2 * rad, 2 * rad), 90.0, -360.0 * s.charge);
        pie.closeSubpath();
        p.setBrush(alpha(accent, 0.30));
        p.drawPath(pie);
        p.restore();
    }

    // flash result
    if (s.flash > 0.0) {
        const QColor fc = s.flash_ok ? kGreen : kRed;
        p.setBrush(alpha(fc, 0.25 * s.flash));
        p.drawPath(path);
    }

    // border
    if (s.spec.active || s.charge > 0.0 || s.flash > 0.0) {
        QColor bc = s.flash > 0.0 ? (s.flash_ok ? kGreen : kRed) : accent;
        if (s.spec.active && s.flash <= 0.0 && !reducedMotion()) {
            bc = alpha(bc, 0.7 + 0.3 * std::sin(time_ * 5.0));
        }
        drawGlowPath(p, path, bc, 1.4, 6.0);
    } else {
        p.setBrush(Qt::NoBrush);
        p.setPen(QPen(alpha(en ? accent : kMuted, 0.25 + 0.4 * s.hover), 1.0));
        p.drawPath(path);
    }

    // icon
    const QRectF icon_r(r.center().x() - 15, r.top() + 11, 30, 30);
    const QPointF ic = icon_r.center();
    QColor icon_c = en ? (s.spec.active ? accent : mix(kText, accent, 0.35 + 0.5 * s.hover)) : alpha(kMuted, 0.55);
    drawDockIcon(p, s.spec.icon, icon_r, icon_c);

    // progress ring (segmented)
    if (s.progress >= 0.0) {
        const int segs = 16;
        const int lit = static_cast<int>(std::floor(std::clamp(s.progress, 0.0, 1.0) * segs + 1e-6));
        const double rad = 21.0;
        for (int k = 0; k < segs; ++k) {
            const double a0 = 90.0 - k * (360.0 / segs);
            QPainterPath arc;
            const QRectF ar(ic.x() - rad, ic.y() - rad, 2 * rad, 2 * rad);
            arc.arcMoveTo(ar, a0 - 2);
            arc.arcTo(ar, a0 - 2, -(360.0 / segs) + 6);
            p.setBrush(Qt::NoBrush);
            p.setPen(QPen(k < lit ? accent : alpha(kMuted, 0.3), 3, Qt::SolidLine, Qt::FlatCap));
            p.drawPath(arc);
        }
    }

    // pending ACK: rotating cooldown arc
    if (s.pending) {
        const double rad = 21.0;
        const QRectF ar(ic.x() - rad, ic.y() - rad, 2 * rad, 2 * rad);
        const double start = reducedMotion() ? 90.0 : 90.0 - std::fmod(time_ * 360.0, 360.0);
        p.setBrush(Qt::NoBrush);
        p.setPen(QPen(alpha(accent, 0.25), 3));
        p.drawEllipse(ar);
        p.setPen(QPen(accent, 3, Qt::SolidLine, Qt::RoundCap));
        p.drawArc(ar, static_cast<int>(start * 16), static_cast<int>(-100 * 16));
    }

    // result glyph
    if (s.flash > 0.3 && !s.pending) {
        const double a = std::min(1.0, (s.flash - 0.3) / 0.3);
        QPen pen(alpha(s.flash_ok ? kGreen : kRed, a), 3.0, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
        p.setPen(pen);
        const QPointF o(r.right() - 16, r.top() + 16);
        if (s.flash_ok) {
            p.drawPolyline(QPolygonF({o + QPointF(-6, 0), o + QPointF(-2, 4), o + QPointF(6, -5)}));
        } else {
            p.drawLine(o + QPointF(-5, -5), o + QPointF(5, 5));
            p.drawLine(o + QPointF(5, -5), o + QPointF(-5, 5));
        }
    }

    // label
    // label: shrink to fit rather than clip ("TAKEOFF 40M")
    QFont lf = labelFont(11.5);
    for (double px = 11.5; px > 8.0; px -= 0.5) {
        lf = labelFont(px);
        if (QFontMetricsF(lf).horizontalAdvance(s.spec.label.toUpper()) <= r.width() - 10) {
            break;
        }
    }
    drawText(p, QRectF(r.left() + 4, r.bottom() - 26, r.width() - 8, 20), Qt::AlignCenter, s.spec.label, lf,
             en ? (s.spec.active ? accent : kText) : alpha(kMuted, 0.8));

    // key badge
    const QRectF kb(r.left() + 6, r.top() + 6, 15, 15);
    p.setPen(QPen(alpha(en ? accent : kMuted, 0.6), 1));
    p.setBrush(alpha(kBg, 0.7));
    p.drawRect(kb);
    drawText(p, kb, Qt::AlignCenter, QString::number(i + 1), monoFont(10, true), alpha(en ? kText : kMuted, 0.9));

    // lock glyph when disabled
    if (!en) {
        const QPointF lo(r.right() - 15, r.top() + 13);
        p.setPen(QPen(alpha(kMuted, 0.9), 1.4));
        p.setBrush(Qt::NoBrush);
        p.drawArc(QRectF(lo.x() - 4, lo.y() - 6, 8, 9), 0, 180 * 16);
        p.setBrush(alpha(kMuted, 0.9));
        p.setPen(Qt::NoPen);
        p.drawRect(QRectF(lo.x() - 5.5, lo.y() - 1, 11, 8));
    }
    p.restore();
}

// Vector icons: no asset files, everything is a path in a 30x30 box.
void drawDockIcon(QPainter& p, DockIcon icon, const QRectF& r, const QColor& c) {
    p.save();
    p.translate(r.topLeft());
    p.scale(r.width() / 30.0, r.height() / 30.0);
    QPen pen(c, 2.0, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
    p.setPen(pen);
    p.setBrush(Qt::NoBrush);

    switch (icon) {
    case DockIcon::Plan: {
        p.drawPolyline(QPolygonF({{4, 24}, {12, 10}, {20, 18}, {27, 6}}));
        p.setBrush(theme::kBg);
        for (QPointF pt : {QPointF(4, 24), QPointF(12, 10), QPointF(20, 18), QPointF(27, 6)}) {
            p.drawEllipse(pt, 2.8, 2.8);
        }
        break;
    }
    case DockIcon::Upload: {
        p.drawLine(QPointF(15, 21), QPointF(15, 4));
        p.drawPolyline(QPolygonF({{8, 11}, {15, 4}, {22, 11}}));
        p.drawPolyline(QPolygonF({{4, 19}, {4, 26}, {26, 26}, {26, 19}}));
        break;
    }
    case DockIcon::Launch: {
        // aircraft climbing
        QPainterPath plane;
        plane.moveTo(15, 3);
        plane.lineTo(18, 12);
        plane.lineTo(27, 17);
        plane.lineTo(18, 17);
        plane.lineTo(17, 23);
        plane.lineTo(20, 26);
        plane.lineTo(10, 26);
        plane.lineTo(13, 23);
        plane.lineTo(12, 17);
        plane.lineTo(3, 17);
        plane.lineTo(12, 12);
        plane.closeSubpath();
        p.setBrush(alpha(c, 0.25));
        p.drawPath(plane);
        break;
    }
    case DockIcon::Takeoff: {
        for (QPointF pt : {QPointF(7, 21), QPointF(23, 21)}) {
            p.drawEllipse(pt, 4.5, 2.2);
        }
        p.drawLine(QPointF(7, 21), QPointF(23, 21));
        p.drawLine(QPointF(15, 17), QPointF(15, 3));
        p.drawPolyline(QPolygonF({{9, 9}, {15, 3}, {21, 9}}));
        p.drawLine(QPointF(4, 27), QPointF(26, 27));
        break;
    }
    case DockIcon::Auto: {
        p.drawPolyline(QPolygonF({{3, 25}, {10, 12}, {17, 20}}));
        QPainterPath tri;
        tri.moveTo(18, 6);
        tri.lineTo(28, 12);
        tri.lineTo(18, 18);
        tri.closeSubpath();
        p.setBrush(alpha(c, 0.5));
        p.drawPath(tri);
        break;
    }
    case DockIcon::Guide: {
        p.drawEllipse(QPointF(15, 15), 7, 7);
        p.drawLine(QPointF(15, 2), QPointF(15, 8));
        p.drawLine(QPointF(15, 22), QPointF(15, 28));
        p.drawLine(QPointF(2, 15), QPointF(8, 15));
        p.drawLine(QPointF(22, 15), QPointF(28, 15));
        p.setBrush(c);
        p.drawEllipse(QPointF(15, 15), 1.6, 1.6);
        break;
    }
    case DockIcon::Preflight: {
        // clipboard with ticks
        p.drawRoundedRect(QRectF(6, 5, 18, 22), 2, 2);
        p.drawRect(QRectF(11, 3, 8, 4));
        p.drawPolyline(QPolygonF({QPointF(9, 13), QPointF(11, 15), QPointF(14, 11)}));
        p.drawLine(QPointF(16, 13), QPointF(21, 13));
        p.drawPolyline(QPolygonF({QPointF(9, 20), QPointF(11, 22), QPointF(14, 18)}));
        p.drawLine(QPointF(16, 20), QPointF(21, 20));
        break;
    }
    case DockIcon::Hold: {
        p.drawEllipse(QPointF(15, 15), 12, 12);
        p.setBrush(c);
        p.drawRect(QRectF(10.5, 9, 3, 12));
        p.drawRect(QRectF(16.5, 9, 3, 12));
        break;
    }
    case DockIcon::Land: {
        // circling down on the wing, then straight down onto the pad
        QPainterPath spiral;
        spiral.moveTo(15, 4);
        spiral.cubicTo(27, 4, 27, 13, 15, 13);
        spiral.cubicTo(7, 13, 7, 7, 13, 7);
        p.drawPath(spiral);
        p.drawLine(QPointF(15, 13), QPointF(15, 23));
        p.drawPolyline(QPolygonF({{11, 19}, {15, 23}, {19, 19}}));
        p.drawLine(QPointF(5, 27), QPointF(25, 27));
        break;
    }
    }
    p.restore();
}

}  // namespace gcs::ui
