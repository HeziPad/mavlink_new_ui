#include "settings_screen.hpp"

#include "anim.hpp"
#include "theme.hpp"

#include <QFontMetricsF>
#include <QMouseEvent>
#include <QPainter>

#include <algorithm>
#include <cmath>

namespace gcs::ui {

using namespace theme;

SettingsScreen::SettingsScreen(QWidget* parent) : QWidget(parent) {
    hide();
}

void SettingsScreen::open(int alt_unit) {
    alt_unit_ = alt_unit;
    alt_knob_ = alt_unit;
    // freeze a blurred copy of what's behind: cheap down/up-scale blur
    hide();
    if (parentWidget()) {
        const QPixmap shot = parentWidget()->grab();
        const QSize small = (QSizeF(shot.size()) / 10.0).toSize().expandedTo(QSize(1, 1));
        backdrop_ = shot.scaled(small, Qt::IgnoreAspectRatio, Qt::SmoothTransformation)
                        .scaled(shot.size(), Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
        backdrop_.setDevicePixelRatio(shot.devicePixelRatio());
        setGeometry(parentWidget()->rect());
    }
    open_target_ = 1.0;
    if (reducedMotion()) {
        open_ = 1.0;
    }
    show();
    raise();
    setFocus();
}

void SettingsScreen::close() {
    if (open_target_ == 0.0) {
        return;
    }
    open_target_ = 0.0;
    if (reducedMotion()) {
        open_ = 0.0;
        hide();
    }
    emit closed();
}

void SettingsScreen::tick(double dt) {
    if (!isVisible()) {
        return;
    }
    time_ += dt;
    open_ += std::clamp(open_target_ - open_, -dt * 4.5, dt * 4.5);
    if (open_ <= 0.0 && open_target_ == 0.0) {
        hide();
        backdrop_ = QPixmap();
        return;
    }
    const double k = reducedMotion() ? 1.0 : 1.0 - std::exp(-16.0 * dt);
    alt_knob_ += (alt_unit_ - alt_knob_) * k;
    flash_ = std::max(0.0, flash_ - dt * 2.0);
    update();
}

QRectF SettingsScreen::panelRect() const {
    const double w = std::min(820.0, width() - 48.0);
    const double h = std::min(470.0, height() - 120.0);
    return {(width() - w) / 2.0, (height() - h) / 2.0, w, h};
}

void SettingsScreen::mousePressEvent(QMouseEvent* e) {
    const QPointF pos = e->position();
    if (open_target_ < 0.5) {
        return;
    }
    if (close_rect_.contains(pos) || !panelRect().contains(pos)) {
        close();
        return;
    }
    for (int i = 0; i < static_cast<int>(alt_seg_.size()); ++i) {
        if (alt_seg_[i].contains(pos) && i != alt_unit_) {
            alt_unit_ = i;
            flash_ = 1.0;
            emit altUnitChanged(i);
            update();
            return;
        }
    }
}

void SettingsScreen::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    const double e = easeOutCubic(open_);
    const QRectF r = rect();

    // ---- backdrop ---------------------------------------------------------------
    if (!backdrop_.isNull()) {
        p.setOpacity(e);
        p.drawPixmap(r.toRect(), backdrop_);
        p.setOpacity(1.0);
    }
    p.fillRect(r, alpha(kBg, 0.62 * e));
    if (!reducedMotion()) {
        drawScanlines(p, r, 0.5 * e);
    }

    // ---- panel: slides up and un-skews as it opens ------------------------------
    const QRectF pr = panelRect();
    p.save();
    p.setOpacity(e);
    p.translate(0, (1.0 - e) * 40.0);
    const QPainterPath shape = chamferPath(pr, 22);
    drawGlass(p, shape, kCyan, 0.75, alpha(kGlassDeep, 0.96));
    drawBrackets(p, pr.adjusted(-6, -6, 6, 6), 26, alpha(kCyan, 0.8), 2.0);
    p.setPen(QPen(kCyan, 2));
    p.drawLine(QPointF(pr.left() + 1, pr.top() + 22), QPointF(pr.left() + 22, pr.top() + 1));

    // title
    drawText(p, QRectF(pr.left() + 34, pr.top() + 18, 400, 34), Qt::AlignLeft | Qt::AlignVCenter,
             QStringLiteral("SETTINGS"), titleFont(26), kText);
    drawText(p, QRectF(pr.left() + 36, pr.top() + 50, 400, 16), Qt::AlignLeft | Qt::AlignVCenter,
             QStringLiteral("changes apply immediately"), labelFont(10), kMuted);

    // close
    close_rect_ = QRectF(pr.right() - 72, pr.top() + 16, 52, 52);
    drawGlass(p, chamferPath(close_rect_, 8), kCyan, 0.6, kGlassDeep);
    drawText(p, close_rect_, Qt::AlignCenter, QStringLiteral("✕"), monoFont(20, true), kText);

    // ---- categories ----------------------------------------------------------------
    const double top = pr.top() + 92;
    const QRectF cat_col(pr.left() + 24, top, 190, pr.bottom() - top - 24);
    p.setPen(QPen(alpha(kCyan, 0.18), 1));
    p.drawLine(QPointF(cat_col.right() + 16, cat_col.top()), QPointF(cat_col.right() + 16, cat_col.bottom()));
    {
        const QRectF item(cat_col.left(), cat_col.top(), cat_col.width(), 52);
        drawGlass(p, chamferPath(item, 8), kCyan, 0.9, alpha(kCyan, 0.12));
        p.setPen(Qt::NoPen);
        p.setBrush(kCyan);
        p.drawRect(QRectF(item.left(), item.top() + 10, 3, item.height() - 20));
        drawText(p, item.adjusted(18, 0, 0, 0), Qt::AlignLeft | Qt::AlignVCenter, QStringLiteral("UNITS"),
                 labelFont(14), kCyan);
    }

    // ---- rows ----------------------------------------------------------------------
    const QRectF body(cat_col.right() + 40, top, pr.right() - cat_col.right() - 64, cat_col.height());
    {
        const QRectF row(body.left(), body.top(), body.width(), 120);
        drawText(p, QRectF(row.left(), row.top(), row.width(), 24), Qt::AlignLeft | Qt::AlignVCenter,
                 QStringLiteral("ALTITUDE"), labelFont(14), kText);
        drawText(p, QRectF(row.left(), row.top() + 24, row.width(), 18), Qt::AlignLeft | Qt::AlignVCenter,
                 QStringLiteral("HUD tape, telemetry bar, map labels, panels and steppers"), labelFont(10, false),
                 kMuted);

        // segmented toggle with a sliding highlight
        const double sw = std::min(row.width(), 360.0);
        const QRectF track(row.left(), row.top() + 54, sw, 58);
        drawGlass(p, chamferPath(track, 10), kCyan, 0.5, alpha(kBg, 0.7));
        const double segw = sw / 2.0;
        alt_seg_ = {QRectF(track.left(), track.top(), segw, track.height()),
                    QRectF(track.left() + segw, track.top(), segw, track.height())};
        const QRectF knob(track.left() + 4 + alt_knob_ * segw, track.top() + 4, segw - 8, track.height() - 8);
        const QColor kc = mix(kCyan, kText, 0.15 * flash_);
        drawGlass(p, chamferPath(knob, 8), kc, 1.0, alpha(kCyan, 0.22 + 0.25 * flash_));
        if (flash_ > 0.0) {
            p.setPen(QPen(alpha(kCyan, 0.5 * flash_), 6));
            p.setBrush(Qt::NoBrush);
            p.drawPath(chamferPath(knob, 8));
        }
        const char* names[] = {"METERS", "FEET"};
        const char* units[] = {"m", "ft"};
        for (int i = 0; i < 2; ++i) {
            const double sel = 1.0 - std::min(1.0, std::abs(alt_knob_ - i));
            const QColor c = mix(kMuted, kCyan, sel);
            drawText(p, alt_seg_[i].adjusted(0, -8, 0, -8), Qt::AlignCenter, QString::fromLatin1(names[i]),
                     labelFont(15), c);
            drawText(p, alt_seg_[i].adjusted(0, 18, 0, 0), Qt::AlignHCenter | Qt::AlignTop,
                     QString::fromLatin1(units[i]), monoFont(11), alpha(c, 0.8));
        }
        // live example
        const QString example = alt_unit_ == 1 ? QStringLiteral("e.g. ALT 394 ft") : QStringLiteral("e.g. ALT 120 m");
        drawText(p, QRectF(track.right() + 18, track.top(), std::max(0.0, row.right() - track.right() - 18), track.height()),
                 Qt::AlignLeft | Qt::AlignVCenter, example, monoFont(12), alpha(kText, 0.7));
    }
    drawText(p, QRectF(body.left(), body.bottom() - 18, body.width(), 18), Qt::AlignLeft | Qt::AlignVCenter,
             QStringLiteral("distances and speeds stay metric"), labelFont(10, false), alpha(kMuted, 0.8));
    p.restore();
}

}  // namespace gcs::ui
