#include "fx_overlay.hpp"

#include "anim.hpp"
#include "theme.hpp"

#include <QFontMetricsF>
#include <QPainter>
#include <QRadialGradient>
#include <QTime>

#include <cmath>

namespace gcs::ui {

using namespace theme;

namespace {
constexpr double kToastLife = 8.0;
constexpr double kToastSlide = 0.28;
constexpr size_t kToastVisible = 4;
constexpr double kBannerLife = 1.6;
}  // namespace

FxOverlay::FxOverlay(QWidget* parent) : QWidget(parent) {
    setAttribute(Qt::WA_TransparentForMouseEvents);
}

void FxOverlay::toast(const QString& text, const QColor& color, const QString& tag) {
    // collapse repeats of a recent identical message into a counter
    for (auto& t : toasts_) {
        if (t.text == text && t.age < kToastLife - 1.0) {
            ++t.count;
            t.age = std::min(t.age, kToastSlide);
            update();
            return;
        }
    }
    Toast t;
    t.text = text;
    t.tag = tag;
    t.color = color;
    t.time = QTime::currentTime().toString(QStringLiteral("HH:mm:ss"));
    t.age = reducedMotion() ? kToastSlide : 0.0;
    toasts_.push_front(t);
    while (toasts_.size() > 12) {
        toasts_.pop_back();
    }
    update();
}

void FxOverlay::banner(const QString& text, const QColor& color) {
    banner_.text = text;
    banner_.color = color;
    banner_.t = 0.0;
    update();
}

void FxOverlay::setAlert(const QString& text) {
    if (text != alert_) {
        if (!text.isEmpty()) {
            alert_t_ = 0.0;
        }
        alert_ = text.isEmpty() ? alert_ : text;
        if (text.isEmpty()) {
            alert_fade_ = std::min(alert_fade_, 1.0);
            alert_.clear();
        }
        update();
    }
}

void FxOverlay::tick(double dt) {
    bool dirty = false;
    for (auto& t : toasts_) {
        t.age += dt;
    }
    while (!toasts_.empty() && toasts_.back().age > kToastLife) {
        toasts_.pop_back();
        dirty = true;
    }
    dirty |= !toasts_.empty();

    if (banner_.t >= 0.0) {
        banner_.t += dt;
        if (banner_.t > kBannerLife) {
            banner_.t = -1.0;
        }
        dirty = true;
    }

    const double target = alert_.isEmpty() ? 0.0 : 1.0;
    if (alert_fade_ != target) {
        alert_fade_ += (target - alert_fade_) * std::min(1.0, dt * 6.0);
        if (std::abs(alert_fade_ - target) < 0.01) {
            alert_fade_ = target;
        }
        dirty = true;
    }
    if (!alert_.isEmpty()) {
        alert_t_ += dt;
        dirty = true;
    }
    if (dirty) {
        update();
    }
}

void FxOverlay::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    paintAlert(p);
    paintBanner(p);
    paintToasts(p);
}

void FxOverlay::paintToasts(QPainter& p) {
    const double w = std::min(400.0, width() * 0.36);
    const double h = 42.0;
    double y = top_inset_ + 14;
    size_t shown = 0;
    for (const auto& t : toasts_) {
        if (shown >= kToastVisible) {
            break;
        }
        const double in = easeOutCubic(t.age / kToastSlide);
        const double out = std::clamp((kToastLife - t.age) / 0.6, 0.0, 1.0);
        // older entries shrink into a compact feed line
        const double compact = shown >= 1 ? 0.84 : 1.0;
        const double rh = h * compact;
        const QRectF r(width() - 16 - w + (1.0 - in) * (w + 30), y, w, rh);
        p.save();
        p.setOpacity(out * (shown >= 2 ? 0.75 : 1.0));
        drawGlass(p, chamferPath(r, 9), t.color, 0.45, kGlassDeep);
        p.fillRect(QRectF(r.left(), r.top() + 6, 3, r.height() - 12), t.color);

        const QFont tf = monoFont(10);
        drawText(p, QRectF(r.left() + 12, r.top(), 64, rh), Qt::AlignLeft | Qt::AlignVCenter, t.time, tf,
                 alpha(kMuted, 0.9));
        double tx = r.left() + 76;
        if (!t.tag.isEmpty()) {
            const QFont lf = labelFont(10);
            const double tw = QFontMetricsF(lf).horizontalAdvance(t.tag) + 12;
            const QRectF tagr(tx, r.center().y() - 9, tw, 18);
            p.setPen(QPen(t.color, 1));
            p.setBrush(alpha(t.color, 0.15));
            p.drawRect(tagr);
            drawText(p, tagr, Qt::AlignCenter, t.tag, lf, t.color);
            tx += tw + 8;
        }
        QString text = t.text;
        if (t.count > 1) {
            text += QStringLiteral("  ×%1").arg(t.count);
        }
        const QFont bf = monoFont(12.5);
        const double avail = r.right() - 10 - tx;
        text = QFontMetricsF(bf).elidedText(text, Qt::ElideRight, avail);
        drawText(p, QRectF(tx, r.top(), avail, rh), Qt::AlignLeft | Qt::AlignVCenter, text, bf,
                 mix(kText, t.color, 0.2));
        p.restore();
        y += rh + 6;
        ++shown;
    }
}

void FxOverlay::paintBanner(QPainter& p) {
    if (banner_.t < 0.0) {
        return;
    }
    const double t = banner_.t;
    const double sweep = easeOutCubic(t / 0.25);
    const double fade = std::clamp((kBannerLife - t) / 0.45, 0.0, 1.0);
    const double cy = height() * 0.27;
    const double fullw = std::min(width() * 0.7, 720.0);
    const double w = fullw * sweep;
    const QRectF band(width() / 2.0 - w / 2.0, cy - 30, w, 60);

    p.save();
    p.setOpacity(fade);
    QLinearGradient g(band.left(), 0, band.right(), 0);
    g.setColorAt(0.0, alpha(kGlassDeep, 0.0));
    g.setColorAt(0.2, kGlassDeep);
    g.setColorAt(0.8, kGlassDeep);
    g.setColorAt(1.0, alpha(kGlassDeep, 0.0));
    p.fillRect(band, g);
    QLinearGradient lg(band.left(), 0, band.right(), 0);
    lg.setColorAt(0.0, alpha(banner_.color, 0.0));
    lg.setColorAt(0.5, banner_.color);
    lg.setColorAt(1.0, alpha(banner_.color, 0.0));
    p.fillRect(QRectF(band.left(), band.top(), band.width(), 1.5), lg);
    p.fillRect(QRectF(band.left(), band.bottom() - 1.5, band.width(), 1.5), lg);

    p.setClipRect(band);
    const double letter = 1.0 + 0.6 * (1.0 - sweep);
    QFont f = titleFont(24);
    f.setLetterSpacing(QFont::AbsoluteSpacing, 24 * 0.28 * letter);
    drawText(p, band, Qt::AlignCenter, banner_.text, f, banner_.color);
    p.restore();
}

void FxOverlay::paintAlert(QPainter& p) {
    if (alert_fade_ <= 0.0) {
        return;
    }
    const double pulse = reducedMotion() ? 0.7 : 0.55 + 0.45 * std::abs(std::sin(alert_t_ * 3.0));

    // edge vignette
    const QRectF r = rect();
    QRadialGradient g(r.center(), std::hypot(r.width(), r.height()) / 2.0);
    g.setColorAt(0.0, QColor(0, 0, 0, 0));
    g.setColorAt(0.62, QColor(0, 0, 0, 0));
    g.setColorAt(1.0, alpha(kRed, 0.38 * pulse * alert_fade_));
    p.fillRect(r, g);

    if (alert_.isEmpty()) {
        return;
    }
    const QString text = QStringLiteral("⚠  %1").arg(alert_);
    const QFont f = titleFont(15);
    const double w = QFontMetricsF(f).horizontalAdvance(text) + 56;
    const QRectF box(width() / 2.0 - w / 2.0, top_inset_ + 14, w, 34);
    p.save();
    p.setOpacity(alert_fade_);
    drawGlass(p, chamferPath(box, 10), kRed, 0.9, QColor(40, 6, 12, 220));
    drawGlowPath(p, chamferPath(box, 10), alpha(kRed, pulse), 1.2, 6);
    drawText(p, box, Qt::AlignCenter, text, f, kRed);
    p.restore();
}

}  // namespace gcs::ui
