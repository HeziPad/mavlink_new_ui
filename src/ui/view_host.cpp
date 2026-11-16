#include "view_host.hpp"

#include "anim.hpp"
#include "theme.hpp"

#include <QEvent>
#include <QFontMetricsF>
#include <QMouseEvent>
#include <QPainter>

#include <cmath>

namespace gcs::ui {

namespace {
constexpr double kSwapSeconds = 0.22;
constexpr double kPipCut = 18.0;

QRectF lerpRect(const QRectF& a, const QRectF& b, double t) {
    return {lerp(a.x(), b.x(), t), lerp(a.y(), b.y(), t), lerp(a.width(), b.width(), t),
            lerp(a.height(), b.height(), t)};
}
}  // namespace

// ---- HudView -------------------------------------------------------------------

HudView::HudView(QWidget* parent) : QWidget(parent) {
    setMouseTracking(true);
}

void HudView::setCompact(bool compact) {
    if (compact_ != compact) {
        compact_ = compact;
        setCursor(compact ? Qt::PointingHandCursor : Qt::ArrowCursor);
        compactChanged();
        update();
    }
}

void HudView::setGlitch(double amount) {
    if (std::abs(glitch_ - amount) > 1e-4) {
        glitch_ = amount;
        update();
    }
}

QColor HudView::pipTagColor() const {
    return theme::kCyan;
}

QPainterPath HudView::viewPath() const {
    QPainterPath path;
    if (compact_) {
        path = theme::chamferPath(QRectF(rect()).adjusted(1, 1, -1, -1), kPipCut);
    } else {
        path.addRect(rect());
    }
    return path;
}

void HudView::beginView(QPainter& p) const {
    p.setRenderHint(QPainter::Antialiasing, true);
    if (compact_) {
        p.setClipPath(viewPath());
    }
}

void HudView::finishView(QPainter& p) const {
    const QRectF r = rect();
    if (glitch_ > 0.0) {
        // horizontal tear bands, deterministic per frame so it reads as a wipe
        p.save();
        const int bands = 7;
        for (int i = 0; i < bands; ++i) {
            const double phase = std::sin(time_ * 37.0 + i * 12.9898) * 43758.5453;
            const double frac = phase - std::floor(phase);
            const double y = r.top() + frac * r.height();
            const double h = 2.0 + 10.0 * glitch_ * frac;
            const double shift = (frac - 0.5) * 40.0 * glitch_;
            p.fillRect(QRectF(r.left() + shift, y, r.width(), h),
                       theme::alpha(i % 2 ? theme::kCyan : theme::kBg, 0.35 * glitch_));
        }
        p.fillRect(r, theme::alpha(theme::kCyan, 0.06 * glitch_));
        p.restore();
    }

    if (!compact_) {
        return;
    }
    if (!theme::reducedMotion()) {
        theme::drawScanlines(p, r, 0.8);
    }
    p.setClipping(false);

    const QPainterPath frame = viewPath();
    theme::drawGlowPath(p, frame, theme::alpha(theme::kCyan, 0.85), 1.2, 5.0);
    theme::drawBrackets(p, r.adjusted(5, 5, -5, -5), 12, theme::kCyan, 1.6);

    // header tag
    const QString tag = pipTag();
    const QFont f = theme::labelFont(10.5);
    const double tw = QFontMetricsF(f).horizontalAdvance(tag) + 20;
    const QRectF tr(r.left() + kPipCut + 2, r.top() + 8, tw, 18);
    theme::drawGlass(p, theme::chamferPath(tr, 5), pipTagColor(), 0.6, theme::kGlassDeep);
    theme::drawText(p, tr, Qt::AlignCenter, tag, f, pipTagColor());
}

// ---- ViewHost ------------------------------------------------------------------

ViewHost::ViewHost(HudView* a, HudView* b, QWidget* parent) : QWidget(parent), views_{a, b} {
    for (HudView* v : views_) {
        v->setParent(this);
        v->installEventFilter(this);
        v->show();
    }
    views_[0]->setCompact(false);
    views_[1]->setCompact(true);
    views_[1]->raise();
}

void ViewHost::setPipMaxWidth(double w) {
    pip_max_w_ = w;
    layoutViews();
}

QRectF ViewHost::pipRect() const {
    const double w = std::max(180.0, std::min({std::clamp(width() * 0.30, 240.0, 420.0), pip_max_w_}));
    const double h = w * 10.0 / 16.0;
    return {16.0, height() - h - 16.0, w, h};
}

void ViewHost::swap() {
    main_ = 1 - main_;
    anim_ = theme::reducedMotion() ? 1.0 : 0.0;
    views_[main_]->setCompact(false);
    views_[1 - main_]->setCompact(true);
    views_[1 - main_]->raise();  // the shrinking view stays on top
    layoutViews();
    emit swapped();
}

void ViewHost::tick(double dt) {
    if (anim_ >= 1.0) {
        return;
    }
    anim_ = std::min(1.0, anim_ + dt / kSwapSeconds);
    const double g = theme::reducedMotion() ? 0.0 : (1.0 - anim_);
    views_[0]->setGlitch(g);
    views_[1]->setGlitch(g);
    layoutViews();
}

void ViewHost::resizeEvent(QResizeEvent* e) {
    QWidget::resizeEvent(e);
    layoutViews();
}

void ViewHost::layoutViews() {
    const QRectF full = rect();
    const QRectF pip = pipRect();
    const double t = easeOutCubic(anim_);
    views_[main_]->setGeometry(lerpRect(pip, full, t).toAlignedRect());
    views_[1 - main_]->setGeometry(lerpRect(full, pip, t).toAlignedRect());
}

bool ViewHost::eventFilter(QObject* obj, QEvent* e) {
    if (obj == pipView() && anim_ >= 1.0) {
        switch (e->type()) {
        case QEvent::MouseButtonPress:
            if (static_cast<QMouseEvent*>(e)->button() == Qt::LeftButton) {
                swap();
            }
            return true;
        case QEvent::MouseButtonRelease:
        case QEvent::MouseButtonDblClick:
        case QEvent::Wheel:
            return true;
        default:
            break;
        }
    }
    return QWidget::eventFilter(obj, e);
}

}  // namespace gcs::ui
