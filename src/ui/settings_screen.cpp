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

const std::vector<int>& SettingsScreen::trailChoices() {
    static const std::vector<int> c = {30, 60, 120, 300, 600, 0};
    return c;
}

SettingsScreen::SettingsScreen(QWidget* parent) : QWidget(parent) {
    alt_.labels = {QStringLiteral("METERS"), QStringLiteral("FEET")};
    trail_on_.labels = {QStringLiteral("OFF"), QStringLiteral("ON")};
    trail_len_.labels = {QStringLiteral("30 S"), QStringLiteral("1 MIN"), QStringLiteral("2 MIN"),
                         QStringLiteral("5 MIN"), QStringLiteral("10 MIN"), QStringLiteral("ALL")};
    hide();
}

void SettingsScreen::open(const Values& v) {
    v_ = v;
    alt_.selected = v.alt_unit;
    trail_on_.selected = v.trail_on ? 1 : 0;
    const auto& c = trailChoices();
    const auto it = std::find(c.begin(), c.end(), v.trail_secs);
    trail_len_.selected = it != c.end() ? static_cast<int>(it - c.begin()) : 2;
    for (Segmented* s : {&alt_, &trail_on_, &trail_len_}) {
        s->knob = s->selected;
    }
    trail_len_.enabled = v.trail_on;

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
    for (Segmented* s : {&alt_, &trail_on_, &trail_len_}) {
        s->knob += (s->selected - s->knob) * k;
    }
    cat_knob_ += (category_ - cat_knob_) * k;
    flash_ = std::max(0.0, flash_ - dt * 2.0);
    update();
}

QRectF SettingsScreen::panelRect() const {
    const double w = std::min(860.0, width() - 48.0);
    const double h = std::min(480.0, height() - 120.0);
    return {(width() - w) / 2.0, (height() - h) / 2.0, w, h};
}

bool SettingsScreen::pressSegmented(Segmented& seg, const QPointF& pos) {
    if (!seg.enabled) {
        return false;
    }
    for (int i = 0; i < static_cast<int>(seg.cells.size()); ++i) {
        if (seg.cells[i].contains(pos) && i != seg.selected) {
            seg.selected = i;
            flash_ = 1.0;
            return true;
        }
    }
    return false;
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
    for (int i = 0; i < static_cast<int>(cat_rects_.size()); ++i) {
        if (cat_rects_[i].contains(pos)) {
            category_ = i;
            update();
            return;
        }
    }
    if (category_ == kUnits) {
        if (pressSegmented(alt_, pos)) {
            emit altUnitChanged(alt_.selected);
        }
    } else if (category_ == kMap) {
        bool changed = false;
        if (pressSegmented(trail_on_, pos)) {
            trail_len_.enabled = trail_on_.selected == 1;
            changed = true;
        } else if (pressSegmented(trail_len_, pos)) {
            changed = true;
        }
        if (changed) {
            emit trailChanged(trail_on_.selected == 1, trailChoices()[trail_len_.selected]);
        }
    }
    update();
}

void SettingsScreen::paintSegmented(QPainter& p, const QRectF& track, Segmented& seg) {
    const int n = static_cast<int>(seg.labels.size());
    const double a = seg.enabled ? 1.0 : 0.35;
    p.save();
    p.setOpacity(p.opacity() * a);
    drawGlass(p, chamferPath(track, 10), kCyan, 0.5, alpha(kBg, 0.7));
    const double cw = track.width() / n;
    seg.cells.clear();
    for (int i = 0; i < n; ++i) {
        seg.cells.push_back(QRectF(track.left() + i * cw, track.top(), cw, track.height()));
    }
    const QRectF knob(track.left() + 4 + seg.knob * cw, track.top() + 4, cw - 8, track.height() - 8);
    const QColor kc = mix(kCyan, kText, 0.15 * flash_);
    drawGlass(p, chamferPath(knob, 8), kc, 1.0, alpha(kCyan, 0.22 + 0.25 * flash_));
    const QFont f = labelFont(n > 3 ? 12.5 : 15);
    for (int i = 0; i < n; ++i) {
        const double sel = 1.0 - std::min(1.0, std::abs(seg.knob - i));
        drawText(p, seg.cells[i], Qt::AlignCenter, seg.labels[i], f, mix(kMuted, kCyan, sel));
    }
    p.restore();
}

void SettingsScreen::paintRow(QPainter& p, const QRectF& r, const QString& title, const QString& sub) {
    drawText(p, QRectF(r.left(), r.top(), r.width(), 24), Qt::AlignLeft | Qt::AlignVCenter, title, labelFont(14),
             kText);
    drawText(p, QRectF(r.left(), r.top() + 24, r.width(), 18), Qt::AlignLeft | Qt::AlignVCenter, sub,
             labelFont(10, false), kMuted);
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

    // ---- panel: slides up as it opens ----------------------------------------------
    const QRectF pr = panelRect();
    p.save();
    p.setOpacity(e);
    p.translate(0, (1.0 - e) * 40.0);
    const QPainterPath shape = chamferPath(pr, 22);
    drawGlass(p, shape, kCyan, 0.75, alpha(kGlassDeep, 0.96));
    drawBrackets(p, pr.adjusted(-6, -6, 6, 6), 26, alpha(kCyan, 0.8), 2.0);
    p.setPen(QPen(kCyan, 2));
    p.drawLine(QPointF(pr.left() + 1, pr.top() + 22), QPointF(pr.left() + 22, pr.top() + 1));

    drawText(p, QRectF(pr.left() + 34, pr.top() + 18, 400, 34), Qt::AlignLeft | Qt::AlignVCenter,
             QStringLiteral("SETTINGS"), titleFont(26), kText);
    drawText(p, QRectF(pr.left() + 36, pr.top() + 50, 400, 16), Qt::AlignLeft | Qt::AlignVCenter,
             QStringLiteral("changes apply immediately"), labelFont(10), kMuted);

    close_rect_ = QRectF(pr.right() - 72, pr.top() + 16, 52, 52);
    drawGlass(p, chamferPath(close_rect_, 8), kCyan, 0.6, kGlassDeep);
    drawText(p, close_rect_, Qt::AlignCenter, QStringLiteral("✕"), monoFont(20, true), kText);

    // ---- categories ----------------------------------------------------------------
    const double top = pr.top() + 92;
    const QRectF cat_col(pr.left() + 24, top, 190, pr.bottom() - top - 24);
    p.setPen(QPen(alpha(kCyan, 0.18), 1));
    p.drawLine(QPointF(cat_col.right() + 16, cat_col.top()), QPointF(cat_col.right() + 16, cat_col.bottom()));
    const char* names[kCategories] = {"UNITS", "MAP"};
    cat_rects_.clear();
    for (int i = 0; i < kCategories; ++i) {
        const QRectF item(cat_col.left(), cat_col.top() + i * 60, cat_col.width(), 52);
        cat_rects_.push_back(item);
        const double sel = 1.0 - std::min(1.0, std::abs(cat_knob_ - i));
        drawGlass(p, chamferPath(item, 8), mix(kMuted, kCyan, sel), 0.4 + 0.5 * sel, alpha(kCyan, 0.12 * sel));
        if (sel > 0.01) {
            p.setPen(Qt::NoPen);
            p.setBrush(alpha(kCyan, sel));
            p.drawRect(QRectF(item.left(), item.top() + 10, 3, item.height() - 20));
        }
        drawText(p, item.adjusted(18, 0, 0, 0), Qt::AlignLeft | Qt::AlignVCenter, QString::fromLatin1(names[i]),
                 labelFont(14), mix(kMuted, kCyan, sel));
    }

    // ---- rows of the selected category ----------------------------------------------------
    const QRectF body(cat_col.right() + 40, top, pr.right() - cat_col.right() - 64, cat_col.height());
    if (category_ == kUnits) {
        paintRow(p, body, QStringLiteral("ALTITUDE"),
                 QStringLiteral("HUD tape, telemetry bar, map labels, panels and steppers"));
        paintSegmented(p, QRectF(body.left(), body.top() + 54, std::min(body.width(), 360.0), 58), alt_);
        drawText(p, QRectF(body.left(), body.bottom() - 18, body.width(), 18), Qt::AlignLeft | Qt::AlignVCenter,
                 QStringLiteral("distances and speeds stay metric"), labelFont(10, false), alpha(kMuted, 0.8));
    } else {
        paintRow(p, body, QStringLiteral("FLIGHT TRAIL"), QStringLiteral("the line the aircraft leaves behind on the map"));
        paintSegmented(p, QRectF(body.left(), body.top() + 54, std::min(body.width(), 220.0), 58), trail_on_);
        const QRectF lr(body.left(), body.top() + 134, body.width(), 42);
        drawText(p, QRectF(lr.left(), lr.top(), lr.width(), 18), Qt::AlignLeft | Qt::AlignVCenter,
                 QStringLiteral("LENGTH"), labelFont(11), trail_len_.enabled ? kText : alpha(kMuted, 0.6));
        drawText(p, QRectF(lr.left(), lr.top() + 18, lr.width(), 18), Qt::AlignLeft | Qt::AlignVCenter,
                 QStringLiteral("how far back in time it reaches"), labelFont(10, false),
                 alpha(kMuted, trail_len_.enabled ? 1.0 : 0.6));
        paintSegmented(p, QRectF(body.left(), body.top() + 182, body.width(), 58), trail_len_);
    }
    p.restore();
}

}  // namespace gcs::ui
