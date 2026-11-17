#include "alerts_panel.hpp"

#include "anim.hpp"
#include "theme.hpp"

#include <QFontMetricsF>
#include <QMouseEvent>
#include <QPainter>
#include <QWheelEvent>

#include <algorithm>
#include <array>
#include <cmath>

namespace gcs::ui {

using namespace theme;

namespace {

constexpr double kWidth = 470.0;
constexpr double kHeader = 60.0;
constexpr double kPills = 46.0;
constexpr double kPad = 12.0;
constexpr double kGap = 10.0;
constexpr double kStripe = 5.0;

QFont bodyFont(double px, bool bold = false) {
    QFont f(QStringLiteral("DejaVu Sans"));
    f.setPixelSize(static_cast<int>(std::round(px)));
    f.setBold(bold);
    return f;
}

QColor severityColor(Severity s) {
    switch (s) {
    case Severity::Critical:
    case Severity::Warning: return kRed;
    case Severity::Caution: return kAmber;
    case Severity::Info: return kCyan;
    }
    return kMuted;
}

double wrapHeight(const QFont& f, double w, const QString& text) {
    if (text.isEmpty()) {
        return 0.0;
    }
    return QFontMetricsF(f).boundingRect(QRectF(0, 0, w, 1e6), Qt::TextWordWrap, text).height();
}

// status pill groups
enum Group { kLink, kGps, kNav, kSensors, kPower, kArming, kGroups };
const char* const kGroupNames[kGroups] = {"LINK", "GPS", "NAV", "SENSORS", "POWER", "ARMING"};

int groupOf(std::string id) {
    if (id == "prearm" || id.rfind("prearm.other", 0) == 0 || id == "prearm.txt.mode_not_armable" ||
        id == "prearm.txt.safety" || id == "prearm.txt.throttle" || id == "prearm.txt.rc") {
        return kArming;
    }
    if (id.rfind("prearm.", 0) == 0) {
        id = id.substr(7);
    }
    auto starts = [&](const char* p) { return id.rfind(p, 0) == 0; };
    if (starts("link.")) return kLink;
    if (starts("gps.") || id == "txt.gps" || id == "sensor.gps") return kGps;
    if (starts("ekf.") || id == "sensor.ahrs" || starts("txt.ekf")) return kNav;
    if (starts("battery.") || id == "sensor.battery" || id == "txt.battery" || id == "txt.board_voltage") return kPower;
    if (starts("sensor.") || starts("txt.compass") || id == "txt.mag_field" || id == "txt.accel" ||
        id == "txt.gyro" || id == "txt.airspeed" || id == "txt.baro") {
        return kSensors;
    }
    if (id == "txt.rc" || id == "txt.throttle" || id == "txt.safety") return kArming;
    return -1;
}

QString qs(const std::string& s) {
    return QString::fromStdString(s);
}

}  // namespace

AlertsPanel::AlertsPanel(QWidget* parent) : QWidget(parent) {
    hide();
}

void AlertsPanel::setAdvisories(const std::vector<Advisory>& list, bool link_alive) {
    alive_ = link_alive;
    list_ = list;
    std::map<std::string, double> next;
    for (const auto& a : list_) {
        const auto it = age_.find(a.id);
        next[a.id] = it != age_.end() ? it->second : 0.0;
    }
    age_.swap(next);
    if (isVisible()) {
        place(area_);
        update();
    }
}

double AlertsPanel::cardHeight(const Advisory& a, double w) const {
    const double inner = w - kStripe - 2 * kPad;
    double h = kPad;
    h += 18;  // severity tag line
    h += wrapHeight(bodyFont(15, true), inner, qs(a.title)) + 4;
    h += wrapHeight(bodyFont(12.5), inner, qs(a.cause));
    if (!a.suggestions.empty()) {
        h += 10 + 16;  // "TRY" label
        for (const auto& s : a.suggestions) {
            h += wrapHeight(bodyFont(12.5), inner - 16, qs(s)) + 4;
        }
    }
    if (!a.detail.empty()) {
        h += 6 + wrapHeight(monoFont(10.5), inner, qs(a.detail));
    }
    return h + kPad;
}

double AlertsPanel::contentHeight(double w) const {
    if (list_.empty()) {
        return 76.0;
    }
    double h = 0.0;
    for (const auto& a : list_) {
        h += cardHeight(a, w) + kGap;
    }
    return h;
}

void AlertsPanel::place(const QRect& area) {
    area_ = area;
    const double w = std::min(kWidth, area.width() - 16.0);
    const double want = kHeader + kPills + kPad + contentHeight(w - 2 * kPad) + kPad;
    const double h = std::min(want, static_cast<double>(area.height()));
    setGeometry(static_cast<int>(area.right() - w), area.top(), static_cast<int>(w), static_cast<int>(h));
    clampScroll();
}

void AlertsPanel::clampScroll() {
    const double view = height() - kHeader - kPills - kPad;
    const double content = contentHeight(width() - 2 * kPad) + kPad;
    scroll_ = std::clamp(scroll_, 0.0, std::max(0.0, content - view));
}

void AlertsPanel::tick(double dt) {
    if (!isVisible()) {
        return;
    }
    time_ += dt;
    for (auto& [id, t] : age_) {
        t += dt;
    }
    update();
}

void AlertsPanel::mousePressEvent(QMouseEvent* e) {
    pressing_ = true;
    dragged_ = false;
    press_pos_ = e->position();
    press_scroll_ = scroll_;
}

void AlertsPanel::mouseMoveEvent(QMouseEvent* e) {
    if (!pressing_) {
        return;
    }
    const double dy = e->position().y() - press_pos_.y();
    if (!dragged_ && std::abs(dy) > 8) {
        dragged_ = true;
    }
    if (dragged_) {
        scroll_ = press_scroll_ - dy;
        clampScroll();
        update();
    }
}

void AlertsPanel::mouseReleaseEvent(QMouseEvent* e) {
    if (pressing_ && !dragged_ && close_rect_.contains(e->position())) {
        emit closeRequested();
    }
    pressing_ = false;
}

void AlertsPanel::wheelEvent(QWheelEvent* e) {
    scroll_ -= e->angleDelta().y() / 2.0;
    clampScroll();
    update();
}

void AlertsPanel::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    const QRectF r = QRectF(rect()).adjusted(1, 1, -1, -1);
    drawGlass(p, chamferPath(r, 14), kCyan, 0.6, alpha(kGlassDeep, 0.97));
    p.setPen(QPen(kCyan, 2));
    p.drawLine(QPointF(r.left() + 1, r.top() + 14), QPointF(r.left() + 14, r.top() + 1));

    // ---- header -------------------------------------------------------------------
    int crit = 0, fault = 0, check = 0;
    for (const auto& a : list_) {
        crit += a.severity == Severity::Critical;
        fault += a.severity == Severity::Warning;
        check += a.severity == Severity::Caution;
    }
    drawText(p, QRectF(r.left() + 18, r.top() + 10, 200, 22), Qt::AlignLeft | Qt::AlignVCenter,
             QStringLiteral("SYSTEMS"), titleFont(18), kText);
    QString summary;
    QColor sc = kGreen;
    if (!alive_) {
        summary = QStringLiteral("no live link");
        sc = kMuted;
    } else if (crit + fault + check == 0) {
        summary = QStringLiteral("all systems go");
    } else {
        QStringList parts;
        if (crit) parts << QStringLiteral("%1 critical").arg(crit);
        if (fault) parts << QStringLiteral("%1 fault").arg(fault);
        if (check) parts << QStringLiteral("%1 to check").arg(check);
        summary = parts.join(QStringLiteral(" · "));
        sc = crit + fault ? kRed : kAmber;
    }
    drawText(p, QRectF(r.left() + 18, r.top() + 33, 300, 16), Qt::AlignLeft | Qt::AlignVCenter, summary,
             labelFont(10.5), sc);
    close_rect_ = QRectF(r.right() - 58, r.top() + 6, 50, 48);
    drawGlass(p, chamferPath(close_rect_.adjusted(4, 4, -4, -4), 6), kCyan, 0.5, kGlassDeep);
    drawText(p, close_rect_, Qt::AlignCenter, QStringLiteral("✕"), monoFont(17, true), kText);

    // ---- group pills ----------------------------------------------------------------
    std::array<int, kGroups> worst;
    worst.fill(-1);
    for (const auto& a : list_) {
        const int g = groupOf(a.id);
        if (g >= 0) {
            worst[g] = std::max(worst[g], static_cast<int>(a.severity));
        }
    }
    {
        const double y = r.top() + kHeader;
        const double pw = (r.width() - 2 * kPad - (kGroups - 1) * 6) / kGroups;
        for (int g = 0; g < kGroups; ++g) {
            const QRectF pill(r.left() + kPad + g * (pw + 6), y, pw, 34);
            QColor c = kGreen;
            QString mark = QStringLiteral("✓");
            if (!alive_ && g != kLink) {
                c = kMuted;
                mark = QStringLiteral("–");
            } else if (worst[g] >= static_cast<int>(Severity::Warning)) {
                c = kRed;
                mark = QStringLiteral("✗");
            } else if (worst[g] == static_cast<int>(Severity::Caution)) {
                c = kAmber;
                mark = QStringLiteral("!");
            } else if (worst[g] == static_cast<int>(Severity::Info)) {
                c = kCyan;
                mark = QStringLiteral("·");
            }
            drawGlass(p, chamferPath(pill, 5), c, 0.55, alpha(c, 0.08));
            drawText(p, pill.adjusted(0, 3, 0, -15), Qt::AlignCenter, QString::fromLatin1(kGroupNames[g]),
                     labelFont(8.5), alpha(kText, 0.8));
            drawText(p, pill.adjusted(0, 15, 0, -1), Qt::AlignCenter, mark, monoFont(12, true), c);
        }
    }

    // ---- cards ------------------------------------------------------------------------
    const QRectF view(r.left() + kPad, r.top() + kHeader + kPills, r.width() - 2 * kPad,
                      r.height() - kHeader - kPills - kPad);
    p.save();
    p.setClipRect(view);
    double y = view.top() - scroll_;
    const double w = view.width();
    if (list_.empty()) {
        const QRectF card(view.left(), y, w, 66);
        drawGlass(p, chamferPath(card, 8), kGreen, 0.6, alpha(kGreen, 0.06));
        drawText(p, card.adjusted(18, 10, -10, -30), Qt::AlignLeft | Qt::AlignVCenter,
                 alive_ ? QStringLiteral("All systems go") : QStringLiteral("Nothing to report yet"),
                 bodyFont(15, true), alive_ ? kGreen : kMuted);
        drawText(p, card.adjusted(18, 34, -10, -8), Qt::AlignLeft | Qt::AlignVCenter,
                 alive_ ? QStringLiteral("Nothing needs your attention.")
                        : QStringLiteral("Cards appear here when the vehicle reports a problem."),
                 bodyFont(12.5), kMuted);
    }
    for (const auto& a : list_) {
        const double h = cardHeight(a, w);
        if (y > view.bottom()) {
            break;
        }
        if (y + h < view.top()) {
            y += h + kGap;
            continue;
        }
        const auto it = age_.find(a.id);
        const double age = it != age_.end() ? it->second : 1.0;
        const double in = reducedMotion() ? 1.0 : easeOutCubic(std::min(1.0, age / 0.35));
        const QColor c = severityColor(a.severity);
        p.save();
        p.setOpacity(in);
        p.translate((1.0 - in) * 40.0, 0);
        const QRectF card(view.left(), y, w, h);
        drawGlass(p, chamferPath(card, 8), c, 0.45, alpha(c, 0.05));
        double stripe_a = 1.0;
        if (a.severity == Severity::Critical && !reducedMotion()) {
            stripe_a = 0.55 + 0.45 * std::abs(std::sin(time_ * 3.0));
        }
        p.setPen(Qt::NoPen);
        p.setBrush(alpha(c, stripe_a));
        p.drawRect(QRectF(card.left(), card.top() + 8, kStripe, card.height() - 16));

        const double x = card.left() + kStripe + kPad;
        const double inner = w - kStripe - 2 * kPad;
        double cy = card.top() + kPad;
        drawText(p, QRectF(x, cy, inner, 14), Qt::AlignLeft | Qt::AlignVCenter,
                 QString::fromLatin1(severityName(a.severity)), labelFont(9), c);
        cy += 18;
        const QString title = qs(a.title);
        const double th = wrapHeight(bodyFont(15, true), inner, title);
        p.setFont(bodyFont(15, true));
        p.setPen(kText);
        p.drawText(QRectF(x, cy, inner, th), Qt::TextWordWrap, title);
        cy += th + 4;
        const QString cause = qs(a.cause);
        const double ch = wrapHeight(bodyFont(12.5), inner, cause);
        p.setFont(bodyFont(12.5));
        p.setPen(alpha(kText, 0.78));
        p.drawText(QRectF(x, cy, inner, ch), Qt::TextWordWrap, cause);
        cy += ch;
        if (!a.suggestions.empty()) {
            cy += 10;
            drawText(p, QRectF(x, cy, inner, 14), Qt::AlignLeft | Qt::AlignVCenter, QStringLiteral("TRY"),
                     labelFont(9), kCyan);
            cy += 16;
            for (const auto& s : a.suggestions) {
                const QString t = qs(s);
                const double sh = wrapHeight(bodyFont(12.5), inner - 16, t);
                p.setPen(Qt::NoPen);
                p.setBrush(kCyan);
                p.drawPolygon(QPolygonF({QPointF(x + 2, cy + 4), QPointF(x + 8, cy + 8), QPointF(x + 2, cy + 12)}));
                p.setFont(bodyFont(12.5));
                p.setPen(kText);
                p.drawText(QRectF(x + 16, cy, inner - 16, sh), Qt::TextWordWrap, t);
                cy += sh + 4;
            }
        }
        if (!a.detail.empty()) {
            cy += 6;
            const QString d = qs(a.detail);
            const double dh = wrapHeight(monoFont(10.5), inner, d);
            p.setFont(monoFont(10.5));
            p.setPen(alpha(kMuted, 0.85));
            p.drawText(QRectF(x, cy, inner, dh), Qt::TextWordWrap, d);
        }
        p.restore();
        y += h + kGap;
    }
    p.restore();

    // scroll hint
    const double content = contentHeight(w) + kPad;
    if (content > view.height() + 1) {
        const double frac = view.height() / content;
        const double th = std::max(30.0, view.height() * frac);
        const double ty = view.top() + (view.height() - th) * (scroll_ / std::max(1.0, content - view.height()));
        p.setPen(Qt::NoPen);
        p.setBrush(alpha(kCyan, 0.5));
        p.drawRoundedRect(QRectF(r.right() - 6, ty, 3, th), 1.5, 1.5);
    }
}

}  // namespace gcs::ui
