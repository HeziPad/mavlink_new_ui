#include "telemetry_bar.hpp"

#include "theme.hpp"

#include <QFontMetricsF>
#include <QMouseEvent>
#include <QPainter>

#include <cmath>

namespace gcs::ui {

using namespace theme;

// Small panel shown under the SYSTEMS chip.
class ChecklistPanel : public QWidget {
public:
    struct Row {
        QString name;
        QString detail;
        QColor color;
        QString mark;
    };

    explicit ChecklistPanel(QWidget* parent) : QWidget(parent) {
        setAttribute(Qt::WA_TransparentForMouseEvents);
        hide();
    }

    void setRows(std::vector<Row> rows) {
        rows_ = std::move(rows);
        resize(250, 40 + static_cast<int>(rows_.size()) * 24);
        update();
    }

protected:
    void paintEvent(QPaintEvent*) override {
        QPainter p(this);
        p.setRenderHint(QPainter::Antialiasing);
        const QRectF r = QRectF(rect()).adjusted(1, 1, -1, -1);
        drawGlass(p, chamferPath(r, 10), kCyan, 0.5, kGlassDeep);
        drawText(p, QRectF(14, 8, r.width() - 28, 18), Qt::AlignLeft | Qt::AlignVCenter,
                 QStringLiteral("Pre-flight systems"), labelFont(10.5), kMuted);
        double y = 32;
        for (const auto& row : rows_) {
            const QRectF line(14, y, r.width() - 28, 22);
            p.setPen(Qt::NoPen);
            p.setBrush(row.color);
            p.drawRect(QRectF(line.left(), line.center().y() - 4, 3, 8));
            drawText(p, line.adjusted(10, 0, 0, 0), Qt::AlignLeft | Qt::AlignVCenter, row.name,
                     labelFont(11), kText);
            drawText(p, line.adjusted(70, 0, -24, 0), Qt::AlignLeft | Qt::AlignVCenter, row.detail,
                     monoFont(11.5), kMuted);
            drawText(p, line, Qt::AlignRight | Qt::AlignVCenter, row.mark, monoFont(13, true),
                     row.color);
            y += 24;
        }
    }

private:
    std::vector<Row> rows_;
};

namespace {

const char* fixName(int fix) {
    switch (fix) {
    case 0:
    case 1: return "NO FIX";
    case 2: return "2D";
    case 3: return "3D";
    case 4: return "DGPS";
    case 5: return "RTK FL";
    case 6: return "RTK";
    default: return "?";
    }
}

QString shortVtol(VtolState v) {
    switch (v) {
    case VtolState::Hover: return QStringLiteral("HOVER");
    case VtolState::TransitionToCruise: return QStringLiteral("→ CRUISE");
    case VtolState::Cruise: return QStringLiteral("CRUISE");
    case VtolState::TransitionToHover: return QStringLiteral("→ HOVER");
    case VtolState::Unknown: break;
    }
    return QStringLiteral("—");
}

QColor vtolColor(VtolState v) {
    switch (v) {
    case VtolState::Hover: return kCyan;
    case VtolState::Cruise: return kGreen;
    case VtolState::TransitionToCruise:
    case VtolState::TransitionToHover: return kAmber;
    case VtolState::Unknown: break;
    }
    return kMuted;
}

QColor batteryColor(int pct) {
    if (pct < 0) {
        return kMuted;
    }
    if (pct < 20) {
        return kRed;
    }
    if (pct < 40) {
        return kAmber;
    }
    return kGreen;
}

// label on top, value below
void chip(QPainter& p, const QRectF& r, const QString& label, const QString& value,
          const QColor& vc, double flash = 0.0) {
    if (flash > 0.0) {
        p.fillRect(r.adjusted(2, 4, -2, -4), alpha(vc, 0.22 * flash));
    }
    drawText(p, QRectF(r.left(), r.top() + 6, r.width(), 13), Qt::AlignHCenter | Qt::AlignVCenter,
             label, labelFont(9.5), kMuted);
    drawText(p, QRectF(r.left(), r.top() + 19, r.width(), 24), Qt::AlignHCenter | Qt::AlignVCenter,
             value, monoFont(15, true), vc);
}

}  // namespace

TelemetryBar::TelemetryBar(QWidget* parent) : QWidget(parent) {
    setMouseTracking(true);
    panel_ = new ChecklistPanel(parent);
    alt_.rate = as_.rate = gs_.rate = 9.0;
    batt_.rate = 4.0;
}

void TelemetryBar::setState(const VehicleState& s, Clock::time_point now) {
    s_ = s;
    hb_age_ = heartbeatAge(s, now);
    alive_ = linkAlive(s, now);

    const double since = std::chrono::duration<double>(now - rate_t_).count();
    if (since >= 1.0 || rate_t_ == Clock::time_point{}) {
        if (rate_t_ != Clock::time_point{} && s.messages >= rate_msgs_) {
            msg_rate_ = static_cast<double>(s.messages - rate_msgs_) / since;
        }
        rate_msgs_ = s.messages;
        rate_t_ = now;
    }

    if (s.has_position) {
        alt_.set(s.alt_rel);
    }
    if (s.has_vfr_hud) {
        as_.set(s.airspeed);
        gs_.set(s.groundspeed);
    }
    if (s.battery_pct >= 0) {
        batt_.set(s.battery_pct);
    }

    if (s.has_heartbeat && s.mode_name != last_mode_) {
        if (!last_mode_.empty() && !reducedMotion()) {
            mode_flash_.start(0.9);
        }
        last_mode_ = s.mode_name;
    }
    const int v = static_cast<int>(effectiveVtolState(s));
    if (v != last_vtol_) {
        if (last_vtol_ >= 0 && !reducedMotion()) {
            vtol_flash_.start(0.9);
        }
        last_vtol_ = v;
    }

    if (sys_hover_) {
        showChecklist(true);  // refresh rows
    }
    update();
}

void TelemetryBar::tick(double dt) {
    time_ += dt;
    bool moving = false;
    moving |= alt_.step(dt);
    moving |= as_.step(dt);
    moving |= gs_.step(dt);
    moving |= batt_.step(dt);
    moving |= mode_flash_.step(dt);
    moving |= vtol_flash_.step(dt);
    if (moving || !alive_) {
        update();
    }
}

std::vector<TelemetryBar::CheckItem> TelemetryBar::checklist() const {
    std::vector<CheckItem> items;
    items.push_back({QStringLiteral("Link"),
                     alive_ ? QStringLiteral("%1 msg/s").arg(msg_rate_, 0, 'f', 0)
                            : QStringLiteral("no heartbeat"),
                     alive_ ? Check::Ok : Check::Bad});
    if (!s_.has_gps) {
        items.push_back({QStringLiteral("GPS"), QStringLiteral("no data"), Check::Bad});
    } else {
        items.push_back({QStringLiteral("GPS"),
                         QStringLiteral("%1 · %2 sats").arg(QLatin1String(fixName(s_.gps_fix))).arg(s_.gps_sats),
                         gpsHealthy(s_) ? Check::Ok : Check::Bad});
    }
    items.push_back({QStringLiteral("EKF"), s_.has_ekf ? QStringLiteral("flags 0x%1").arg(s_.ekf_flags, 0, 16)
                                                       : QStringLiteral("no report"),
                     !s_.has_ekf ? Check::Warn : (ekfHealthy(s_) ? Check::Ok : Check::Bad)});
    items.push_back({QStringLiteral("PreArm"),
                     s_.has_sys_status ? (prearmHealthy(s_) ? QStringLiteral("passing")
                                                            : QStringLiteral("see comms log"))
                                       : QStringLiteral("unknown"),
                     !s_.has_sys_status ? Check::Warn : (prearmHealthy(s_) ? Check::Ok : Check::Bad)});
    items.push_back({QStringLiteral("Home"), s_.has_home ? QStringLiteral("set") : QStringLiteral("not set"),
                     s_.has_home ? Check::Ok : Check::Warn});
    if (s_.battery_pct >= 0) {
        items.push_back({QStringLiteral("Battery"), QStringLiteral("%1%").arg(s_.battery_pct),
                         s_.battery_pct >= 40 ? Check::Ok : (s_.battery_pct >= 20 ? Check::Warn : Check::Bad)});
    } else {
        items.push_back({QStringLiteral("Battery"),
                         s_.has_battery ? QStringLiteral("%1 V").arg(s_.battery_v, 0, 'f', 1)
                                        : QStringLiteral("not reported"),
                         Check::NotApplicable});
    }
    return items;
}

void TelemetryBar::showChecklist(bool on) {
    if (!on) {
        panel_->hide();
        return;
    }
    std::vector<ChecklistPanel::Row> rows;
    for (const auto& it : checklist()) {
        QColor c = kMuted;
        QString mark = QStringLiteral("–");
        switch (it.status) {
        case Check::Ok: c = kGreen; mark = QStringLiteral("✓"); break;
        case Check::Warn: c = kAmber; mark = QStringLiteral("!"); break;
        case Check::Bad: c = kRed; mark = QStringLiteral("✗"); break;
        case Check::NotApplicable: break;
        }
        rows.push_back({it.name, it.detail, c, mark});
    }
    panel_->setRows(std::move(rows));
    const QPoint at = mapToParent(QPointF(sys_rect_.right() - panel_->width(), sys_rect_.bottom() + 6).toPoint());
    panel_->move(at);
    panel_->raise();
    panel_->show();
}

void TelemetryBar::mouseMoveEvent(QMouseEvent* e) {
    const bool hover = sys_rect_.contains(e->position());
    if (hover != sys_hover_) {
        sys_hover_ = hover;
        showChecklist(hover);
    }
}

void TelemetryBar::leaveEvent(QEvent* e) {
    sys_hover_ = false;
    showChecklist(false);
    QWidget::leaveEvent(e);
}

void TelemetryBar::hideEvent(QHideEvent* e) {
    showChecklist(false);
    QWidget::hideEvent(e);
}

void TelemetryBar::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);

    // chip widths at scale 1
    const double kLink = 104, kMode = 128, kVtol = 112, kArm = 84, kAlt = 92, kSpd = 132, kBat = 156,
                 kGps = 92, kSys = 108;
    const double widths[] = {kLink, kMode, kVtol, kArm, kAlt, kSpd, kBat, kGps, kSys};
    double total = 0;
    for (double w : widths) {
        total += w;
    }
    const double pad = 14;
    const double scale = std::min(1.0, (width() - 4.0) / (total + 2 * pad));
    const double h = 46;

    p.translate(width() / 2.0, 0);
    p.scale(scale, scale);
    p.translate(-(total + 2 * pad) / 2.0, 2);

    const QRectF strip(0, 0, total + 2 * pad, h);
    drawGlass(p, chamferPath(strip, 14), kCyan, 0.4);
    // accent ticks on the chamfers
    p.setPen(QPen(alpha(kCyan, 0.9), 2));
    p.drawLine(QPointF(2, 14), QPointF(14, 2));
    p.drawLine(QPointF(strip.right() - 14, h - 1), QPointF(strip.right() - 1, h - 14));

    double x = pad;
    auto next = [&](double w) {
        QRectF r(x, 0, w, h);
        x += w;
        if (x < total + pad - 1) {
            p.setPen(QPen(alpha(kCyan, 0.16), 1));
            p.drawLine(QPointF(x, 10), QPointF(x, h - 10));
        }
        return r;
    };

    // ---- link --------------------------------------------------------------
    {
        const QRectF r = next(kLink);
        QString text;
        QColor c;
        int bars = 0;
        if (!s_.error.empty()) {
            text = QStringLiteral("ERROR");
            c = kRed;
        } else if (!s_.has_heartbeat) {
            text = QStringLiteral("WAIT");
            c = kMuted;
        } else if (hb_age_ < 3.0) {
            text = QStringLiteral("LIVE");
            c = kGreen;
            bars = msg_rate_ > 20 ? 4 : (msg_rate_ > 5 ? 3 : 2);
        } else if (hb_age_ < 10.0) {
            text = QStringLiteral("STALE");
            c = kAmber;
            bars = 1;
        } else {
            text = QStringLiteral("LOST");
            c = kRed;
        }
        if (!alive_ && s_.has_heartbeat && !reducedMotion()) {
            c = alpha(c, 0.55 + 0.45 * std::abs(std::sin(time_ * 4.0)));
        }
        // signal bars
        const double bx = r.left() + 12, by = r.top() + 33;
        for (int i = 0; i < 4; ++i) {
            const double bh = 6 + i * 4;
            const QRectF b(bx + i * 6, by - bh, 4, bh);
            p.fillRect(b, i < bars ? c : alpha(kMuted, 0.25));
        }
        drawText(p, QRectF(r.left() + 40, r.top() + 6, r.width() - 44, 13), Qt::AlignLeft | Qt::AlignVCenter,
                 QStringLiteral("Link"), labelFont(9.5), kMuted);
        drawText(p, QRectF(r.left() + 40, r.top() + 19, r.width() - 44, 24),
                 Qt::AlignLeft | Qt::AlignVCenter, text, monoFont(14, true), c);
    }

    // ---- mode --------------------------------------------------------------
    {
        const QRectF r = next(kMode);
        const QString mode = s_.has_heartbeat ? QString::fromStdString(s_.mode_name) : QStringLiteral("—");
        const double f = mode_flash_.active() ? 1.0 - mode_flash_.progress() : 0.0;
        QRectF rr = r;
        if (f > 0) {
            rr.translate(0, -3.0 * f);
        }
        chip(p, rr, QStringLiteral("Mode"), mode, mix(kCyan, kText, 0.3), f);
    }

    // ---- vtol --------------------------------------------------------------
    {
        const QRectF r = next(kVtol);
        const VtolState v = effectiveVtolState(s_);
        const double f = vtol_flash_.active() ? 1.0 - vtol_flash_.progress() : 0.0;
        chip(p, r, QStringLiteral("VTOL"), shortVtol(v), vtolColor(v), f);
    }

    // ---- armed -------------------------------------------------------------
    {
        const QRectF r = next(kArm);
        if (!s_.has_heartbeat) {
            chip(p, r, QStringLiteral("State"), QStringLiteral("—"), kMuted);
        } else if (s_.armed) {
            const QRectF box = r.adjusted(10, 12, -10, -10);
            p.fillRect(box, alpha(kRed, 0.18));
            p.setPen(QPen(kRed, 1));
            p.drawRect(box);
            drawText(p, box, Qt::AlignCenter, QStringLiteral("ARMED"), labelFont(12.5), kRed);
        } else {
            chip(p, r, QStringLiteral("State"), QStringLiteral("SAFE"), kGreen);
        }
    }

    // ---- altitude ----------------------------------------------------------
    {
        const QRectF r = next(kAlt);
        const QString v = s_.has_position ? QString::number(alt_.value, 'f', 1) : QStringLiteral("—");
        chip(p, r, QStringLiteral("Alt m"), v, kText);
    }

    // ---- speeds ------------------------------------------------------------
    {
        const QRectF r = next(kSpd);
        const QString v = s_.has_vfr_hud ? QStringLiteral("%1·%2").arg(as_.value, 0, 'f', 1).arg(gs_.value, 0, 'f', 1)
                                         : QStringLiteral("—");
        chip(p, r, QStringLiteral("AS · GS m/s"), v, kText);
    }

    // ---- battery -----------------------------------------------------------
    {
        const QRectF r = next(kBat);
        const int pct = s_.battery_pct >= 0 ? static_cast<int>(std::lround(batt_.value)) : -1;
        const QColor c = batteryColor(pct);
        drawText(p, QRectF(r.left() + 10, r.top() + 6, r.width() - 20, 13), Qt::AlignLeft | Qt::AlignVCenter,
                 QStringLiteral("Batt"), labelFont(9.5), kMuted);
        QString right = s_.has_battery ? QStringLiteral("%1V").arg(s_.battery_v, 0, 'f', 1) : QStringLiteral("—");
        drawText(p, QRectF(r.left() + 10, r.top() + 5, r.width() - 20, 15), Qt::AlignRight | Qt::AlignVCenter,
                 right, monoFont(12), kText);
        // segmented gauge
        const int segs = 10;
        const double gx = r.left() + 10, gy = r.top() + 25, gw = r.width() - 58, gh = 11;
        const double sw = (gw - (segs - 1) * 2) / segs;
        const int lit = pct < 0 ? 0 : static_cast<int>(std::ceil(pct / 10.0));
        for (int i = 0; i < segs; ++i) {
            const QRectF seg(gx + i * (sw + 2), gy, sw, gh);
            p.fillRect(seg, i < lit ? c : alpha(kMuted, 0.18));
        }
        drawText(p, QRectF(gx + gw + 4, gy - 4, 44, gh + 8), Qt::AlignRight | Qt::AlignVCenter,
                 pct < 0 ? QStringLiteral("—%") : QStringLiteral("%1%").arg(pct), monoFont(13, true), c);
    }

    // ---- GPS ---------------------------------------------------------------
    {
        const QRectF r = next(kGps);
        if (!s_.has_gps) {
            chip(p, r, QStringLiteral("GPS"), QStringLiteral("—"), kMuted);
        } else {
            const QColor c = gpsHealthy(s_) ? kGreen : (s_.gps_fix >= 2 ? kAmber : kRed);
            chip(p, r, QStringLiteral("GPS"),
                 QStringLiteral("%1·%2").arg(QLatin1String(fixName(s_.gps_fix))).arg(s_.gps_sats), c);
        }
    }

    // ---- systems -----------------------------------------------------------
    {
        const QRectF r = next(kSys);
        int bad = 0, warn = 0;
        for (const auto& it : checklist()) {
            bad += it.status == Check::Bad;
            warn += it.status == Check::Warn;
        }
        const QColor c = !alive_ ? kMuted : (bad ? kRed : (warn ? kAmber : kGreen));
        const QString text = !alive_ ? QStringLiteral("—") : (bad ? QStringLiteral("%1 FAULT").arg(bad)
                                                                       : (warn ? QStringLiteral("CHECK") : QStringLiteral("READY")));
        const QPointF dot(r.left() + 16, r.center().y() + 7);
        p.setPen(Qt::NoPen);
        p.setBrush(alpha(c, 0.25));
        p.drawEllipse(dot, 8, 8);
        p.setBrush(c);
        p.drawEllipse(dot, 4, 4);
        drawText(p, QRectF(r.left() + 30, r.top() + 6, r.width() - 34, 13), Qt::AlignLeft | Qt::AlignVCenter,
                 QStringLiteral("Systems"), labelFont(9.5), kMuted);
        drawText(p, QRectF(r.left() + 30, r.top() + 19, r.width() - 34, 24), Qt::AlignLeft | Qt::AlignVCenter,
                 text, labelFont(12.5), c);
        if (sys_hover_) {
            p.setPen(QPen(alpha(kCyan, 0.6), 1));
            p.setBrush(Qt::NoBrush);
            p.drawRect(r.adjusted(3, 6, -3, -6));
        }
        // map back to widget coordinates for hover testing
        sys_rect_ = p.transform().mapRect(r);
    }
}

}  // namespace gcs::ui
