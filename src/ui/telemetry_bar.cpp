#include "telemetry_bar.hpp"

#include "core/endurance.hpp"
#include "core/geo.hpp"

#include "theme.hpp"
#include "units.hpp"

#include <QFontMetricsF>
#include <QMouseEvent>
#include <QPainter>

#include <algorithm>
#include <cmath>

namespace gcs::ui {

using namespace theme;


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
    (void)parent;
    alt_.rate = as_.rate = gs_.rate = 9.0;
    batt_.rate = 4.0;
}

void TelemetryBar::setState(const VehicleState& s, Clock::time_point now) {
    s_ = s;
    now_ = now;
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

    update();
}

void TelemetryBar::setAdvisories(const std::vector<Advisory>& list) {
    adv_bad_ = adv_warn_ = 0;
    adv_critical_ = false;
    for (const auto& a : list) {
        adv_bad_ += a.severity >= Severity::Warning;
        adv_warn_ += a.severity == Severity::Caution;
        adv_critical_ = adv_critical_ || a.severity == Severity::Critical;
    }
}

void TelemetryBar::setEndurance(bool valid, double time_left_s, int level, Clock::time_point at) {
    endurance_valid_ = valid;
    endurance_s_ = time_left_s;
    endurance_level_ = level;
    endurance_at_ = at;
}

void TelemetryBar::setFlightStatus(const FlightStatus& f) {
    if (f.label != flight_.label && !flight_.label.empty() && !reducedMotion()) {
        mode_flash_.start(0.9);
    }
    flight_ = f;
}

void TelemetryBar::setAltitudes(std::optional<double> agl, bool agl_rangefinder, std::optional<double> afl) {
    agl_ = agl;
    agl_rangefinder_ = agl_rangefinder;
    afl_ = afl;
}

void TelemetryBar::setSystemsOpen(bool open) {
    sys_open_ = open;
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
    if (safety_down_) {
        safety_hold_ += dt;
        if (safety_hold_ >= kSafetyHold) {
            safety_down_ = false;
            safety_hold_ = 0.0;
            safety_flash_ = 1.0;
            safety_pending_ = 3.0;
            safety_sent_from_ = safetyState(s_);
            emit safetyToggleRequested(safety_sent_from_ == SafetyState::Live);
        }
        moving = true;
    }
    if (safety_pending_ > 0.0) {
        safety_pending_ = safetyState(s_) != safety_sent_from_ ? 0.0 : std::max(0.0, safety_pending_ - dt);
        moving = true;
    }
    if (safety_flash_ > 0.0) {
        safety_flash_ = std::max(0.0, safety_flash_ - dt * 2.0);
        moving = true;
    }
    if (gear_spin_ > 0.0) {
        gear_spin_ = std::max(0.0, gear_spin_ - dt * 2.5);
        moving = true;
    }
    if (moving || !alive_) {
        update();
    }
}

QString TelemetryBar::safetyLockReason() const {
    if (!alive_) {
        return QStringLiteral("No vehicle link");
    }
    if (safetyState(s_) == SafetyState::Unknown) {
        return QStringLiteral("The vehicle doesn't report a safety switch");
    }
    if (s_.armed) {
        return QStringLiteral("Can't change safety while armed: it would cut the pusher and servos too");
    }
    return {};
}

void TelemetryBar::mouseReleaseEvent(QMouseEvent*) {
    if (safety_down_ && safety_hold_ < kSafetyHold) {
        // released early: nothing happens (a hint, like the dock's hold buttons)
        emit safetyBlocked(QStringLiteral("Hold SAFETY for 1 s to change it"));
    }
    safety_down_ = false;
    safety_hold_ = 0.0;
    update();
}

void TelemetryBar::mousePressEvent(QMouseEvent* e) {
    if (safety_rect_.contains(e->position())) {
        const QString reason = safetyLockReason();
        if (!reason.isEmpty()) {
            emit safetyBlocked(reason);
            return;
        }
        safety_down_ = true;
        safety_hold_ = 0.0;
        return;
    }
    if (alt_rect_.contains(e->position())) {
        alt_ref_ = (alt_ref_ + 1) % 3;
        emit altRefChanged(alt_ref_);
        update();
        return;
    }
    if (gear_rect_.contains(e->position())) {
        gear_spin_ = 1.0;
        emit settingsRequested();
        return;
    }
    if (sys_rect_.contains(e->position()) || (endurance_valid_ && batt_rect_.contains(e->position()))) {
        emit systemsTapped();
    }
}

void TelemetryBar::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);

    // chip widths at scale 1
    const double kLink = 104, kFlight = 250, kEta = 112, kArm = 100, kSafe = 104, kAlt = 200, kSpd = 120, kBat = 156, kGps = 92,
                 kSys = 108, kGear = 60;
    const double widths[] = {kLink, kFlight, kEta, kArm, kSafe, kAlt, kSpd, kBat, kGps, kSys, kGear};
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

    // ---- flight: one plain state instead of mode + VTOL + lift rotors -------------
    {
        const QRectF r = next(kFlight);
        const double f = mode_flash_.active() ? 1.0 - mode_flash_.progress() : 0.0;
        QColor c = kMuted;
        switch (flight_.tone) {
        case Tone::Good: c = kGreen; break;
        case Tone::Busy: c = kCyan; break;
        case Tone::Warn: c = kAmber; break;
        case Tone::Bad: c = kRed; break;
        case Tone::Neutral: c = mix(kCyan, kText, 0.5); break;
        }
        if (flight_.assist && !reducedMotion()) {
            c = alpha(c, 0.55 + 0.45 * std::abs(std::sin(time_ * 5.0)));
        }
        QRectF rr = r;
        if (f > 0) {
            rr.translate(0, -3.0 * f);
        }
        // small print: the autopilot's own mode name, for whoever needs it
        const QString mode = s_.has_heartbeat ? QString::fromStdString(s_.mode_name) : QString();
        drawText(p, QRectF(rr.left() + 12, rr.top() + 6, rr.width() - 44, 13), Qt::AlignLeft | Qt::AlignVCenter,
                 mode.isEmpty() ? QStringLiteral("Flight") : QStringLiteral("Flight · %1").arg(mode), labelFont(9.5),
                 kMuted);
        // the state, shrunk to fit
        const QString label = QString::fromStdString(flight_.label);
        QFont vf = monoFont(15, true);
        const double avail = rr.width() - 46;
        for (double px = 15; px > 10 && QFontMetricsF(vf).horizontalAdvance(label) > avail; px -= 0.5) {
            vf = monoFont(px, true);
        }
        if (f > 0) {
            p.fillRect(rr.adjusted(4, 6, -4, -6), alpha(c, 0.18 * f));
        }
        drawText(p, QRectF(rr.left() + 12, rr.top() + 19, avail, 24), Qt::AlignLeft | Qt::AlignVCenter, label, vf, c);
        // lift-rotor glyph: lit while the lift rotors spin (takeoff / landing; amber = assist)
        const QPointF g(rr.right() - 20, rr.center().y() + 2);
        const QColor gc = flight_.assist ? kAmber : (flight_.rotors ? kCyan : alpha(kMuted, 0.45));
        p.setPen(QPen(gc, 1.4));
        p.setBrush(Qt::NoBrush);
        const double spin = flight_.rotors && !reducedMotion() ? time_ * 9.0 : 0.0;
        for (int k = 0; k < 4; ++k) {
            const double ang = spin + k * geo::kPi / 2.0 + geo::kPi / 4.0;
            const QPointF hub = g + QPointF(std::cos(geo::kPi / 4.0 + k * geo::kPi / 2.0) * 6.0,
                                            std::sin(geo::kPi / 4.0 + k * geo::kPi / 2.0) * 6.0);
            p.drawLine(hub - QPointF(std::cos(ang) * 4.0, std::sin(ang) * 4.0),
                       hub + QPointF(std::cos(ang) * 4.0, std::sin(ang) * 4.0));
        }
        p.drawEllipse(g, 1.5, 1.5);
    }

    // ---- ETA to what it is flying to ------------------------------------------------
    {
        const QRectF r = next(kEta);
        const QString name = nav_.valid ? QString::fromStdString(nav_.name) : QString();
        chip(p, r, name.isEmpty() ? QStringLiteral("ETA") : QStringLiteral("ETA → %1").arg(name),
             nav_.valid ? units::durationText(nav_.eta_s) : QStringLiteral("—"), nav_.valid ? kCyan : kMuted);
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
            chip(p, r, QStringLiteral("State"), QStringLiteral("DISARMED"), kGreen);
        }
    }

    // ---- safety switch (hold 1 s to toggle) ------------------------------------
    {
        const QRectF r = next(kSafe);
        safety_rect_ = p.transform().mapRect(r);
        const SafetyState st = safetyState(s_);
        QString text = QStringLiteral("—");
        QColor c = kMuted;
        if (alive_ && st == SafetyState::Safe) {
            text = QStringLiteral("ON");  // safety on: motors cut
            c = kAmber;
        } else if (alive_ && st == SafetyState::Live) {
            text = QStringLiteral("OFF");  // safety off: motors live
            c = kRed;
        }
        if (safety_pending_ > 0.0) {
            text = QStringLiteral("…");
        }
        const QRectF box = r.adjusted(8, 5, -8, -5);
        p.setPen(QPen(alpha(c, 0.6 + 0.4 * safety_flash_), 1));
        p.setBrush(alpha(c, 0.08 + 0.2 * safety_flash_));
        p.drawRect(box);
        // charge bar while held
        if (safety_down_) {
            const double f = std::clamp(safety_hold_ / kSafetyHold, 0.0, 1.0);
            p.setPen(Qt::NoPen);
            p.setBrush(alpha(c == kMuted ? kCyan : (st == SafetyState::Live ? kAmber : kRed), 0.45));
            p.drawRect(QRectF(box.left(), box.bottom() - 4, box.width() * f, 4));
        }
        drawText(p, QRectF(box.left(), box.top() + 1, box.width(), 13), Qt::AlignHCenter | Qt::AlignVCenter,
                 QStringLiteral("Safety"), labelFont(9.5), kMuted);
        // padlock: closed when safe
        const QPointF lc(box.left() + 18, box.top() + 25);
        p.setPen(QPen(c, 1.6));
        p.setBrush(Qt::NoBrush);
        p.drawRect(QRectF(lc.x() - 6, lc.y() - 2, 12, 9));
        QPainterPath shackle;
        const bool closed = st != SafetyState::Live;
        shackle.moveTo(lc.x() - 4, lc.y() - 2);
        shackle.lineTo(lc.x() - 4, lc.y() - 6);
        shackle.arcTo(QRectF(lc.x() - 4, lc.y() - 10, 8, 8), 180, -180);
        shackle.lineTo(lc.x() + 4, closed ? lc.y() - 2 : lc.y() - 5);
        p.drawPath(shackle);
        drawText(p, QRectF(box.left() + 28, box.top() + 14, box.width() - 30, 22), Qt::AlignLeft | Qt::AlignVCenter,
                 text, labelFont(13), c);
    }

    // ---- altitude: tap to switch the reference (ASL / AGL / AFL) --------------
    {
        const QRectF r = next(kAlt);
        alt_rect_ = p.transform().mapRect(r);
        static const char* kRefs[] = {"ASL", "AGL", "AFL"};
        QString label = QStringLiteral("Alt %1 %2").arg(QLatin1String(kRefs[std::clamp(alt_ref_, 0, 2)]), units::altUnitLabel());
        std::optional<double> value;
        if (s_.has_position) {
            if (alt_ref_ == 0) {
                value = s_.alt_amsl;
            } else if (alt_ref_ == 1) {
                value = agl_;
                if (agl_ && !agl_rangefinder_) {
                    label += QStringLiteral(" · terrain");  // not measured: from the elevation map
                }
            } else {
                value = afl_;
            }
        }
        QString v = value ? units::altText(*value, false) : QStringLiteral("—");
        // commanded altitude when the autopilot is navigating to one (same reference)
        if (value && s_.has_nav && s_.armed && std::abs(s_.nav_alt_error) >= 1.5) {
            v += QStringLiteral("→%1").arg(units::altText(*value + s_.nav_alt_error, false));
        }
        chip(p, r, label, v, kText);
        // the chip is a button: a small cycle mark, bottom right
        p.setPen(QPen(alpha(kCyan, 0.6), 1.2));
        p.drawArc(QRectF(r.right() - 18, r.bottom() - 18, 10, 10), 30 * 16, 300 * 16);
    }

    // ---- speeds: whole numbers ----------------------------------------------------------
    {
        const QRectF r = next(kSpd);
        const QString v = s_.has_vfr_hud ? QStringLiteral("%1·%2").arg(units::speedText(as_.value, 0, false),
                                                                        units::speedText(gs_.value, 0, false))
                                         : QStringLiteral("—");
        chip(p, r, QStringLiteral("AS · GS %1").arg(units::speedUnitLabel()), v, kText);
    }

    // ---- battery -----------------------------------------------------------
    {
        const QRectF r = next(kBat);
        const int pct = s_.battery_pct >= 0 ? static_cast<int>(std::lround(batt_.value)) : -1;
        const QColor c = batteryColor(pct);
        drawText(p, QRectF(r.left() + 10, r.top() + 6, r.width() - 20, 13), Qt::AlignLeft | Qt::AlignVCenter,
                 QStringLiteral("Batt"), labelFont(9.5), kMuted);
        batt_rect_ = p.transform().mapRect(r);
        if (endurance_valid_ && isAirborne(s_)) {
            // in flight: the flying time left before it must head back (it
            // already keeps the return, the landing and the reserve)
            const double left = endurance_s_ - std::chrono::duration<double>(now_ - endurance_at_).count();
            const bool land_now = endurance_level_ >= 2 || left <= 0.0;
            const QColor tc = land_now ? kRed : (endurance_level_ == 1 ? kAmber : kText);
            const QString t = land_now ? QStringLiteral("LAND NOW")
                                       : QStringLiteral("%1 left").arg(QString::fromStdString(endurance::clockText(left)));
            p.save();
            if (land_now && !reducedMotion()) {
                p.setOpacity(0.55 + 0.45 * std::abs(std::sin(time_ * 3.0)));
            }
            drawText(p, QRectF(r.left() + 10, r.top() + 5, r.width() - 20, 15), Qt::AlignRight | Qt::AlignVCenter, t,
                     monoFont(12, true), tc);
            p.restore();
        } else {
            QString right = s_.has_battery ? QStringLiteral("%1V").arg(s_.battery_v, 0, 'f', 1) : QStringLiteral("—");
            drawText(p, QRectF(r.left() + 10, r.top() + 5, r.width() - 20, 15), Qt::AlignRight | Qt::AlignVCenter,
                     right, monoFont(12), kText);
        }
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
        const int bad = adv_bad_, warn = adv_warn_;
        const QColor c = !alive_ ? kMuted : (bad ? kRed : (warn ? kAmber : kGreen));
        const QString text = !alive_ ? QStringLiteral("—")
                                     : (bad ? QStringLiteral("%1 FAULT").arg(bad)
                                            : (warn ? QStringLiteral("%1 CHECK").arg(warn) : QStringLiteral("READY")));
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
        if (adv_critical_ && alive_ && !reducedMotion()) {
            const double pulse = 0.5 + 0.5 * std::abs(std::sin(time_ * 3.0));
            p.setPen(QPen(alpha(kRed, 0.8 * pulse), 1.5));
            p.setBrush(alpha(kRed, 0.10 * pulse));
            p.drawRect(r.adjusted(3, 6, -3, -6));
        }
        if (sys_open_) {
            p.setPen(QPen(alpha(kCyan, 0.6), 1));
            p.setBrush(Qt::NoBrush);
            p.drawRect(r.adjusted(3, 6, -3, -6));
        }
        // map back to widget coordinates for hover testing
        sys_rect_ = p.transform().mapRect(r);
    }

    // ---- settings ------------------------------------------------------------
    {
        const QRectF r = next(kGear);
        const QPointF c = r.center();
        p.save();
        p.translate(c);
        p.rotate(easeOutCubic(1.0 - gear_spin_) * 90.0 * (gear_spin_ > 0.0 ? 1.0 : 0.0));
        // cog: ring + 8 teeth
        p.setPen(QPen(kCyan, 2.2));
        p.setBrush(Qt::NoBrush);
        p.drawEllipse(QPointF(0, 0), 8.5, 8.5);
        p.drawEllipse(QPointF(0, 0), 3.5, 3.5);
        for (int i = 0; i < 8; ++i) {
            p.save();
            p.rotate(i * 45.0);
            p.drawLine(QPointF(0, -9), QPointF(0, -13.5));
            p.restore();
        }
        p.restore();
        // the whole bar height is the hit target (>= 46 px)
        gear_rect_ = p.transform().mapRect(r.adjusted(-2, -2, 2, 2));
    }
}

}  // namespace gcs::ui
