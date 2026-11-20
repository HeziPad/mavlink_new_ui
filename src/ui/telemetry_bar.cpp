#include "telemetry_bar.hpp"

#include "core/endurance.hpp"
#include "core/geo.hpp"
#include "core/speed_mode.hpp"

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

void TelemetryBar::setAltitudes(std::optional<double> agl, std::optional<double> afl) {
    agl_ = agl;
    afl_ = afl;
}

void TelemetryBar::setSpeed(int target, int mode, bool airborne) {
    speed_target_ = target;
    speed_mode_ = mode;
    speed_airborne_ = airborne;
}

void TelemetryBar::setSystemsOpen(bool open) {
    sys_open_ = open;
    update();
}

void TelemetryBar::tick(double dt) {
    time_ += dt;
    bool moving = false;
    moving |= batt_.step(dt);
    moving |= mode_flash_.step(dt);
    if (gear_spin_ > 0.0) {
        gear_spin_ = std::max(0.0, gear_spin_ - dt * 2.5);
        moving = true;
    }
    if (spd_flash_ > 0.0) {
        spd_flash_ = std::max(0.0, spd_flash_ - dt * 3.0);
        moving = true;
    }
    if (moving || !alive_) {
        update();
    }
}

void TelemetryBar::mousePressEvent(QMouseEvent* e) {
    if (spd_rect_.contains(e->position())) {
        spd_flash_ = 1.0;
        emit speedTapped();
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

namespace {
// chip widths at scale 1
constexpr double kLink = 64, kBat = 128, kTime = 104, kFlight = 230, kEta = 112, kAlt = 172, kSpd = 152, kSys = 108,
                 kGear = 60;
constexpr double kChipsTotal = kLink + kBat + kTime + kFlight + kEta + kAlt + kSpd + kSys + kGear;

// "0:23", "1:05": hours and minutes (rounded down)
QString hmText(double seconds) {
    const long m = seconds <= 0.0 ? 0 : static_cast<long>(seconds / 60.0);
    return QStringLiteral("%1:%2").arg(m / 60).arg(m % 60, 2, 10, QLatin1Char('0'));
}
constexpr double kStripPad = 14;
constexpr double kStripH = 46;
constexpr double kBarH = 52;  // strip + 2 px above + room for the glow below
}  // namespace

double TelemetryBar::effectiveScale(double w) const {
    return std::max(0.1, std::min(uiScale(), (w - 4.0) / (kChipsTotal + 2 * kStripPad)));
}

int TelemetryBar::heightFor(int w) const {
    return static_cast<int>(std::ceil(kBarH * std::max(1.0, effectiveScale(w))));
}

void TelemetryBar::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);

    const double total = kChipsTotal;
    const double pad = kStripPad;
    const double scale = effectiveScale();
    const double h = kStripH;

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

    // ---- link: bars only ------------------------------------------------------
    {
        const QRectF r = next(kLink);
        const bool error = !s_.error.empty();
        const bool waiting = !error && !s_.has_heartbeat;
        const bool lost = error || (!waiting && hb_age_ >= 10.0);
        int bars = 0;
        QColor c = kMuted;
        if (!waiting && !lost) {
            if (hb_age_ < 3.0) {
                bars = msg_rate_ > 20 ? 4 : (msg_rate_ > 5 ? 3 : 2);
                c = bars >= 3 ? kGreen : kAmber;
            } else {
                bars = 1;  // stale
                c = kAmber;
            }
        }
        const double bx = r.center().x() - 14, by = r.top() + 35;
        for (int i = 0; i < 4; ++i) {
            const double bh = 7 + i * 5;
            const QRectF b(bx + i * 7.5, by - bh, 5, bh);
            QColor bc = i < bars ? c : alpha(kMuted, lost ? 0.18 : 0.28);
            if (waiting && !reducedMotion()) {
                // looking for the vehicle: a slow sweep along the bars
                const double ph = std::fmod(time_ * 1.2, 1.6) * 4.0 - i;
                bc = alpha(kMuted, 0.25 + 0.45 * std::max(0.0, 1.0 - std::abs(ph - 1.0)));
            }
            p.fillRect(b, bc);
        }
        if (lost) {
            // link lost: a red cross over the bars
            const double pulse = reducedMotion() ? 1.0 : 0.55 + 0.45 * std::abs(std::sin(time_ * 4.0));
            const QPointF xc(bx + 13, by - 11);
            p.setPen(QPen(alpha(kRed, pulse), 3.0, Qt::SolidLine, Qt::RoundCap));
            p.drawLine(xc + QPointF(-9, -9), xc + QPointF(9, 9));
            p.drawLine(xc + QPointF(-9, 9), xc + QPointF(9, -9));
        }
    }

    // ---- battery: gauge and percent ---------------------------------------------------
    {
        const QRectF r = next(kBat);
        batt_rect_ = p.transform().mapRect(r);
        const int pct = s_.battery_pct >= 0 ? static_cast<int>(std::lround(batt_.value)) : -1;
        const QColor c = batteryColor(pct);
        const int segs = 5;
        const double gx = r.left() + 12, gh = 18, gy = r.center().y() - gh / 2.0, gw = 58;
        // a battery outline with a nub, five cells inside
        p.setPen(QPen(alpha(c == kMuted ? kMuted : c, 0.8), 1.4));
        p.setBrush(Qt::NoBrush);
        p.drawRect(QRectF(gx - 3, gy - 3, gw + 6, gh + 6));
        p.fillRect(QRectF(gx + gw + 3, gy + gh / 2.0 - 4, 3, 8), alpha(c == kMuted ? kMuted : c, 0.8));
        const double sw = (gw - (segs - 1) * 2) / segs;
        const int lit = pct < 0 ? 0 : static_cast<int>(std::ceil(pct / 20.0));
        for (int i = 0; i < segs; ++i) {
            p.fillRect(QRectF(gx + i * (sw + 2), gy, sw, gh), i < lit ? c : alpha(kMuted, 0.18));
        }
        drawText(p, QRectF(gx + gw + 8, r.top(), r.right() - (gx + gw + 8) - 6, r.height()),
                 Qt::AlignRight | Qt::AlignVCenter, pct < 0 ? QStringLiteral("—") : QStringLiteral("%1%").arg(pct),
                 monoFont(14, true), c);
    }

    // ---- time left (battery, core/endurance) -------------------------------------------
    {
        const QRectF r = next(kTime);
        if (endurance_valid_ && isAirborne(s_)) {
            // the flying time left before it must head back (it already keeps
            // the return, the landing and the reserve)
            const double left = endurance_s_ - std::chrono::duration<double>(now_ - endurance_at_).count();
            const bool land_now = endurance_level_ >= 2 || left <= 0.0;
            const QColor tc = land_now ? kRed : (endurance_level_ == 1 ? kAmber : kText);
            p.save();
            if (land_now && !reducedMotion()) {
                p.setOpacity(0.55 + 0.45 * std::abs(std::sin(time_ * 3.0)));
            }
            if (land_now) {
                drawText(p, QRectF(r.left(), r.top() + 6, r.width(), 13), Qt::AlignHCenter | Qt::AlignVCenter,
                         QStringLiteral("Time left"), labelFont(9.5), kMuted);
                drawText(p, QRectF(r.left(), r.top() + 19, r.width(), 24), Qt::AlignHCenter | Qt::AlignVCenter,
                         QStringLiteral("LAND NOW"), labelFont(12.5), tc);
            } else {
                chip(p, r, QStringLiteral("Time left"), hmText(left), tc);
            }
            p.restore();
        } else {
            chip(p, r, QStringLiteral("Time left"), QStringLiteral("—"), kMuted);
        }
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
        drawText(p, QRectF(rr.left() + 12, rr.top() + 6, rr.width() - 44, 13), Qt::AlignLeft | Qt::AlignVCenter,
                 QStringLiteral("Flight mode"), labelFont(9.5), kMuted);
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
             !nav_.valid     ? QStringLiteral("—")
             : nav_.arrived  ? QStringLiteral("HOLDING")  // circling it: no time to go
                             : units::durationText(nav_.eta_s),
             nav_.valid ? (nav_.arrived ? kGreen : kCyan) : kMuted);
    }

    // ---- altitude: tap to switch the reference (ASL / AGL / AFL) --------------
    {
        const QRectF r = next(kAlt);
        alt_rect_ = p.transform().mapRect(r);
        static const char* kRefs[] = {"ASL", "AGL", "AFL"};
        const int ref = std::clamp(alt_ref_, 0, 2);
        const QString label = QStringLiteral("Alt %1 %2").arg(QLatin1String(kRefs[ref]), units::altUnitLabel());
        std::optional<double> value;
        if (s_.has_position) {
            value = ref == 0 ? std::optional<double>(s_.alt_amsl) : (ref == 1 ? agl_ : afl_);
        }
        // a new reference or unit starts the sticky rounding afresh
        const int key = ref * 2 + (units::feet() ? 1 : 0);
        if (key != alt_shown_ref_) {
            alt_shown_ref_ = key;
            alt_shown_.reset();
            cmd_shown_.reset();
        }
        QString v = QStringLiteral("—");
        if (value) {
            const double shown = alt_shown_.update(units::altToDisplay(*value));
            v = QString::number(std::lround(shown));
            // what the GCS commanded, in the same reference, when it differs
            if (cmd_alt_ && s_.armed) {
                const double cmd = cmd_shown_.update(units::altToDisplay(*cmd_alt_ + (*value - s_.alt_rel)));
                if (std::lround(cmd) != std::lround(shown)) {
                    v += QStringLiteral("→%1").arg(std::lround(cmd));
                }
            } else {
                cmd_shown_.reset();
            }
        }
        chip(p, r, label, v, kText);
        // the chip is a button: a small cycle mark, bottom right
        p.setPen(QPen(alpha(kCyan, 0.6), 1.2));
        p.drawArc(QRectF(r.right() - 18, r.bottom() - 18, 10, 10), 30 * 16, 300 * 16);
    }

    // ---- speed: what is commanded, in words, and the speed mode ------------------------
    {
        const QRectF r = next(kSpd);
        spd_rect_ = p.transform().mapRect(r);
        const int mode = std::clamp(speed_mode_, 0, 2);
        static const char* kModes[] = {"ECO", "NORMAL", "FAST"};
        const double sf = spd_flash_;
        if (sf > 0.0) {
            p.fillRect(r.adjusted(2, 4, -2, -4), alpha(kCyan, 0.18 * sf));
        }
        // three chevrons (one lit per step of the mode) and its name, centred as one
        const QString mname = QString::fromLatin1(kModes[mode]);
        const QFont mf = labelFont(9.5);
        const double chev_w = 2 * 6 + 4, gap = 6;
        const double group = chev_w + gap + QFontMetricsF(mf).horizontalAdvance(mname);
        const double cy = r.top() + 12.5;
        double cx = r.center().x() - group / 2.0;
        p.setBrush(Qt::NoBrush);
        for (int i = 0; i < 3; ++i) {
            p.setPen(QPen(i <= mode ? kCyan : alpha(kMuted, 0.35), 1.8, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
            p.drawPolyline(QPolygonF({QPointF(cx, cy - 4), QPointF(cx + 4, cy), QPointF(cx, cy + 4)}));
            cx += 6;
        }
        drawText(p, QRectF(cx - 6 + 4 + gap, r.top() + 6, group, 13), Qt::AlignLeft | Qt::AlignVCenter, mname, mf, kCyan);
        // the chip opens the speed mode: the same quiet mark as the ALT chip's, a caret
        p.setPen(QPen(alpha(kCyan, 0.6), 1.2, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
        p.drawPolyline(QPolygonF({QPointF(r.right() - 17, r.bottom() - 14), QPointF(r.right() - 13, r.bottom() - 10),
                                  QPointF(r.right() - 9, r.bottom() - 14)}));
        QString word = QStringLiteral("—");
        QColor wc = kMuted;
        if (speed_airborne_) {
            const auto t = static_cast<speed::Target>(speed_target_);
            word = t == speed::Target::Default ? QStringLiteral("CRUISE")
                                                : QString::fromUtf8(speed::targetName(t));
            wc = t == speed::Target::Max ? kAmber : kText;
        }
        drawText(p, QRectF(r.left(), r.top() + 19, r.width(), 24), Qt::AlignHCenter | Qt::AlignVCenter, word,
                 monoFont(15, true), wc);
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
