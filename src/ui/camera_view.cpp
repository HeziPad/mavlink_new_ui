#include "camera_view.hpp"

#include "core/geo.hpp"
#include "theme.hpp"
#include "video_receiver.hpp"

#include <QFontMetricsF>
#include <QLinearGradient>
#include <QMouseEvent>
#include <QPainter>
#include <QRadialGradient>

#include <cmath>

namespace gcs::ui {

using namespace theme;

namespace {
constexpr double kNoVideoAfter = 2.0;

double tanDeg(double d) {
    return std::tan(d * geo::kDegToRad);
}
}  // namespace

CameraView::CameraView(VideoReceiver* video, QWidget* parent) : HudView(parent), video_(video) {
    // targets move continuously (predicted every frame), so these only take
    // the edge off; they are no longer what makes the motion smooth
    roll_.rate = pitch_.rate = cam_roll_.rate = cam_el_.rate = pan_rel_.rate = tilt_rel_.rate = 25.0;
    yaw_.rate = cam_az_.rate = 25.0;
    as_.rate = 10.0;
    alt_.rate = 20.0;
    cam_el_.set(mount_pitch_);
    tilt_rel_.set(mount_pitch_);
}

bool CameraView::live() const {
    return frame_age_ < kNoVideoAfter;
}

QString CameraView::pipTag() const {
    return live() ? QStringLiteral("Feed · Live") : QStringLiteral("Feed · Sim");
}

QColor CameraView::pipTagColor() const {
    return live() ? kGreen : kAmber;
}

void CameraView::setState(const VehicleState& s) {
    s_ = s;
    if (s.has_vfr_hud) {
        as_.set(s.airspeed);
    }
}

void CameraView::setMotion(const geo::Attitude& aircraft, const CameraLos& los) {
    roll_.set(aircraft.roll_deg);
    pitch_.set(aircraft.pitch_deg);
    yaw_.set(geo::wrap360(aircraft.yaw_deg));
    cam_roll_.set(los.earth.roll_deg);
    cam_el_.set(los.earth.pitch_deg);
    cam_az_.set(geo::wrap360(los.earth.yaw_deg));
    pan_rel_.set(los.pan_rel);
    tilt_rel_.set(los.tilt_rel);
    gimbal_ = los.gimbal;
}

void CameraView::setCamera(double hfov_deg, double mount_pitch_deg) {
    hfov_ = hfov_deg;
    mount_pitch_ = mount_pitch_deg;
    update();
}

void CameraView::setGuideMode(bool on) {
    guide_ = on;
    setCursor(compact_ ? Qt::PointingHandCursor : (on ? Qt::CrossCursor : Qt::ArrowCursor));
    update();
}

void CameraView::tick(double dt) {
    time_ += dt;
    for (Tween* t : {&roll_, &pitch_, &cam_roll_, &cam_el_, &pan_rel_, &tilt_rel_}) {
        t->step(dt);
    }
    yaw_.step(dt);
    cam_az_.step(dt);
    as_.step(dt);
    alt_.step(dt);
    travelled_ += s_.groundspeed * dt;
    if (click_t_ >= 0.0) {
        click_t_ += dt;
        if (click_t_ > 1.2) {
            click_t_ = -1.0;
        }
    }
    if (video_) {
        video_->takeFrame(frame_, frame_seq_);
        frame_age_ = video_->frameAge();
    }
    if (isVisible()) {
        update();
    }
}

double CameraView::focal() const {
    return (width() / 2.0) / tanDeg(hfov_ / 2.0);
}

// A point at `elevation` above the horizon, in the frame rotated by the image
// roll and centred on the boresight (whose elevation is the LOS elevation).
double CameraView::elevationToY(double elevation_deg) const {
    const double rel = std::clamp(elevation_deg - cam_el_.value, -89.0, 89.0);
    return -focal() * tanDeg(rel);
}

void CameraView::mousePressEvent(QMouseEvent* e) {
    if (compact_ || !guide_ || e->button() != Qt::LeftButton) {
        return;
    }
    click_ = e->position();
    click_t_ = 0.0;
    emit groundClicked(click_.x(), click_.y(), width(), height());
}

void CameraView::paintEvent(QPaintEvent*) {
    QPainter p(this);
    beginView(p);
    const QRectF r = rect();
    p.fillRect(r, kBg);

    if (live() && !frame_.isNull()) {
        const QSize fs = frame_.size().scaled(size(), Qt::KeepAspectRatio);
        if (scaled_seq_ != frame_seq_ || scaled_.size() != fs) {
            scaled_ = frame_.scaled(fs, Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
            scaled_seq_ = frame_seq_;
        }
        p.drawImage(QPointF(r.center().x() - fs.width() / 2.0, r.center().y() - fs.height() / 2.0), scaled_);
    } else {
        paintSynthetic(p);
    }

    // vignette + scanlines make it read as a sensor feed
    QRadialGradient vg(r.center(), std::hypot(r.width(), r.height()) / 2.0);
    vg.setColorAt(0.55, QColor(0, 0, 0, 0));
    vg.setColorAt(1.0, QColor(0, 0, 0, 150));
    p.fillRect(r, vg);
    if (!reducedMotion() && !compact_) {
        drawScanlines(p, r, 0.6);
    }

    if (live() && !compact_) {
        // Over real video the thin green symbology vanishes on bright scenes:
        // render it once, then stamp a dark copy at +-1 px underneath as an outline.
        const qreal dpr = devicePixelRatioF();
        QImage hud((QSizeF(size()) * dpr).toSize(), QImage::Format_ARGB32_Premultiplied);
        hud.setDevicePixelRatio(dpr);
        hud.fill(Qt::transparent);
        {
            QPainter hp(&hud);
            hp.setRenderHint(QPainter::Antialiasing);
            paintHud(hp);
        }
        QImage shadow = hud;
        {
            QPainter sp(&shadow);
            sp.setCompositionMode(QPainter::CompositionMode_SourceIn);
            sp.fillRect(QRectF(QPointF(0, 0), QSizeF(size())), QColor(0, 0, 0, 210));
        }
        for (const QPointF off : {QPointF(-1, 0), QPointF(1, 0), QPointF(0, -1), QPointF(0, 1)}) {
            p.drawImage(off, shadow);
        }
        p.drawImage(QPointF(0, 0), hud);
    } else {
        paintHud(p);
    }
    finishView(p);
}

void CameraView::paintSynthetic(QPainter& p) {
    const QPointF c = rect().center();
    const double diag = std::hypot(width(), height());
    const double hy = elevationToY(0.0);
    const double f = focal();
    const double h = std::max(2.0, s_.has_position ? s_.alt_rel + 1.5 : 1.5);

    p.save();
    p.translate(c);
    p.rotate(-cam_roll_.value);

    // sky
    QLinearGradient sky(0, hy - diag, 0, hy);
    sky.setColorAt(0.0, QColor(4, 8, 14));
    sky.setColorAt(0.75, QColor(10, 26, 38));
    sky.setColorAt(1.0, QColor(24, 58, 72));
    p.fillRect(QRectF(-diag, hy - diag * 2, diag * 2, diag * 2), sky);
    // ground
    QLinearGradient ground(0, hy, 0, hy + diag);
    ground.setColorAt(0.0, QColor(10, 22, 26));
    ground.setColorAt(1.0, QColor(4, 9, 12));
    p.fillRect(QRectF(-diag, hy, diag * 2, diag * 2), ground);

    // perspective grid: lines of constant along-track distance scroll with
    // ground speed, radial lines converge on the vanishing point
    p.setRenderHint(QPainter::Antialiasing);
    // grid cell scales with height so there is texture both parked and at altitude
    const double spacing = std::clamp(h * 0.5, 1.5, 40.0);
    const double phase = std::fmod(travelled_, spacing);
    for (int k = 1; k < 80; ++k) {
        const double dist = k * spacing - phase;
        if (dist < 1.0) {
            continue;
        }
        const double elev = -std::atan2(h, dist) * geo::kRadToDeg;
        const double y = elevationToY(elev);
        const double a = std::clamp(0.35 * (1.0 - dist / (spacing * 80.0)), 0.03, 0.35);
        p.setPen(QPen(alpha(kCyan, a), 1.0));
        p.drawLine(QPointF(-diag, y), QPointF(diag, y));
    }
    for (int k = -30; k <= 30; ++k) {
        const double lateral = k * spacing;
        // ground point (forward d, lateral L) projects to x = f * L / d
        const double near_d = 5.0, far_d = 4000.0;
        const QPointF a(f * lateral / near_d, elevationToY(-std::atan2(h, near_d) * geo::kRadToDeg));
        const QPointF b(f * lateral / far_d, elevationToY(-std::atan2(h, far_d) * geo::kRadToDeg));
        p.setPen(QPen(alpha(kCyan, k == 0 ? 0.3 : 0.14), 1.0));
        p.drawLine(a, b);
    }
    // horizon glow
    p.setPen(QPen(alpha(kCyan, 0.18), 6));
    p.drawLine(QPointF(-diag, hy), QPointF(diag, hy));
    p.restore();

    if (!compact_) {
        const QString text = video_ && VideoReceiver::compiledIn()
                                 ? QStringLiteral("NO VIDEO · udp:%1").arg(video_->port())
                                 : QStringLiteral("NO VIDEO · built without GStreamer");
        const QFont fnt = labelFont(11);
        const double w = QFontMetricsF(fnt).horizontalAdvance(text) + 26;
        const QRectF box(16, 74, w, 24);
        const double blink = reducedMotion() ? 1.0 : 0.6 + 0.4 * std::abs(std::sin(time_ * 2.0));
        drawGlass(p, chamferPath(box, 6), kAmber, 0.7 * blink, kGlassDeep);
        drawText(p, box, Qt::AlignCenter, text, fnt, alpha(kAmber, blink));
    }
}

void CameraView::paintHud(QPainter& p) {
    const QRectF r = rect();
    const QPointF c = r.center();
    const QColor hud = kGreen;
    const double f = focal();

    // ---- horizon + ladder (rotated with roll) -------------------------------
    p.save();
    p.setClipRect(QRectF(c.x() - r.width() * 0.32, c.y() - r.height() * 0.30, r.width() * 0.64, r.height() * 0.57));
    p.translate(c);
    p.rotate(-cam_roll_.value);
    const double hy = elevationToY(0.0);
    p.setPen(QPen(alpha(hud, 0.9), compact_ ? 1.2 : 1.6));
    p.drawLine(QPointF(-r.width() * 0.4, hy), QPointF(-60, hy));
    p.drawLine(QPointF(60, hy), QPointF(r.width() * 0.4, hy));

    if (!compact_) {
        const QFont lf = monoFont(11);
        for (int e = -80; e <= 80; e += 5) {
            if (e == 0) {
                continue;
            }
            const double y = elevationToY(e);
            if (std::abs(y) > r.height() * 0.45) {
                continue;
            }
            const bool major = e % 10 == 0;
            const double half = major ? 70 : 34;
            QPen pen(alpha(hud, major ? 0.85 : 0.5), 1.3);
            if (e < 0) {
                pen.setStyle(Qt::DashLine);  // dashed below the horizon, like a real HUD
            }
            p.setPen(pen);
            const double tick = e > 0 ? 7 : -7;
            p.drawLine(QPointF(-half, y), QPointF(-half * 0.35, y));
            p.drawLine(QPointF(half * 0.35, y), QPointF(half, y));
            p.drawLine(QPointF(-half, y), QPointF(-half, y + tick));
            p.drawLine(QPointF(half, y), QPointF(half, y + tick));
            if (major) {
                drawText(p, QRectF(half + 4, y - 8, 30, 16), Qt::AlignLeft | Qt::AlignVCenter, QString::number(e), lf,
                         alpha(hud, 0.85));
                drawText(p, QRectF(-half - 34, y - 8, 30, 16), Qt::AlignRight | Qt::AlignVCenter, QString::number(e),
                         lf, alpha(hud, 0.85));
            }
        }

        // flight path marker: where the aircraft is actually going
        if (s_.has_position && s_.groundspeed > 2.0) {
            const double track = std::atan2(s_.ve, s_.vn) * geo::kRadToDeg;
            const double drift = std::clamp(geo::wrap180(track - cam_az_.value), -40.0, 40.0);
            const double gamma = std::atan2(-s_.vd, std::hypot(s_.vn, s_.ve)) * geo::kRadToDeg;
            const QPointF fpm(f * tanDeg(drift), elevationToY(gamma));
            p.setPen(QPen(hud, 1.6));
            p.setBrush(Qt::NoBrush);
            p.drawEllipse(fpm, 7, 7);
            p.drawLine(fpm + QPointF(-18, 0), fpm + QPointF(-7, 0));
            p.drawLine(fpm + QPointF(7, 0), fpm + QPointF(18, 0));
            p.drawLine(fpm + QPointF(0, -7), fpm + QPointF(0, -14));
        }
    }
    p.restore();

    // ---- boresight crosshair ------------------------------------------------
    p.setPen(QPen(hud, 1.6));
    const double g = compact_ ? 4 : 8, l = compact_ ? 10 : 22;
    p.drawLine(QPointF(c.x() - l, c.y()), QPointF(c.x() - g, c.y()));
    p.drawLine(QPointF(c.x() + g, c.y()), QPointF(c.x() + l, c.y()));
    p.drawLine(QPointF(c.x(), c.y() - l), QPointF(c.x(), c.y() - g));
    p.drawLine(QPointF(c.x(), c.y() + g), QPointF(c.x(), c.y() + l));
    p.drawPoint(c);

    if (compact_) {
        return;
    }

    // bank indicator arc
    {
        p.save();
        p.translate(c.x(), c.y());
        const double rad = std::min(r.width(), r.height()) * 0.30;
        p.setPen(QPen(alpha(hud, 0.6), 1.2));
        p.drawArc(QRectF(-rad, -rad, 2 * rad, 2 * rad), (90 - 45) * 16, 90 * 16);
        for (int b : {-45, -30, -20, -10, 0, 10, 20, 30, 45}) {
            p.save();
            p.rotate(b);
            p.drawLine(QPointF(0, -rad), QPointF(0, -rad - (b % 30 == 0 ? 10 : 6)));
            p.restore();
        }
        p.rotate(-roll_.value);
        QPolygonF caret({QPointF(0, -rad + 2), QPointF(-6, -rad + 12), QPointF(6, -rad + 12)});
        p.setBrush(hud);
        p.setPen(Qt::NoPen);
        p.drawPolygon(caret);
        p.restore();
    }

    paintHeadingTape(p, QRectF(c.x() - r.width() * 0.22, 66, r.width() * 0.44, 38));
    paintReadouts(p);
    const double tape_h = std::min(r.height() * 0.46, 360.0);
    paintVerticalTape(p, QRectF(c.x() - r.width() * 0.36 - 70, c.y() - tape_h / 2, 70, tape_h), as_.value, 2.0, true,
                      QStringLiteral("AS"));
    paintVerticalTape(p, QRectF(c.x() + r.width() * 0.36, c.y() - tape_h / 2, 70, tape_h), alt_.value, 5.0, false,
                      QStringLiteral("ALT"));

    // status line under the crosshair
    {
        const VtolState v = effectiveVtolState(s_);
        QString text = QStringLiteral("%1  ·  %2  ·  GS %3  ·  VS %4")
                           .arg(s_.has_heartbeat ? QString::fromStdString(s_.mode_name) : QStringLiteral("—"))
                           .arg(QString::fromUtf8(vtolStateName(v)))
                           .arg(s_.groundspeed, 0, 'f', 1)
                           .arg(s_.climb, 0, 'f', 1);
        const QFont fnt = monoFont(12, true);
        // under the heading box: the bottom centre is where the dock panels open
        drawText(p, QRectF(c.x() - 300, 130, 600, 20), Qt::AlignCenter, text, fnt, alpha(hud, 0.9));
    }

    if (guide_) {
        const QString text = QStringLiteral("GUIDE · tap the ground to send the aircraft");
        const QFont fnt = labelFont(11);
        const double w = QFontMetricsF(fnt).horizontalAdvance(text) + 30;
        const QRectF box(c.x() - w / 2, 156, w, 26);
        drawGlass(p, chamferPath(box, 7), kCyan, 0.8, kGlassDeep);
        drawText(p, box, Qt::AlignCenter, text, fnt, kCyan);
        drawBrackets(p, r.adjusted(8, 8, -8, -8), 28, alpha(kCyan, 0.7), 2.0);
    }
    if (click_t_ >= 0.0) {
        const double e = easeOutCubic(std::min(1.0, click_t_ / 0.25));
        const double a = 1.0 - std::clamp((click_t_ - 0.6) / 0.6, 0.0, 1.0);
        const double s = 34 - 16 * e;
        drawBrackets(p, QRectF(click_.x() - s, click_.y() - s, 2 * s, 2 * s), 8, alpha(kCyan, a), 2.0);
    }
}

void CameraView::paintHeadingTape(QPainter& p, const QRectF& r) {
    p.save();
    p.setClipRect(r.adjusted(0, 0, 0, 14));
    const QColor hud = kGreen;
    const double span = 60.0;
    const double ppd = r.width() / span;
    const double hdg = cam_az_.value;  // the tape follows what the camera sees
    p.setPen(QPen(alpha(hud, 0.6), 1));
    p.drawLine(QPointF(r.left(), r.bottom()), QPointF(r.right(), r.bottom()));
    const QFont f = monoFont(11);
    for (int d = static_cast<int>(std::floor((hdg - span / 2) / 5.0)) * 5; d <= hdg + span / 2 + 5; d += 5) {
        const double x = r.center().x() + (d - hdg) * ppd;
        const int dd = ((d % 360) + 360) % 360;
        const bool major = dd % 10 == 0;
        p.setPen(QPen(alpha(hud, 0.85), 1.2));
        p.drawLine(QPointF(x, r.bottom()), QPointF(x, r.bottom() - (major ? 10 : 5)));
        if (major) {
            QString label;
            switch (dd) {
            case 0: label = QStringLiteral("N"); break;
            case 90: label = QStringLiteral("E"); break;
            case 180: label = QStringLiteral("S"); break;
            case 270: label = QStringLiteral("W"); break;
            default: label = QString::number(dd / 10).rightJustified(2, QLatin1Char('0'));
            }
            drawText(p, QRectF(x - 16, r.top(), 32, 16), Qt::AlignCenter, label, f, alpha(hud, 0.85));
        }
    }
    // aircraft nose on the LOS tape (when it's in view)
    const double nose = geo::wrap180(yaw_.value - hdg);
    if (std::abs(nose) < span / 2) {
        const double x = r.center().x() + nose * ppd;
        p.setPen(Qt::NoPen);
        p.setBrush(alpha(kCyan, 0.9));
        p.drawPolygon(QPolygonF({QPointF(x, r.top() + 2), QPointF(x - 5, r.top() - 6), QPointF(x + 5, r.top() - 6)}));
    }
    p.restore();
    drawText(p, QRectF(r.left() - 48, r.bottom() - 16, 44, 14), Qt::AlignRight | Qt::AlignVCenter,
             QStringLiteral("LOS"), labelFont(10), alpha(hud, 0.7));
    // boxed readout + caret
    const QRectF box(r.center().x() - 28, r.bottom() + 2, 56, 20);
    p.setPen(QPen(kGreen, 1.2));
    p.setBrush(alpha(kBg, 0.7));
    p.drawRect(box);
    drawText(p, box, Qt::AlignCenter, QStringLiteral("%1°").arg(static_cast<int>(std::lround(hdg)) % 360, 3, 10, QLatin1Char('0')),
             monoFont(12, true), kGreen);
    p.setBrush(kGreen);
    p.setPen(Qt::NoPen);
    p.drawPolygon(QPolygonF({QPointF(r.center().x(), r.bottom() - 1), QPointF(r.center().x() - 5, r.bottom() - 8),
                             QPointF(r.center().x() + 5, r.bottom() - 8)}));
}

// Aircraft attitude (left) and camera line of sight (right). The HUD lines
// follow the camera; these give the platform's own angles and where the
// camera points relative to it.
void CameraView::paintReadouts(QPainter& p) {
    const QColor hud = kGreen;
    const QFont lf = labelFont(10);
    const QFont vf = monoFont(13, true);
    auto signedDeg = [](double v) {
        return QStringLiteral("%1%2°").arg(v >= 0 ? QStringLiteral("+") : QString()).arg(v, 0, 'f', 0);
    };
    auto row = [&](const QRectF& box, int i, const QString& k, const QString& v, const QColor& c) {
        const double y = box.top() + 26 + i * 19;
        drawText(p, QRectF(box.left() + 10, y, 50, 16), Qt::AlignLeft | Qt::AlignVCenter, k, lf, alpha(hud, 0.65));
        drawText(p, QRectF(box.left() + 56, y, box.width() - 64, 16), Qt::AlignRight | Qt::AlignVCenter, v, vf, c);
    };
    const double top = 120;

    // ---- aircraft ---------------------------------------------------------------
    {
        const QRectF box(16, top, 168, 92);
        p.setPen(QPen(alpha(hud, 0.5), 1));
        p.setBrush(alpha(kBg, 0.55));
        p.drawRect(box);
        drawText(p, QRectF(box.left() + 10, box.top() + 5, 120, 16), Qt::AlignLeft | Qt::AlignVCenter,
                 QStringLiteral("AIRCRAFT"), labelFont(10.5), hud);
        row(box, 0, QStringLiteral("ROLL"), signedDeg(roll_.value), hud);
        row(box, 1, QStringLiteral("PITCH"), signedDeg(pitch_.value), hud);
        row(box, 2, QStringLiteral("HDG"),
            QStringLiteral("%1°").arg(static_cast<int>(std::lround(yaw_.value)) % 360, 3, 10, QLatin1Char('0')), hud);
        // mini attitude: aircraft symbol over a horizon that rolls/pitches
        const QPointF ac(box.right() + 34, box.center().y());
        const double rad = 26;
        p.save();
        p.setClipRect(QRectF(ac.x() - rad, ac.y() - rad, 2 * rad, 2 * rad));
        p.translate(ac);
        p.setPen(QPen(alpha(hud, 0.4), 1));
        p.setBrush(alpha(kBg, 0.55));
        p.drawEllipse(QPointF(0, 0), rad, rad);
        p.rotate(-roll_.value);
        const double py = std::clamp(pitch_.value, -30.0, 30.0) * rad / 30.0;
        p.setPen(QPen(hud, 1.4));
        p.drawLine(QPointF(-rad, py), QPointF(rad, py));
        p.restore();
        p.setPen(QPen(kCyan, 2));
        p.drawLine(ac + QPointF(-14, 0), ac + QPointF(-5, 0));
        p.drawLine(ac + QPointF(5, 0), ac + QPointF(14, 0));
        p.drawPoint(ac);
    }

    // ---- line of sight ------------------------------------------------------------
    {
        // bottom right: the top right is where toasts stack
        const QRectF box(width() - 16 - 196, height() - 250, 196, 111);
        p.setPen(QPen(alpha(hud, 0.5), 1));
        p.setBrush(alpha(kBg, 0.55));
        p.drawRect(box);
        drawText(p, QRectF(box.left() + 10, box.top() + 5, 176, 16), Qt::AlignLeft | Qt::AlignVCenter,
                 gimbal_ ? QStringLiteral("GIMBAL · LOS") : QStringLiteral("FIXED MOUNT · LOS"), labelFont(10.5),
                 gimbal_ ? hud : kAmber);
        // relative to the platform: the main numbers
        row(box, 0, QStringLiteral("PAN"), signedDeg(pan_rel_.value), hud);
        row(box, 1, QStringLiteral("TILT"), signedDeg(tilt_rel_.value), hud);
        // earth-frame LOS underneath, smaller
        const QString earth = QStringLiteral("AZ %1°  EL %2")
                                  .arg(static_cast<int>(std::lround(cam_az_.value)) % 360, 3, 10, QLatin1Char('0'))
                                  .arg(signedDeg(cam_el_.value));
        drawText(p, QRectF(box.left() + 10, box.top() + 26 + 2 * 19 + 4, box.width() - 20, 16),
                 Qt::AlignLeft | Qt::AlignVCenter, QStringLiteral("earth"), lf, alpha(hud, 0.5));
        drawText(p, QRectF(box.left() + 10, box.top() + 26 + 3 * 19 + 2, box.width() - 18, 16),
                 Qt::AlignRight | Qt::AlignVCenter, earth, monoFont(11), alpha(hud, 0.8));
    }
}

void CameraView::paintVerticalTape(QPainter& p, const QRectF& r, double value, double step, bool left,
                                   const QString& label) {
    const QColor hud = kGreen;
    const double ppu = r.height() / (step * 10.0);
    p.save();
    p.setClipRect(r);
    p.setPen(QPen(alpha(hud, 0.6), 1));
    const double edge = left ? r.right() : r.left();
    p.drawLine(QPointF(edge, r.top()), QPointF(edge, r.bottom()));
    const QFont f = monoFont(11);
    const double first = std::floor((value - step * 6) / step) * step;
    for (double v = first; v <= value + step * 6; v += step) {
        const double y = r.center().y() - (v - value) * ppu;
        // labels under the value box would peek out beside it
        const bool major = std::fmod(std::abs(v) + 1e-6, step * 2) < 1e-3 && std::abs(y - r.center().y()) > 18;
        const double len = major ? 10 : 5;
        p.setPen(QPen(alpha(hud, 0.85), 1.2));
        if (left) {
            p.drawLine(QPointF(edge, y), QPointF(edge - len, y));
            if (major) {
                drawText(p, QRectF(r.left(), y - 8, r.width() - 14, 16), Qt::AlignRight | Qt::AlignVCenter,
                         QString::number(v, 'f', 0), f, alpha(hud, 0.85));
            }
        } else {
            p.drawLine(QPointF(edge, y), QPointF(edge + len, y));
            if (major) {
                drawText(p, QRectF(r.left() + 14, y - 8, r.width() - 14, 16), Qt::AlignLeft | Qt::AlignVCenter,
                         QString::number(v, 'f', 0), f, alpha(hud, 0.85));
            }
        }
    }
    p.restore();

    // current value box with a pointer toward the centre
    const double bh = 26;
    const QRectF box(r.left() + (left ? -4 : 4), r.center().y() - bh / 2, r.width(), bh);
    QPainterPath shape;
    if (left) {
        shape.moveTo(box.topLeft());
        shape.lineTo(box.right(), box.top());
        shape.lineTo(box.right() + 8, box.center().y());
        shape.lineTo(box.bottomRight());
        shape.lineTo(box.bottomLeft());
    } else {
        shape.moveTo(box.topRight());
        shape.lineTo(box.left(), box.top());
        shape.lineTo(box.left() - 8, box.center().y());
        shape.lineTo(box.bottomLeft());
        shape.lineTo(box.bottomRight());
    }
    shape.closeSubpath();
    p.setPen(QPen(hud, 1.4));
    p.setBrush(alpha(kBg, 0.85));
    p.drawPath(shape);
    drawText(p, box, Qt::AlignCenter, QString::number(value, 'f', 1), monoFont(14, true), hud);
    drawText(p, QRectF(r.left(), r.top() - 20, r.width(), 16), Qt::AlignCenter, label, labelFont(10.5),
             alpha(hud, 0.8));
}

}  // namespace gcs::ui
