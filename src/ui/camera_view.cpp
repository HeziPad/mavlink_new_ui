#include "camera_view.hpp"

#include "attitude_ball.hpp"
#include "wind_gauge.hpp"
#include "core/geo.hpp"
#include "theme.hpp"
#include "units.hpp"
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
    fpm_drift_.rate = fpm_gamma_.rate = 8.0;
    cam_el_.set(mount_pitch_);
    tilt_rel_.set(mount_pitch_);
}

bool CameraView::live() const {
    return frame_age_ < kNoVideoAfter;
}

void CameraView::setVelocity(double vn, double ve, double vd) {
    vn_ = vn;
    ve_ = ve;
    vd_ = vd;
    has_vel_ = true;
}

double CameraView::aspect() const {
    if (live() && !frame_.isNull() && frame_.height() > 0) {
        return static_cast<double>(frame_.width()) / frame_.height();
    }
    return 16.0 / 9.0;
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
    // flight-path marker: track from the predicted velocity (turns with the
    // heading), relative to the ruler reference, then lightly smoothed
    if (has_vel_ && std::hypot(vn_, ve_) > 2.0) {
        const double track = std::atan2(ve_, vn_) * geo::kRadToDeg;
        fpm_drift_.set(std::clamp(geo::wrap180(track - refYaw()), -40.0, 40.0));
        fpm_gamma_.set(std::atan2(-vd_, std::hypot(vn_, ve_)) * geo::kRadToDeg);
    }
    fpm_drift_.step(dt);
    fpm_gamma_.step(dt);
    const double target = ref_ == HudRef::Body ? 1.0 : 0.0;
    if (reducedMotion()) {
        ref_blend_ = target;
    } else {
        ref_blend_ += std::clamp(target - ref_blend_, -dt * 3.5, dt * 3.5);
    }
    ref_glitch_ = std::max(0.0, ref_glitch_ - dt * 3.0);
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

double CameraView::hudY(double elevation_deg) const {
    const double rel = std::clamp(elevation_deg - refPitch(), -89.0, 89.0);
    return -focal() * tanDeg(rel);
}

// eased so the ladder sweeps from one reference to the other
double CameraView::refRoll() const {
    return cam_roll_.value + (roll_.value - cam_roll_.value) * easeInOutCubic(ref_blend_);
}
double CameraView::refPitch() const {
    return cam_el_.value + (pitch_.value - cam_el_.value) * easeInOutCubic(ref_blend_);
}
double CameraView::refYaw() const {
    return geo::wrap360(cam_az_.value + geo::wrap180(yaw_.value - cam_az_.value) * easeInOutCubic(ref_blend_));
}

void CameraView::setHudReference(HudRef ref, bool animate) {
    if (ref == ref_) {
        return;
    }
    ref_ = ref;
    if (!animate || reducedMotion()) {
        ref_blend_ = ref == HudRef::Body ? 1.0 : 0.0;
    } else {
        ref_glitch_ = 1.0;
    }
    update();
}

void CameraView::mousePressEvent(QMouseEvent* e) {
    if (compact_ || e->button() != Qt::LeftButton) {
        return;
    }
    if (wind_rect_.contains(e->position())) {
        wind_north_up_ = !wind_north_up_;
        emit windNorthUpChanged(wind_north_up_);
        update();
        return;
    }
    if (ref_chip_.contains(e->position())) {
        setHudReference(ref_ == HudRef::Los ? HudRef::Body : HudRef::Los, true);
        emit hudReferenceChanged(static_cast<int>(ref_));
        return;
    }
    if (!guide_) {
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

    // vignette + scanlines make it read as a sensor feed (one cached layer)
    const bool scan = !reducedMotion() && !compact_;
    const qreal dpr = devicePixelRatioF();
    if (overlay_.isNull() || overlay_size_ != size() || overlay_scan_ != scan || overlay_.devicePixelRatio() != dpr) {
        overlay_ = QPixmap((QSizeF(size()) * dpr).toSize());
        overlay_.setDevicePixelRatio(dpr);
        overlay_.fill(Qt::transparent);
        QPainter op(&overlay_);
        QRadialGradient vg(r.center(), std::hypot(r.width(), r.height()) / 2.0);
        vg.setColorAt(0.55, QColor(0, 0, 0, 0));
        vg.setColorAt(1.0, QColor(0, 0, 0, 150));
        op.fillRect(r, vg);
        if (scan) {
            drawScanlines(op, r, 0.6);
        }
        overlay_size_ = size();
        overlay_scan_ = scan;
    }
    p.drawPixmap(QPointF(0, 0), overlay_);

    if (live() && !compact_) {
        // Over real video the thin green symbology vanishes on bright scenes:
        // render it once, then stamp a dark copy at +-1 px underneath as an outline.
        const QSize px = (QSizeF(size()) * dpr).toSize();
        if (hud_img_.size() != px) {
            hud_img_ = QImage(px, QImage::Format_ARGB32_Premultiplied);
            hud_shadow_ = QImage(px, QImage::Format_ARGB32_Premultiplied);
        }
        hud_img_.setDevicePixelRatio(dpr);
        hud_shadow_.setDevicePixelRatio(dpr);
        hud_img_.fill(Qt::transparent);
        {
            QPainter hp(&hud_img_);
            hp.setRenderHint(QPainter::Antialiasing);
            paintHud(hp);
        }
        {
            // the same shapes in near-black: copy, then recolour keeping alpha
            QPainter sp(&hud_shadow_);
            sp.setCompositionMode(QPainter::CompositionMode_Source);
            sp.drawImage(QPointF(0, 0), hud_img_);
            sp.setCompositionMode(QPainter::CompositionMode_SourceIn);
            sp.fillRect(QRectF(QPointF(0, 0), QSizeF(size())), QColor(0, 0, 0, 210));
        }
        // a diagonal pair of offsets outlines every edge well enough, at half the cost of four
        for (const QPointF off : {QPointF(-1, -1), QPointF(1, 1)}) {
            p.drawImage(off, hud_shadow_);
        }
        p.drawImage(QPointF(0, 0), hud_img_);
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

CameraView::Layout CameraView::layout() const {
    Layout l;
    const QRectF r = rect();
    const QPointF c = r.center();
    constexpr double kColX = 16, kColW = 200;
    l.aircraft = QRectF(kColX, 120, kColW, 92);
    l.los = QRectF(kColX, l.aircraft.bottom() + 8, kColW, 104);
    l.chip = QRectF(kColX, l.los.bottom() + 8, kColW, 48);
    l.wind = QRectF(kColX, l.chip.bottom() + 8, kColW, 120);
    const double tape_h = std::min(r.height() * 0.46, 360.0);
    const double tape_x = l.aircraft.right() + 12;  // just inside the column, mirrored on the right
    l.as_tape = QRectF(tape_x, c.y() - tape_h / 2, 70, tape_h);
    l.alt_tape = QRectF(r.width() - tape_x - 70, c.y() - tape_h / 2, 70, tape_h);
    l.ladder_clip = QRectF(l.as_tape.right() + 14, c.y() - r.height() * 0.30,
                           std::max(40.0, l.alt_tape.left() - 14 - (l.as_tape.right() + 14)), r.height() * 0.57);
    return l;
}

void CameraView::paintHud(QPainter& p) {
    const QRectF r = rect();
    const QPointF c = r.center();
    const QColor hud = kGreen;
    const double f = focal();

    // ---- horizon + ladder (rotated with roll) -------------------------------
    p.save();
    p.setClipRect(compact_ ? QRectF(c.x() - r.width() * 0.32, c.y() - r.height() * 0.30, r.width() * 0.64,
                                    r.height() * 0.57)
                           : layout().ladder_clip);
    p.translate(c);
    p.rotate(-refRoll());
    const double hy = hudY(0.0);
    p.setPen(QPen(alpha(hud, 0.9), compact_ ? 1.2 : 1.6));
    p.drawLine(QPointF(-r.width() * 0.4, hy), QPointF(-60, hy));
    p.drawLine(QPointF(60, hy), QPointF(r.width() * 0.4, hy));

    if (!compact_) {
        const QFont lf = monoFont(11);
        for (int e = -80; e <= 80; e += 5) {
            if (e == 0) {
                continue;
            }
            const double y = hudY(e);
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
        if (has_vel_ && std::hypot(vn_, ve_) > 2.0) {
            const QPointF fpm(f * tanDeg(fpm_drift_.value), hudY(fpm_gamma_.value));
            p.setPen(QPen(hud, 1.6));
            p.setBrush(Qt::NoBrush);
            p.drawEllipse(fpm, 7, 7);
            p.drawLine(fpm + QPointF(-18, 0), fpm + QPointF(-7, 0));
            p.drawLine(fpm + QPointF(7, 0), fpm + QPointF(18, 0));
            p.drawLine(fpm + QPointF(0, -7), fpm + QPointF(0, -14));
        }

        // body reference: where the camera looks, relative to the nose
        if (ref_blend_ > 0.02) {
            const double dx = geo::wrap180(cam_az_.value - refYaw());
            if (std::abs(dx) < 60.0) {
                const QPointF cb(f * tanDeg(dx), hudY(cam_el_.value));
                p.setPen(QPen(alpha(kAmber, 0.9 * ref_blend_), 1.6));
                p.setBrush(Qt::NoBrush);
                p.drawRect(QRectF(cb.x() - 9, cb.y() - 9, 18, 18));
                drawText(p, QRectF(cb.x() + 12, cb.y() - 8, 40, 16), Qt::AlignLeft | Qt::AlignVCenter,
                         QStringLiteral("CAM"), labelFont(9.5), alpha(kAmber, 0.9 * ref_blend_));
            }
        }
    }
    p.restore();

    // ---- boresight crosshair ------------------------------------------------
    const double body = compact_ ? 0.0 : ref_blend_;
    if (body < 0.98) {
        p.setPen(QPen(alpha(hud, 1.0 - body), 1.6));
        const double g = compact_ ? 4 : 8, l = compact_ ? 10 : 22;
        p.drawLine(QPointF(c.x() - l, c.y()), QPointF(c.x() - g, c.y()));
        p.drawLine(QPointF(c.x() + g, c.y()), QPointF(c.x() + l, c.y()));
        p.drawLine(QPointF(c.x(), c.y() - l), QPointF(c.x(), c.y() - g));
        p.drawLine(QPointF(c.x(), c.y() + g), QPointF(c.x(), c.y() + l));
        p.drawPoint(c);
    }
    if (body > 0.02) {
        // waterline "W": the aircraft nose
        p.setPen(QPen(alpha(hud, body), 2.0));
        p.setBrush(Qt::NoBrush);
        p.drawPolyline(QPolygonF({c + QPointF(-30, 0), c + QPointF(-14, 0), c + QPointF(-7, 9), c + QPointF(0, 0),
                                  c + QPointF(7, 9), c + QPointF(14, 0), c + QPointF(30, 0)}));
    }

    if (compact_) {
        return;
    }

    // the arc's numbers must stay below the heading tape and status line (~y 160)
    paintRollScale(p, c, std::max(60.0, std::min(std::min(r.width(), r.height()) * 0.30, c.y() - 190.0)));

    paintHeadingTape(p, QRectF(c.x() - r.width() * 0.22, 66, r.width() * 0.44, 38));
    paintReadouts(p);
    paintRefChip(p);
    paintWind(p);
    const Layout lay = layout();
    paintVerticalTape(p, lay.as_tape, as_.value, 2.0, true, QStringLiteral("AS"));
    paintVerticalTape(p, lay.alt_tape, units::altToDisplay(alt_.value), units::feet() ? 20.0 : 5.0, false,
                      QStringLiteral("ALT %1").arg(units::altUnitLabel()));

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
    if (ref_glitch_ > 0.0 && !reducedMotion()) {
        // short tear across the rulers while the reference switches
        for (int i = 0; i < 7; ++i) {
            const double fy = std::fmod(std::sin(i * 12.9898 + time_ * 31.0) * 43758.5453, 1.0);
            const double y = c.y() + (std::abs(fy) - 0.5) * r.height() * 0.6;
            p.fillRect(QRectF(c.x() - r.width() * 0.32 + fy * 30.0, y, r.width() * 0.64, 2.0 + 6.0 * ref_glitch_),
                       alpha(i % 2 ? kGreen : kAmber, 0.25 * ref_glitch_));
        }
    }
    if (click_t_ >= 0.0) {
        const double e = easeOutCubic(std::min(1.0, click_t_ / 0.25));
        const double a = 1.0 - std::clamp((click_t_ - 0.6) / 0.6, 0.0, 1.0);
        const double s = 34 - 16 * e;
        drawBrackets(p, QRectF(click_.x() - s, click_.y() - s, 2 * s, 2 * s), 8, alpha(kCyan, a), 2.0);
    }
}

void CameraView::paintRollScale(QPainter& p, const QPointF& c, double rad) {
    const QColor hud = kGreen;
    const double roll = refRoll();
    p.save();
    p.translate(c);
    p.setPen(QPen(alpha(hud, 0.6), 1.2));
    p.setBrush(Qt::NoBrush);
    p.drawArc(QRectF(-rad, -rad, 2 * rad, 2 * rad), (90 - 60) * 16, 120 * 16);
    const QFont lf = monoFont(10.5, true);
    for (int b : {-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60}) {
        p.save();
        p.rotate(b);
        const bool major = b % 30 == 0 || std::abs(b) == 45;
        p.setPen(QPen(alpha(hud, 0.8), major ? 1.6 : 1.2));
        p.drawLine(QPointF(0, -rad), QPointF(0, -rad - (major ? 11 : 6)));
        if (b != 0) {
            // numbers ride along the arc, just outside the ticks
            drawText(p, QRectF(-16, -rad - 28, 32, 15), Qt::AlignCenter, QString::number(std::abs(b)), lf,
                     alpha(hud, 0.85));
        } else {
            p.setPen(Qt::NoPen);
            p.setBrush(alpha(hud, 0.8));
            p.drawPolygon(QPolygonF({QPointF(0, -rad - 12), QPointF(-5, -rad - 21), QPointF(5, -rad - 21)}));
        }
        p.restore();
    }
    // pointer rides with the horizon (sky pointer)
    const bool steep = std::abs(roll) > 45.0;
    const QColor pc = steep ? kAmber : hud;
    p.save();
    p.rotate(-std::clamp(roll, -70.0, 70.0));
    p.setBrush(pc);
    p.setPen(Qt::NoPen);
    p.drawPolygon(QPolygonF({QPointF(0, -rad + 2), QPointF(-7, -rad + 14), QPointF(7, -rad + 14)}));
    p.restore();
    // live readout under the pointer
    const int deg = static_cast<int>(std::lround(std::abs(roll)));
    const QString text = deg == 0 ? QStringLiteral("0°")
                                   : QStringLiteral("%1 %2°").arg(roll > 0 ? QLatin1Char('R') : QLatin1Char('L')).arg(deg);
    const QRectF box(-30, -rad + 18, 60, 20);
    p.setPen(QPen(alpha(pc, 0.9), 1.2));
    p.setBrush(alpha(kBg, 0.7));
    p.drawRect(box);
    drawText(p, box, Qt::AlignCenter, text, monoFont(12, true), pc);
    drawText(p, QRectF(-50, -rad + 40, 100, 14), Qt::AlignCenter,
             ref_ == HudRef::Body ? QStringLiteral("BANK") : QStringLiteral("LOS ROLL"), labelFont(9),
             alpha(hud, 0.55));
    p.restore();
}

void CameraView::paintRefChip(QPainter& p) {
    // under the AIRCRAFT box (see paintReadouts)
    const bool body = ref_ == HudRef::Body;
    const QColor col = body ? kGreen : kAmber;  // matches the AIRCRAFT (green) and LOS (yellow) boxes
    ref_chip_ = layout().chip;
    drawGlass(p, chamferPath(ref_chip_, 7), col, 0.85, kGlassDeep);
    const bool misaligned = body && live();
    drawText(p, QRectF(ref_chip_.left() + 10, ref_chip_.top() + 5, ref_chip_.width() - 20, 14),
             Qt::AlignLeft | Qt::AlignVCenter,
             misaligned ? QStringLiteral("HUD RULERS · NOT ON VIDEO") : QStringLiteral("HUD RULERS · TAP"),
             labelFont(9), misaligned ? alpha(kAmber, 0.9) : alpha(kMuted, 0.95));
    drawText(p, QRectF(ref_chip_.left() + 10, ref_chip_.top() + 20, 70, 22), Qt::AlignLeft | Qt::AlignVCenter,
             body ? QStringLiteral("BODY") : QStringLiteral("LOS"), labelFont(14), col);
    // two-position indicator on the right
    const QRectF track(ref_chip_.right() - 38, ref_chip_.top() + 25, 28, 12);
    p.setPen(QPen(alpha(col, 0.6), 1));
    p.setBrush(alpha(kBg, 0.6));
    p.drawRoundedRect(track, 6, 6);
    const double kx = track.left() + 6 + (track.width() - 12) * easeInOutCubic(ref_blend_);
    p.setPen(Qt::NoPen);
    p.setBrush(col);
    p.drawEllipse(QPointF(kx, track.center().y()), 5, 5);
}

void CameraView::paintWind(QPainter& p) {
    // tap: relative to the nose <-> north up
    const QColor hud = kGreen;
    WindInfo w;
    w.valid = s_.has_wind;
    w.from_deg = s_.wind_from_deg;
    w.speed = s_.wind_speed;
    wind_rect_ = layout().wind;
    // the PiP sits bottom-left (ViewHost::pipRect: 30 % of the width, 16:10);
    // in a small window there's no room for the gauge above it
    const double pip_top = height() - 16.0 - std::clamp(width() * 0.30, 240.0, 420.0) * 10.0 / 16.0;
    if (wind_rect_.bottom() > pip_top - 8.0) {
        wind_rect_ = QRectF();
        return;
    }
    const QRectF& wb = wind_rect_;
    p.setPen(QPen(alpha(hud, 0.5), 1));
    p.setBrush(alpha(kBg, 0.55));
    p.drawRect(wb);
    drawText(p, QRectF(wb.left() + 10, wb.top() + 4, wb.width() - 20, 14), Qt::AlignLeft | Qt::AlignVCenter,
             QStringLiteral("WIND"), labelFont(10.5), hud);
    drawText(p, QRectF(wb.left() + 10, wb.top() + 4, wb.width() - 20, 14), Qt::AlignRight | Qt::AlignVCenter,
             wind_north_up_ ? QStringLiteral("N-UP · TAP") : QStringLiteral("REL · TAP"), labelFont(9),
             wind_north_up_ ? alpha(kCyan, 0.9) : alpha(kMuted, 0.95));
    paintWindGauge(p, wb.adjusted(6, 20, -6, -4), w, yaw_.value, time_, wind_north_up_, false);
}

void CameraView::paintHeadingTape(QPainter& p, const QRectF& r) {
    p.save();
    p.setClipRect(r.adjusted(0, 0, 0, 14));
    const QColor hud = kGreen;
    const double span = 60.0;
    const double ppd = r.width() / span;
    const double hdg = refYaw();  // LOS: what the camera sees; body: the nose
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
    // the other reference on the tape (when it's in view): the nose on the LOS
    // tape, the camera on the body tape
    const bool body = ref_ == HudRef::Body;
    const double other = geo::wrap180((body ? cam_az_.value : yaw_.value) - hdg);
    if (std::abs(other) < span / 2) {
        const double x = r.center().x() + other * ppd;
        p.setPen(Qt::NoPen);
        p.setBrush(alpha(body ? kAmber : kGreen, 0.9));
        p.drawPolygon(QPolygonF({QPointF(x, r.top() + 2), QPointF(x - 5, r.top() - 6), QPointF(x + 5, r.top() - 6)}));
    }
    p.restore();
    drawText(p, QRectF(r.left() - 48, r.bottom() - 16, 44, 14), Qt::AlignRight | Qt::AlignVCenter,
             ref_ == HudRef::Body ? QStringLiteral("HDG") : QStringLiteral("LOS"), labelFont(10), alpha(hud, 0.7));
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
    const Layout lay = layout();

    // ---- aircraft ---------------------------------------------------------------
    {
        const QRectF& box = lay.aircraft;
        auto row = [&](int i, const QString& k, const QString& v) {
            // values stop short of the ball on the right
            const double y = box.top() + 26 + i * 19;
            drawText(p, QRectF(box.left() + 10, y, 50, 16), Qt::AlignLeft | Qt::AlignVCenter, k, lf, alpha(hud, 0.65));
            drawText(p, QRectF(box.left() + 56, y, box.width() - 56 - 66, 16), Qt::AlignRight | Qt::AlignVCenter, v, vf,
                     hud);
        };
        p.setPen(QPen(alpha(hud, 0.5), 1));
        p.setBrush(alpha(kBg, 0.55));
        p.drawRect(box);
        drawText(p, QRectF(box.left() + 10, box.top() + 5, 120, 16), Qt::AlignLeft | Qt::AlignVCenter,
                 QStringLiteral("AIRCRAFT"), labelFont(10.5), hud);
        row(0, QStringLiteral("ROLL"), signedDeg(roll_.value));
        row(1, QStringLiteral("PITCH"), signedDeg(pitch_.value));
        row(2, QStringLiteral("HDG"),
            QStringLiteral("%1°").arg(static_cast<int>(std::lround(yaw_.value)) % 360, 3, 10, QLatin1Char('0')));
        // mini attitude ball, inside the box on the right
        paintAttitudeBall(p, QPointF(box.right() - 34, box.center().y() + 6), 24, roll_.value, pitch_.value,
                          yaw_.value, true);
    }

    // ---- line of sight ------------------------------------------------------------
    {
        // under AIRCRAFT: the platform's angles, then where its camera points
        const QRectF& box = lay.los;
        p.setPen(QPen(alpha(hud, 0.5), 1));
        p.setBrush(alpha(kBg, 0.55));
        p.drawRect(box);
        drawText(p, QRectF(box.left() + 10, box.top() + 5, box.width() - 20, 16), Qt::AlignLeft | Qt::AlignVCenter,
                 gimbal_ ? QStringLiteral("GIMBAL · LOS") : QStringLiteral("FIXED MOUNT · LOS"), labelFont(10.5),
                 kAmber);
        // relative to the platform: the main numbers
        row(box, 0, QStringLiteral("PAN"), signedDeg(pan_rel_.value), hud);
        row(box, 1, QStringLiteral("TILT"), signedDeg(tilt_rel_.value), hud);
        // earth-frame LOS underneath, smaller
        const QString earth = QStringLiteral("AZ %1°  EL %2")
                                  .arg(static_cast<int>(std::lround(cam_az_.value)) % 360, 3, 10, QLatin1Char('0'))
                                  .arg(signedDeg(cam_el_.value));
        drawText(p, QRectF(box.left() + 10, box.top() + 26 + 2 * 19 + 2, 50, 16), Qt::AlignLeft | Qt::AlignVCenter,
                 QStringLiteral("EARTH"), lf, alpha(hud, 0.5));
        drawText(p, QRectF(box.left() + 56, box.top() + 26 + 2 * 19 + 2, box.width() - 64, 16),
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
