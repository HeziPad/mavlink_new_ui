#pragma once

#include "anim.hpp"
#include "core/vehicle_state.hpp"
#include "view_host.hpp"

#include <QImage>
#include <QPixmap>

namespace gcs::ui {

class VideoReceiver;

// Video feed (or a synthetic perspective-grid world when there is none) with
// a fighter-style HUD. The horizon, elevation ladder, LOS heading tape and
// flight-path marker are drawn in camera geometry (from the gimbal's line of
// sight, or the fixed mount), so they overlay the video. Separate readouts show
// the aircraft's own attitude and the LOS relative to the platform. In guide
// mode a click is reported as a pixel so the MainWindow can ray-cast it.
//
// The main rulers (horizon/ladder, roll scale, heading tape) follow either the
// camera LOS (lines up with the video) or the aircraft body (nose = screen
// centre); a chip under the AIRCRAFT box switches, with an animated sweep.
class CameraView : public HudView {
    Q_OBJECT

public:
    explicit CameraView(VideoReceiver* video, QWidget* parent = nullptr);

    enum class HudRef { Los = 0, Body = 1 };
    void setHudReference(HudRef ref, bool animate = false);
    HudRef hudReference() const { return ref_; }
    void setWindNorthUp(bool on) { wind_north_up_ = on; update(); }
    bool windNorthUp() const { return wind_north_up_; }

    QString pipTag() const override;
    QColor pipTagColor() const override;

    void setState(const VehicleState& s);
    // predicted every frame: aircraft attitude and camera line of sight
    void setMotion(const geo::Attitude& aircraft, const CameraLos& los);
    void setAltitude(double alt_rel) { alt_.set(alt_rel); }
    // predicted ground velocity (NED, m/s), for the flight-path marker
    void setVelocity(double vn, double ve, double vd);
    void setCamera(double hfov_deg, double mount_pitch_deg);
    void setGuideMode(bool on);
    // the plain flight state, shown in the status line
    void setFlightLabel(const QString& label) { flight_label_ = label; }
    // where the line of sight meets the ground (bottom-right box)
    struct LosTarget {
        bool valid = false;        // there is an estimate at all (position, attitude)
        bool hit = false;          // false: above the horizon / beyond range
        bool approximate = false;  // terrain unknown: flat ground at home level
        double lat = 0.0, lon = 0.0;
        double ground = 0.0;       // AMSL
        double slant = 0.0;        // m
    };
    void setLosTarget(const LosTarget& t) { los_target_ = t; }
    void tick(double dt);
    bool live() const;
    // width / height of the picture: the video's when live, else 16:9
    double aspect() const;

signals:
    void groundClicked(double px, double py, double width, double height);
    void hudReferenceChanged(int ref);  // HudRef
    void windNorthUpChanged(bool on);

protected:
    void paintEvent(QPaintEvent* e) override;

public:
    int paints_ = 0;  // paint events since the last GCS_FPS report

protected:
    void mousePressEvent(QMouseEvent* e) override;
    void compactChanged() override { setGuideMode(guide_); }

private:
    void paintLosTarget(QPainter& p);
    QString flight_label_;
    LosTarget los_target_;
    double focal() const;
    double elevationToY(double elevation_deg) const;  // camera frame, roll-rotated
    // the same for the HUD rulers, centred on the selected reference
    double hudY(double elevation_deg) const;
    double refRoll() const;
    double refPitch() const;
    double refYaw() const;
    void paintRollScale(QPainter& p, const QPointF& c, double rad);
    void paintRefChip(QPainter& p);
    void paintWind(QPainter& p);

    // Left column (AIRCRAFT, LOS, HUD chip, WIND), the tapes just inside it
    // and the ladder between the tapes. One place, so paint and hit-tests agree.
    struct Layout {
        QRectF target, aircraft, los, chip, wind;
        QRectF as_tape, alt_tape;
        QRectF ladder_clip;
    };
    Layout layout() const;
    void paintSynthetic(QPainter& p);
    void paintHud(QPainter& p);
    void paintHeadingTape(QPainter& p, const QRectF& r);
    void paintReadouts(QPainter& p);
    void paintVerticalTape(QPainter& p, const QRectF& r, double value, double step, bool left,
                           const QString& label);

    VideoReceiver* video_;
    QImage frame_;
    uint64_t frame_seq_ = 0;
    QImage scaled_;             // frame_ scaled to the widget, redone per new frame / resize
    uint64_t scaled_seq_ = 0;
    // vignette + scanlines: the same every frame for a given size, so drawn once
    QPixmap overlay_;
    QSize overlay_size_;
    bool overlay_scan_ = false;
    // the HUD over live video and its dark outline, reused between frames
    QImage hud_img_, hud_shadow_;
    double frame_age_ = 1e9;

    VehicleState s_;
    Tween roll_, pitch_, as_, alt_;  // aircraft
    AngleTween yaw_;
    Tween cam_roll_, cam_el_;        // camera LOS, earth frame
    AngleTween cam_az_;
    Tween pan_rel_, tilt_rel_;       // LOS relative to the platform
    double vn_ = 0.0, ve_ = 0.0, vd_ = 0.0;
    bool has_vel_ = false;
    Tween fpm_drift_, fpm_gamma_;    // flight-path marker angles, lightly smoothed
    bool gimbal_ = false;
    double hfov_ = 70.0;
    double mount_pitch_ = -30.0;
    double travelled_ = 0.0;
    bool guide_ = false;

    HudRef ref_ = HudRef::Los;
    double ref_blend_ = 0.0;  // 0 = LOS, 1 = body; animates toward ref_
    double ref_glitch_ = 0.0;
    QRectF ref_chip_;
    QRectF wind_rect_;
    bool wind_north_up_ = false;  // HUD wind gauge: false = relative to the nose

    QPointF click_;
    double click_t_ = -1.0;
};

}  // namespace gcs::ui
