#pragma once

#include "anim.hpp"
#include "core/hud_config.hpp"
#include "core/vehicle_state.hpp"
#include "view_host.hpp"

#include <QImage>
#include <QRectF>

#include <array>
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
// centre); the HUD panel switches it, with an animated sweep.
class CameraView : public HudView {
    Q_OBJECT

public:
    explicit CameraView(VideoReceiver* video, QWidget* parent = nullptr);

    enum class HudRef { Los = 0, Body = 1 };
    void setHudReference(HudRef ref, bool animate = false);
    HudRef hudReference() const { return ref_; }
    void setWindNorthUp(bool on) { wind_north_up_ = on; update(); }
    bool windNorthUp() const { return wind_north_up_; }

    // What the HUD draws (HudPanel edits it; box folds and the measuring
    // brackets are changed here too and reported by hudConfigChanged).
    void setHudConfig(const hud::Config& c);
    const hud::Config& hudConfig() const { return cfg_; }
    // the HUD colour swatches (hud::Config::color)
    static QColor swatch(int index);
    // a crosshair style centred on c, scaled by k (also the HUD panel's previews)
    static void drawReticle(QPainter& p, const QPointF& c, hud::Crosshair style, const QColor& color,
                            double line_factor, double k, const QRectF& bounds);
    // the HUD chip in widget pixels (empty in PiP), for MainWindow's outside-tap test
    QRectF hudChipRect() const { return QRectF(hud_chip_.topLeft() * hit_k_, hud_chip_.size() * hit_k_); }
    // the chip lights while the HUD panel is open
    void setHudPanelOpen(bool open) { hud_panel_open_ = open; update(); }
    // the PAYLOAD chip (next to HUD): shown while a payload ICD is loaded
    QRectF payloadChipRect() const { return QRectF(payload_chip_.topLeft() * hit_k_, payload_chip_.size() * hit_k_); }
    void setPayloadChip(bool shown, bool open, bool alive) {
        payload_chip_shown_ = shown;
        payload_panel_open_ = open;
        payload_alive_ = alive;
    }

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
    void windNorthUpChanged(bool on);
    void hudConfigChanged();      // a box folded / unfolded, the brackets resized
    void hudPanelRequested();     // the HUD chip
    void payloadPanelRequested(); // the PAYLOAD chip

protected:
    void paintEvent(QPaintEvent* e) override;

public:
    int paints_ = 0;  // paint events since the last GCS_FPS report

protected:
    void mousePressEvent(QMouseEvent* e) override;
    void mouseMoveEvent(QMouseEvent* e) override;
    void mouseReleaseEvent(QMouseEvent* e) override;
    void wheelEvent(QWheelEvent* e) override;
    bool event(QEvent* e) override;
    void compactChanged() override { setGuideMode(guide_); }

private:
    void paintLosTarget(QPainter& p);
    // ---- configurable HUD -----------------------------------------------------------
    hud::Config cfg_ = hud::defaultConfig();
    QColor hudColor() const { return swatch(cfg_.color); }
    double lw(double w) const { return w * cfg_.lineFactor(); }  // a HUD pen width
    bool shows(hud::Element e) const { return cfg_.shows(e); }
    // the rulers follow the body only while some ruler is shown
    double bodyBlend() const;
    // the picture on screen, physical px: the letterboxed video when live
    QRectF pictureRect() const;
    void paintCrosshair(QPainter& p, const QPointF& c, double alpha_k);
    void paintMeasure(QPainter& p, const QPointF& c);
    void paintHudChip(QPainter& p);
    void paintVideoTag(QPainter& p);
    QRectF hud_chip_;
    bool hud_panel_open_ = false;
    QRectF payload_chip_;
    bool payload_chip_shown_ = false, payload_panel_open_ = false, payload_alive_ = false;
    // side boxes fold to one line: 0 folded .. 1 open, animated
    std::array<double, hud::kBoxes> box_open_{};
    std::array<QRectF, hud::kBoxes> box_hit_{};
    void paintBoxFrame(QPainter& p, const QRectF& box, const QString& title, const QColor& title_color,
                       hud::Box which);
    bool boxOpen(hud::Box b) const { return box_open_[static_cast<size_t>(b)] > 0.5; }
    // measuring brackets: the shown size eases toward cfg_.measure_*
    double measure_w_shown_ = 4.0, measure_h_shown_ = 2.5;
    enum class MeasureDrag { None, Width, Height, Both };
    MeasureDrag measure_drag_ = MeasureDrag::None;
    double measure_glow_ = 0.0;
    bool pinching_ = false;
    double pinch_start_dist_ = 0.0, pinch_w0_ = 0.0, pinch_h0_ = 0.0;
    MeasureDrag measureHandleAt(const QPointF& logical) const;
    QSizeF measureHalfPx() const;  // half width / height of the brackets, logical px
    void scaleMeasure(double factor);
    // one path for mouse and a single finger (physical px)
    void pointerPress(const QPointF& pos);
    void pointerMove(const QPointF& pos);
    void pointerRelease();
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
    void paintWind(QPainter& p);

    // Left column (TARGET, AIRCRAFT, LOS, WIND), the tapes just inside it
    // and the ladder between the tapes. One place, so paint and hit-tests agree.
    struct Layout {
        QRectF target, aircraft, los, wind;
        QRectF as_tape, alt_tape;
        QRectF ladder_clip;
    };
    Layout layout() const;
    void paintSynthetic(QPainter& p);
    void paintHud(QPainter& p);
    // The HUD grows with the text size: it is painted scaled by hud_k_ in a
    // logical rect (hudRect); focal() follows, so the ladder stays true to the FOV.
    void paintHudScaled(QPainter& p);
    QRectF hudRect() const;
    double hud_k_ = 1.0;   // while painting the HUD
    double hit_k_ = 1.0;   // the scale the HUD's tap targets were laid out at
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
    QRectF wind_rect_;
    bool wind_north_up_ = false;  // HUD wind gauge: false = relative to the nose

    QPointF click_;
    double click_t_ = -1.0;
};

}  // namespace gcs::ui
