#pragma once

#include "anim.hpp"
#include "core/vehicle_state.hpp"
#include "view_host.hpp"

#include <QImage>

namespace gcs::ui {

class VideoReceiver;

// Video feed (or a synthetic perspective-grid world when there is none) with
// a fighter-style HUD. The horizon, elevation ladder, LOS heading tape and
// flight-path marker are drawn in camera geometry (from the gimbal's line of
// sight, or the fixed mount), so they overlay the video. Separate readouts show
// the aircraft's own attitude and the LOS relative to the platform. In guide
// mode a click is reported as a pixel so the MainWindow can ray-cast it.
class CameraView : public HudView {
    Q_OBJECT

public:
    explicit CameraView(VideoReceiver* video, QWidget* parent = nullptr);

    QString pipTag() const override;
    QColor pipTagColor() const override;

    void setState(const VehicleState& s);
    // predicted every frame: aircraft attitude and camera line of sight
    void setMotion(const geo::Attitude& aircraft, const CameraLos& los);
    void setAltitude(double alt_rel) { alt_.set(alt_rel); }
    void setCamera(double hfov_deg, double mount_pitch_deg);
    void setGuideMode(bool on);
    void tick(double dt);
    bool live() const;

signals:
    void groundClicked(double px, double py, double width, double height);

protected:
    void paintEvent(QPaintEvent* e) override;
    void mousePressEvent(QMouseEvent* e) override;
    void compactChanged() override { setGuideMode(guide_); }

private:
    double focal() const;
    double elevationToY(double elevation_deg) const;  // in the roll-rotated frame
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
    double frame_age_ = 1e9;

    VehicleState s_;
    Tween roll_, pitch_, as_, alt_;  // aircraft
    AngleTween yaw_;
    Tween cam_roll_, cam_el_;        // camera LOS, earth frame
    AngleTween cam_az_;
    Tween pan_rel_, tilt_rel_;       // LOS relative to the platform
    bool gimbal_ = false;
    double hfov_ = 70.0;
    double mount_pitch_ = -30.0;
    double travelled_ = 0.0;
    bool guide_ = false;

    QPointF click_;
    double click_t_ = -1.0;
};

}  // namespace gcs::ui
