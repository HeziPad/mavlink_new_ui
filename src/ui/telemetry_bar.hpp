#pragma once

#include "anim.hpp"
#include "core/health.hpp"

#include <QWidget>

#include <string>
#include <vector>

namespace gcs::ui {

// Floating glass strip of segmented telemetry chips. Numbers ease towards new
// values; mode / VTOL chips flash on change. SYSTEMS summarises the
// advisories (tap: the window opens the alerts panel). LIFT ROTORS shows whether
// the VTOL motors are idle, in expected use, or assisting in cruise.
class TelemetryBar : public QWidget {
    Q_OBJECT

public:
    explicit TelemetryBar(QWidget* parent);

    void setState(const VehicleState& s, Clock::time_point now);
    void setAdvisories(const std::vector<Advisory>& list);
    void setSystemsOpen(bool open);
    void tick(double dt);
    QSize sizeHint() const override { return {1330, 50}; }

signals:
    void settingsRequested();
    void systemsTapped();
    // SAFETY held for kSafetyHold: toggle it (`make_safe` = engage)
    void safetyToggleRequested(bool make_safe);
    void safetyBlocked(const QString& reason);

protected:
    void paintEvent(QPaintEvent* e) override;
    void mousePressEvent(QMouseEvent* e) override;
    void mouseReleaseEvent(QMouseEvent* e) override;

private:
    VehicleState s_;
    Clock::time_point now_{};
    bool alive_ = false;
    double hb_age_ = 1e9;
    double msg_rate_ = 0.0;
    uint64_t rate_msgs_ = 0;
    Clock::time_point rate_t_{};

    Tween alt_, as_, gs_, batt_;
    Pulse mode_flash_, vtol_flash_;
    std::string last_mode_;
    int last_vtol_ = -1;
    double time_ = 0.0;

    QRectF sys_rect_;
    QRectF gear_rect_;
    QRectF safety_rect_;
    static constexpr double kSafetyHold = 1.0;  // s, both directions
    bool safety_down_ = false;
    double safety_hold_ = 0.0;     // s held
    double safety_flash_ = 0.0;
    double safety_pending_ = 0.0;  // s left waiting for the state to change
    SafetyState safety_sent_from_ = SafetyState::Unknown;
    QString safetyLockReason() const;
    double gear_spin_ = 0.0;
    bool sys_open_ = false;
    int adv_bad_ = 0;   // Warning + Critical
    int adv_warn_ = 0;  // Caution
    bool adv_critical_ = false;
};

}  // namespace gcs::ui
