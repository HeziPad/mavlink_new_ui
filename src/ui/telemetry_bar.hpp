#pragma once

#include "anim.hpp"
#include "core/vehicle_state.hpp"

#include <QWidget>

#include <string>
#include <vector>

namespace gcs::ui {

class ChecklistPanel;

// Floating glass strip of segmented telemetry chips. Numbers ease towards new
// values; mode / VTOL chips flash on change. Hovering SYSTEMS opens a
// pre-flight checklist panel underneath.
class TelemetryBar : public QWidget {
    Q_OBJECT

public:
    explicit TelemetryBar(QWidget* parent);

    void setState(const VehicleState& s, Clock::time_point now);
    void tick(double dt);
    QSize sizeHint() const override { return {1040, 50}; }

protected:
    void paintEvent(QPaintEvent* e) override;
    void mouseMoveEvent(QMouseEvent* e) override;
    void leaveEvent(QEvent* e) override;
    void hideEvent(QHideEvent* e) override;

private:
    enum class Check { Ok, Warn, Bad, NotApplicable };
    struct CheckItem {
        QString name;
        QString detail;
        Check status = Check::NotApplicable;
    };

    std::vector<CheckItem> checklist() const;
    void showChecklist(bool on);

    VehicleState s_;
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
    bool sys_hover_ = false;
    ChecklistPanel* panel_ = nullptr;
};

}  // namespace gcs::ui
