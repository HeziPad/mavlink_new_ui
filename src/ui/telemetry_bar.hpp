#pragma once

#include "anim.hpp"
#include "core/flight_status.hpp"
#include "core/display_round.hpp"
#include "core/health.hpp"

#include <optional>

#include <QWidget>

#include <string>
#include <vector>

namespace gcs::ui {

// Floating glass strip of the essentials, left to right: LINK (bars only),
// BATTERY, TIME LEFT, FLIGHT, ETA, ALT, SPEED (in words), SYSTEMS, settings.
// Numbers ease towards new values; the flight chip flashes on a change.
// SYSTEMS summarises the advisories (tap: the window opens the alerts panel).
// Altitudes are shown to the nearest 10 (display unit) and only move on once
// clearly past a boundary (core/display_round).
class TelemetryBar : public QWidget {
    Q_OBJECT

public:
    explicit TelemetryBar(QWidget* parent);

    // The bar grows with the text size (theme::uiScale) but never past the
    // width it is given: all chips stay on one row.
    double effectiveScale(double width) const;
    double effectiveScale() const { return effectiveScale(width()); }
    int heightFor(int width) const;

    void setState(const VehicleState& s, Clock::time_point now);
    void setAdvisories(const std::vector<Advisory>& list);
    void setSystemsOpen(bool open);
    void setFlightStatus(const FlightStatus& f);
    void setNavTarget(const NavTarget& t) { nav_ = t; }  // the ETA chip
    // the SPEED chip: what is commanded (speed::Target) and the operator's
    // speed mode (speed::Mode: ECO / NORMAL / FAST)
    void setSpeed(int target, int mode, bool airborne);
    // the altitude chip's other references (ASL comes from the state):
    // AGL (rangefinder or terrain) and AFL (above the landing point's ground); nullopt = unknown
    void setAltitudes(std::optional<double> agl, std::optional<double> afl);
    // what the GCS asked the aircraft to fly at (height above home); nullopt = nothing
    void setCommandedAlt(std::optional<double> alt_rel) { cmd_alt_ = alt_rel; }
    void setAltRef(int ref) { alt_ref_ = ref; }
    // Battery time left (core/endurance): flying time before it must head
    // back, estimated at `at`; the chip counts down from there. level:
    // endurance::Level (0 none .. 3 critical).
    void setEndurance(bool valid, double time_left_s, int level, Clock::time_point at);
    void tick(double dt);
    QSize sizeHint() const override { return {1440, 50}; }

signals:
    void settingsRequested();
    void altRefChanged(int ref);  // tapped: 0 ASL, 1 AGL, 2 AFL
    void speedTapped();           // the SPEED chip: the airspeed panel
    void systemsTapped();

protected:
    void paintEvent(QPaintEvent* e) override;
    void mousePressEvent(QMouseEvent* e) override;

private:
    VehicleState s_;
    Clock::time_point now_{};
    bool alive_ = false;
    double hb_age_ = 1e9;
    double msg_rate_ = 0.0;
    uint64_t rate_msgs_ = 0;
    Clock::time_point rate_t_{};

    Tween batt_;
    Pulse mode_flash_;  // the flight chip, on a change of state
    FlightStatus flight_;
    NavTarget nav_;
    int alt_ref_ = 0;
    std::optional<double> agl_, afl_;
    std::optional<double> cmd_alt_;
    // to the nearest 10 in the display unit, moving on 2 past the boundary
    display::StickyRound alt_shown_{10.0, 2.0}, cmd_shown_{10.0, 2.0};
    int alt_shown_ref_ = -1;  // the reference + unit the sticky values belong to
    QRectF alt_rect_;
    QRectF spd_rect_;
    int speed_target_ = 0;  // speed::Target
    int speed_mode_ = 1;    // speed::Mode
    bool speed_airborne_ = false;
    double spd_flash_ = 0.0;  // a tap on the chip
    double time_ = 0.0;

    QRectF sys_rect_;
    QRectF gear_rect_;
    double gear_spin_ = 0.0;
    bool sys_open_ = false;
    int adv_bad_ = 0;   // Warning + Critical
    int adv_warn_ = 0;  // Caution
    bool adv_critical_ = false;

    bool endurance_valid_ = false;
    double endurance_s_ = 0.0;
    int endurance_level_ = 0;
    Clock::time_point endurance_at_{};
    QRectF batt_rect_;
};

}  // namespace gcs::ui
