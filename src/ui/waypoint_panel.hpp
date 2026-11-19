#pragma once

#include <QWidget>

#include <utility>
#include <vector>

class QLabel;
class QPushButton;

namespace gcs::ui {

// Finger-sized editor for the selected plan point: altitude steppers
// (press-and-hold repeats), insert-after and delete, and for the final hold
// point (the end of a one-way route, or a single point) the hold circle's
// radius and direction. Shown above the dock while a point is selected in
// Plan mode.
class WaypointPanel : public QWidget {
    Q_OBJECT

public:
    explicit WaypointPanel(QWidget* parent);

    void showWaypoint(int index, float alt_rel);
    void showLanding(float approach_alt);
    // the HOLD row. Final: the end of a one-way route (or a single point),
    // circled until told otherwise (radius / direction). Timed: any other
    // waypoint, a time (0 = fly through) and a radius. The GCS ends the hold
    // when the time is up, whatever the radius.
    enum class HoldMode { None, Final, Timed };
    struct HoldView {
        HoldMode mode = HoldMode::None;
        float hold_s = 0.0f;
        float radius = 0.0f;  // 0 = default
        bool ccw = false;
        double default_radius = 80.0;
    };
    void setHold(const HoldView& v);
    // relabel the steppers after a display unit change (they step in m or ft)
    void unitsChanged();
    void applyScale();  // text size changed (theme::uiScale)

signals:
    void altitudeDelta(float delta);
    void insertRequested();
    void deleteRequested();
    void closeRequested();
    void holdRadiusDelta(float delta);
    void holdDirectionToggled();
    void holdTimeDelta(float seconds);
    void holdValueTapped();     // type the time
    void coordRequested();   // type the point's coordinates
    void altitudeTapped();   // type its altitude

protected:
    void paintEvent(QPaintEvent* e) override;

private:
    void relayout();

    QLabel* title_;
    QLabel* alt_;
    QLabel* note_;
    QWidget* steppers_;
    QPushButton* insert_;
    QPushButton* delete_;
    bool landing_ = false;
    QWidget* hold_row_;
    QLabel* hold_title_;
    QWidget* hold_time_;     // time steppers
    QLabel* hold_time_label_;  // the time (tap: type it)
    QWidget* hold_circle_;   // radius label + steppers + direction
    QLabel* hold_radius_;
    QPushButton* hold_dir_;
    HoldMode hold_mode_ = HoldMode::None;

protected:
    bool eventFilter(QObject* obj, QEvent* e) override;
    std::vector<std::pair<QPushButton*, int>> alt_steps_;  // button, metres-mode step
};

}  // namespace gcs::ui
