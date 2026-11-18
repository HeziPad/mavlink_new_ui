#pragma once

#include "core/mission.hpp"

#include <QWidget>

class QLabel;
class QPushButton;

namespace gcs::ui {

// Floating strip shown while planning: the add tool (waypoint / landing
// point), waypoint count, route length and time, landing point status,
// Undo / Clear / Cancel, and how the route is flown (loop / back and forth /
// one way then hold). All buttons are finger-sized. Keeps the dock at five.
class PlanStrip : public QWidget {
    Q_OBJECT

public:
    explicit PlanStrip(QWidget* parent);

    // route_m / route_s: one cycle of the pattern (see routeLength)
    void setInfo(int waypoints, double route_m, double route_s, bool has_land, bool can_undo, Pattern pattern);
    void setLandingTool(bool on);  // reflects MapView's add tool

signals:
    void landingToolToggled(bool on);
    void patternChanged(gcs::Pattern pattern);
    void undoRequested();
    void addByCoordRequested(bool landing);  // type a position for a new waypoint (or L)
    void clearRequested();
    void cancelRequested();

protected:
    void paintEvent(QPaintEvent* e) override;

private:
    QLabel* info_;
    QLabel* warn_;
    QPushButton* wp_tool_;
    QPushButton* land_tool_;
    QPushButton* coord_;
    QPushButton* undo_;
    QPushButton* clear_;
    QPushButton* cancel_;
    QPushButton* loop_;
    QPushButton* back_forth_;
    QPushButton* one_way_;
    QLabel* hint_;
};

}  // namespace gcs::ui
