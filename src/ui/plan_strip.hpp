#pragma once

#include <QWidget>

class QLabel;
class QPushButton;

namespace gcs::ui {

// Floating strip shown while planning: the add tool (waypoint / landing
// point), waypoint count, lap length and time, landing point status, and
// Undo / Clear / Cancel. All buttons are finger-sized. Keeps the dock at six.
class PlanStrip : public QWidget {
    Q_OBJECT

public:
    explicit PlanStrip(QWidget* parent);

    void setInfo(int waypoints, double lap_m, double lap_s, bool has_land, bool can_undo);
    void setLandingTool(bool on);  // reflects MapView's add tool

signals:
    void landingToolToggled(bool on);
    void undoRequested();
    void clearRequested();
    void cancelRequested();

protected:
    void paintEvent(QPaintEvent* e) override;

private:
    QLabel* info_;
    QLabel* warn_;
    QPushButton* wp_tool_;
    QPushButton* land_tool_;
    QPushButton* undo_;
    QPushButton* clear_;
    QPushButton* cancel_;
};

}  // namespace gcs::ui
