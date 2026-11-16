#pragma once

#include <QWidget>

class QLabel;
class QPushButton;

namespace gcs::ui {

// Floating strip shown while planning: waypoint count, route length, ETA,
// final-approach warning, and Undo / Clear / Cancel. Keeps the dock at six.
class PlanStrip : public QWidget {
    Q_OBJECT

public:
    explicit PlanStrip(QWidget* parent);

    // approach_m < 0 hides the warning
    void setInfo(int waypoints, double route_m, double eta_s, double approach_m, double min_approach_m,
                 bool can_undo);

signals:
    void undoRequested();
    void clearRequested();
    void cancelRequested();

protected:
    void paintEvent(QPaintEvent* e) override;

private:
    QLabel* info_;
    QLabel* warn_;
    QPushButton* undo_;
    QPushButton* clear_;
    QPushButton* cancel_;
};

}  // namespace gcs::ui
