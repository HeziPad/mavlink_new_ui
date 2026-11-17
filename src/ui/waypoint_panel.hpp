#pragma once

#include <QWidget>

#include <utility>
#include <vector>

class QLabel;
class QPushButton;

namespace gcs::ui {

// Finger-sized editor for the selected plan point: altitude steppers
// (press-and-hold repeats), insert-after and delete. Shown above the dock
// while a point is selected in Plan mode.
class WaypointPanel : public QWidget {
    Q_OBJECT

public:
    explicit WaypointPanel(QWidget* parent);

    void showWaypoint(int index, float alt_rel);
    void showLanding(float approach_alt);
    // relabel the steppers after a display unit change (they step in m or ft)
    void unitsChanged();

signals:
    void altitudeDelta(float delta);
    void insertRequested();
    void deleteRequested();
    void closeRequested();

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
    std::vector<std::pair<QPushButton*, int>> alt_steps_;  // button, metres-mode step
};

}  // namespace gcs::ui
