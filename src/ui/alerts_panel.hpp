#pragma once

#include "core/health.hpp"

#include <QWidget>

#include <map>
#include <set>
#include <string>
#include <vector>

namespace gcs::ui {

// The SYSTEMS panel: a row of group status pills (link, GPS, navigation,
// sensors, power, arming) over a scrollable stack of advisory cards. Each card
// says what's wrong, why it matters and what to try, with the raw data in small
// print. Opened by tapping SYSTEMS or the red alert banner; drag or wheel to
// scroll; ✕ closes.
class AlertsPanel : public QWidget {
    Q_OBJECT

public:
    explicit AlertsPanel(QWidget* parent);

    void setAdvisories(const std::vector<Advisory>& list, bool link_alive);
    void place(const QRect& area);  // right-aligned inside `area`
    void tick(double dt);

protected:
    void paintEvent(QPaintEvent* e) override;
    void mousePressEvent(QMouseEvent* e) override;
    void mouseMoveEvent(QMouseEvent* e) override;
    void mouseReleaseEvent(QMouseEvent* e) override;
    void wheelEvent(QWheelEvent* e) override;

private:
    double cardHeight(const Advisory& a, double w) const;
    double contentHeight(double w) const;
    void clampScroll();

    std::vector<Advisory> list_;
    bool alive_ = false;
    std::map<std::string, double> age_;  // per id, for the slide-in
    // cards show their title only until tapped; criticals arrive open
    std::set<std::string> expanded_;
    bool isExpanded(const Advisory& a) const { return expanded_.count(a.id) > 0; }
    int cardAt(const QPointF& pos) const;  // index into list_, -1 none
    double time_ = 0.0;
    double scroll_ = 0.0;
    QRect area_;

    bool pressing_ = false;
    bool dragged_ = false;
    QPointF press_pos_;
    double press_scroll_ = 0.0;
};

}  // namespace gcs::ui
