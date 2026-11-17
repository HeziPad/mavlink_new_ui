#pragma once

#include <QElapsedTimer>
#include <QWidget>

#include <vector>

class QLabel;
class QPushButton;

namespace gcs::ui {

// Finger-sized control for the circle the aircraft is flying in GUIDED (Hold
// or a Guide target): altitude and radius steppers (press-and-hold repeats)
// and the turn direction. Purely presentational: it reports deltas in metres.
// Holding a stepper accelerates it (x1, then x5 after 1 s, x10 after 2.5 s) so
// the far ends of the range are reachable by touch. Altitude follows the
// display unit (m / ft); the radius is a distance and stays in metres.
class OrbitPanel : public QWidget {
    Q_OBJECT

public:
    explicit OrbitPanel(QWidget* parent);

    // target values; `current_alt` < 0 hides the "now" readout
    // `flown` < 0: not measured yet; `trimmed`: flown matches the request
    void setValues(const QString& title, float alt, float radius, bool ccw, double current_alt, double flown = -1.0,
                   bool trimmed = false);
    // relabel the altitude steppers after a unit change
    void unitsChanged();

signals:
    void altitudeDelta(float delta);
    void radiusDelta(float delta);
    void directionToggled();

protected:
    void paintEvent(QPaintEvent* e) override;

private:
    struct Step {
        QPushButton* button;
        double meters;  // nominal step in metres mode
        bool altitude;  // follows the display unit
    };
    QWidget* column(const QString& name, QLabel*& value, const std::initializer_list<int>& steps, bool altitude,
                    void (OrbitPanel::*signal)(float), QLabel** note = nullptr);
    double stepFor(const Step& st) const;
    int multiplier() const;

    std::vector<Step> steps_;
    QElapsedTimer press_t_;
    QString title_text_;
    int shown_mult_ = 1;
    bool holding_ = false;
    bool skip_click_ = false;

    QLabel* title_;
    QLabel* alt_;
    QLabel* alt_now_;
    QLabel* radius_;
    QLabel* radius_flown_ = nullptr;
    int flown_style_ = -1;  // 0 none, 1 approx, 2 trimmed: restyle only on change
    QPushButton* dir_;
};

}  // namespace gcs::ui
