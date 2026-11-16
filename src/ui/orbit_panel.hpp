#pragma once

#include <QWidget>

class QLabel;
class QPushButton;

namespace gcs::ui {

// Finger-sized control for the circle the aircraft is flying in GUIDED (Hold
// or a Guide target): altitude and radius steppers (press-and-hold repeats)
// and the turn direction. Purely presentational: it reports deltas.
class OrbitPanel : public QWidget {
    Q_OBJECT

public:
    explicit OrbitPanel(QWidget* parent);

    // target values; `current_alt` < 0 hides the "now" readout
    void setValues(const QString& title, float alt, float radius, bool ccw, double current_alt);

signals:
    void altitudeDelta(float delta);
    void radiusDelta(float delta);
    void directionToggled();

protected:
    void paintEvent(QPaintEvent* e) override;

private:
    QWidget* column(const QString& name, QLabel*& value, const std::initializer_list<int>& steps,
                    void (OrbitPanel::*signal)(float));

    QLabel* title_;
    QLabel* alt_;
    QLabel* alt_now_;
    QLabel* radius_;
    QPushButton* dir_;
};

}  // namespace gcs::ui
