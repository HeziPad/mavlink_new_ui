#pragma once

#include <QWidget>

class QLabel;
class QPushButton;

namespace gcs::ui {

// Opened from the AS · GS chip: the aircraft's three airspeeds (read-only:
// they are its own parameters, LOITER = AIRSPEED_MIN, CRUISE =
// AIRSPEED_CRUISE, MAX = AIRSPEED_MAX) and the SPEED MODE, the one choice
// the operator makes: which of them the GCS asks for in each situation.
class SpeedPanel : public QWidget {
    Q_OBJECT

public:
    explicit SpeedPanel(QWidget* parent);

    // < 0: unknown (not read yet)
    // loiter_flown: what is actually commanded when circling (kept above Q_ASSIST_SPEED)
    void applyScale();  // text size changed (theme::uiScale)
    void setValues(double loiter, double cruise, double max, int mode, const QString& active, double loiter_flown,
                   double q_assist);

signals:
    void modeChanged(int mode);  // speed::Mode
    void closeRequested();

protected:
    void paintEvent(QPaintEvent* e) override;

private:
    QLabel* loiter_;
    QLabel* cruise_;
    QLabel* max_;
    QLabel* explain_;
    QLabel* active_;
    QPushButton* modes_[3];
};

}  // namespace gcs::ui
