#pragma once

#include "mavlink_receiver.hpp"

#include <QMainWindow>

class QLabel;
class QTimer;

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() override;

private slots:
    void refresh();

private:
    QLabel* addField(class QFormLayout* form, const QString& name);

    MavlinkReceiver receiver_;
    QTimer* timer_ = nullptr;
    bool announced_ = false;
    uint64_t rate_datagrams_ = 0;
    qint64 rate_ms_ = 0;
    double packet_rate_ = 0.0;

    QLabel* link_pill_ = nullptr;
    QLabel* link_detail_ = nullptr;

    QLabel* sysid_ = nullptr;
    QLabel* vehicle_ = nullptr;
    QLabel* autopilot_ = nullptr;
    QLabel* mode_ = nullptr;
    QLabel* armed_ = nullptr;
    QLabel* status_ = nullptr;
    QLabel* mavlink_ver_ = nullptr;

    QLabel* roll_ = nullptr;
    QLabel* pitch_ = nullptr;
    QLabel* yaw_ = nullptr;
    QLabel* rates_ = nullptr;

    QLabel* airspeed_ = nullptr;
    QLabel* groundspeed_ = nullptr;
    QLabel* heading_ = nullptr;
    QLabel* throttle_ = nullptr;
    QLabel* hud_alt_ = nullptr;
    QLabel* climb_ = nullptr;

    QLabel* lat_ = nullptr;
    QLabel* lon_ = nullptr;
    QLabel* alt_amsl_ = nullptr;
    QLabel* alt_rel_ = nullptr;
    QLabel* velocity_ = nullptr;
    QLabel* pos_heading_ = nullptr;
};
