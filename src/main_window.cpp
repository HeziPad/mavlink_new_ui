#include "main_window.hpp"

#include <QDateTime>
#include <QFormLayout>
#include <QFrame>
#include <QHBoxLayout>
#include <QLabel>
#include <QStyle>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include <cmath>

namespace {

QString dash() {
    return QStringLiteral("—");
}

QString deg1(double value) {
    return QString::number(value, 'f', 1) + QChar(0x00B0);
}

QString mps(double value) {
    return QString::number(value, 'f', 2) + QStringLiteral(" m/s");
}

QString meters(double value) {
    return QString::number(value, 'f', 2) + QStringLiteral(" m");
}

QFrame* makeCard(const QString& title, QFormLayout** form_out) {
    auto* card = new QFrame;
    card->setObjectName(QStringLiteral("card"));
    auto* layout = new QVBoxLayout(card);
    layout->setContentsMargins(16, 14, 16, 14);
    layout->setSpacing(8);

    auto* heading = new QLabel(title);
    heading->setObjectName(QStringLiteral("cardTitle"));
    layout->addWidget(heading);

    auto* form = new QFormLayout;
    form->setHorizontalSpacing(18);
    form->setVerticalSpacing(6);
    form->setLabelAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    form->setFormAlignment(Qt::AlignTop);
    layout->addLayout(form);
    *form_out = form;
    return card;
}

const char* kStyle = R"(
QMainWindow, QWidget {
    background: #10131a;
    color: #e7ebf3;
    font-family: "Ubuntu", "Segoe UI", sans-serif;
    font-size: 13px;
}
QLabel#title {
    font-size: 22px;
    font-weight: 650;
    color: #f4f7fb;
}
QLabel#subtitle {
    color: #8b95a8;
    font-size: 12px;
}
QLabel#cardTitle {
    color: #9aa6ff;
    font-size: 12px;
    font-weight: 700;
    letter-spacing: 1.2px;
}
QLabel#key {
    color: #8b95a8;
}
QLabel#value {
    color: #d7fff2;
    font-family: "Ubuntu Mono", "DejaVu Sans Mono", monospace;
    font-size: 14px;
}
QLabel#pillLive {
    background: #143528;
    color: #5ee0a8;
    border: 1px solid #1f6b4a;
    border-radius: 11px;
    padding: 3px 12px;
    font-weight: 700;
}
QLabel#pillWait {
    background: #3a2e12;
    color: #f0c36a;
    border: 1px solid #7a5b1e;
    border-radius: 11px;
    padding: 3px 12px;
    font-weight: 700;
}
QLabel#pillDown {
    background: #3a1820;
    color: #ff8b9a;
    border: 1px solid #7a3140;
    border-radius: 11px;
    padding: 3px 12px;
    font-weight: 700;
}
QFrame#card {
    background: #181c27;
    border: 1px solid #2a3144;
    border-radius: 12px;
}
QFrame#header {
    background: #181c27;
    border: 1px solid #2a3144;
    border-radius: 12px;
}
)";

}  // namespace

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent), receiver_("127.0.0.1", 14551) {
    setWindowTitle(QStringLiteral("ArduCopter Telemetry"));
    resize(980, 680);
    setStyleSheet(QString::fromUtf8(kStyle));

    auto* central = new QWidget;
    auto* root = new QVBoxLayout(central);
    root->setContentsMargins(18, 16, 18, 16);
    root->setSpacing(12);

    auto* header = new QFrame;
    header->setObjectName(QStringLiteral("header"));
    auto* header_layout = new QHBoxLayout(header);
    header_layout->setContentsMargins(16, 12, 16, 12);

    auto* titles = new QVBoxLayout;
    auto* title = new QLabel(QStringLiteral("ArduCopter"));
    title->setObjectName(QStringLiteral("title"));
    auto* subtitle = new QLabel(QStringLiteral("MAVLink v2  ·  UDP 127.0.0.1:14551"));
    subtitle->setObjectName(QStringLiteral("subtitle"));
    titles->addWidget(title);
    titles->addWidget(subtitle);
    titles->setSpacing(0);

    link_pill_ = new QLabel(QStringLiteral("WAITING"));
    link_pill_->setObjectName(QStringLiteral("pillWait"));
    link_detail_ = new QLabel(dash());
    link_detail_->setObjectName(QStringLiteral("subtitle"));
    link_detail_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);

    auto* link_col = new QVBoxLayout;
    link_col->addWidget(link_pill_, 0, Qt::AlignRight);
    link_col->addWidget(link_detail_, 0, Qt::AlignRight);
    link_col->setSpacing(4);

    header_layout->addLayout(titles);
    header_layout->addStretch();
    header_layout->addLayout(link_col);
    root->addWidget(header);

    auto* grid = new QHBoxLayout;
    grid->setSpacing(12);
    auto* left = new QVBoxLayout;
    auto* right = new QVBoxLayout;
    left->setSpacing(12);
    right->setSpacing(12);

    QFormLayout* heartbeat_form = nullptr;
    QFormLayout* attitude_form = nullptr;
    QFormLayout* hud_form = nullptr;
    QFormLayout* pos_form = nullptr;
    left->addWidget(makeCard(QStringLiteral("HEARTBEAT"), &heartbeat_form), 1);
    left->addWidget(makeCard(QStringLiteral("ATTITUDE"), &attitude_form), 1);
    right->addWidget(makeCard(QStringLiteral("VFR HUD"), &hud_form), 1);
    right->addWidget(makeCard(QStringLiteral("GLOBAL POSITION"), &pos_form), 1);
    grid->addLayout(left, 1);
    grid->addLayout(right, 1);
    root->addLayout(grid, 1);
    setCentralWidget(central);

    sysid_ = addField(heartbeat_form, QStringLiteral("System"));
    vehicle_ = addField(heartbeat_form, QStringLiteral("Type"));
    autopilot_ = addField(heartbeat_form, QStringLiteral("Autopilot"));
    mode_ = addField(heartbeat_form, QStringLiteral("Mode"));
    armed_ = addField(heartbeat_form, QStringLiteral("Armed"));
    status_ = addField(heartbeat_form, QStringLiteral("Status"));
    mavlink_ver_ = addField(heartbeat_form, QStringLiteral("MAVLink"));

    roll_ = addField(attitude_form, QStringLiteral("Roll"));
    pitch_ = addField(attitude_form, QStringLiteral("Pitch"));
    yaw_ = addField(attitude_form, QStringLiteral("Yaw"));
    rates_ = addField(attitude_form, QStringLiteral("Rates"));

    airspeed_ = addField(hud_form, QStringLiteral("Airspeed"));
    groundspeed_ = addField(hud_form, QStringLiteral("Groundspeed"));
    heading_ = addField(hud_form, QStringLiteral("Heading"));
    throttle_ = addField(hud_form, QStringLiteral("Throttle"));
    hud_alt_ = addField(hud_form, QStringLiteral("Altitude"));
    climb_ = addField(hud_form, QStringLiteral("Climb"));

    lat_ = addField(pos_form, QStringLiteral("Latitude"));
    lon_ = addField(pos_form, QStringLiteral("Longitude"));
    alt_amsl_ = addField(pos_form, QStringLiteral("AMSL"));
    alt_rel_ = addField(pos_form, QStringLiteral("Relative"));
    velocity_ = addField(pos_form, QStringLiteral("Velocity NED"));
    pos_heading_ = addField(pos_form, QStringLiteral("Heading"));

    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, &MainWindow::refresh);
    timer_->start(100);
    receiver_.start();
    rate_ms_ = QDateTime::currentMSecsSinceEpoch();
}

MainWindow::~MainWindow() {
    receiver_.stop();
}

QLabel* MainWindow::addField(QFormLayout* form, const QString& name) {
    auto* key = new QLabel(name);
    key->setObjectName(QStringLiteral("key"));
    auto* value = new QLabel(dash());
    value->setObjectName(QStringLiteral("value"));
    value->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    form->addRow(key, value);
    return value;
}

void MainWindow::refresh() {
    const TelemetrySnapshot snap = receiver_.snapshot();
    const auto now = std::chrono::steady_clock::now();
    const qint64 now_ms = QDateTime::currentMSecsSinceEpoch();

    double age_s = -1.0;
    if (snap.last_rx.time_since_epoch().count() != 0) {
        age_s = std::chrono::duration<double>(now - snap.last_rx).count();
    }

    const double dt = (now_ms - rate_ms_) / 1000.0;
    if (dt >= 0.5) {
        packet_rate_ = static_cast<double>(snap.datagrams - rate_datagrams_) / dt;
        rate_datagrams_ = snap.datagrams;
        rate_ms_ = now_ms;
    }

    if (!snap.error.empty() && !snap.socket_open) {
        link_pill_->setText(QStringLiteral("ERROR"));
        link_pill_->setObjectName(QStringLiteral("pillDown"));
        link_detail_->setText(QString::fromStdString(snap.error));
    } else if (age_s >= 0.0 && age_s < 2.0) {
        link_pill_->setText(QStringLiteral("LIVE"));
        link_pill_->setObjectName(QStringLiteral("pillLive"));
        link_detail_->setText(
            QStringLiteral("%1 pkt/s   %2 msg   %3")
                .arg(packet_rate_, 0, 'f', 0)
                .arg(snap.messages)
                .arg(QString::fromStdString(snap.endpoint)));
    } else if (snap.socket_open) {
        link_pill_->setText(age_s < 0.0 ? QStringLiteral("WAITING") : QStringLiteral("STALE"));
        link_pill_->setObjectName(QStringLiteral("pillWait"));
        link_detail_->setText(
            age_s < 0.0
                ? QStringLiteral("listening on %1").arg(QString::fromStdString(snap.endpoint))
                : QStringLiteral("last packet %1 s ago").arg(age_s, 0, 'f', 1));
    }
    link_pill_->style()->unpolish(link_pill_);
    link_pill_->style()->polish(link_pill_);

    if (snap.has_heartbeat) {
        sysid_->setText(QStringLiteral("%1 / %2").arg(snap.sysid).arg(snap.compid));
        vehicle_->setText(QString::fromStdString(snap.vehicle_type));
        autopilot_->setText(QString::fromStdString(snap.autopilot));
        mode_->setText(QString::fromStdString(snap.flight_mode));
        armed_->setText(snap.armed ? QStringLiteral("ARMED") : QStringLiteral("DISARMED"));
        armed_->setStyleSheet(snap.armed ? QStringLiteral("color: #ff8b9a;")
                                          : QStringLiteral("color: #5ee0a8;"));
        status_->setText(QString::fromStdString(snap.system_status));
        mavlink_ver_->setText(QString::number(snap.mavlink_version));
    }

    if (snap.has_attitude) {
        roll_->setText(deg1(snap.roll_deg));
        pitch_->setText(deg1(snap.pitch_deg));
        yaw_->setText(deg1(snap.yaw_deg));
        rates_->setText(QStringLiteral("%1  %2  %3 °/s")
                            .arg(snap.roll_rate_dps, 0, 'f', 1)
                            .arg(snap.pitch_rate_dps, 0, 'f', 1)
                            .arg(snap.yaw_rate_dps, 0, 'f', 1));
    }

    if (snap.has_vfr_hud) {
        airspeed_->setText(mps(snap.airspeed_mps));
        groundspeed_->setText(mps(snap.groundspeed_mps));
        heading_->setText(QString::number(snap.heading_deg) + QChar(0x00B0));
        throttle_->setText(QString::number(snap.throttle_pct) + QStringLiteral(" %"));
        hud_alt_->setText(meters(snap.hud_alt_m));
        climb_->setText(mps(snap.climb_mps));
    }

    if (snap.has_global_position) {
        lat_->setText(QString::number(snap.latitude_deg, 'f', 7) + QChar(0x00B0));
        lon_->setText(QString::number(snap.longitude_deg, 'f', 7) + QChar(0x00B0));
        alt_amsl_->setText(meters(snap.alt_amsl_m));
        alt_rel_->setText(meters(snap.alt_rel_m));
        velocity_->setText(QStringLiteral("%1  %2  %3 m/s")
                               .arg(snap.vn_mps, 0, 'f', 2)
                               .arg(snap.ve_mps, 0, 'f', 2)
                               .arg(snap.vd_mps, 0, 'f', 2));
        if (snap.heading_valid) {
            pos_heading_->setText(QString::number(snap.heading_cdeg / 100.0, 'f', 1) + QChar(0x00B0));
        } else {
            pos_heading_->setText(dash());
        }
    }

    if (!announced_ && snap.has_heartbeat && snap.has_attitude && snap.has_vfr_hud &&
        snap.has_global_position) {
        announced_ = true;
        qInfo().noquote()
            << QStringLiteral("link live sysid=%1 mode=%2 armed=%3 lat=%4 lon=%5 alt=%6")
                   .arg(snap.sysid)
                   .arg(QString::fromStdString(snap.flight_mode))
                   .arg(snap.armed ? QStringLiteral("yes") : QStringLiteral("no"))
                   .arg(snap.latitude_deg, 0, 'f', 7)
                   .arg(snap.longitude_deg, 0, 'f', 7)
                   .arg(snap.alt_rel_m, 0, 'f', 2);
    }
}
