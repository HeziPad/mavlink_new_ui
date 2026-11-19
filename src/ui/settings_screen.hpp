#pragma once

#include "link/serial_transport.hpp"

#include <QColor>
#include <QPixmap>
#include <QStringList>
#include <QWidget>

#include <map>
#include <vector>

namespace gcs::ui {

// Full-window, game-menu style settings overlay. Opening freezes a blurred,
// dimmed copy of the screen behind a chamfered glass panel that slides in;
// categories on the left, big segmented toggles on the right. Everything is
// custom painted and finger-sized. Changes apply live: the panel only
// reports them.
class SettingsScreen : public QWidget {
    Q_OBJECT

public:
    // how the GCS reaches the aircraft (CONNECTION page)
    struct Link {
        bool serial = false;
        QString host = QStringLiteral("0.0.0.0");  // UDP: listen here
        int port = 14550;
        QString remote_host;  // UDP: also send here from the start (empty = only reply)
        int remote_port = 0;
        QString serial_port;  // "/dev/ttyACM0", "COM4"
        int baud = 57600;
        int video_port = 5600;
        bool operator==(const Link& o) const {
            return serial == o.serial && host == o.host && port == o.port && remote_host == o.remote_host &&
                   remote_port == o.remote_port && serial_port == o.serial_port && baud == o.baud &&
                   video_port == o.video_port;
        }
        bool operator!=(const Link& o) const { return !(*this == o); }
    };
    enum LinkField { kListenField = 0, kSendToField = 1, kVideoField = 2 };

    struct Values {
        Link link;
        int alt_unit = 0;     // units::Alt
        int speed_unit = 0;   // units::Speed
        int coord_format = 0; // coords::Fmt
        bool trail_on = true;
        int trail_secs = 120; // 0 = everything kept
        bool labels_english = false;
        int reserve_pct = 20;  // battery time left: never plan into this much of the pack
        int text_size = 0;     // theme::TextSize
    };
    // the landing reserves offered (% of the pack)
    static const std::vector<int>& reserveChoices();
    // the trail lengths offered (seconds, 0 = all)
    static const std::vector<int>& trailChoices();
    // what the OFFLINE page shows; MainWindow keeps it current
    struct Offline {
        bool street_map = false;  // the vector street map is installed
        QString country;          // country pack status line
        int areas = 0;            // full-detail areas downloaded
        qint64 area_bytes = 0;
        bool running = false;
        QString stage;            // what the download is doing
        double fraction = 0.0;
        qint64 disk_bytes = -1;   // the whole offline folder (-1 unknown)
        QString folder;
        QString status;           // last result line
        bool online = false;      // online tiles while browsing
    };
    void setOffline(const Offline& o);

    // the link being edited (not applied until CONNECT)
    const Link& link() const { return link_; }
    void setLink(const Link& l);
    // what the link is doing now, shown under CONNECT; `armed` asks before switching
    void setLinkStatus(const QString& text, const QColor& color, bool armed);

    explicit SettingsScreen(QWidget* parent);

    void open(const Values& v);
    void close();
    bool isOpen() const { return open_target_ > 0.5; }
    void tick(double dt);

signals:
    void altUnitChanged(int unit);
    void speedUnitChanged(int unit);
    void coordFormatChanged(int fmt);
    void reserveChanged(int pct);
    void textSizeChanged(int size);  // theme::TextSize
    void trailChanged(bool on, int secs);
    void offlineCountry();       // download / update the country pack
    void offlineCancel();
    void offlineSelectArea();    // close and draw a full-detail area on the map
    void offlineClearDetail();   // confirmed
    void offlineDeleteAll();     // confirmed
    void offlineChooseFolder();
    void onlineChanged(bool on);
    void labelsEnglishChanged(bool english);
    void linkEdit(int field);              // LinkField: open the entry pad for it
    void linkConnect(const Link& link);    // apply (confirmed when armed)
    void closed();

protected:
    void paintEvent(QPaintEvent* e) override;
    void mousePressEvent(QMouseEvent* e) override;

private:
    enum Category { kLink = 0, kUnits, kDisplay, kMap, kBattery, kOffline, kCategories };
    // a segmented control: one finger-sized cell per option, sliding highlight
    struct Segmented {
        QStringList labels;
        int selected = 0;
        double knob = 0.0;  // animated position, in cells
        bool enabled = true;
        std::vector<QRectF> cells;
    };

    QRectF panelRect() const;
    QSizeF lsize() const;  // the widget in logical (text-size scaled) pixels
    void paintSegmented(QPainter& p, const QRectF& r, Segmented& seg);
    void paintRow(QPainter& p, const QRectF& r, const QString& title, const QString& sub);
    void paintOffline(QPainter& p, const QRectF& body);
    void paintLink(QPainter& p, const QRectF& body);
    void paintField(QPainter& p, const QRectF& r, const QString& name, const QString& value, const QString& note);
    void refreshPorts();
    bool pressLink(const QPointF& pos);
    void paintAction(QPainter& p, const QRectF& r, const QString& label, const QColor& c, bool enabled);
    bool pressSegmented(Segmented& seg, const QPointF& pos);

    QPixmap backdrop_;
    double open_ = 0.0;  // 0 hidden .. 1 shown
    double open_target_ = 0.0;
    double time_ = 0.0;
    double flash_ = 0.0;  // brief glow on change
    int category_ = kUnits;
    double cat_knob_ = 0.0;

    Values v_;
    Segmented alt_;
    Segmented speed_;
    Segmented coord_;
    Segmented trail_on_;
    Segmented trail_len_;
    Segmented labels_;
    Segmented online_;
    Segmented reserve_;
    Segmented text_size_;
    Offline off_;
    QRectF country_rect_, area_rect_, clear_rect_, delete_rect_, folder_rect_;
    // CONNECTION
    Link link_;          // being edited
    Link applied_;       // in use
    Segmented link_type_;
    Segmented baud_;
    std::vector<link::SerialPortInfo> ports_;
    double ports_age_ = 0.0;  // s since the list was read
    std::vector<QRectF> port_rects_;
    QRectF listen_rect_, sendto_rect_, video_rect_, connect_rect_, refresh_rect_;
    QString link_status_;
    QColor link_status_color_;
    bool armed_ = false;
    // "are you sure?" over the panel
    enum class Confirm { None, ClearDetail, DeleteAll, Relink } confirm_ = Confirm::None;
    QRectF confirm_yes_, confirm_no_;
    void paintConfirm(QPainter& p, const QRectF& panel);

    QRectF close_rect_;
    std::vector<QRectF> cat_rects_;
};

}  // namespace gcs::ui
