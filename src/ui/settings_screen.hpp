#pragma once

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
    struct Values {
        int alt_unit = 0;     // units::Alt
        int speed_unit = 0;   // units::Speed
        int coord_format = 0; // coords::Fmt
        bool trail_on = true;
        int trail_secs = 120; // 0 = everything kept
        bool labels_english = false;
        int reserve_pct = 25;  // battery time left: never plan into this much of the pack
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
    void trailChanged(bool on, int secs);
    void offlineCountry();       // download / update the country pack
    void offlineCancel();
    void offlineSelectArea();    // close and draw a full-detail area on the map
    void offlineClearDetail();   // confirmed
    void offlineDeleteAll();     // confirmed
    void offlineChooseFolder();
    void onlineChanged(bool on);
    void labelsEnglishChanged(bool english);
    void closed();

protected:
    void paintEvent(QPaintEvent* e) override;
    void mousePressEvent(QMouseEvent* e) override;

private:
    enum Category { kUnits = 0, kMap = 1, kBattery = 2, kOffline = 3, kCategories };
    // a segmented control: one finger-sized cell per option, sliding highlight
    struct Segmented {
        QStringList labels;
        int selected = 0;
        double knob = 0.0;  // animated position, in cells
        bool enabled = true;
        std::vector<QRectF> cells;
    };

    QRectF panelRect() const;
    void paintSegmented(QPainter& p, const QRectF& r, Segmented& seg);
    void paintRow(QPainter& p, const QRectF& r, const QString& title, const QString& sub);
    void paintOffline(QPainter& p, const QRectF& body);
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
    Offline off_;
    QRectF country_rect_, area_rect_, clear_rect_, delete_rect_, folder_rect_;
    // "are you sure?" over the panel
    enum class Confirm { None, ClearDetail, DeleteAll } confirm_ = Confirm::None;
    QRectF confirm_yes_, confirm_no_;
    void paintConfirm(QPainter& p, const QRectF& panel);

    QRectF close_rect_;
    std::vector<QRectF> cat_rects_;
};

}  // namespace gcs::ui
