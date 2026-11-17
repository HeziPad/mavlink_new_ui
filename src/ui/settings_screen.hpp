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
        bool trail_on = true;
        int trail_secs = 120; // 0 = everything kept
    };
    // the trail lengths offered (seconds, 0 = all)
    static const std::vector<int>& trailChoices();
    // offline area radii offered (km)
    static const std::vector<int>& offlineRadii();

    // what the OFFLINE page shows; MainWindow keeps it current
    struct Offline {
        QString center;         // "around the aircraft" / "around the map centre"
        int radius_idx = 1;
        size_t tiles = 0;       // estimate for the selected radius
        qint64 bytes = 0;
        bool running = false;
        size_t done = 0, total = 0, failed = 0;
        qint64 disk_bytes = -1; // in the folder now (-1 unknown)
        QString folder;
        QString status;         // last result line
    };
    void setOffline(const Offline& o);

    explicit SettingsScreen(QWidget* parent);

    void open(const Values& v);
    void close();
    bool isOpen() const { return open_target_ > 0.5; }
    void tick(double dt);

signals:
    void altUnitChanged(int unit);
    void trailChanged(bool on, int secs);
    void offlineRadiusChanged(int km);
    void offlineDownload();
    void offlineCancel();
    void offlineDelete();
    void offlineChooseFolder();
    void closed();

protected:
    void paintEvent(QPaintEvent* e) override;
    void mousePressEvent(QMouseEvent* e) override;

private:
    enum Category { kUnits = 0, kMap = 1, kOffline = 2, kCategories };
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
    Segmented trail_on_;
    Segmented trail_len_;
    Segmented radius_;
    Offline off_;
    QRectF dl_rect_, del_rect_, folder_rect_;
    bool del_armed_ = false;  // DELETE asks once more

    QRectF close_rect_;
    std::vector<QRectF> cat_rects_;
};

}  // namespace gcs::ui
