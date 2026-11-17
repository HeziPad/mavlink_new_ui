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

    explicit SettingsScreen(QWidget* parent);

    void open(const Values& v);
    void close();
    bool isOpen() const { return open_target_ > 0.5; }
    void tick(double dt);

signals:
    void altUnitChanged(int unit);
    void trailChanged(bool on, int secs);
    void closed();

protected:
    void paintEvent(QPaintEvent* e) override;
    void mousePressEvent(QMouseEvent* e) override;

private:
    enum Category { kUnits = 0, kMap = 1, kCategories };
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

    QRectF close_rect_;
    std::vector<QRectF> cat_rects_;
};

}  // namespace gcs::ui
