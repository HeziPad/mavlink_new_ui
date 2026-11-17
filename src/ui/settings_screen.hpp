#pragma once

#include <QPixmap>
#include <QWidget>

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
    explicit SettingsScreen(QWidget* parent);

    void open(int alt_unit);  // units::Alt
    void close();
    bool isOpen() const { return open_target_ > 0.5; }
    void tick(double dt);

signals:
    void altUnitChanged(int unit);
    void closed();

protected:
    void paintEvent(QPaintEvent* e) override;
    void mousePressEvent(QMouseEvent* e) override;

private:
    QRectF panelRect() const;

    QPixmap backdrop_;
    double open_ = 0.0;         // 0 hidden .. 1 shown
    double open_target_ = 0.0;
    double time_ = 0.0;
    int alt_unit_ = 0;
    double alt_knob_ = 0.0;     // segmented toggle highlight, animated
    double flash_ = 0.0;        // brief glow on change

    QRectF close_rect_;
    std::vector<QRectF> alt_seg_;  // [meters, feet]
};

}  // namespace gcs::ui
