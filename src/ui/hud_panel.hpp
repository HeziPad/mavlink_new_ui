#pragma once

#include "core/hud_config.hpp"

#include <QRectF>
#include <QWidget>

#include <array>
#include <vector>

namespace gcs::ui {

// "HUD LOADOUT": a glass panel over the right of the video that edits the
// camera HUD live (hud::Config): presets, which elements show, the crosshair,
// line width and colour. Custom painted, finger-sized, scrolls when it is
// taller than the window; painted scaled by the text size. Every change is
// reported at once (configChanged), so the video behind it is the preview.
class HudPanel : public QWidget {
    Q_OBJECT

public:
    explicit HudPanel(QWidget* parent);

    void setConfig(const hud::Config& c);
    const hud::Config& config() const { return cfg_; }
    // right side of `area` (window coordinates), as tall as its content allows
    void place(const QRect& area);
    void tick(double dt);

signals:
    void configChanged();
    void closeRequested();

protected:
    void paintEvent(QPaintEvent* e) override;
    void mousePressEvent(QMouseEvent* e) override;
    void mouseMoveEvent(QMouseEvent* e) override;
    void mouseReleaseEvent(QMouseEvent* e) override;
    void wheelEvent(QWheelEvent* e) override;

private:
    struct Hit {
        QRectF rect;  // content coordinates (before scrolling)
        int kind;     // Kind
        int index;
    };
    enum Kind { kClose, kPreset, kElement, kCross, kLine, kColor };

    double contentHeight() const;
    void layoutHits(double width);  // fills hits_ for a panel `width` logical px wide
    void press(const Hit& h);
    void clampScroll();
    QRectF logicalRect() const;
    double viewTop() const;  // where the scrolling part starts (under the title)

    hud::Config cfg_;
    std::vector<Hit> hits_;
    QRect area_;
    double scroll_ = 0.0;
    bool pressing_ = false, dragged_ = false;
    QPointF press_pos_;
    double press_scroll_ = 0.0;
    double time_ = 0.0;
    // a short glow on what was just tapped
    int flash_kind_ = -1, flash_index_ = -1;
    double flash_ = 0.0;
};

}  // namespace gcs::ui
