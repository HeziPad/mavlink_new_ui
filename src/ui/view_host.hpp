#pragma once

#include <QPainterPath>
#include <QWidget>

class QPainter;

namespace gcs::ui {

// Base for the two swappable views (map, camera). In "compact" (PiP) mode the
// view clips itself to a chamfered frame and paints the minimap/feed chrome;
// it stays non-opaque so the main view shows through the cut corners.
class HudView : public QWidget {
    Q_OBJECT

public:
    explicit HudView(QWidget* parent = nullptr);

    void setCompact(bool compact);
    bool compact() const { return compact_; }
    void setGlitch(double amount);

    virtual QString pipTag() const = 0;
    virtual QColor pipTagColor() const;

protected:
    virtual void compactChanged() {}
    QPainterPath viewPath() const;
    void beginView(QPainter& p) const;   // antialiasing + PiP clip
    void finishView(QPainter& p) const;  // PiP frame, scanlines, swap glitch

    bool compact_ = false;
    double glitch_ = 0.0;
    double time_ = 0.0;  // seconds, advanced by subclasses for ambient motion
};

// Main viewport + PiP overlay. Swapping animates geometry; clicking the PiP
// swaps too. Layout of the rest of the HUD is the MainWindow's business.
class ViewHost : public QWidget {
    Q_OBJECT

public:
    ViewHost(HudView* a, HudView* b, QWidget* parent = nullptr);

    void swap();
    HudView* mainView() const { return views_[main_]; }
    HudView* pipView() const { return views_[1 - main_]; }
    void setPipMaxWidth(double w);
    QRectF pipRect() const;
    void tick(double dt);

signals:
    void swapped();

protected:
    void resizeEvent(QResizeEvent* e) override;
    bool eventFilter(QObject* obj, QEvent* e) override;

private:
    void layoutViews();

    HudView* views_[2];
    int main_ = 0;
    double anim_ = 1.0;
    double pip_max_w_ = 1e9;
};

}  // namespace gcs::ui
