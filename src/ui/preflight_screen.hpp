#pragma once

#include "preflight_runner.hpp"

#include <QWidget>

#include <string>
#include <vector>

namespace gcs::ui {

// The preflight window, a full-window overlay in three views that follow
// the runner's flow:
//   look    "look at the aircraft": big tick rows and CONTINUE
//   checks  the list filling in, one row at a time, and on the right what
//           is being checked now (or the hold before the moving parts)
//   result  READY TO FLY, or the problems and notes in plain words, each
//           with at most one action
// Touch-first: every target is at least 48 px; whatever spins or changes
// the vehicle needs a hold.
class PreflightScreen : public QWidget {
    Q_OBJECT

public:
    PreflightScreen(QWidget* parent, PreflightRunner* runner);

    void open(bool turnaround);
    void close();
    bool isOpen() const { return open_target_ > 0.5; }
    void tick(double dt);

signals:
    void closed();
    void launchOverride();  // the 3 s "launch without preflight" hold completed

protected:
    void paintEvent(QPaintEvent* e) override;
    void mousePressEvent(QMouseEvent* e) override;
    void mouseReleaseEvent(QMouseEvent* e) override;
    void mouseMoveEvent(QMouseEvent* e) override;
    void wheelEvent(QWheelEvent* e) override;

private:
    struct Hit {
        QRectF rect;
        std::string kind;    // "close" "look" "continue" "gate" "stop" "override" "rerun" "restart"
                             // "setup" "cal" "calcancel" "reboot" "ready"
        std::string value;
        bool hold = false;
        double hold_secs = 1.0;
        bool enabled = true;
        QString reason;
    };

    void paintTop(QPainter& p, const QRectF& r);
    void paintLook(QPainter& p, const QRectF& r);
    void paintChecks(QPainter& p, const QRectF& r);
    void paintNow(QPainter& p, const QRectF& r);   // the right side of the checks view
    void paintGate(QPainter& p, const QRectF& r);
    void paintResult(QPainter& p, const QRectF& r);
    void paintCal(QPainter& p, const QRectF& r);
    void paintArt(QPainter& p, const QRectF& r, const std::string& id);
    void paintButton(QPainter& p, const QRectF& r, const QString& label, PreflightRunner::Style style, bool enabled,
                     bool hold, double charge);
    void addButton(QPainter& p, const QRectF& r, const QString& label, PreflightRunner::Style style, Hit h);
    void fire(const Hit& h);
    void clampScroll();

    PreflightRunner* runner_;
    double open_ = 0.0;
    double open_target_ = 0.0;
    double time_ = 0.0;
    preflight::Phase last_phase_ = preflight::Phase::Idle;

    // the one scrolling list of the view shown
    double list_scroll_ = 0.0;
    double list_content_h_ = 0.0;
    QRectF list_rect_;
    double user_scrolled_t_ = -100.0;  // when the operator last dragged the list
    int last_row_ = -1;

    std::vector<Hit> hits_;
    int pressed_ = -1;          // index into hits_ while held
    double hold_t_ = 0.0;
    bool dragging_ = false;
    QPointF press_pos_;
    double press_scroll_ = 0.0;
    QString toast_;
    double toast_t_ = 0.0;
};

}  // namespace gcs::ui
