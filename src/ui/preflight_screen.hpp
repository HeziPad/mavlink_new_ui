#pragma once

#include "preflight_runner.hpp"

#include <QWidget>

#include <string>
#include <vector>

namespace gcs::ui {

// The preflight window: a full-window overlay with the stage rail on the
// left, the steps of the stage, the step card (depiction, what to do, live
// result, actions) and a live data column. Touch-first: every target is at
// least 48 px; actions that spin or change the vehicle need a 1 s hold.
class PreflightScreen : public QWidget {
    Q_OBJECT

public:
    PreflightScreen(QWidget* parent, PreflightRunner* runner);

    void open(bool turnaround);
    void close();
    bool isOpen() const { return open_target_ > 0.5; }
    void tick(double dt);
    void selectStep(const std::string& id);
    const std::string& currentStep() const { return step_; }

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
        std::string kind;    // "stage" "step" "btn" "close" "prev" "next" "stop" "override" "full" "turn" "ready"
        std::string value;   // stage index / step id / action
        bool hold = false;
        double hold_secs = 1.0;
        bool enabled = true;
        QString reason;
    };

    void paintRail(QPainter& p, const QRectF& r);
    void paintSteps(QPainter& p, const QRectF& r);
    void paintCard(QPainter& p, const QRectF& r);
    void paintArt(QPainter& p, const QRectF& r);
    void paintData(QPainter& p, const QRectF& r);
    void paintReady(QPainter& p, const QRectF& r);
    void paintButton(QPainter& p, const QRectF& r, const QString& label, PreflightRunner::Style style, bool enabled,
                     bool hold, double charge);
    std::vector<const preflight::StepDef*> stageSteps(int stage) const;
    void selectStage(int stage);
    void stepBy(int delta);
    void fire(const Hit& h);

    PreflightRunner* runner_;
    double open_ = 0.0;
    double open_target_ = 0.0;
    double time_ = 0.0;
    int stage_ = 0;
    std::string step_;
    double list_scroll_ = 0.0;
    double list_content_h_ = 0.0;
    QRectF list_rect_;
    bool ready_shown_ = false;
    bool ready_dismissed_ = false;

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
