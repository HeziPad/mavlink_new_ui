#pragma once

#include <QColor>
#include <QString>
#include <QWidget>

#include <deque>

namespace gcs::ui {

// Full-window, mouse-transparent layer for transient feedback:
//  - comms log: toasts sliding in from the right, collapsing to a short feed
//  - banner: centre sweep for mode / VTOL changes ("MODE · QLOITER")
//  - alert: persistent red banner + pulsing edge vignette (link lost, battery)
class FxOverlay : public QWidget {
    Q_OBJECT

public:
    explicit FxOverlay(QWidget* parent);

    void toast(const QString& text, const QColor& color, const QString& tag = {});
    void banner(const QString& text, const QColor& color);
    void setAlert(const QString& text);  // empty clears
    void setTopInset(double px) { top_inset_ = px; }
    // toasts keep clear of a panel open on the right (animated)
    void setRightInset(double px) { right_inset_target_ = px; }
    // where the alert banner is drawn (empty when none): the window makes it tappable
    QRectF alertRect() const { return alert_rect_; }
    void tick(double dt);

protected:
    void paintEvent(QPaintEvent* e) override;

private:
    struct Toast {
        QString text;
        QString tag;
        QString time;
        QColor color;
        double age = 0.0;
        int count = 1;
    };
    struct Banner {
        QString text;
        QColor color;
        double t = -1.0;
    };

    void paintToasts(QPainter& p);
    void paintBanner(QPainter& p);
    void paintAlert(QPainter& p);

    std::deque<Toast> toasts_;
    Banner banner_;
    QString alert_;
    double alert_t_ = 0.0;
    double alert_fade_ = 0.0;
    double top_inset_ = 64.0;
    QRectF alert_rect_;
    double right_inset_ = 0.0;
    double right_inset_target_ = 0.0;
};

}  // namespace gcs::ui
