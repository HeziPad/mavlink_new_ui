#pragma once

#include "theme.hpp"

#include <QColor>
#include <QString>
#include <QWidget>

#include <array>
#include <chrono>

namespace gcs::ui {

enum class DockIcon { Plan, Upload, Launch, Takeoff, Auto, Guide, Hold, Land, Qrtl, Preflight };

struct DockSlotSpec {
    QString label;
    DockIcon icon = DockIcon::Plan;
    bool enabled = true;
    QString reason;            // tooltip / toast when disabled
    QString hint;              // tooltip when enabled
    bool active = false;       // toggled on (glowing border)
    QColor accent = theme::kCyan;
    bool hold_to_confirm = false;  // fires after a 1 s charge
    bool long_press = false;       // tap = triggered, hold = longPressed
};

// Floating "ability bar" with exactly six slots. Purely presentational: it
// reports intents (triggered / longPressed / blocked) and the MainWindow
// decides what they mean. Keys 1-6 are routed in via pressSlot/releaseSlot.
class ControlDock : public QWidget {
    Q_OBJECT

public:
    static constexpr int kSlots = 6;
    static constexpr double kHoldSeconds = 1.0;
    static constexpr double kLongPressSeconds = 0.8;

    explicit ControlDock(QWidget* parent);

    void setSlot(int i, const DockSlotSpec& spec);
    void setPending(int i, bool pending);
    void flash(int i, bool ok);
    void setProgress(int i, double fraction);  // < 0 hides the ring
    void pressSlot(int i);
    void releaseSlot(int i);
    void tick(double dt);

    QSize sizeHint() const override;

signals:
    void triggered(int slot);
    void longPressed(int slot);
    void blocked(int slot, const QString& reason);
    void holdCancelled(int slot);

protected:
    void paintEvent(QPaintEvent* e) override;
    void mousePressEvent(QMouseEvent* e) override;
    void mouseReleaseEvent(QMouseEvent* e) override;
    void mouseMoveEvent(QMouseEvent* e) override;
    void leaveEvent(QEvent* e) override;
    bool event(QEvent* e) override;

private:
    struct Slot {
        DockSlotSpec spec;
        double hover = 0.0;
        bool hovered = false;
        double press = 0.0;
        bool down = false;
        bool consumed = false;
        double hold_t = 0.0;  // wall-clock seconds since press (not summed frame dt)
        std::chrono::steady_clock::time_point pressed_at{};
        double charge = 0.0;
        bool pending = false;
        double flash = 0.0;
        bool flash_ok = true;
        double shake = 0.0;
        double progress = -1.0;
    };

    QRectF slotRect(int i) const;
    int slotAt(const QPointF& pos) const;
    void paintSlot(QPainter& p, int i);

    std::array<Slot, kSlots> slots_{};
    int mouse_slot_ = -1;
    double time_ = 0.0;
};

void drawDockIcon(QPainter& p, DockIcon icon, const QRectF& r, const QColor& c);

}  // namespace gcs::ui
