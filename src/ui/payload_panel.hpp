#pragma once

#include "payload/adapter.hpp"
#include "payload/payload_manager.hpp"

#include <QRectF>
#include <QString>
#include <QWidget>

#include <deque>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace gcs::ui {

// "PAYLOAD": a glass panel over the right of the window, generated from the
// payload ICDs (icds/*.json): the adapter's one-tap actions, every report as
// it arrives (decoded, with units and labels), every command with editable
// parameters, and the result of each command. Custom painted, finger-sized,
// scrolls; works for any ICD, with or without an adapter.
class PayloadPanel : public QWidget {
    Q_OBJECT

public:
    explicit PayloadPanel(QWidget* parent);

    void setSnapshot(const payload::PayloadSnapshot& s);
    // the selected device's adapter output and actions (empty without an adapter)
    void setAdapterInfo(const std::optional<payload::PayloadView>& view,
                        const std::vector<std::pair<std::string, payload::CommandCall>>& actions);
    void addResult(const QString& text, bool ok);
    std::string selectedIcd() const { return selected_; }
    void place(const QRect& area);
    void tick(double dt);

signals:
    void commandRequested(const QString& icd_id, const QString& name, const std::vector<double>& args);
    // an integer parameter tapped: the window opens its number pad
    void numberRequested(const QString& title, int value, int min, int max);
    void closeRequested();

public slots:
    void setEditedNumber(int value);  // the number pad's answer for the parameter being edited

protected:
    void paintEvent(QPaintEvent* e) override;
    void mousePressEvent(QMouseEvent* e) override;
    void mouseMoveEvent(QMouseEvent* e) override;
    void mouseReleaseEvent(QMouseEvent* e) override;
    void wheelEvent(QWheelEvent* e) override;

private:
    enum Kind { kDevice, kQuick, kCmdToggle, kCmdRow, kMinus, kValue, kPlus, kSend, kReport, kFilter, kReportsLabel, kResults };
    struct Item {
        QRectF rect;   // content coordinates
        int kind = -1;
        int index = 0;
    };

    const payload::DeviceSnapshot* device() const;
    const payload::CommandDef* command() const;
    double layout(double width, std::vector<Item>* items) const;  // -> content height
    void press(const Item& it);
    void selectCommand(int index);
    double step(const payload::ParamDef& p) const;
    void clampScroll();
    QRectF logicalRect() const;
    QString paramText(const payload::ParamDef& p, double v) const;

    payload::PayloadSnapshot snap_;
    std::string selected_;
    std::optional<payload::PayloadView> view_;
    std::vector<std::pair<std::string, payload::CommandCall>> actions_;
    bool commands_open_ = false;
    int filter_ = 0;               // command group shown (0 = all)
    int command_ = -1;             // index into the ICD's commands
    std::vector<double> args_;
    int editing_param_ = -1;
    std::map<int, bool> open_reports_;
    std::deque<std::pair<QString, bool>> log_;
    QRect area_;
    double scroll_ = 0.0;
    bool pressing_ = false, dragged_ = false;
    QPointF press_pos_;
    double press_scroll_ = 0.0;
    double time_ = 0.0;
    double flash_ = 0.0;
    int flash_kind_ = -1, flash_index_ = -1;
};

}  // namespace gcs::ui
