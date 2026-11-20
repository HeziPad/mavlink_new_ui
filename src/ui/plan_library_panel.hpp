#pragma once

#include "ui/plan_library.hpp"

#include <QWidget>

#include <functional>
#include <optional>
#include <vector>

namespace gcs::ui {

// The PLANS panel, opened from the plan strip: the saved plans as a list
// (tap one to select it, then OPEN / RENAME / DELETE) under SAVE, SAVE AS and
// NO PLAN for what is in the editor. Drag or wheel to scroll. Questions
// (delete? discard the edits?) are asked inside the panel. It has no close
// button: a tap outside closes it (MainWindow). Chrome widget: scaled by
// uiScale(), laid out in logical pixels.
class PlanLibraryPanel : public QWidget {
    Q_OBJECT

public:
    explicit PlanLibraryPanel(QWidget* parent);

    struct EditorInfo {
        QString name;          // empty: not saved yet
        QString id;            // the saved plan it came from, if any
        bool empty = true;     // nothing in the editor
        bool edited = false;   // differs from its saved copy (or not saved at all)
    };
    void setEntries(std::vector<PlanLibrary::Entry> entries, const EditorInfo& editor);
    void place(const QRect& area);  // right-aligned inside `area`
    // a question inside the panel; `yes` runs on the first button
    void ask(const QString& title, const QString& detail, const QString& yes_label, const QColor& yes_color,
             std::function<void()> yes);

signals:
    void openRequested(const QString& id);
    void renameRequested(const QString& id);
    void deleteRequested(const QString& id);  // already confirmed
    void saveRequested();                     // SAVE (asks for a name when there is none)
    void saveAsRequested();
    void noPlanRequested();

protected:
    void paintEvent(QPaintEvent* e) override;
    void mousePressEvent(QMouseEvent* e) override;
    void mouseMoveEvent(QMouseEvent* e) override;
    void mouseReleaseEvent(QMouseEvent* e) override;
    void wheelEvent(QWheelEvent* e) override;

private:
    struct Hit {
        QRectF rect;
        std::function<void()> fn;
        bool enabled = true;
    };
    double rowHeight(int i) const;
    double contentHeight() const;
    QRectF listRect() const;
    void clampScroll();
    void tapAt(const QPointF& pos);

    std::vector<PlanLibrary::Entry> entries_;
    EditorInfo editor_;
    QString selected_;
    QRect area_;
    double scroll_ = 0.0;

    // the open question, if any
    struct Question {
        QString title, detail, yes_label;
        QColor yes_color;
        std::function<void()> yes;
    };
    std::optional<Question> question_;

    std::vector<Hit> hits_;  // rebuilt on each paint (logical coordinates)
    bool pressing_ = false;
    bool dragged_ = false;
    QPointF press_pos_;
    double press_scroll_ = 0.0;
};

}  // namespace gcs::ui
