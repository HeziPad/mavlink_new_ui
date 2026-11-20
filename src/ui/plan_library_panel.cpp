#include "ui/plan_library_panel.hpp"

#include "ui/theme.hpp"

#include <QFontMetricsF>
#include <QMouseEvent>
#include <QPainter>
#include <QWheelEvent>

#include <algorithm>
#include <cmath>

namespace gcs::ui {

using namespace theme;

namespace {

constexpr double kWidth = 460.0;
constexpr double kHeader = 64.0;
constexpr double kActions = 60.0;  // SAVE · SAVE AS · NO PLAN
constexpr double kPad = 12.0;
constexpr double kGap = 8.0;
constexpr double kRow = 64.0;
constexpr double kRowButtons = 56.0;  // OPEN · RENAME · DELETE under the selected row

QFont bodyFont(double px, bool bold = false) {
    QFont f(QString::fromLatin1(kSansFamily));
    f.setPixelSize(static_cast<int>(std::round(px)));
    f.setBold(bold);
    return f;
}

QRectF logicalRect(const QWidget* w) {
    return {QPointF(0, 0), logicalSize(w->size())};
}

// a name inside a sentence: a right-to-left name stays a unit and doesn't reorder the line
QString isolated(const QString& name) {
    return QChar(0x2068) + name + QChar(0x2069);
}

QString lengthText(double m) {
    return m >= 1000.0 ? QStringLiteral("%1 km").arg(m / 1000.0, 0, 'f', 1)
                       : QStringLiteral("%1 m").arg(std::lround(m));
}

}  // namespace

PlanLibraryPanel::PlanLibraryPanel(QWidget* parent) : QWidget(parent) {
    hide();
}

void PlanLibraryPanel::setEntries(std::vector<PlanLibrary::Entry> entries, const EditorInfo& editor) {
    entries_ = std::move(entries);
    editor_ = editor;
    const bool still = std::any_of(entries_.begin(), entries_.end(),
                                   [this](const PlanLibrary::Entry& e) { return e.id == selected_; });
    if (!still) {
        selected_.clear();
    }
    if (isVisible()) {
        place(area_);
    }
    update();
}

void PlanLibraryPanel::ask(const QString& title, const QString& detail, const QString& yes_label,
                           const QColor& yes_color, std::function<void()> yes) {
    question_ = Question{title, detail, yes_label, yes_color, std::move(yes)};
    update();
}

double PlanLibraryPanel::rowHeight(int i) const {
    return kRow + (entries_[static_cast<size_t>(i)].id == selected_ ? kRowButtons : 0.0);
}

double PlanLibraryPanel::contentHeight() const {
    if (entries_.empty()) {
        return 76.0;
    }
    double h = 0.0;
    for (int i = 0; i < static_cast<int>(entries_.size()); ++i) {
        h += rowHeight(i) + kGap;
    }
    return h;
}

QRectF PlanLibraryPanel::listRect() const {
    const QRectF r = logicalRect(this).adjusted(1, 1, -1, -1);
    return {r.left() + kPad, r.top() + kHeader + kActions, r.width() - 2 * kPad,
            r.height() - kHeader - kActions - kPad};
}

void PlanLibraryPanel::place(const QRect& area) {
    area_ = area;
    const double k = uiScale();
    const double w = std::min(kWidth, area.width() / k - 16.0);
    const double want = kHeader + kActions + contentHeight() + kPad;
    const double h = std::min(std::max(want, 280.0), area.height() / k);
    setGeometry(static_cast<int>(area.right() - w * k), area.top(), static_cast<int>(w * k), static_cast<int>(h * k));
    clampScroll();
}

void PlanLibraryPanel::clampScroll() {
    const double view = listRect().height();
    scroll_ = std::clamp(scroll_, 0.0, std::max(0.0, contentHeight() - view));
}

void PlanLibraryPanel::mousePressEvent(QMouseEvent* e) {
    pressing_ = true;
    dragged_ = false;
    press_pos_ = toLogical(e->position());
    press_scroll_ = scroll_;
}

void PlanLibraryPanel::mouseMoveEvent(QMouseEvent* e) {
    if (!pressing_ || question_) {
        return;
    }
    const double dy = toLogical(e->position()).y() - press_pos_.y();
    if (!dragged_ && std::abs(dy) > 8) {
        dragged_ = true;
    }
    if (dragged_) {
        scroll_ = press_scroll_ - dy;
        clampScroll();
        update();
    }
}

void PlanLibraryPanel::mouseReleaseEvent(QMouseEvent* e) {
    const bool tap = pressing_ && !dragged_;
    pressing_ = false;
    if (tap) {
        tapAt(toLogical(e->position()));
    }
}

void PlanLibraryPanel::wheelEvent(QWheelEvent* e) {
    if (question_) {
        return;
    }
    scroll_ -= e->angleDelta().y() / 2.0;
    clampScroll();
    update();
}

void PlanLibraryPanel::tapAt(const QPointF& pos) {
    // hits_ is in paint order: the question (drawn last) wins, so search backwards
    for (auto it = hits_.rbegin(); it != hits_.rend(); ++it) {
        if (it->rect.contains(pos)) {
            if (it->enabled && it->fn) {
                auto fn = it->fn;  // may change hits_ / delete rows
                fn();
                update();
            }
            return;
        }
    }
    if (question_) {
        return;  // a tap beside the question's buttons does nothing
    }
}

void PlanLibraryPanel::paintEvent(QPaintEvent*) {
    hits_.clear();
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    p.scale(uiScale(), uiScale());
    const QRectF r = logicalRect(this).adjusted(1, 1, -1, -1);
    drawGlass(p, chamferPath(r, 14), kAmber, 0.6, alpha(kGlassDeep, 0.97));
    p.setPen(QPen(kAmber, 2));
    p.drawLine(QPointF(r.left() + 1, r.top() + 14), QPointF(r.left() + 14, r.top() + 1));

    // header: what is in the editor
    drawText(p, QRectF(r.left() + 18, r.top() + 10, r.width() - 36, 22), Qt::AlignLeft | Qt::AlignVCenter,
             QStringLiteral("PLANS"), titleFont(18), kText);
    QString sub;
    QColor sub_c = kMuted;
    if (editor_.empty && editor_.name.isEmpty()) {
        sub = QStringLiteral("Editing: no plan · lands at home");
    } else if (editor_.name.isEmpty()) {
        sub = QStringLiteral("Editing: a new plan, not saved");
        sub_c = kAmber;
    } else if (editor_.id.isEmpty()) {
        sub = QStringLiteral("Editing: %1 · not saved").arg(isolated(editor_.name));
        sub_c = kAmber;
    } else {
        sub = QStringLiteral("Editing: %1%2").arg(isolated(editor_.name), editor_.edited ? QStringLiteral(" · edited") : QString());
        sub_c = editor_.edited ? kAmber : kCyan;
    }
    drawText(p, QRectF(r.left() + 18, r.top() + 34, r.width() - 36, 20), Qt::AlignLeft | Qt::AlignVCenter,
             QFontMetricsF(bodyFont(13)).elidedText(sub, Qt::ElideRight, r.width() - 36), bodyFont(13), sub_c);

    auto button = [&](const QRectF& b, const QString& label, const QColor& c, bool enabled, std::function<void()> fn) {
        const QColor cc = enabled ? c : kMuted;
        drawGlass(p, chamferPath(b, 6), cc, enabled ? 0.7 : 0.3, alpha(cc, enabled ? 0.10 : 0.03));
        drawText(p, b, Qt::AlignCenter, label, labelFont(12), enabled ? cc : alpha(kMuted, 0.7));
        hits_.push_back({b, std::move(fn), enabled});
    };

    // actions for the editor
    {
        const double y = r.top() + kHeader;
        const double bw = (r.width() - 2 * kPad - 2 * kGap) / 3.0;
        const bool can_save = !editor_.empty && (editor_.id.isEmpty() || editor_.edited);
        button(QRectF(r.left() + kPad, y, bw, 48),
               editor_.id.isEmpty() || !editor_.edited ? QStringLiteral("SAVE") : QStringLiteral("SAVE CHANGES"),
               kGreen, can_save, [this] { emit saveRequested(); });
        button(QRectF(r.left() + kPad + bw + kGap, y, bw, 48), QStringLiteral("SAVE AS…"), kCyan, !editor_.empty,
               [this] { emit saveAsRequested(); });
        button(QRectF(r.left() + kPad + 2 * (bw + kGap), y, bw, 48), QStringLiteral("NO PLAN"), kAmber,
               !(editor_.empty && editor_.name.isEmpty()), [this] {
                   if (editor_.edited && !editor_.empty) {
                       ask(QStringLiteral("Clear the editor?"),
                           QStringLiteral("The changes to this plan are not saved."), QStringLiteral("CLEAR"), kAmber,
                           [this] { emit noPlanRequested(); });
                   } else {
                       emit noPlanRequested();
                   }
               });
    }

    // the saved plans
    const QRectF view = listRect();
    p.save();
    p.setClipRect(view);
    double y = view.top() - scroll_;
    const double w = view.width();
    if (entries_.empty()) {
        const QRectF card(view.left(), y, w, 66);
        drawGlass(p, chamferPath(card, 8), kMuted, 0.5, alpha(kMuted, 0.05));
        drawText(p, card.adjusted(16, 10, -10, -30), Qt::AlignLeft | Qt::AlignVCenter,
                 QStringLiteral("No saved plans yet"), bodyFont(15, true), kText);
        drawText(p, card.adjusted(16, 34, -10, -8), Qt::AlignLeft | Qt::AlignVCenter,
                 QStringLiteral("Build a route on the map, then SAVE AS."), bodyFont(12.5), kMuted);
    }
    for (int i = 0; i < static_cast<int>(entries_.size()); ++i) {
        const auto& e = entries_[static_cast<size_t>(i)];
        const double h = rowHeight(i);
        if (y > view.bottom()) {
            break;
        }
        if (y + h < view.top()) {
            y += h + kGap;
            continue;
        }
        const bool sel = e.id == selected_;
        const bool current = e.id == editor_.id;
        const QColor c = sel ? kCyan : (current ? kAmber : kMuted);
        const QRectF card(view.left(), y, w, h);
        drawGlass(p, chamferPath(card, 8), c, sel ? 0.8 : 0.45, alpha(c, sel ? 0.10 : 0.04));
        const QRectF head(card.left(), card.top(), w, kRow);
        const double tag_w = current ? 76.0 : 0.0;
        drawText(p, QRectF(head.left() + 14, head.top() + 8, w - 28 - tag_w, 24), Qt::AlignLeft | Qt::AlignVCenter,
                 QFontMetricsF(bodyFont(16, true)).elidedText(e.name, Qt::ElideRight, w - 28 - tag_w),
                 bodyFont(16, true), kText);
        QStringList facts;
        facts << (e.waypoints == 0   ? QStringLiteral("no route")
                  : e.waypoints == 1 ? QStringLiteral("1 WP · circle")
                                     : QStringLiteral("%1 WP · %2").arg(e.waypoints).arg(lengthText(e.route_m)));
        facts << (e.has_land ? QStringLiteral("L") : QStringLiteral("lands at home"));
        if (e.saved.isValid()) {
            facts << e.saved.toLocalTime().toString(QStringLiteral("d MMM yyyy HH:mm"));
        }
        drawText(p, QRectF(head.left() + 14, head.top() + 34, w - 28, 20), Qt::AlignLeft | Qt::AlignVCenter,
                 facts.join(QStringLiteral(" · ")), monoFont(11.5), kMuted);
        if (current) {
            const QRectF tag(head.right() - 14 - tag_w, head.top() + 10, tag_w, 20);
            drawGlass(p, chamferPath(tag, 4), kAmber, 0.6, alpha(kAmber, 0.12));
            drawText(p, tag, Qt::AlignCenter, QStringLiteral("EDITING"), labelFont(9), kAmber);
        }
        const QString id = e.id;
        const QString name = e.name;
        hits_.push_back({head.intersected(view), [this, id] {
                             selected_ = selected_ == id ? QString() : id;
                             place(area_);
                         }});
        if (sel) {
            const double by = card.top() + kRow;
            const double bw = (w - 2 * 10 - 2 * kGap) / 3.0;
            auto rowButton = [&](int k, const QString& label, const QColor& bc, std::function<void()> fn) {
                const QRectF b(card.left() + 10 + k * (bw + kGap), by, bw, 46);
                if (b.intersects(view)) {
                    button(b.intersected(view), label, bc, true, std::move(fn));
                }
            };
            rowButton(0, QStringLiteral("OPEN"), kGreen, [this, id, name] {
                if (editor_.edited && !editor_.empty && editor_.id != id) {
                    ask(QStringLiteral("Open “%1”?").arg(name),
                        QStringLiteral("The plan in the editor has changes that are not saved."),
                        QStringLiteral("OPEN"), kAmber, [this, id] { emit openRequested(id); });
                } else {
                    emit openRequested(id);
                }
            });
            rowButton(1, QStringLiteral("RENAME"), kCyan, [this, id] { emit renameRequested(id); });
            rowButton(2, QStringLiteral("DELETE"), kRed, [this, id, name] {
                ask(QStringLiteral("Delete “%1”?").arg(name),
                    QStringLiteral("The saved plan is removed from this computer. What is on the vehicle stays."),
                    QStringLiteral("DELETE"), kRed, [this, id] { emit deleteRequested(id); });
            });
        }
        y += h + kGap;
    }
    p.restore();

    // the open question covers the panel; it takes every tap
    if (question_) {
        hits_.push_back({r, nullptr, false});  // swallow taps around the card
        p.fillRect(r, alpha(kBg, 0.75));
        const double cw = r.width() - 2 * kPad;
        const double dh = QFontMetricsF(bodyFont(13))
                              .boundingRect(QRectF(0, 0, cw - 32, 1e6), Qt::TextWordWrap, question_->detail)
                              .height();
        const double ch = 16 + 26 + 8 + dh + 18 + 48 + 16;
        const QRectF card(r.left() + kPad, r.center().y() - ch / 2, cw, ch);
        drawGlass(p, chamferPath(card, 10), question_->yes_color, 0.8, alpha(kGlassDeep, 0.98));
        drawText(p, QRectF(card.left() + 16, card.top() + 16, cw - 32, 26), Qt::AlignLeft | Qt::AlignVCenter,
                 question_->title, bodyFont(17, true), kText);
        p.setFont(bodyFont(13));
        p.setPen(kMuted);
        p.drawText(QRectF(card.left() + 16, card.top() + 50, cw - 32, dh), Qt::TextWordWrap, question_->detail);
        const double bw = (cw - 32 - kGap) / 2.0;
        const double by = card.bottom() - 16 - 48;
        button(QRectF(card.left() + 16, by, bw, 48), question_->yes_label, question_->yes_color, true, [this] {
            auto yes = std::move(question_->yes);
            question_.reset();
            if (yes) {
                yes();
            }
        });
        button(QRectF(card.left() + 16 + bw + kGap, by, bw, 48), QStringLiteral("BACK"), kMuted, true,
               [this] { question_.reset(); });
    }
}

}  // namespace gcs::ui
