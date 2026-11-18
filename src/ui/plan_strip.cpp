#include "plan_strip.hpp"

#include "theme.hpp"

#include <QHBoxLayout>
#include <QLabel>
#include <QPainter>
#include <QPushButton>
#include <QVBoxLayout>

namespace gcs::ui {

using namespace theme;

namespace {

QString formatDistance(double m) {
    if (m >= 1000.0) {
        return QStringLiteral("%1 km").arg(m / 1000.0, 0, 'f', 2);
    }
    return QStringLiteral("%1 m").arg(m, 0, 'f', 0);
}

QString formatDuration(double s) {
    const int total = static_cast<int>(s + 0.5);
    return QStringLiteral("%1:%2").arg(total / 60).arg(total % 60, 2, 10, QLatin1Char('0'));
}

}  // namespace

PlanStrip::PlanStrip(QWidget* parent) : QWidget(parent) {
    setAttribute(Qt::WA_StyledBackground, false);
    auto* outer = new QVBoxLayout(this);
    outer->setContentsMargins(20, 8, 16, 8);
    outer->setSpacing(2);

    auto* row = new QHBoxLayout;
    row->setSpacing(8);
    auto* title = new QLabel(QStringLiteral("PLAN"));
    title->setObjectName(QStringLiteral("planTitle"));
    info_ = new QLabel;
    info_->setObjectName(QStringLiteral("planInfo"));
    wp_tool_ = new QPushButton(QStringLiteral("+ WAYPOINT"));
    land_tool_ = new QPushButton(QStringLiteral("+ LANDING"));
    undo_ = new QPushButton(QStringLiteral("UNDO"));
    clear_ = new QPushButton(QStringLiteral("CLEAR"));
    cancel_ = new QPushButton(QStringLiteral("CANCEL"));
    for (auto* b : {wp_tool_, land_tool_, undo_, clear_, cancel_}) {
        b->setFocusPolicy(Qt::NoFocus);
        b->setCursor(Qt::PointingHandCursor);
        b->setMinimumHeight(48);  // finger-sized
    }
    wp_tool_->setObjectName(QStringLiteral("planTool"));
    land_tool_->setObjectName(QStringLiteral("planTool"));
    wp_tool_->setCheckable(true);
    land_tool_->setCheckable(true);
    wp_tool_->setChecked(true);
    cancel_->setObjectName(QStringLiteral("planCancel"));
    row->addWidget(title);
    row->addWidget(wp_tool_);
    row->addWidget(land_tool_);
    row->addSpacing(8);
    row->addWidget(info_, 1);
    row->addSpacing(8);
    row->addWidget(undo_);
    row->addWidget(clear_);
    row->addWidget(cancel_);
    outer->addLayout(row);

    // how the route is flown: one exclusive, finger-sized choice
    auto* route_row = new QHBoxLayout;
    route_row->setSpacing(0);
    auto* route_title = new QLabel(QStringLiteral("ROUTE"));
    route_title->setObjectName(QStringLiteral("planTitle"));
    loop_ = new QPushButton(QStringLiteral("⟳ LOOP"));
    back_forth_ = new QPushButton(QStringLiteral("⇄ BACK && FORTH"));
    one_way_ = new QPushButton(QStringLiteral("→ ONE WAY, THEN HOLD"));
    route_row->addWidget(route_title);
    route_row->addSpacing(8);
    for (auto* b : {loop_, back_forth_, one_way_}) {
        b->setObjectName(QStringLiteral("planTool"));
        b->setCheckable(true);
        b->setFocusPolicy(Qt::NoFocus);
        b->setCursor(Qt::PointingHandCursor);
        b->setMinimumHeight(48);
        route_row->addWidget(b);
    }
    loop_->setChecked(true);
    route_row->addSpacing(12);
    warn_ = new QLabel;
    warn_->setObjectName(QStringLiteral("planWarn"));
    warn_->hide();
    route_row->addWidget(warn_, 1);
    outer->addSpacing(4);
    outer->addLayout(route_row);

    hint_ = new QLabel;
    hint_->setObjectName(QStringLiteral("planHint"));
    outer->addWidget(hint_);

    setStyleSheet(QStringLiteral(R"(
        QLabel { background: transparent; color: #e6f1f8; }
        QLabel#planTitle { color: #ffb547; font: bold 13px "DejaVu Sans"; padding-right: 6px; }
        QLabel#planInfo { font: 13px "DejaVu Sans Mono"; }
        QLabel#planWarn { color: #ffb547; font: 12px "DejaVu Sans Mono"; }
        QLabel#planHint { color: #7d93a6; font: 11px "DejaVu Sans"; }
        QPushButton {
            background: rgba(63, 224, 255, 0.06); color: #e6f1f8;
            border: 1px solid rgba(63, 224, 255, 0.45); padding: 4px 14px;
            font: bold 12px "DejaVu Sans";
        }
        QPushButton:hover { background: rgba(63, 224, 255, 0.18); border-color: #3fe0ff; }
        QPushButton:pressed { background: rgba(0, 0, 0, 0.5); }
        QPushButton:disabled { color: #4a5a68; border-color: rgba(125, 147, 166, 0.25); }
        QPushButton#planTool:checked { background: rgba(255, 181, 71, 0.25); border-color: #ffb547; color: #ffb547; }
        QPushButton#planCancel { border-color: rgba(255, 77, 94, 0.6); }
        QPushButton#planCancel:hover { background: rgba(255, 77, 94, 0.18); border-color: #ff4d5e; }
    )"));

    connect(wp_tool_, &QPushButton::clicked, this, [this] { emit landingToolToggled(false); });
    connect(land_tool_, &QPushButton::clicked, this, [this] { emit landingToolToggled(true); });
    connect(undo_, &QPushButton::clicked, this, &PlanStrip::undoRequested);
    connect(clear_, &QPushButton::clicked, this, &PlanStrip::clearRequested);
    connect(cancel_, &QPushButton::clicked, this, &PlanStrip::cancelRequested);
    connect(loop_, &QPushButton::clicked, this, [this] { emit patternChanged(Pattern::Loop); });
    connect(back_forth_, &QPushButton::clicked, this, [this] { emit patternChanged(Pattern::BackForth); });
    connect(one_way_, &QPushButton::clicked, this, [this] { emit patternChanged(Pattern::OneWay); });
    setInfo(0, 0, 0, false, false, Pattern::Loop);
}

void PlanStrip::setLandingTool(bool on) {
    wp_tool_->setChecked(!on);
    land_tool_->setChecked(on);
}

void PlanStrip::setInfo(int waypoints, double route_m, double route_s, bool has_land, bool can_undo,
                        Pattern pattern) {
    QString text = QStringLiteral("%1 pts").arg(waypoints);
    if (waypoints > 1) {
        const QString what = pattern == Pattern::OneWay      ? QStringLiteral("one way")
                             : pattern == Pattern::BackForth ? QStringLiteral("out & back")
                                                             : QStringLiteral("lap");
        text += QStringLiteral("  ·  %1 %2  ·  %3").arg(what, formatDistance(route_m), formatDuration(route_s));
    } else if (waypoints == 1) {
        text += QStringLiteral("  ·  circle");
    }
    text += has_land ? QStringLiteral("  ·  L set") : QStringLiteral("  ·  no L");
    info_->setText(text);
    warn_->setVisible(!has_land && waypoints > 0);
    warn_->setText(QStringLiteral("No landing point: it will land at home (Land and failsafes)"));
    info_->setMinimumWidth(info_->fontMetrics().horizontalAdvance(info_->text()) + 8);
    loop_->setChecked(pattern == Pattern::Loop);
    back_forth_->setChecked(pattern == Pattern::BackForth);
    one_way_->setChecked(pattern == Pattern::OneWay);
    const QString flown = pattern == Pattern::OneWay
                              ? QStringLiteral("flies the route once, then circles the last point until you Land")
                          : pattern == Pattern::BackForth
                              ? QStringLiteral("flies to the last point and back, over and over, until you Land")
                              : QStringLiteral("the route loops until you Land");
    hint_->setText(QStringLiteral("tap: add  ·  tap a point: edit altitude / delete  ·  drag: move  ·  pinch: zoom  ·  ") +
                   flown);
    undo_->setEnabled(can_undo);
    clear_->setEnabled(waypoints > 0 || has_land);
    adjustSize();
}

void PlanStrip::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    const QRectF r = QRectF(rect()).adjusted(1, 1, -1, -1);
    drawGlass(p, chamferPath(r, 12), kAmber, 0.55, kGlassDeep);
    p.setPen(QPen(kAmber, 2));
    p.drawLine(QPointF(r.left() + 1, r.top() + 12), QPointF(r.left() + 12, r.top() + 1));
}

}  // namespace gcs::ui
