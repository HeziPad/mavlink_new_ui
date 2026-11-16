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

    warn_ = new QLabel;
    warn_->setObjectName(QStringLiteral("planWarn"));
    warn_->hide();
    outer->addWidget(warn_);

    auto hint = new QLabel(QStringLiteral("tap: add  ·  tap a point: edit altitude / delete  ·  drag: move  ·  pinch: zoom  ·  the route loops until you Land"));
    hint->setObjectName(QStringLiteral("planHint"));
    outer->addWidget(hint);

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
    setInfo(0, 0, 0, false, false);
}

void PlanStrip::setLandingTool(bool on) {
    wp_tool_->setChecked(!on);
    land_tool_->setChecked(on);
}

void PlanStrip::setInfo(int waypoints, double lap_m, double lap_s, bool has_land, bool can_undo) {
    QString text = QStringLiteral("%1 pts").arg(waypoints);
    if (waypoints > 1) {
        text += QStringLiteral("  ·  lap %1  ·  %2").arg(formatDistance(lap_m), formatDuration(lap_s));
    } else if (waypoints == 1) {
        text += QStringLiteral("  ·  circle");
    }
    text += has_land ? QStringLiteral("  ·  L set") : QStringLiteral("  ·  no L");
    info_->setText(text);
    warn_->setVisible(!has_land && waypoints > 0);
    warn_->setText(QStringLiteral("No landing point: the Land button stays locked; QRTL still returns home"));
    info_->setMinimumWidth(info_->fontMetrics().horizontalAdvance(info_->text()) + 8);
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
