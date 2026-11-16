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
    undo_ = new QPushButton(QStringLiteral("UNDO"));
    clear_ = new QPushButton(QStringLiteral("CLEAR"));
    cancel_ = new QPushButton(QStringLiteral("CANCEL"));
    for (auto* b : {undo_, clear_, cancel_}) {
        b->setFocusPolicy(Qt::NoFocus);
        b->setCursor(Qt::PointingHandCursor);
    }
    cancel_->setObjectName(QStringLiteral("planCancel"));
    row->addWidget(title);
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

    auto hint = new QLabel(QStringLiteral("click: add  ·  drag: move  ·  right-click: delete  ·  wheel on point: altitude"));
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
            border: 1px solid rgba(63, 224, 255, 0.45); padding: 4px 12px;
            font: bold 11px "DejaVu Sans";
        }
        QPushButton:hover { background: rgba(63, 224, 255, 0.18); border-color: #3fe0ff; }
        QPushButton:pressed { background: rgba(0, 0, 0, 0.5); }
        QPushButton:disabled { color: #4a5a68; border-color: rgba(125, 147, 166, 0.25); }
        QPushButton#planCancel { border-color: rgba(255, 77, 94, 0.6); }
        QPushButton#planCancel:hover { background: rgba(255, 77, 94, 0.18); border-color: #ff4d5e; }
    )"));

    connect(undo_, &QPushButton::clicked, this, &PlanStrip::undoRequested);
    connect(clear_, &QPushButton::clicked, this, &PlanStrip::clearRequested);
    connect(cancel_, &QPushButton::clicked, this, &PlanStrip::cancelRequested);
    setInfo(0, 0, 0, -1, 0, false);
}

void PlanStrip::setInfo(int waypoints, double route_m, double eta_s, double approach_m, double min_approach_m,
                        bool can_undo) {
    info_->setText(QStringLiteral("%1 pts  ·  %2  ·  ETA %3")
                       .arg(waypoints)
                       .arg(formatDistance(route_m))
                       .arg(formatDuration(eta_s)));
    const bool warn = approach_m >= 0.0 && approach_m < min_approach_m;
    warn_->setVisible(warn);
    if (warn) {
        warn_->setText(QStringLiteral("⚠ final approach %1 m — ArduPlane needs ≥ ~%2 m to land")
                           .arg(approach_m, 0, 'f', 0)
                           .arg(min_approach_m, 0, 'f', 0));
    }
    info_->setMinimumWidth(info_->fontMetrics().horizontalAdvance(info_->text()) + 8);
    undo_->setEnabled(can_undo);
    clear_->setEnabled(waypoints > 0);
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
