#include "waypoint_panel.hpp"

#include "theme.hpp"

#include <QFontMetrics>
#include <QHBoxLayout>
#include <QLabel>
#include <QPainter>
#include <QPushButton>
#include <QVBoxLayout>

#include <algorithm>

namespace gcs::ui {

using namespace theme;

namespace {

constexpr int kTouch = 52;  // px; comfortably above the ~44-48 px touch-target minimum

QPushButton* makeButton(const QString& text, const QString& name = {}) {
    auto* b = new QPushButton(text);
    b->setFocusPolicy(Qt::NoFocus);
    b->setCursor(Qt::PointingHandCursor);
    // the stylesheet font is not applied yet, so size from it explicitly:
    // the panel re-lays out when rows hide, and must never squeeze a label
    QFont f(QStringLiteral("DejaVu Sans Mono"));
    f.setBold(true);
    f.setPixelSize(14);
    const int text_w = QFontMetrics(f).horizontalAdvance(text) + 28;
    b->setMinimumSize(std::max(kTouch, text_w), kTouch);
    if (!name.isEmpty()) {
        b->setObjectName(name);
    }
    return b;
}

}  // namespace

WaypointPanel::WaypointPanel(QWidget* parent) : QWidget(parent) {
    auto* outer = new QHBoxLayout(this);
    outer->setContentsMargins(18, 10, 12, 10);
    outer->setSpacing(10);

    auto* info = new QVBoxLayout;
    info->setSpacing(0);
    title_ = new QLabel;
    title_->setObjectName(QStringLiteral("wpTitle"));
    alt_ = new QLabel;
    alt_->setObjectName(QStringLiteral("wpAlt"));
    note_ = new QLabel(QStringLiteral("spiral down on the wing to this\napproach altitude, then land vertically"));
    note_->setObjectName(QStringLiteral("wpNote"));
    {
        // stylesheet fonts are applied late; reserve the width the note needs
        QFont nf(QStringLiteral("DejaVu Sans"));
        nf.setPixelSize(11);
        const QFontMetrics fm(nf);
        int w = 0;
        for (const auto& line : note_->text().split(QLatin1Char('\n'))) {
            w = std::max(w, fm.horizontalAdvance(line));
        }
        note_->setMinimumWidth(w + 6);
    }
    info->addWidget(title_);
    info->addWidget(alt_);
    info->addWidget(note_);
    outer->addLayout(info);

    steppers_ = new QWidget;
    auto* st = new QHBoxLayout(steppers_);
    st->setContentsMargins(0, 0, 0, 0);
    st->setSpacing(6);
    for (int d : {-10, -1, 1, 10}) {
        auto* b = makeButton(d > 0 ? QStringLiteral("+%1").arg(d) : QString::number(d));
        b->setAutoRepeat(true);  // press and hold keeps stepping
        b->setAutoRepeatDelay(400);
        b->setAutoRepeatInterval(90);
        connect(b, &QPushButton::clicked, this, [this, d] { emit altitudeDelta(static_cast<float>(d)); });
        st->addWidget(b);
    }
    outer->addWidget(steppers_);

    insert_ = makeButton(QStringLiteral("+ INSERT"));
    delete_ = makeButton(QStringLiteral("DELETE"), QStringLiteral("wpDelete"));
    auto* close = makeButton(QStringLiteral("✕"));
    outer->addWidget(insert_);
    outer->addWidget(delete_);
    outer->addWidget(close);

    setStyleSheet(QStringLiteral(R"(
        QLabel { background: transparent; color: #e6f1f8; }
        QLabel#wpTitle { color: #ffb547; font: bold 12px "DejaVu Sans"; }
        QLabel#wpAlt { font: bold 24px "DejaVu Sans Mono"; min-width: 110px; }
        QLabel#wpNote { color: #7d93a6; font: 11px "DejaVu Sans"; }
        QPushButton {
            background: rgba(63, 224, 255, 0.06); color: #e6f1f8;
            border: 1px solid rgba(63, 224, 255, 0.45); padding: 0 12px;
            font: bold 14px "DejaVu Sans Mono";
        }
        QPushButton:hover { background: rgba(63, 224, 255, 0.18); border-color: #3fe0ff; }
        QPushButton:pressed { background: rgba(63, 224, 255, 0.30); }
        QPushButton#wpDelete { border-color: rgba(255, 77, 94, 0.6); }
        QPushButton#wpDelete:hover { background: rgba(255, 77, 94, 0.18); border-color: #ff4d5e; }
        QPushButton#wpDelete:pressed { background: rgba(255, 77, 94, 0.30); }
    )"));

    connect(insert_, &QPushButton::clicked, this, &WaypointPanel::insertRequested);
    connect(delete_, &QPushButton::clicked, this, &WaypointPanel::deleteRequested);
    connect(close, &QPushButton::clicked, this, &WaypointPanel::closeRequested);
    // Apply the stylesheet fonts now: the panel is laid out while hidden, and
    // sizing it with the default fonts squeezes the 24 px readout and labels.
    ensurePolished();
    for (auto* w : findChildren<QWidget*>()) {
        w->ensurePolished();
    }
    showWaypoint(0, 80.0f);
}

void WaypointPanel::showWaypoint(int index, float alt_rel) {
    landing_ = false;
    title_->setText(QStringLiteral("WAYPOINT %1").arg(index + 1));
    title_->setStyleSheet(QStringLiteral("color: #ffb547;"));
    alt_->setText(QStringLiteral("%1 m").arg(alt_rel, 0, 'f', 0));
    alt_->show();
    note_->hide();
    steppers_->show();
    insert_->show();
    relayout();
}

void WaypointPanel::showLanding(float approach_alt) {
    landing_ = true;
    title_->setText(QStringLiteral("LANDING POINT"));
    title_->setStyleSheet(QStringLiteral("color: #5cff9d;"));
    alt_->setText(QStringLiteral("%1 m").arg(approach_alt, 0, 'f', 0));
    alt_->show();
    note_->show();
    steppers_->show();
    insert_->hide();
    relayout();
}

void WaypointPanel::relayout() {
    // children were just shown / hidden: make the layout see that before
    // sizing, or the panel keeps the previous mode's (smaller) size
    layout()->invalidate();
    layout()->activate();
    setMinimumSize(layout()->minimumSize());
    adjustSize();
    update();
}

void WaypointPanel::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    const QColor accent = landing_ ? kGreen : kAmber;
    const QRectF r = QRectF(rect()).adjusted(1, 1, -1, -1);
    drawGlass(p, chamferPath(r, 12), accent, 0.6, kGlassDeep);
    p.setPen(QPen(accent, 2));
    p.drawLine(QPointF(r.left() + 1, r.top() + 12), QPointF(r.left() + 12, r.top() + 1));
}

}  // namespace gcs::ui
