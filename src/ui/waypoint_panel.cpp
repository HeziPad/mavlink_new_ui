#include "waypoint_panel.hpp"

#include "theme.hpp"
#include "units.hpp"

#include <QFontMetrics>
#include <QHBoxLayout>
#include <QLabel>
#include <QPainter>
#include <QEvent>
#include <QPushButton>
#include <QVBoxLayout>

#include <algorithm>
#include <functional>
#include <cmath>

namespace gcs::ui {

using namespace theme;

namespace {

constexpr int kTouch = 52;  // px; comfortably above the ~44-48 px touch-target minimum

// written at text size Small; applyScale() scales every px
const char* const kSheet = R"(
        QLabel { background: transparent; color: #e6f1f8; }
        QLabel#wpTitle { color: #ffb547; font: bold 12px "DejaVu Sans"; }
        QLabel#wpAlt { font: bold 24px "DejaVu Sans Mono"; min-width: 110px;
                       border-bottom: 1px dashed rgba(63, 224, 255, 0.6); }
        QLabel#wpNote { color: #7d93a6; font: 11px "DejaVu Sans"; }
        QLabel#wpHold { font: bold 16px "DejaVu Sans Mono"; }
        QLabel#wpHoldValue { font: bold 18px "DejaVu Sans Mono";
                             border-bottom: 1px dashed rgba(63, 224, 255, 0.6); }
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
    )";

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
    auto* rows = new QVBoxLayout(this);
    rows->setContentsMargins(18, 10, 12, 10);
    rows->setSpacing(8);
    auto* outer = new QHBoxLayout;
    outer->setSpacing(10);
    rows->addLayout(outer);

    auto* info = new QVBoxLayout;
    info->setSpacing(0);
    title_ = new QLabel;
    title_->setObjectName(QStringLiteral("wpTitle"));
    alt_ = new QLabel;
    alt_->setObjectName(QStringLiteral("wpAlt"));
    alt_->setCursor(Qt::PointingHandCursor);
    alt_->installEventFilter(this);  // tap: type the altitude
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
        auto* b = makeButton(QStringLiteral("+50"));  // widest label (feet) sizes the button
        b->setText(units::altStepLabel(d));
        b->setAutoRepeat(true);  // press and hold keeps stepping
        b->setAutoRepeatDelay(400);
        b->setAutoRepeatInterval(90);
        connect(b, &QPushButton::clicked, this,
                [this, d] { emit altitudeDelta(static_cast<float>(units::altStepMeters(d))); });
        st->addWidget(b);
        alt_steps_.push_back({b, d});
    }
    outer->addWidget(steppers_);

    auto* coord = makeButton(QStringLiteral("⌖ COORD"));
    connect(coord, &QPushButton::clicked, this, &WaypointPanel::coordRequested);
    outer->addWidget(coord);
    insert_ = makeButton(QStringLiteral("+ INSERT"));
    delete_ = makeButton(QStringLiteral("DELETE"), QStringLiteral("wpDelete"));
    auto* close = makeButton(QStringLiteral("✕"));
    outer->addWidget(insert_);
    outer->addWidget(delete_);
    outer->addWidget(close);

    // the hold: how long (always a time), the circle's
    // radius and its direction (like the Fly To panel)
    hold_row_ = new QWidget;
    auto* hr = new QHBoxLayout(hold_row_);
    hr->setContentsMargins(0, 0, 0, 0);
    hr->setSpacing(6);
    hold_title_ = new QLabel(QStringLiteral("HOLD\nCIRCLE"));
    hold_title_->setObjectName(QStringLiteral("wpTitle"));
    hr->addWidget(hold_title_);
    hold_time_label_ = new QLabel;  // tap: type it
    hold_time_label_->setObjectName(QStringLiteral("wpHoldValue"));
    hold_time_label_->setMinimumWidth(170);
    hold_time_label_->setCursor(Qt::PointingHandCursor);
    hold_time_label_->installEventFilter(this);
    hr->addWidget(hold_time_label_);
    auto steppers = [&](QWidget*& box, const std::initializer_list<int>& steps, const QString& unit,
                        std::function<void(int)> emitter) {
        box = new QWidget;
        auto* h = new QHBoxLayout(box);
        h->setContentsMargins(0, 0, 0, 0);
        h->setSpacing(6);
        for (int d : steps) {
            auto* b = makeButton(QStringLiteral("%1%2%3").arg(d > 0 ? QStringLiteral("+") : QString()).arg(d).arg(unit));
            b->setAutoRepeat(true);
            b->setAutoRepeatDelay(400);
            b->setAutoRepeatInterval(120);
            connect(b, &QPushButton::clicked, this, [emitter, d] { emitter(d); });
            h->addWidget(b);
        }
        h->addSpacing(10);
        hr->addWidget(box);
    };
    steppers(hold_time_, {-60, -10, 10, 60}, QStringLiteral("s"),
             [this](int d) { emit holdTimeDelta(static_cast<float>(d)); });
    hold_circle_ = new QWidget;
    auto* ch = new QHBoxLayout(hold_circle_);
    ch->setContentsMargins(0, 0, 0, 0);
    ch->setSpacing(6);
    hold_radius_ = new QLabel;
    hold_radius_->setObjectName(QStringLiteral("wpHold"));
    hold_radius_->setMinimumWidth(170);
    ch->addWidget(hold_radius_);
    for (int d : {-50, -10, 10, 50}) {
        auto* b = makeButton(QStringLiteral("%1%2").arg(d > 0 ? QStringLiteral("+") : QString()).arg(d));
        b->setAutoRepeat(true);
        b->setAutoRepeatDelay(400);
        b->setAutoRepeatInterval(90);
        connect(b, &QPushButton::clicked, this, [this, d] { emit holdRadiusDelta(static_cast<float>(d)); });
        ch->addWidget(b);
    }
    hold_dir_ = makeButton(QStringLiteral("⟲ COUNTER-CW"));
    connect(hold_dir_, &QPushButton::clicked, this, &WaypointPanel::holdDirectionToggled);
    ch->addWidget(hold_dir_);
    hr->addWidget(hold_circle_);
    hr->addStretch(1);
    rows->addWidget(hold_row_);
    hold_row_->hide();

    setStyleSheet(scaleCss(QString::fromUtf8(kSheet)));
    scaleWidgetTree(this);

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
    alt_->setText(units::altText(alt_rel));
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
    alt_->setText(units::altText(approach_alt));
    alt_->show();
    note_->show();
    steppers_->show();
    insert_->hide();
    relayout();
}

void WaypointPanel::setHold(const HoldView& v) {
    const bool show = v.mode != HoldMode::None;
    const bool timed = v.mode == HoldMode::Timed;
    if (show) {
        hold_title_->setText(v.mode == HoldMode::Final ? QStringLiteral("HOLD\nCIRCLE") : QStringLiteral("HOLD\nHERE"));
        auto clock = [](double sec) {
            const long t = std::lround(sec);
            return QStringLiteral("%1:%2").arg(t / 60).arg(t % 60, 2, 10, QLatin1Char('0'));
        };
        hold_time_label_->setText(v.hold_s <= 0.0f ? QStringLiteral("fly through") : clock(v.hold_s));
        const double r = v.radius > 0.0f ? v.radius : v.default_radius;
        hold_radius_->setText(QStringLiteral("Radius: %1 m").arg(std::lround(r)));
        hold_dir_->setText(v.ccw ? QStringLiteral("⟲ COUNTER-CW") : QStringLiteral("⟳ CLOCKWISE"));
    }
    const bool relay = hold_row_->isVisible() != show || hold_time_label_->isVisible() != timed ||
                       hold_time_->isVisible() != timed || !hold_circle_->isVisible();
    hold_mode_ = v.mode;
    hold_time_label_->setVisible(timed);
    hold_time_->setVisible(timed);
    hold_circle_->setVisible(true);
    hold_row_->setVisible(show);
    if (relay) {
        relayout();
    }
}

bool WaypointPanel::eventFilter(QObject* obj, QEvent* e) {
    if (obj == alt_ && e->type() == QEvent::MouseButtonRelease) {
        emit altitudeTapped();  // type it
        return true;
    }
    if (obj == hold_time_label_ && e->type() == QEvent::MouseButtonRelease) {
        emit holdValueTapped();  // type the time
        return true;
    }
    return QWidget::eventFilter(obj, e);
}

void WaypointPanel::applyScale() {
    setStyleSheet(scaleCss(QString::fromUtf8(kSheet)));
    scaleWidgetTree(this);
    ensurePolished();
    for (auto* w : findChildren<QWidget*>()) {
        w->ensurePolished();
    }
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

void WaypointPanel::unitsChanged() {
    for (const auto& [b, d] : alt_steps_) {
        b->setText(units::altStepLabel(d));
    }
}

}  // namespace gcs::ui
