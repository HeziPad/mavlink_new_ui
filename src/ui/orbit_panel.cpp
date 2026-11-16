#include "orbit_panel.hpp"

#include "theme.hpp"

#include <QFontMetrics>
#include <QHBoxLayout>
#include <QLabel>
#include <QLayout>
#include <QPainter>
#include <QPushButton>
#include <QVBoxLayout>

#include <algorithm>

namespace gcs::ui {

using namespace theme;

namespace {

constexpr int kTouch = 52;

QPushButton* makeButton(const QString& text) {
    auto* b = new QPushButton(text);
    b->setFocusPolicy(Qt::NoFocus);
    b->setCursor(Qt::PointingHandCursor);
    // size from the stylesheet font explicitly (it is applied late)
    QFont f(QStringLiteral("DejaVu Sans Mono"));
    f.setBold(true);
    f.setPixelSize(14);
    b->setMinimumSize(std::max(kTouch, QFontMetrics(f).horizontalAdvance(text) + 24), kTouch);
    return b;
}

}  // namespace

QWidget* OrbitPanel::column(const QString& name, QLabel*& value, const std::initializer_list<int>& steps,
                            void (OrbitPanel::*signal)(float)) {
    auto* w = new QWidget;
    auto* h = new QHBoxLayout(w);
    h->setContentsMargins(0, 0, 0, 0);
    h->setSpacing(6);
    auto* info = new QVBoxLayout;
    info->setSpacing(0);
    auto* label = new QLabel(name);
    label->setObjectName(QStringLiteral("orbName"));
    value = new QLabel;
    value->setObjectName(QStringLiteral("orbValue"));
    info->addWidget(label);
    info->addWidget(value);
    h->addLayout(info);
    for (int d : steps) {
        auto* b = makeButton(d > 0 ? QStringLiteral("+%1").arg(d) : QString::number(d));
        b->setObjectName(name.toLower() + (d > 0 ? QStringLiteral("+%1").arg(d) : QString::number(d)));  // e.g. "alt+10"
        b->setAutoRepeat(true);
        b->setAutoRepeatDelay(400);
        b->setAutoRepeatInterval(110);
        connect(b, &QPushButton::clicked, this, [this, d, signal] { emit(this->*signal)(static_cast<float>(d)); });
        h->addWidget(b);
    }
    return w;
}

OrbitPanel::OrbitPanel(QWidget* parent) : QWidget(parent) {
    auto* outer = new QHBoxLayout(this);
    outer->setContentsMargins(18, 10, 12, 10);
    outer->setSpacing(14);

    title_ = new QLabel;
    title_->setObjectName(QStringLiteral("orbTitle"));
    outer->addWidget(title_);
    outer->addWidget(column(QStringLiteral("ALT"), alt_, {-10, -1, 1, 10}, &OrbitPanel::altitudeDelta));
    alt_now_ = new QLabel;
    alt_now_->setObjectName(QStringLiteral("orbNow"));
    outer->addWidget(alt_now_);
    outer->addWidget(column(QStringLiteral("RADIUS"), radius_, {-50, -10, 10, 50}, &OrbitPanel::radiusDelta));
    dir_ = makeButton(QStringLiteral("↻ CW"));
    dir_->setMinimumWidth(84);
    connect(dir_, &QPushButton::clicked, this, &OrbitPanel::directionToggled);
    outer->addWidget(dir_);

    setStyleSheet(QStringLiteral(R"(
        QLabel { background: transparent; color: #e6f1f8; }
        QLabel#orbTitle { color: #3fe0ff; font: bold 12px "DejaVu Sans"; }
        QLabel#orbName { color: #7d93a6; font: bold 11px "DejaVu Sans"; }
        QLabel#orbValue { font: bold 20px "DejaVu Sans Mono"; min-width: 76px; }
        QLabel#orbNow { color: #7d93a6; font: 11px "DejaVu Sans Mono"; }
        QPushButton {
            background: rgba(63, 224, 255, 0.06); color: #e6f1f8;
            border: 1px solid rgba(63, 224, 255, 0.45); padding: 0 10px;
            font: bold 14px "DejaVu Sans Mono";
        }
        QPushButton:hover { background: rgba(63, 224, 255, 0.18); border-color: #3fe0ff; }
        QPushButton:pressed { background: rgba(63, 224, 255, 0.30); }
    )"));
    ensurePolished();
    for (auto* w : findChildren<QWidget*>()) {
        w->ensurePolished();
    }
    setValues(QStringLiteral("HOLD"), 80.0f, 80.0f, false, -1.0);
}

void OrbitPanel::setValues(const QString& title, float alt, float radius, bool ccw, double current_alt) {
    title_->setText(title);
    alt_->setText(QStringLiteral("%1 m").arg(alt, 0, 'f', 0));
    radius_->setText(QStringLiteral("%1 m").arg(radius, 0, 'f', 0));
    alt_now_->setText(current_alt >= 0.0 ? QStringLiteral("now\n%1 m").arg(current_alt, 0, 'f', 0) : QString());
    dir_->setText(ccw ? QStringLiteral("↺ CCW") : QStringLiteral("↻ CW"));
    layout()->invalidate();
    layout()->activate();
    setMinimumSize(layout()->minimumSize());
    adjustSize();
    update();
}

void OrbitPanel::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    const QRectF r = QRectF(rect()).adjusted(1, 1, -1, -1);
    drawGlass(p, chamferPath(r, 12), kCyan, 0.6, kGlassDeep);
    p.setPen(QPen(kCyan, 2));
    p.drawLine(QPointF(r.left() + 1, r.top() + 12), QPointF(r.left() + 12, r.top() + 1));
}

}  // namespace gcs::ui
