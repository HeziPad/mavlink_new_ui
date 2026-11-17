#include "orbit_panel.hpp"

#include "theme.hpp"
#include "units.hpp"

#include <QFontMetrics>
#include <QHBoxLayout>
#include <QLabel>
#include <QLayout>
#include <QPainter>
#include <QPushButton>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>

namespace gcs::ui {

using namespace theme;

namespace {

constexpr int kTouch = 48;  // finger-sized (>= 44-48 px), compact enough to fit beside the PiP

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
                            bool altitude, void (OrbitPanel::*signal)(float), QLabel** note) {
    auto* w = new QWidget;
    auto* h = new QHBoxLayout(w);
    h->setContentsMargins(0, 0, 0, 0);
    h->setSpacing(5);
    auto* info = new QVBoxLayout;
    info->setSpacing(0);
    auto* label = new QLabel(name);
    label->setObjectName(QStringLiteral("orbName"));
    value = new QLabel;
    value->setObjectName(QStringLiteral("orbValue"));
    info->addWidget(label);
    info->addWidget(value);
    if (note) {
        *note = new QLabel;
        (*note)->setObjectName(QStringLiteral("orbNote"));
        info->addWidget(*note);
    }
    h->addLayout(info);
    for (int d : steps) {
        auto* b = makeButton(QStringLiteral("+50"));  // widest label (m or ft) sizes the button
        b->setText(d > 0 ? QStringLiteral("+%1").arg(d) : QString::number(d));
        b->setObjectName(name.toLower() + (d > 0 ? QStringLiteral("+%1").arg(d) : QString::number(d)));  // e.g. "alt+10"
        b->setAutoRepeat(true);
        b->setAutoRepeatDelay(400);
        b->setAutoRepeatInterval(110);
        const size_t idx = steps_.size();
        steps_.push_back({b, static_cast<double>(d), altitude});
        // Auto-repeat emits released/clicked/pressed on every repeat while the
        // button stays down; the real release comes with isDown() == false and
        // is followed by one more clicked(), which must not add a step.
        connect(b, &QPushButton::pressed, this, [this] {
            if (!holding_) {
                holding_ = true;
                skip_click_ = false;
                press_t_.restart();
            }
        });
        connect(b, &QPushButton::released, this, [this, b] {
            if (!b->isDown()) {
                skip_click_ = press_t_.isValid() && press_t_.elapsed() > 350;  // it repeated
                holding_ = false;
                press_t_.invalidate();
                shown_mult_ = 1;
                title_->setText(title_text_);
            }
        });
        connect(b, &QPushButton::clicked, this, [this, idx, signal] {
            if (skip_click_) {
                skip_click_ = false;
                return;
            }
            const Step& st = steps_[idx];
            const int m = multiplier();
            if (m != shown_mult_) {
                shown_mult_ = m;
                title_->setText(m > 1 ? QStringLiteral("%1\n×%2").arg(title_text_).arg(m) : title_text_);
            }
            emit(this->*signal)(static_cast<float>(stepFor(st) * m));
        });
        h->addWidget(b);
    }
    return w;
}

double OrbitPanel::stepFor(const Step& st) const {
    return st.altitude ? units::altStepMeters(st.meters) : st.meters;
}

int OrbitPanel::multiplier() const {
    if (!press_t_.isValid()) {
        return 1;
    }
    const qint64 ms = press_t_.elapsed();
    return ms >= 2500 ? 10 : (ms >= 1000 ? 5 : 1);
}

void OrbitPanel::unitsChanged() {
    for (const Step& st : steps_) {
        if (st.altitude) {
            st.button->setText(units::altStepLabel(st.meters));
        }
    }
}

OrbitPanel::OrbitPanel(QWidget* parent) : QWidget(parent) {
    auto* outer = new QHBoxLayout(this);
    outer->setContentsMargins(14, 10, 10, 10);
    outer->setSpacing(10);

    title_ = new QLabel;
    title_->setObjectName(QStringLiteral("orbTitle"));
    outer->addWidget(title_);
    // the current altitude sits under the target, like "flown" under the radius
    outer->addWidget(column(QStringLiteral("ALT"), alt_, {-10, -1, 1, 10}, true, &OrbitPanel::altitudeDelta,
                            &alt_now_));
    outer->addWidget(column(QStringLiteral("RADIUS"), radius_, {-50, -10, 10, 50}, false, &OrbitPanel::radiusDelta,
                             &radius_flown_));
    dir_ = makeButton(QStringLiteral("↻ CW"));
    dir_->setMinimumWidth(76);
    connect(dir_, &QPushButton::clicked, this, &OrbitPanel::directionToggled);
    outer->addWidget(dir_);

    setStyleSheet(QStringLiteral(R"(
        QLabel { background: transparent; color: #e6f1f8; }
        QLabel#orbTitle { color: #3fe0ff; font: bold 12px "DejaVu Sans"; }
        QLabel#orbName { color: #7d93a6; font: bold 11px "DejaVu Sans"; }
        QLabel#orbValue { font: bold 20px "DejaVu Sans Mono"; min-width: 96px; }
        QLabel#orbNow { color: #7d93a6; font: 11px "DejaVu Sans Mono"; }
        QLabel#orbNote { color: #7d93a6; font: 10px "DejaVu Sans Mono"; }
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

void OrbitPanel::setValues(const QString& title, float alt, float radius, bool ccw, double current_alt, double flown,
                           bool trimmed) {
    if (flown > 0.0) {
        radius_flown_->setText(QStringLiteral("%1 flown %2").arg(trimmed ? QStringLiteral("✓") : QStringLiteral("≈"))
                                   .arg(static_cast<int>(std::lround(flown))));
    } else {
        radius_flown_->setText(QStringLiteral("measuring…"));
    }
    const int style = flown <= 0.0 ? 0 : (trimmed ? 2 : 1);
    if (style != flown_style_) {
        flown_style_ = style;
        radius_flown_->setStyleSheet(style == 2   ? QStringLiteral("color: #5cff9d;")
                                     : style == 1 ? QStringLiteral("color: #ffb547;")
                                                  : QString());
    }
    title_text_ = title;
    if (shown_mult_ <= 1) {
        title_->setText(title);
    }
    alt_->setText(units::altText(alt));
    radius_->setText(QStringLiteral("%1 m").arg(radius, 0, 'f', 0));
    alt_now_->setText(current_alt >= 0.0 ? QStringLiteral("now %1").arg(units::altText(current_alt)) : QString());
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
