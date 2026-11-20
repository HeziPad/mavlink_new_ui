#include "speed_panel.hpp"

#include "theme.hpp"
#include "units.hpp"

#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPainter>
#include <QPushButton>
#include <QVBoxLayout>

namespace gcs::ui {

using namespace theme;

namespace {
// written at text size Small; applyScale() scales every px
const char* const kSheet = R"(
        QLabel { background: transparent; color: #e6f1f8; }
        QLabel#spTitle { color: #3fe0ff; font: bold 14px "DejaVu Sans"; letter-spacing: 2px; }
        QLabel#spName { color: #7d93a6; font: bold 11px "DejaVu Sans"; }
        QLabel#spValue { font: bold 22px "DejaVu Sans Mono"; }
        QLabel#spParam { color: #5d7386; font: 10px "DejaVu Sans Mono"; }
        QLabel#spExplain { color: #9fb4c6; font: 12px "DejaVu Sans"; }
        QLabel#spActive { color: #3fe0ff; font: bold 12px "DejaVu Sans Mono"; }
        QPushButton {
            background: rgba(63, 224, 255, 0.06); color: #e6f1f8;
            border: 1px solid rgba(63, 224, 255, 0.45); padding: 0 16px;
            font: bold 14px "DejaVu Sans";
        }
        QPushButton:checked { background: rgba(63, 224, 255, 0.22); border-color: #3fe0ff; color: #3fe0ff; }
    )";
}  // namespace

SpeedPanel::SpeedPanel(QWidget* parent) : QWidget(parent) {
    auto* v = new QVBoxLayout(this);
    v->setContentsMargins(20, 14, 16, 14);
    v->setSpacing(8);

    auto* head = new QHBoxLayout;
    auto* title = new QLabel(QStringLiteral("AIRSPEED"));
    title->setObjectName(QStringLiteral("spTitle"));
    // no close button: tap the AS · GS chip again, or anywhere outside
    head->addWidget(title);
    head->addStretch(1);
    v->addLayout(head);

    auto* grid = new QGridLayout;
    grid->setHorizontalSpacing(18);
    grid->setVerticalSpacing(2);
    auto cell = [&](int col, const QString& name, const QString& param, QLabel*& value) {
        auto* n = new QLabel(name);
        n->setObjectName(QStringLiteral("spName"));
        value = new QLabel(QStringLiteral("—"));
        value->setObjectName(QStringLiteral("spValue"));
        auto* p = new QLabel(param);
        p->setObjectName(QStringLiteral("spParam"));
        grid->addWidget(n, 0, col);
        grid->addWidget(value, 1, col);
        grid->addWidget(p, 2, col);
    };
    cell(0, QStringLiteral("LOITER"), QStringLiteral("AIRSPEED_MIN"), loiter_);
    cell(1, QStringLiteral("CRUISE"), QStringLiteral("AIRSPEED_CRUISE"), cruise_);
    cell(2, QStringLiteral("MAX"), QStringLiteral("AIRSPEED_MAX"), max_);
    v->addLayout(grid);

    auto* note = new QLabel(QStringLiteral("The aircraft's own parameters (change them in Mission Planner / QGC)."));
    note->setObjectName(QStringLiteral("spParam"));
    v->addWidget(note);

    auto* mode_title = new QLabel(QStringLiteral("SPEED MODE"));
    mode_title->setObjectName(QStringLiteral("spName"));
    v->addWidget(mode_title);
    auto* row = new QHBoxLayout;
    row->setSpacing(0);
    const char* names[] = {"ECO", "NORMAL", "FAST"};
    for (int i = 0; i < 3; ++i) {
        modes_[i] = new QPushButton(QString::fromLatin1(names[i]));
        modes_[i]->setCheckable(true);
        modes_[i]->setMinimumHeight(50);
        modes_[i]->setFocusPolicy(Qt::NoFocus);
        connect(modes_[i], &QPushButton::clicked, this, [this, i] { emit modeChanged(i); });
        row->addWidget(modes_[i]);
    }
    v->addLayout(row);
    explain_ = new QLabel;
    explain_->setObjectName(QStringLiteral("spExplain"));
    explain_->setWordWrap(true);
    explain_->setMinimumWidth(420);
    v->addWidget(explain_);
    descent_ = new QLabel;
    descent_->setObjectName(QStringLiteral("spExplain"));
    descent_->setWordWrap(true);
    v->addWidget(descent_);
    active_ = new QLabel;
    active_->setObjectName(QStringLiteral("spActive"));
    v->addWidget(active_);

    applyScale();
    hide();
}

void SpeedPanel::setValues(double loiter, double cruise, double max, int mode, const QString& active,
                           double loiter_flown, double q_assist) {
    auto txt = [](double v) { return v > 0 ? units::speedText(v) : QStringLiteral("—"); };
    // LOITER is AIRSPEED_MIN, raised when Q_ASSIST would spin the lift rotors there
    loiter_->setText(loiter > 0 && loiter_flown > loiter + 0.05
                         ? QStringLiteral("%1 → %2").arg(units::speedText(loiter, 0, false), txt(loiter_flown))
                         : txt(loiter));
    loiter_->setToolTip(q_assist > 0 ? QStringLiteral("kept %1 above Q_ASSIST_SPEED (%2) so the lift rotors stay off")
                                           .arg(units::speedText(2.0), units::speedText(q_assist))
                                     : QString());
    cruise_->setText(txt(cruise));
    max_->setText(txt(max));
    for (int i = 0; i < 3; ++i) {
        modes_[i]->setChecked(i == mode);
    }
    static const char* explain[] = {
        "Saves battery. Flies gently and makes the most of the wind, for the longest flight time and range.",
        "Balanced. Flies at the aircraft's usual cruising speed.",
        "Gets there sooner. Flies at full speed between points; the battery runs down faster."};
    explain_->setText(QString::fromLatin1(explain[std::clamp(mode, 0, 2)]));
    active_->setText(active.isEmpty() ? QStringLiteral("now: autopilot default")
                                      : QStringLiteral("now: %1").arg(active));
    adjustSize();
}

void SpeedPanel::setDescent(double airspeed, bool learned) {
    descent_->setText(airspeed > 0 && learned
                          ? QStringLiteral("Descent speed learned from your flights: %1").arg(units::speedText(airspeed))
                          : QString());
    descent_->setVisible(!descent_->text().isEmpty());
    adjustSize();
}

void SpeedPanel::applyScale() {
    setStyleSheet(scaleCss(QString::fromUtf8(kSheet)));
    scaleWidgetTree(this);
    adjustSize();
}

void SpeedPanel::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    const QRectF r = QRectF(rect()).adjusted(1, 1, -1, -1);
    drawGlass(p, chamferPath(r, 12), kCyan, 0.6, kGlassDeep);
    p.setPen(QPen(kCyan, 2));
    p.drawLine(QPointF(r.left() + 1, r.top() + 12), QPointF(r.left() + 12, r.top() + 1));
}

}  // namespace gcs::ui
