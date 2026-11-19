#include "entry_pad.hpp"

#include "core/geo.hpp"
#include "theme.hpp"
#include "units.hpp"

#include <QApplication>
#include <QEvent>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QLabel>
#include <QLineEdit>
#include <QPainter>
#include <QProxyStyle>
#include <QPushButton>
#include <QRegularExpressionValidator>
#include <QStyle>
#include <QTimer>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>

namespace gcs::ui {

using namespace theme;

namespace {

constexpr int kKey = 58;  // px: comfortably finger-sized

// written at text size Small; applyScale() scales every px
const char* const kSheet = R"(
        QLabel { background: transparent; color: #e6f1f8; }
        QLabel#padTitle { color: #3fe0ff; font: bold 16px "DejaVu Sans"; letter-spacing: 2px; }
        QLabel#padRowName { color: #7d93a6; font: bold 13px "DejaVu Sans"; min-width: 84px; }
        QLabel#padUnit { color: #9fb4c6; font: bold 24px "DejaVu Sans Mono"; }
        QLabel#padPreview { font: 13px "DejaVu Sans Mono"; color: #9fb4c6; }
        QLineEdit {
            font: bold 28px "DejaVu Sans Mono"; color: #e6f1f8;
            background: rgba(0, 0, 0, 0.35); border: 1px solid rgba(63, 224, 255, 0.35);
            padding: 4px 8px; selection-background-color: rgba(63, 224, 255, 0.45);
        }
        QLineEdit[active="true"] { border: 2px solid #3fe0ff; background: rgba(63, 224, 255, 0.10); }
        QPushButton {
            background: rgba(63, 224, 255, 0.06); color: #e6f1f8;
            border: 1px solid rgba(63, 224, 255, 0.45); padding: 0 10px;
            font: bold 17px "DejaVu Sans Mono";
        }
        QPushButton:hover { background: rgba(63, 224, 255, 0.18); border-color: #3fe0ff; }
        QPushButton:pressed { background: rgba(63, 224, 255, 0.30); }
        QPushButton:disabled { color: #4a5a68; border-color: rgba(125, 147, 166, 0.25); }
        QPushButton#padSel { font: bold 13px "DejaVu Sans"; }
        QPushButton#padSel:checked { background: rgba(63, 224, 255, 0.22); border-color: #3fe0ff; color: #3fe0ff; }
        QPushButton#padOk { border-color: rgba(92, 255, 157, 0.7); color: #5cff9d; }
        QPushButton#padMin { border-color: rgba(255, 181, 71, 0.7); color: #ffb547; font: bold 14px "DejaVu Sans"; }
        QPushButton#padCancel { border-color: rgba(255, 77, 94, 0.6); }
    )";

QPushButton* key(const QString& text, const QString& name = {}) {
    auto* b = new QPushButton(text);
    b->setFocusPolicy(Qt::NoFocus);  // the box being typed into keeps the focus (and its cursor)
    b->setCursor(Qt::PointingHandCursor);
    b->setMinimumSize(kKey, kKey);
    if (!name.isEmpty()) {
        b->setObjectName(name);
    }
    return b;
}

const char* kRefNames[] = {"ASL", "AGL", "AFL"};

// a thick text cursor, so where a digit goes is obvious at a glance
class ThickCursorStyle : public QProxyStyle {
public:
    int pixelMetric(PixelMetric m, const QStyleOption* o = nullptr, const QWidget* w = nullptr) const override {
        return m == PM_TextCursorWidth ? std::max(3, sci(3)) : QProxyStyle::pixelMetric(m, o, w);
    }
};

QStyle* thickCursor() {
    static ThickCursorStyle* s = [] {
        auto* st = new ThickCursorStyle;
        st->setParent(qApp);
        return st;
    }();
    return s;
}

QString clock(double sec) {
    const long t = std::lround(sec);
    return QStringLiteral("%1:%2").arg(t / 60).arg(t % 60, 2, 10, QLatin1Char('0'));
}

}  // namespace

EntryPad::EntryPad(QWidget* parent) : QWidget(parent) {
    hide();
    setFocusPolicy(Qt::StrongFocus);
    build();
    poll_ = new QTimer(this);
    poll_->setInterval(300);
    connect(poll_, &QTimer::timeout, this, [this] {
        if (kind_ == Kind::Altitude) {
            refresh();  // a terrain tile may have arrived
        }
    });
}

void EntryPad::build() {
    card_ = new QWidget(this);
    auto* v = new QVBoxLayout(card_);
    v->setContentsMargins(26, 18, 26, 20);
    v->setSpacing(10);

    title_ = new QLabel;
    title_->setObjectName(QStringLiteral("padTitle"));
    v->addWidget(title_);

    // altitude reference (a reference, not a unit: kept)
    ref_row_ = new QWidget;
    {
        auto* h = new QHBoxLayout(ref_row_);
        h->setContentsMargins(0, 0, 0, 0);
        h->setSpacing(6);
        const QStringList names = {QStringLiteral("ASL · sea level"), QStringLiteral("AGL · ground here"),
                                   QStringLiteral("AFL · ground at L")};
        for (int i = 0; i < names.size(); ++i) {
            auto* b = key(names[i], QStringLiteral("padSel"));
            b->setCheckable(true);
            b->setMinimumHeight(48);
            connect(b, &QPushButton::clicked, this, [this, i] {
                // keep the same height, shown in the new reference
                const auto g = grounds();
                bool ok = false;
                const double shown = boxValue(0, &ok);
                const auto rel = ok ? altitude::toRelHome(units::altFromDisplay(shown), static_cast<altitude::Ref>(ref_), g)
                                    : std::nullopt;
                ref_ = i;
                if (rel) {
                    if (const auto nv = altitude::fromRelHome(*rel, static_cast<altitude::Ref>(ref_), g)) {
                        setBoxText(0, QString::number(std::lround(units::altToDisplay(*nv))));
                    }
                }
                emit refChanged(ref_);
                refresh();
            });
            h->addWidget(b);
            ref_buttons_.push_back(b);
        }
    }
    v->addWidget(ref_row_);

    presets_ = new QWidget;
    new QHBoxLayout(presets_);
    presets_->layout()->setContentsMargins(0, 0, 0, 0);
    presets_->layout()->setSpacing(6);
    v->addWidget(presets_);

    fields_ = new QWidget;
    auto* fl = new QVBoxLayout(fields_);
    fl->setContentsMargins(0, 4, 0, 4);
    fl->setSpacing(8);
    v->addWidget(fields_);

    preview_ = new QLabel;
    preview_->setObjectName(QStringLiteral("padPreview"));
    preview_->setWordWrap(true);
    preview_->setMinimumHeight(54);
    v->addWidget(preview_);

    // keys: digits, and moving the cursor
    auto* keys = new QHBoxLayout;
    keys->setSpacing(10);
    auto* digits = new QGridLayout;
    digits->setSpacing(6);
    const char* layout[4][3] = {{"7", "8", "9"}, {"4", "5", "6"}, {"1", "2", "3"}, {".", "0", "⌫"}};
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 3; ++c) {
            const QString t = QString::fromUtf8(layout[r][c]);
            auto* b = key(t);
            if (t == QStringLiteral("⌫")) {
                b->setAutoRepeat(true);
                connect(b, &QPushButton::clicked, this, [this] {
                    // delete before the cursor; at the start of a box, step back into the previous one
                    const int i = activeIndex();
                    if (i < 0) {
                        return;
                    }
                    if (active_->cursorPosition() == 0 && !active_->hasSelectedText() && i > 0) {
                        focusBox(i - 1);
                        active_->end(false);
                    }
                    active_->backspace();
                    refresh();
                });
            } else {
                if (t == QStringLiteral(".")) {
                    dot_key_ = b;
                }
                connect(b, &QPushButton::clicked, this, [this, t] { typeText(t); });
            }
            digits->addWidget(b, r, c);
        }
    }
    keys->addLayout(digits);

    auto* nav = new QGridLayout;
    nav->setSpacing(6);
    auto* left = key(QStringLiteral("◀"));
    auto* right = key(QStringLiteral("▶"));
    auto* next = key(QStringLiteral("NEXT ▸"));
    left->setAutoRepeat(true);
    right->setAutoRepeat(true);
    connect(left, &QPushButton::clicked, this, [this] {
        const int i = activeIndex();
        if (i < 0) {
            return;
        }
        if (active_->cursorPosition() == 0 && i > 0) {
            focusBox(i - 1);
            active_->end(false);
        } else {
            active_->cursorBackward(false);
        }
    });
    connect(right, &QPushButton::clicked, this, [this] {
        const int i = activeIndex();
        if (i < 0) {
            return;
        }
        if (active_->cursorPosition() == active_->text().size() && i + 1 < static_cast<int>(boxes_.size())) {
            focusBox(i + 1);
            active_->home(false);
        } else {
            active_->cursorForward(false);
        }
    });
    connect(next, &QPushButton::clicked, this, [this] {
        const int i = activeIndex();
        focusBox(boxes_.empty() ? 0 : (i + 1) % static_cast<int>(boxes_.size()));
    });
    nav->addWidget(left, 0, 0);
    nav->addWidget(right, 0, 1);
    nav->addWidget(next, 1, 0, 1, 2);
    nav->setRowStretch(2, 1);
    keys->addLayout(nav);

    nudge_keys_ = new QWidget;
    auto* ng = new QVBoxLayout(nudge_keys_);
    ng->setContentsMargins(0, 0, 0, 0);
    ng->setSpacing(6);
    for (int d : {10, -10}) {
        auto* b = key(d > 0 ? QStringLiteral("+10") : QStringLiteral("−10"));
        b->setAutoRepeat(true);
        connect(b, &QPushButton::clicked, this, [this, d] {
            setBoxText(0, QString::number(std::max(0L, std::lround(boxValue(0) + d))));
            refresh();
        });
        ng->addWidget(b);
    }
    ng->addStretch(1);
    keys->addWidget(nudge_keys_);
    keys->addStretch(1);
    v->addLayout(keys);

    auto* actions = new QHBoxLayout;
    actions->setSpacing(10);
    auto* cancel = key(QStringLiteral("CANCEL"), QStringLiteral("padCancel"));
    cancel->setMinimumWidth(140);
    use_min_ = key(QString(), QStringLiteral("padMin"));
    use_min_->setMinimumWidth(200);
    ok_ = key(QStringLiteral("OK"), QStringLiteral("padOk"));
    ok_->setMinimumWidth(160);
    actions->addWidget(cancel);
    actions->addStretch(1);
    actions->addWidget(use_min_);
    actions->addWidget(ok_);
    v->addLayout(actions);
    connect(cancel, &QPushButton::clicked, this, &EntryPad::close);
    connect(ok_, &QPushButton::clicked, this, &EntryPad::accept);
    connect(use_min_, &QPushButton::clicked, this, [this] {
        // the lowest allowed height, in the chosen reference (rounded up)
        if (const auto v = altitude::fromRelHome(min_rel_, static_cast<altitude::Ref>(ref_), grounds())) {
            setBoxText(0, QString::number(std::ceil(units::altToDisplay(*v))));
            refresh();
        }
    });

    applyScale();
}

void EntryPad::applyScale() {
    card_->setStyleSheet(scaleCss(QString::fromUtf8(kSheet)));
    scaleWidgetTree(this);  // the fields of the open entry are rebuilt at Small sizes, so it runs on each open too
    if (isVisible()) {
        layoutCard();
    }
}

// ---- the boxes ------------------------------------------------------------------------------

void EntryPad::clearBoxes() {
    active_ = nullptr;
    boxes_.clear();
    toggles_.clear();
    for (auto* w : fields_->findChildren<QWidget*>(QString(), Qt::FindDirectChildrenOnly)) {
        delete w;
    }
}

QWidget* EntryPad::newRow() {
    auto* row = new QWidget(fields_);
    auto* h = new QHBoxLayout(row);
    h->setContentsMargins(0, 0, 0, 0);
    h->setSpacing(8);
    fields_->layout()->addWidget(row);
    return row;
}

void EntryPad::addLabel(QWidget* row, const QString& text, const char* name) {
    auto* l = new QLabel(text, row);
    l->setObjectName(QString::fromLatin1(name));
    static_cast<QHBoxLayout*>(row->layout())->addWidget(l);
}

void EntryPad::addBox(QWidget* row, int chars, int decimals, int advance_at, const QString& placeholder) {
    auto* e = new QLineEdit(row);
    e->setStyle(thickCursor());
    const QString re = decimals > 0 ? QStringLiteral("^\\d{0,%1}(\\.\\d{0,%2})?$").arg(chars).arg(decimals)
                                    : QStringLiteral("^\\d{0,%1}$").arg(chars);
    e->setValidator(new QRegularExpressionValidator(QRegularExpression(re), e));  // digits only
    e->setPlaceholderText(placeholder);
    e->setAlignment(Qt::AlignCenter);
    e->setFocusPolicy(Qt::StrongFocus);
    e->setContextMenuPolicy(Qt::NoContextMenu);
    e->setProperty("decimals", decimals);
    e->setProperty("active", false);
    e->setMinimumHeight(56);
    const int width_chars = chars + (decimals > 0 ? decimals + 1 : 0);
    e->setFixedWidth(std::max(64, 18 * width_chars + 30));
    e->installEventFilter(this);
    const int index = static_cast<int>(boxes_.size());
    connect(e, &QLineEdit::textChanged, this, [this] { refresh(); });
    connect(e, &QLineEdit::textEdited, this, [this, index, e] {
        // a full box moves on to the next one
        const int at = boxes_[index].advance_at;
        if (at > 0 && e->text().size() >= at && e->cursorPosition() == e->text().size() &&
            index + 1 < static_cast<int>(boxes_.size()) && active_ == e) {
            focusBox(index + 1);
            active_->selectAll();  // typing replaces what is there
        }
    });
    connect(e, &QLineEdit::returnPressed, this, &EntryPad::accept);
    static_cast<QHBoxLayout*>(row->layout())->addWidget(e);
    boxes_.push_back({e, advance_at});
}

void EntryPad::addToggle(QWidget* row, const QString& a, const QString& b, bool first) {
    Toggle t;
    t.a = key(a, QStringLiteral("padSel"));
    t.b = key(b, QStringLiteral("padSel"));
    t.first = first;
    for (auto* btn : {t.a, t.b}) {
        btn->setParent(row);
        btn->setCheckable(true);
        btn->setMinimumWidth(kKey);
        static_cast<QHBoxLayout*>(row->layout())->addWidget(btn);
    }
    t.a->setChecked(first);
    t.b->setChecked(!first);
    const int index = static_cast<int>(toggles_.size());
    connect(t.a, &QPushButton::clicked, this, [this, index] {
        toggles_[index].first = true;
        toggles_[index].a->setChecked(true);
        toggles_[index].b->setChecked(false);
        refresh();
    });
    connect(t.b, &QPushButton::clicked, this, [this, index] {
        toggles_[index].first = false;
        toggles_[index].a->setChecked(false);
        toggles_[index].b->setChecked(true);
        refresh();
    });
    toggles_.push_back(t);
}

void EntryPad::focusBox(int i) {
    if (i < 0 || i >= static_cast<int>(boxes_.size())) {
        return;
    }
    for (auto& b : boxes_) {
        const bool on = b.edit == boxes_[i].edit;
        if (b.edit->property("active").toBool() != on) {
            b.edit->setProperty("active", on);
            b.edit->style()->unpolish(b.edit);
            b.edit->style()->polish(b.edit);
        }
    }
    active_ = boxes_[i].edit;
    active_->setFocus(Qt::OtherFocusReason);
    if (dot_key_) {
        // in an IP address '.' moves on to the next number
        dot_key_->setEnabled(active_->property("decimals").toInt() > 0 || kind_ == Kind::Address);
    }
}

int EntryPad::activeIndex() const {
    for (size_t i = 0; i < boxes_.size(); ++i) {
        if (boxes_[i].edit == active_) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

QString EntryPad::boxText(int i) const {
    return i >= 0 && i < static_cast<int>(boxes_.size()) ? boxes_[i].edit->text().trimmed() : QString();
}

double EntryPad::boxValue(int i, bool* ok) const {
    const QString t = boxText(i);
    bool good = false;
    const double v = t.toDouble(&good);
    if (ok) {
        *ok = good;
    }
    return good ? v : 0.0;
}

void EntryPad::setBoxText(int i, const QString& t) {
    if (i >= 0 && i < static_cast<int>(boxes_.size())) {
        boxes_[i].edit->setText(t);
        boxes_[i].edit->end(false);
    }
}

void EntryPad::typeText(const QString& s) {
    if (activeIndex() < 0) {
        focusBox(0);
    }
    const int i = activeIndex();
    if (i < 0) {
        return;
    }
    if (kind_ == Kind::Address && s == QStringLiteral(".")) {
        if (i + 1 < static_cast<int>(boxes_.size())) {
            focusBox(i + 1);
            active_->selectAll();
        }
        return;
    }
    QLineEdit* e = active_;
    e->insert(s);  // at the cursor; the box's validator refuses what doesn't fit
    const int at = boxes_[i].advance_at;
    if (active_ == e && at > 0 && e->text().size() >= at && e->cursorPosition() == e->text().size() &&
        i + 1 < static_cast<int>(boxes_.size())) {
        focusBox(i + 1);
        active_->selectAll();
    }
    refresh();
}

bool EntryPad::eventFilter(QObject* obj, QEvent* e) {
    auto* edit = qobject_cast<QLineEdit*>(obj);
    if (edit && e->type() == QEvent::FocusIn) {
        for (size_t i = 0; i < boxes_.size(); ++i) {
            if (boxes_[i].edit == edit && active_ != edit) {
                focusBox(static_cast<int>(i));
            }
        }
    } else if (edit && e->type() == QEvent::KeyPress) {
        auto* k = static_cast<QKeyEvent*>(e);
        const int i = activeIndex();
        switch (k->key()) {
        case Qt::Key_Tab:
            focusBox(boxes_.empty() ? 0 : (i + 1) % static_cast<int>(boxes_.size()));
            return true;
        case Qt::Key_Backtab:
            focusBox(boxes_.empty() ? 0 : (i + static_cast<int>(boxes_.size()) - 1) % static_cast<int>(boxes_.size()));
            return true;
        case Qt::Key_Backspace:
        case Qt::Key_Left:
            if (edit->cursorPosition() == 0 && !edit->hasSelectedText() && i > 0) {
                focusBox(i - 1);
                active_->end(false);
                if (k->key() == Qt::Key_Backspace) {
                    active_->backspace();
                }
                return true;
            }
            break;
        case Qt::Key_Right:
            if (edit->cursorPosition() == edit->text().size() && i + 1 < static_cast<int>(boxes_.size())) {
                focusBox(i + 1);
                active_->home(false);
                return true;
            }
            break;
        case Qt::Key_Period:
        case Qt::Key_Colon:
            if (kind_ == Kind::Address) {
                typeText(QStringLiteral("."));
                return true;
            }
            break;
        case Qt::Key_Escape:
            return true;  // no keyboard shortcuts: Cancel is a tap
        default:
            break;
        }
    }
    return QWidget::eventFilter(obj, e);
}

// ---- opening --------------------------------------------------------------------------------

void EntryPad::showFor(Kind kind) {
    kind_ = kind;
    ref_row_->setVisible(kind == Kind::Altitude);
    nudge_keys_->setVisible(kind == Kind::Altitude);
    presets_->setVisible((kind == Kind::Duration || kind == Kind::Address || kind == Kind::Number) &&
                         presets_->layout()->count() > 0);
    scaleWidgetTree(this);  // this entry's fields and chips were just built at Small sizes
    for (auto* w : fields_->findChildren<QWidget*>()) {
        w->ensurePolished();
    }
    if (parentWidget()) {
        setGeometry(parentWidget()->rect());  // before the card is centred in it
    }
    layoutCard();
    refresh();
    show();
    raise();
    focusBox(0);
    if (active_) {
        active_->end(false);
    }
}

void EntryPad::openCoordinate(const QString& title, std::optional<coords::LatLon> initial,
                              std::function<void(double, double)> done) {
    coord_done_ = std::move(done);
    alt_done_ = nullptr;
    dur_done_ = nullptr;
    addr_done_ = nullptr;
    num_done_ = nullptr;
    fmt_ = units::coordFormat();  // the format chosen in Settings; not switched here
    title_->setText(title);
    clearBoxes();
    // hemispheres: from the value, else from where the aircraft (or home) is
    std::optional<coords::LatLon> where = initial ? initial : (aircraft_ ? aircraft_ : home_);
    const bool north = where ? where->lat >= 0.0 : true;
    const bool east = where ? where->lon >= 0.0 : true;
    if (fmt_ == coords::Fmt::Utm) {
        QWidget* r1 = newRow();
        addLabel(r1, QStringLiteral("ZONE"), "padRowName");
        addBox(r1, 2, 0, 2, QStringLiteral("36"));
        addToggle(r1, QStringLiteral("NORTH"), QStringLiteral("SOUTH"), north);
        static_cast<QHBoxLayout*>(r1->layout())->addStretch(1);
        QWidget* r2 = newRow();
        addLabel(r2, QStringLiteral("EASTING"), "padRowName");
        addBox(r2, 6, 0, 6, QStringLiteral("668123"));
        addLabel(r2, QStringLiteral("m"));
        QWidget* r3 = newRow();
        addLabel(r3, QStringLiteral("NORTHING"), "padRowName");
        addBox(r3, 7, 0, 0, QStringLiteral("3551234"));
        addLabel(r3, QStringLiteral("m"));
        static_cast<QHBoxLayout*>(r2->layout())->addStretch(1);
        static_cast<QHBoxLayout*>(r3->layout())->addStretch(1);
        if (initial) {
            const auto u = coords::toUtm(initial->lat, initial->lon);
            setBoxText(0, QString::number(u.zone));
            setBoxText(1, QString::number(std::lround(std::floor(u.easting))));
            setBoxText(2, QString::number(std::lround(std::floor(u.northing))));
        }
    } else if (fmt_ == coords::Fmt::Dms) {
        auto row = [&](const QString& name, int deg_chars, const QString& a, const QString& b, bool first) {
            QWidget* r = newRow();
            addLabel(r, name, "padRowName");
            addBox(r, deg_chars, 0, deg_chars, deg_chars == 2 ? QStringLiteral("dd") : QStringLiteral("ddd"));
            addLabel(r, QStringLiteral("°"));
            addBox(r, 2, 0, 2, QStringLiteral("mm"));
            addLabel(r, QStringLiteral("′"));
            addBox(r, 2, 1, 0, QStringLiteral("ss.s"));
            addLabel(r, QStringLiteral("″"));
            addToggle(r, a, b, first);
            static_cast<QHBoxLayout*>(r->layout())->addStretch(1);
        };
        row(QStringLiteral("LAT"), 2, QStringLiteral("N"), QStringLiteral("S"), north);
        row(QStringLiteral("LON"), 3, QStringLiteral("E"), QStringLiteral("W"), east);
        if (initial) {
            const auto la = coords::toDms(initial->lat);
            const auto lo = coords::toDms(initial->lon);
            setBoxText(0, QString::number(la.deg));
            setBoxText(1, QStringLiteral("%1").arg(la.min, 2, 10, QLatin1Char('0')));
            setBoxText(2, QString::number(la.sec, 'f', 1));
            setBoxText(3, QString::number(lo.deg));
            setBoxText(4, QStringLiteral("%1").arg(lo.min, 2, 10, QLatin1Char('0')));
            setBoxText(5, QString::number(lo.sec, 'f', 1));
        }
    } else {
        auto row = [&](const QString& name, int chars, const QString& a, const QString& b, bool first) {
            QWidget* r = newRow();
            addLabel(r, name, "padRowName");
            addBox(r, chars, 6, 0, chars == 2 ? QStringLiteral("dd.dddddd") : QStringLiteral("ddd.dddddd"));
            addLabel(r, QStringLiteral("°"));
            addToggle(r, a, b, first);
            static_cast<QHBoxLayout*>(r->layout())->addStretch(1);
        };
        row(QStringLiteral("LAT"), 2, QStringLiteral("N"), QStringLiteral("S"), north);
        row(QStringLiteral("LON"), 3, QStringLiteral("E"), QStringLiteral("W"), east);
        if (initial) {
            setBoxText(0, QString::number(std::abs(initial->lat), 'f', 6));
            setBoxText(1, QString::number(std::abs(initial->lon), 'f', 6));
        }
    }
    showFor(Kind::Coordinate);
}

void EntryPad::openAltitude(const QString& title, double lat, double lon, double initial_rel, double home_amsl,
                            std::optional<coords::LatLon> land, int ref, double min_agl,
                            std::function<void(double, int)> done) {
    alt_done_ = std::move(done);
    coord_done_ = nullptr;
    dur_done_ = nullptr;
    addr_done_ = nullptr;
    num_done_ = nullptr;
    lat_ = lat;
    lon_ = lon;
    home_amsl_ = home_amsl;
    land_ = land;
    min_agl_ = min_agl;
    ref_ = std::clamp(ref, 0, 2);
    title_->setText(title);
    clearBoxes();
    QWidget* r = newRow();
    addLabel(r, QStringLiteral("HEIGHT"), "padRowName");
    addBox(r, 5, 0, 0, QStringLiteral("0"));
    addLabel(r, units::altUnitLabel());
    static_cast<QHBoxLayout*>(r->layout())->addStretch(1);
    // start from the current height, in the chosen reference (AGL without terrain: ASL)
    const auto g = grounds();
    if (ref_ == 1 && !g.point) {
        ref_ = 0;
    }
    if (const auto v = altitude::fromRelHome(initial_rel, static_cast<altitude::Ref>(ref_), g)) {
        setBoxText(0, QString::number(std::lround(units::altToDisplay(*v))));
    }
    poll_->start();
    showFor(Kind::Altitude);
}

void EntryPad::setPresets(const std::vector<std::pair<QString, std::function<void()>>>& chips) {
    auto* lay = presets_->layout();
    while (auto* item = lay->takeAt(0)) {
        delete item->widget();
        delete item;
    }
    for (const auto& [label, fn] : chips) {
        auto* b = key(label, QStringLiteral("padSel"));
        b->setMinimumHeight(48);
        connect(b, &QPushButton::clicked, this, fn);
        lay->addWidget(b);
    }
}

void EntryPad::setDuration(double seconds) {
    const long t = std::lround(seconds);
    setBoxText(0, t <= 0 ? QString() : QString::number(t / 60));
    setBoxText(1, t <= 0 ? QString() : QStringLiteral("%1").arg(t % 60, 2, 10, QLatin1Char('0')));
}

void EntryPad::openDuration(const QString& title, double seconds, double max_s, std::function<void(double)> done) {
    coord_done_ = nullptr;
    alt_done_ = nullptr;
    dur_done_ = std::move(done);
    addr_done_ = nullptr;
    num_done_ = nullptr;
    max_value_ = max_s;
    title_->setText(title);
    std::vector<std::pair<QString, std::function<void()>>> chips;
    for (const auto& [label, sec] : std::vector<std::pair<QString, double>>{{QStringLiteral("FLY THROUGH"), 0.0},
                                                                             {QStringLiteral("0:30"), 30.0},
                                                                             {QStringLiteral("1:00"), 60.0},
                                                                             {QStringLiteral("2:00"), 120.0},
                                                                             {QStringLiteral("5:00"), 300.0},
                                                                             {QStringLiteral("10:00"), 600.0}}) {
        const double s = sec;
        chips.emplace_back(label, [this, s] {
            setDuration(s);
            refresh();
        });
    }
    setPresets(chips);
    clearBoxes();
    QWidget* r = newRow();
    addLabel(r, QStringLiteral("HOLD"), "padRowName");
    addBox(r, 2, 0, 0, QStringLiteral("0"));
    addLabel(r, QStringLiteral("min"));
    addBox(r, 2, 0, 0, QStringLiteral("00"));
    addLabel(r, QStringLiteral("s"));
    static_cast<QHBoxLayout*>(r->layout())->addStretch(1);
    setDuration(seconds);
    showFor(Kind::Duration);
}

void EntryPad::openAddress(const QString& title, const QString& host, int port, const std::vector<AddressChip>& chips,
                           std::function<void(const QString&, int)> done) {
    coord_done_ = nullptr;
    alt_done_ = nullptr;
    dur_done_ = nullptr;
    num_done_ = nullptr;
    addr_done_ = std::move(done);
    title_->setText(title);
    std::vector<std::pair<QString, std::function<void()>>> presets;
    for (const auto& c : chips) {
        presets.emplace_back(c.label, [this, c] {
            if (c.host.isEmpty()) {
                auto d = addr_done_;
                close();
                if (d) {
                    d(QString(), 0);
                }
                return;
            }
            const QStringList parts = c.host.split(QLatin1Char('.'));
            for (int i = 0; i < 4 && i < parts.size(); ++i) {
                setBoxText(i, parts[i]);
            }
            if (c.port > 0) {
                setBoxText(4, QString::number(c.port));
            }
            refresh();
        });
    }
    setPresets(presets);
    clearBoxes();
    QWidget* r = newRow();
    addLabel(r, QStringLiteral("IP"), "padRowName");
    for (int i = 0; i < 4; ++i) {
        addBox(r, 3, 0, 3, QStringLiteral("0"));
        if (i < 3) {
            addLabel(r, QStringLiteral("."));
        }
    }
    addLabel(r, QStringLiteral(":"));
    addBox(r, 5, 0, 0, QStringLiteral("port"));
    static_cast<QHBoxLayout*>(r->layout())->addStretch(1);
    const QStringList parts = host.split(QLatin1Char('.'));
    if (parts.size() == 4) {
        for (int i = 0; i < 4; ++i) {
            setBoxText(i, parts[i]);
        }
    }
    if (port > 0) {
        setBoxText(4, QString::number(port));
    }
    showFor(Kind::Address);
}

void EntryPad::openNumber(const QString& title, const QString& name, int value, int min, int max,
                          const std::vector<std::pair<QString, int>>& chips, std::function<void(int)> done) {
    coord_done_ = nullptr;
    alt_done_ = nullptr;
    dur_done_ = nullptr;
    addr_done_ = nullptr;
    num_done_ = std::move(done);
    min_int_ = min;
    max_int_ = max;
    title_->setText(title);
    std::vector<std::pair<QString, std::function<void()>>> presets;
    for (const auto& [label, v] : chips) {
        const int val = v;
        presets.emplace_back(label, [this, val] {
            setBoxText(0, QString::number(val));
            refresh();
        });
    }
    setPresets(presets);
    clearBoxes();
    QWidget* r = newRow();
    addLabel(r, name, "padRowName");
    addBox(r, static_cast<int>(QString::number(max).size()), 0, 0, QString::number(min));
    static_cast<QHBoxLayout*>(r->layout())->addStretch(1);
    setBoxText(0, QString::number(value));
    showFor(Kind::Number);
}

void EntryPad::close() {
    poll_->stop();
    hide();
    coord_done_ = nullptr;
    alt_done_ = nullptr;
    dur_done_ = nullptr;
    addr_done_ = nullptr;
    num_done_ = nullptr;
    emit closed();
}

altitude::Grounds EntryPad::grounds() const {
    altitude::Grounds g;
    g.home_amsl = home_amsl_;
    if (ground_) {
        g.point = ground_(lat_, lon_);
        if (land_) {
            g.land = ground_(land_->lat, land_->lon);
        }
    }
    if (land_ && !g.land && home_ && geo::distance(land_->lat, land_->lon, home_->lat, home_->lon) < 30.0) {
        g.land = home_amsl_;  // L at home: home's altitude
    }
    return g;
}

// ---- reading the boxes ------------------------------------------------------------------------

void EntryPad::refresh() {
    QString preview;
    bool ok = false;
    use_min_->setVisible(false);
    if (kind_ == Kind::Coordinate) {
        refreshCoordinate(preview, ok);
    } else if (kind_ == Kind::Duration) {
        refreshDuration(preview, ok);
    } else if (kind_ == Kind::Address) {
        refreshAddress(preview, ok);
    } else if (kind_ == Kind::Number) {
        refreshNumber(preview, ok);
    } else {
        refreshAltitude(preview, ok);
    }
    preview_->setText(preview);
    ok_->setEnabled(ok);
    update();
}

void EntryPad::refreshCoordinate(QString& preview, bool& ok) {
    parsed_.reset();
    bool any = false;
    bool all = true;
    for (size_t i = 0; i < boxes_.size(); ++i) {
        const bool filled = !boxText(static_cast<int>(i)).isEmpty();
        any = any || filled;
        // DMS seconds may be left empty (= 0)
        all = all && (filled || (fmt_ == coords::Fmt::Dms && (i == 2 || i == 5)));
    }
    if (all && !boxes_.empty()) {
        if (fmt_ == coords::Fmt::Utm && toggles_.size() == 1) {
            parsed_ = coords::fromUtmChecked(static_cast<int>(boxValue(0)), toggles_[0].first, boxValue(1), boxValue(2));
        } else if (fmt_ == coords::Fmt::Dms && toggles_.size() == 2) {
            const auto la = coords::angleFromDms(boxValue(0), boxValue(1), boxValue(2), toggles_[0].first, 90.0);
            const auto lo = coords::angleFromDms(boxValue(3), boxValue(4), boxValue(5), toggles_[1].first, 180.0);
            if (la && lo) {
                parsed_ = coords::LatLon{*la, *lo};
            }
        } else if (toggles_.size() == 2) {
            const double la = boxValue(0) * (toggles_[0].first ? 1.0 : -1.0);
            const double lo = boxValue(1) * (toggles_[1].first ? 1.0 : -1.0);
            parsed_ = coords::checkedLatLon(la, lo);
        }
    }
    if (!any) {
        preview = QStringLiteral("type the numbers into the boxes; tap a box to move to it");
    } else if (!all) {
        preview = QStringLiteral("fill in every box");
    } else if (!parsed_) {
        preview = QStringLiteral("<span style='color:#ff4d5e'>%1</span>")
                      .arg(fmt_ == coords::Fmt::Utm   ? QStringLiteral("not a valid UTM position (zone 1-60, easting "
                                                                       "100000-900000)")
                           : fmt_ == coords::Fmt::Dms ? QStringLiteral("out of range (minutes and seconds below 60, "
                                                                       "latitude up to 90°, longitude up to 180°)")
                                                      : QStringLiteral("out of range (latitude up to 90°, longitude up "
                                                                       "to 180°)"));
    } else {
        const auto& p = *parsed_;
        QStringList lines;
        for (int f = 0; f < 3; ++f) {
            if (f != static_cast<int>(fmt_)) {
                lines << QString::fromStdString(coords::format(p.lat, p.lon, static_cast<coords::Fmt>(f)));
            }
        }
        if (aircraft_) {
            lines << QStringLiteral("%1 · %2° from the aircraft")
                         .arg(units::distanceText(geo::distance(aircraft_->lat, aircraft_->lon, p.lat, p.lon)))
                         .arg(std::lround(geo::bearing(aircraft_->lat, aircraft_->lon, p.lat, p.lon)) % 360);
        }
        ok = true;
        if (home_ && geo::distance(home_->lat, home_->lon, p.lat, p.lon) > kMaxFromHome) {
            ok = false;
            lines << QStringLiteral("<span style='color:#ff4d5e'>%1 from home: too far (a typo?)</span>")
                         .arg(units::distanceText(geo::distance(home_->lat, home_->lon, p.lat, p.lon)));
        }
        preview = lines.join(QStringLiteral("<br>"));
    }
}

void EntryPad::refreshDuration(QString& preview, bool& ok) {
    value_.reset();
    const double m = boxValue(0);
    const double s = boxValue(1);
    const double sec = m * 60.0 + s;
    if (s >= 60.0) {
        preview = QStringLiteral("<span style='color:#ff4d5e'>seconds: 0 to 59</span>");
    } else if (sec > max_value_) {
        preview = QStringLiteral("<span style='color:#ff4d5e'>at most %1</span>").arg(clock(max_value_));
    } else {
        value_ = sec;
        ok = true;
        const long t = std::lround(sec);
        QStringList words;
        if (t >= 60) {
            words << QStringLiteral("%1 min").arg(t / 60);
        }
        if (t % 60 != 0) {
            words << QStringLiteral("%1 s").arg(t % 60);
        }
        preview = t <= 0 ? QStringLiteral("fly through (no hold)")
                         : QStringLiteral("circle here %1, then go on").arg(words.join(QLatin1Char(' ')));
    }
}

void EntryPad::refreshAddress(QString& preview, bool& ok) {
    address_.reset();
    QStringList octets;
    bool all = true;
    bool in_range = true;
    for (int i = 0; i < 4; ++i) {
        bool good = false;
        const int v = boxText(i).toInt(&good);
        all = all && good;
        in_range = in_range && good && v >= 0 && v <= 255;
        octets << QString::number(v);
    }
    bool port_ok = false;
    const int port = boxText(4).toInt(&port_ok);
    if (!all || !port_ok) {
        preview = QStringLiteral("four numbers, then the port; '.' moves to the next number");
        return;
    }
    if (!in_range) {
        preview = QStringLiteral("<span style='color:#ff4d5e'>each number is 0 to 255</span>");
        return;
    }
    if (port < 1 || port > 65535) {
        preview = QStringLiteral("<span style='color:#ff4d5e'>port: 1 to 65535</span>");
        return;
    }
    const QString host = octets.join(QLatin1Char('.'));
    address_ = std::make_pair(host, port);
    ok = true;
    if (host == QStringLiteral("0.0.0.0")) {
        preview = QStringLiteral("all networks of this computer (WiFi, Ethernet, radio modems), port %1").arg(port);
    } else if (host.startsWith(QStringLiteral("127."))) {
        preview = QStringLiteral("this computer only, port %1 (a simulator or a program running here)").arg(port);
    } else {
        preview = QStringLiteral("%1 port %2").arg(host).arg(port);
    }
}

void EntryPad::refreshNumber(QString& preview, bool& ok) {
    number_.reset();
    bool good = false;
    const int v = boxText(0).toInt(&good);
    if (!good) {
        preview = QStringLiteral("%1 to %2").arg(min_int_).arg(max_int_);
    } else if (v < min_int_ || v > max_int_) {
        preview = QStringLiteral("<span style='color:#ff4d5e'>%1 to %2</span>").arg(min_int_).arg(max_int_);
    } else {
        number_ = v;
        ok = true;
        preview = QString::number(v);
    }
}

void EntryPad::refreshAltitude(QString& preview, bool& ok) {
    for (size_t i = 0; i < ref_buttons_.size(); ++i) {
        ref_buttons_[i]->setChecked(static_cast<int>(i) == ref_);
    }
    const auto g = grounds();
    ref_buttons_[1]->setEnabled(g.point.has_value());
    bool number = false;
    const double shown = boxValue(0, &number);
    const double meters = units::altFromDisplay(shown);
    const altitude::Check c = altitude::check(meters, static_cast<altitude::Ref>(ref_), g, min_agl_);
    min_rel_ = c.min_rel;
    QStringList lines;
    lines << (g.point ? QStringLiteral("ground here %1 MSL").arg(units::altText(*g.point))
                      : QStringLiteral("<span style='color:#ffb547'>no terrain data here: checked against home "
                                       "level</span>"));
    if (number && c.ok) {
        const double rel = c.rel_home;
        QStringList same;
        for (int r = 0; r < 3; ++r) {
            if (const auto v = altitude::fromRelHome(rel, static_cast<altitude::Ref>(r), g)) {
                same << QStringLiteral("%1 %2").arg(QLatin1String(kRefNames[r]), units::altText(*v));
            }
        }
        lines << same.join(QStringLiteral("  ·  ")) + QStringLiteral("  ·  %1 above home").arg(units::altText(rel));
        rel_ = rel;
        ok = true;
    } else {
        rel_.reset();
        if (number) {
            lines << QStringLiteral("<span style='color:#ff4d5e'>%1</span>").arg(QString::fromLatin1(c.reason));
        }
    }
    const auto min_shown = altitude::fromRelHome(c.min_rel, static_cast<altitude::Ref>(ref_), g);
    use_min_->setVisible(min_shown.has_value() && !ok);
    if (min_shown) {
        use_min_->setText(QStringLiteral("USE MINIMUM %1").arg(units::altText(std::ceil(*min_shown))));
    }
    lines << QStringLiteral("height %1 · minimum: %2 above the ground here")
                 .arg(QLatin1String(kRefNames[ref_]), units::altText(min_agl_));
    preview = lines.join(QStringLiteral("<br>"));
}

void EntryPad::accept() {
    if (!ok_->isEnabled()) {
        return;
    }
    if (kind_ == Kind::Coordinate && parsed_ && coord_done_) {
        auto done = coord_done_;
        const auto p = *parsed_;
        close();
        done(p.lat, p.lon);
    } else if (kind_ == Kind::Duration && value_ && dur_done_) {
        auto done = dur_done_;
        const double v = *value_;
        close();
        done(v);
    } else if (kind_ == Kind::Address && address_ && addr_done_) {
        auto done = addr_done_;
        const auto a = *address_;
        close();
        done(a.first, a.second);
    } else if (kind_ == Kind::Number && number_ && num_done_) {
        auto done = num_done_;
        const int v = *number_;
        close();
        done(v);
    } else if (kind_ == Kind::Altitude && rel_ && alt_done_) {
        auto done = alt_done_;
        const double rel = *rel_;
        const int ref = ref_;
        close();
        done(rel, ref);
    }
}

void EntryPad::layoutCard() {
    fields_->layout()->invalidate();
    card_->layout()->invalidate();
    card_->layout()->activate();
    const QSize s = card_->layout()->sizeHint().expandedTo(QSize(sci(640), 0));
    card_->resize(s);
    card_->move((width() - s.width()) / 2, std::max(20, (height() - s.height()) / 2));
    update();
}

void EntryPad::resizeEvent(QResizeEvent* e) {
    QWidget::resizeEvent(e);
    layoutCard();
}

void EntryPad::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    p.fillRect(rect(), alpha(kBg, 0.72));
    const QRectF r = QRectF(card_->geometry()).adjusted(-2, -2, 2, 2);
    drawGlass(p, chamferPath(r, 16), kCyan, 0.8, alpha(kGlassDeep, 0.97));
    p.setPen(QPen(kCyan, 2));
    p.drawLine(QPointF(r.left() + 1, r.top() + 16), QPointF(r.left() + 16, r.top() + 1));
}

void EntryPad::mousePressEvent(QMouseEvent*) {
    // taps outside the card don't fall through to the map
}

void EntryPad::keyPressEvent(QKeyEvent* e) {
    // reached only when no box has the focus: send the key to the active box
    if (e->key() == Qt::Key_Return || e->key() == Qt::Key_Enter) {
        accept();
        return;
    }
    const QString t = e->text();
    if (!t.isEmpty() && (t.at(0).isDigit() || t.at(0) == QLatin1Char('.'))) {
        typeText(t);
    }
}

}  // namespace gcs::ui
