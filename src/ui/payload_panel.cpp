#include "payload_panel.hpp"

#include "theme.hpp"

#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QWheelEvent>

#include <algorithm>
#include <cmath>
#include <set>

namespace gcs::ui {

using namespace theme;
using payload::CommandDef;
using payload::ParamDef;

namespace {

constexpr double kWidth = 460.0;   // logical px
constexpr double kPad = 16.0;
constexpr double kHeader = 74.0;   // title + status + close, not scrolled
constexpr double kGap = 8.0;
constexpr double kLabelH = 22.0;
constexpr double kTileH = 48.0;    // touch targets
constexpr double kRowH = 44.0;
constexpr double kFieldH = 22.0;
constexpr int kQuickCols = 4;
constexpr size_t kLogLines = 6;

QString q(const std::string& s) { return QString::fromStdString(s); }

// "Illuminator.Safety" -> "Illuminator"; plain names -> "" (general)
std::string groupOf(const CommandDef& c) {
    if (!c.id) {
        return "MAV_CMD";
    }
    const auto dot = c.name.find('.');
    return dot == std::string::npos ? std::string() : c.name.substr(0, dot);
}

std::vector<std::string> groups(const payload::Icd& icd) {
    std::vector<std::string> g{"ALL"};
    std::set<std::string> seen;
    for (const auto& c : icd.commands) {
        const std::string k = groupOf(c);
        if (!k.empty() && seen.insert(k).second) {
            g.push_back(k);
        }
    }
    g.push_back("GENERAL");
    return g;
}

bool inGroup(const CommandDef& c, const std::string& g) {
    if (g == "ALL") {
        return true;
    }
    if (g == "GENERAL") {
        return groupOf(c).empty();
    }
    return groupOf(c) == g;
}

}  // namespace

PayloadPanel::PayloadPanel(QWidget* parent) : QWidget(parent) {
    hide();
}

void PayloadPanel::setSnapshot(const payload::PayloadSnapshot& s) {
    snap_ = s;
    if (!device() && !snap_.devices.empty() && snap_.devices.front().icd) {
        selected_ = snap_.devices.front().icd->id;
        command_ = -1;
    }
}

void PayloadPanel::setAdapterInfo(const std::optional<payload::PayloadView>& view,
                                  const std::vector<std::pair<std::string, payload::CommandCall>>& actions) {
    view_ = view;
    actions_ = actions;
}

void PayloadPanel::addResult(const QString& text, bool ok) {
    log_.emplace_front(text, ok);
    while (log_.size() > kLogLines) {
        log_.pop_back();
    }
    update();
}

const payload::DeviceSnapshot* PayloadPanel::device() const {
    return snap_.find(selected_);
}

const CommandDef* PayloadPanel::command() const {
    const auto* d = device();
    if (!d || !d->icd || command_ < 0 || command_ >= static_cast<int>(d->icd->commands.size())) {
        return nullptr;
    }
    return &d->icd->commands[static_cast<size_t>(command_)];
}

QRectF PayloadPanel::logicalRect() const {
    return {QPointF(0, 0), logicalSize(size())};
}

void PayloadPanel::place(const QRect& area) {
    area_ = area;
    const double k = uiScale();
    const double w = std::min(kWidth, area.width() / k - 16.0);
    const double h = area.height() / k;
    setGeometry(static_cast<int>(area.right() - w * k), area.top(), static_cast<int>(w * k), static_cast<int>(h * k));
    clampScroll();
}

void PayloadPanel::clampScroll() {
    const double view = logicalRect().height() - kHeader - 6.0;
    scroll_ = std::clamp(scroll_, 0.0, std::max(0.0, layout(logicalRect().width(), nullptr) - view));
}

void PayloadPanel::tick(double dt) {
    if (!isVisible()) {
        return;
    }
    time_ += dt;
    flash_ = std::max(0.0, flash_ - dt * 2.5);
    update();  // reports change continuously
}

void PayloadPanel::selectCommand(int index) {
    command_ = index;
    args_.clear();
    if (const CommandDef* c = command()) {
        for (const auto& p : c->params) {
            args_.push_back(p.def);
        }
    }
}

double PayloadPanel::step(const ParamDef& p) const {
    if (p.min && p.max && *p.max - *p.min <= 2.0) {
        return 0.05;
    }
    if (p.unit == "deg") {
        return 1.0;
    }
    return 1.0;
}

QString PayloadPanel::paramText(const ParamDef& p, double v) const {
    const auto* d = device();
    if (d && d->icd && !p.enum_name.empty()) {
        const std::string l = d->icd->enumLabel(p.enum_name, static_cast<int64_t>(std::llround(v)));
        if (!l.empty()) {
            return q(l).toUpper();
        }
    }
    QString s = std::fabs(v - std::round(v)) < 1e-9 ? QString::number(static_cast<long long>(std::llround(v)))
                                                     : QString::number(v, 'f', std::fabs(v) < 10 ? 3 : 1);
    if (!p.unit.empty()) {
        s += QStringLiteral(" ") + q(p.unit);
    }
    return s;
}

// The whole content, top to bottom; fills `items` (hit targets) when given.
double PayloadPanel::layout(double width, std::vector<Item>* items) const {
    const double inner = width - 2 * kPad;
    double y = 0.0;
    auto add = [&](const QRectF& r, int kind, int index) {
        if (items) {
            items->push_back({r, kind, index});
        }
    };
    const auto* d = device();
    // devices
    if (snap_.devices.size() > 1) {
        y += kLabelH;
        const double w = (inner - kGap) / 2.0;
        for (size_t i = 0; i < snap_.devices.size(); ++i) {
            add(QRectF(kPad + (i % 2) * (w + kGap), y + (i / 2) * (kTileH + kGap), w, kTileH), kDevice,
                static_cast<int>(i));
        }
        y += ((snap_.devices.size() + 1) / 2) * (kTileH + kGap) + 6;
    }
    if (!d) {
        return y + 60;
    }
    // adapter: status lines + quick actions
    if (view_) {
        y += kLabelH + 3 * kFieldH + 8;
    }
    if (!actions_.empty()) {
        y += kLabelH;
        const double w = (inner - (kQuickCols - 1) * kGap) / kQuickCols;
        for (size_t i = 0; i < actions_.size(); ++i) {
            const int row = static_cast<int>(i) / kQuickCols, col = static_cast<int>(i) % kQuickCols;
            add(QRectF(kPad + col * (w + kGap), y + row * (kTileH + kGap), w, kTileH), kQuick, static_cast<int>(i));
        }
        y += ((actions_.size() + kQuickCols - 1) / kQuickCols) * (kTileH + kGap) + 6;
    }
    // command editor
    y += kLabelH;
    add(QRectF(kPad, y, inner, kRowH), kCmdToggle, 0);
    y += kRowH + kGap;
    if (commands_open_) {
        add(QRectF(kPad, y, inner, kTileH - 8), kFilter, 0);
        y += kTileH - 8 + kGap;
        const auto gs = groups(*d->icd);
        const std::string g = gs[static_cast<size_t>(filter_) % gs.size()];
        for (size_t i = 0; i < d->icd->commands.size(); ++i) {
            if (!inGroup(d->icd->commands[i], g)) {
                continue;
            }
            add(QRectF(kPad, y, inner, kRowH - 6), kCmdRow, static_cast<int>(i));
            y += kRowH - 6 + 4;
        }
        y += 6;
    } else if (const CommandDef* c = command()) {
        for (size_t i = 0; i < c->params.size(); ++i) {
            const double bw = 54.0;
            add(QRectF(kPad + inner * 0.38, y, bw, kTileH), kMinus, static_cast<int>(i));
            add(QRectF(kPad + inner * 0.38 + bw + 4, y, inner * 0.62 - 2 * bw - 8, kTileH), kValue,
                static_cast<int>(i));
            add(QRectF(kPad + inner - bw, y, bw, kTileH), kPlus, static_cast<int>(i));
            y += kTileH + 6;
        }
        add(QRectF(kPad, y, inner, kTileH), kSend, 0);
        y += kTileH + kGap;
    }
    if (!log_.empty()) {  // command results
        add(QRectF(kPad, y, inner, kLabelH + log_.size() * kFieldH), kResults, 0);
        y += kLabelH + log_.size() * kFieldH + 8;
    }
    // reports
    add(QRectF(kPad, y, inner, kLabelH), kReportsLabel, 0);
    y += kLabelH;
    for (const auto& kv : d->reports) {
        add(QRectF(kPad, y, inner, kRowH - 8), kReport, kv.first);
        y += kRowH - 8 + 2;
        const auto it = open_reports_.find(kv.first);
        if (it != open_reports_.end() && it->second && kv.second.report.def) {
            const auto& rep = kv.second.report;
            y += rep.def->header.fields.size() * kFieldH;
            if (rep.def->has_records) {
                y += rep.records.size() * (kFieldH * (1 + (rep.def->records.fields.size() + 1) / 2)) + kFieldH;
            }
            y += 6;
        }
    }
    if (d->reports.empty()) {
        y += 40;
    }
    return y + kPad;
}

void PayloadPanel::press(const Item& it) {
    flash_kind_ = it.kind;
    flash_index_ = it.index;
    flash_ = 1.0;
    const auto* d = device();
    switch (it.kind) {
    case kDevice:
        if (it.index < static_cast<int>(snap_.devices.size()) && snap_.devices[static_cast<size_t>(it.index)].icd) {
            selected_ = snap_.devices[static_cast<size_t>(it.index)].icd->id;
            command_ = -1;
            commands_open_ = false;
        }
        break;
    case kQuick:
        if (d && it.index < static_cast<int>(actions_.size())) {
            const auto& a = actions_[static_cast<size_t>(it.index)];
            emit commandRequested(q(d->icd->id), q(a.second.name), a.second.args);
        }
        break;
    case kCmdToggle:
        commands_open_ = !commands_open_;
        break;
    case kFilter:
        filter_ += 1;
        break;
    case kCmdRow:
        selectCommand(it.index);
        commands_open_ = false;
        break;
    case kMinus:
    case kPlus: {
        const CommandDef* c = command();
        if (!c || it.index >= static_cast<int>(c->params.size())) {
            break;
        }
        const ParamDef& p = c->params[static_cast<size_t>(it.index)];
        double& v = args_[static_cast<size_t>(it.index)];
        const double dir = it.kind == kPlus ? 1.0 : -1.0;
        if (!p.enum_name.empty() && d && d->icd->enums.count(p.enum_name)) {
            // step through the enum's values
            const auto& vals = d->icd->enums.at(p.enum_name);
            auto cur = std::find_if(vals.begin(), vals.end(), [&](const auto& kv) { return kv.first == std::llround(v); });
            long k = cur == vals.end() ? 0 : static_cast<long>(cur - vals.begin()) + static_cast<long>(dir);
            k = (k + static_cast<long>(vals.size())) % static_cast<long>(vals.size());
            v = static_cast<double>(vals[static_cast<size_t>(k)].first);
        } else {
            v += dir * step(p);
            if (p.min) {
                v = std::max(v, *p.min);
            }
            if (p.max) {
                v = std::min(v, *p.max);
            }
            if (std::fabs(v) < 1e-9) {
                v = 0.0;
            }
        }
        break;
    }
    case kValue: {
        const CommandDef* c = command();
        if (!c || it.index >= static_cast<int>(c->params.size())) {
            break;
        }
        const ParamDef& p = c->params[static_cast<size_t>(it.index)];
        if (p.enum_name.empty()) {
            editing_param_ = it.index;
            const int lo = p.min ? static_cast<int>(std::ceil(*p.min)) : -1000000;
            const int hi = p.max ? static_cast<int>(std::floor(*p.max)) : 1000000;
            emit numberRequested(q(c->name + " · " + p.name), static_cast<int>(std::llround(args_[static_cast<size_t>(it.index)])),
                                 lo, hi);
        }
        break;
    }
    case kSend:
        if (const CommandDef* c = command(); c && d) {
            emit commandRequested(q(d->icd->id), q(c->name), args_);
        }
        break;
    case kReport:
        open_reports_[it.index] = !open_reports_[it.index];
        break;
    default:
        break;
    }
    clampScroll();
    update();
}

void PayloadPanel::setEditedNumber(int value) {
    if (editing_param_ >= 0 && editing_param_ < static_cast<int>(args_.size())) {
        args_[static_cast<size_t>(editing_param_)] = value;
    }
    editing_param_ = -1;
    update();
}

void PayloadPanel::mousePressEvent(QMouseEvent* e) {
    pressing_ = true;
    dragged_ = false;
    press_pos_ = toLogical(e->position());
    press_scroll_ = scroll_;
}

void PayloadPanel::mouseMoveEvent(QMouseEvent* e) {
    if (!pressing_) {
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

void PayloadPanel::mouseReleaseEvent(QMouseEvent* e) {
    const bool tap = pressing_ && !dragged_;
    pressing_ = false;
    if (!tap) {
        return;
    }
    const QPointF pos = toLogical(e->position());
    const QRectF lr = logicalRect();
    if (QRectF(lr.right() - 62, 10, 50, 50).contains(pos)) {
        emit closeRequested();
        return;
    }
    if (pos.y() < kHeader) {
        return;
    }
    const QPointF content(pos.x(), pos.y() - kHeader + scroll_);
    std::vector<Item> items;
    layout(lr.width(), &items);
    for (const Item& it : items) {
        if (it.rect.contains(content)) {
            press(it);
            return;
        }
    }
}

void PayloadPanel::wheelEvent(QWheelEvent* e) {
    scroll_ -= e->angleDelta().y() / 2.0;
    clampScroll();
    update();
}

void PayloadPanel::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    p.scale(uiScale(), uiScale());
    const QRectF lr = logicalRect();
    const QRectF r = lr.adjusted(1, 1, -1, -1);
    drawGlass(p, chamferPath(r, 16), kCyan, 0.6, alpha(kGlassDeep, 0.94));
    p.setPen(QPen(kCyan, 2));
    p.drawLine(QPointF(r.left() + 1, r.top() + 16), QPointF(r.left() + 16, r.top() + 1));

    const auto* d = device();
    // ---- header ---------------------------------------------------------------------
    drawText(p, QRectF(r.left() + 22, r.top() + 12, r.width() - 100, 28), Qt::AlignLeft | Qt::AlignVCenter,
             QStringLiteral("PAYLOAD"), titleFont(18), kText);
    QString sub = QStringLiteral("no payload ICD loaded (icds/*.json)");
    QColor sub_c = kMuted;
    if (d && d->icd) {
        const QString name = q(d->icd->name).toUpper();
        if (d->alive) {
            sub = QStringLiteral("%1  ·  ● LIVE  ·  %2/%3").arg(name).arg(d->sysid).arg(d->compid);
            sub_c = kGreen;
        } else {
            sub = name + (d->reports.empty() ? QStringLiteral("  ·  ○ no reports yet") : QStringLiteral("  ·  ○ reports stopped"));
            sub_c = kAmber;
        }
    }
    drawText(p, QRectF(r.left() + 24, r.top() + 42, r.width() - 100, 18), Qt::AlignLeft | Qt::AlignVCenter, sub,
             labelFont(10, false), sub_c);
    const QRectF close(lr.right() - 62, 10, 50, 50);
    drawGlass(p, chamferPath(close.adjusted(6, 6, -6, -6), 6), alpha(kCyan, 0.6), 0.5, kGlassDeep);
    drawText(p, close, Qt::AlignCenter, QStringLiteral("✕"), labelFont(16), kText);

    // ---- content (scrolled) ---------------------------------------------------------
    p.save();
    p.setClipRect(QRectF(0, kHeader, lr.width(), lr.height() - kHeader - 4));
    p.translate(0, kHeader - scroll_);
    std::vector<Item> items;
    layout(lr.width(), &items);
    const double inner = lr.width() - 2 * kPad;
    auto flashOn = [&](const Item& it) { return flash_ > 0 && flash_kind_ == it.kind && flash_index_ == it.index; };
    auto tile = [&](const QRectF& rect, const QString& text, const QColor& c, bool on, bool flash) {
        drawGlass(p, chamferPath(rect, 6), on ? c : alpha(c, 0.45), on ? 0.9 : 0.5,
                  flash ? alpha(c, 0.25 * flash_ + 0.1) : (on ? alpha(c, 0.14) : kGlassDeep));
        drawText(p, rect.adjusted(6, 0, -6, 0), Qt::AlignCenter, text, labelFont(11), on ? kText : alpha(kText, 0.85));
    };
    auto label = [&](double y, const QString& text) {
        drawText(p, QRectF(kPad + 2, y, inner, kLabelH - 4), Qt::AlignLeft | Qt::AlignVCenter, text,
                 labelFont(9.5), kMuted);
    };
    double y = 0.0;
    if (snap_.devices.size() > 1) {
        label(y, QStringLiteral("DEVICE"));
        y += kLabelH;
    }
    // section labels are drawn just above the first item of each kind
    bool quick_label = false, cmd_label = false;
    if (d && view_) {
        double vy = 0.0;
        for (const Item& it : items) {
            if (it.kind == kDevice) {
                vy = std::max(vy, it.rect.bottom() + kGap + 6);
            }
        }
        label(vy, QStringLiteral("LINE OF SIGHT"));
        vy += kLabelH;
        const auto& v = *view_;
        const QString l1 = QStringLiteral("%1  ·  %2  ·  FOV %3°").arg(q(v.mode).toUpper(), q(v.sensor).toUpper())
                               .arg(v.hfov_deg, 0, 'f', 1);
        const QString l2 = QStringLiteral("AZ %1°  EL %2°  ·  RANGE %3")
                               .arg(v.earth.yaw_deg, 0, 'f', 0).arg(v.earth.pitch_deg, 0, 'f', 0)
                               .arg(v.ground ? QStringLiteral("%1 m").arg(v.slant_range, 0, 'f', 0) : QStringLiteral("--"));
        QString l3 = v.ground ? QStringLiteral("GND %1, %2").arg(v.ground->lat, 0, 'f', 6).arg(v.ground->lon, 0, 'f', 6)
                              : QStringLiteral("NO GROUND CROSSING");
        for (const auto& st : v.status) {
            l3 += QStringLiteral("  ·  ") + q(st);
        }
        drawText(p, QRectF(kPad + 4, vy, inner, kFieldH), Qt::AlignLeft | Qt::AlignVCenter, l1, monoFont(12, true), kCyan);
        drawText(p, QRectF(kPad + 4, vy + kFieldH, inner, kFieldH), Qt::AlignLeft | Qt::AlignVCenter, l2, monoFont(12), kText);
        drawText(p, QRectF(kPad + 4, vy + 2 * kFieldH, inner, kFieldH), Qt::AlignLeft | Qt::AlignVCenter, l3,
                 monoFont(11), v.status.empty() ? kMuted : kAmber);
    }
    const CommandDef* cur = command();
    for (const Item& it : items) {
        const QRectF& rc = it.rect;
        switch (it.kind) {
        case kDevice: {
            const auto& dev = snap_.devices[static_cast<size_t>(it.index)];
            tile(rc, q(dev.icd ? dev.icd->name : std::string("?")).toUpper(), dev.alive ? kGreen : kMuted,
                 dev.icd && dev.icd->id == selected_, flashOn(it));
            break;
        }
        case kQuick:
            if (!quick_label) {
                label(rc.top() - kLabelH, QStringLiteral("QUICK"));
                quick_label = true;
            }
            tile(rc, q(actions_[static_cast<size_t>(it.index)].first), kCyan, false, flashOn(it));
            break;
        case kCmdToggle:
            if (!cmd_label) {
                label(rc.top() - kLabelH, QStringLiteral("COMMAND"));
                cmd_label = true;
            }
            tile(rc,
                 (cur ? q(cur->name) : QStringLiteral("CHOOSE A COMMAND")) +
                     (commands_open_ ? QStringLiteral("  ▴") : QStringLiteral("  ▾")),
                 kCyan, commands_open_, flashOn(it));
            break;
        case kFilter: {
            const auto gs = groups(*d->icd);
            tile(rc, QStringLiteral("GROUP: %1  ▸").arg(q(gs[static_cast<size_t>(filter_) % gs.size()]).toUpper()),
                 kMuted, false, flashOn(it));
            break;
        }
        case kCmdRow: {
            const auto& c = d->icd->commands[static_cast<size_t>(it.index)];
            const bool sel = it.index == command_;
            drawGlass(p, chamferPath(rc, 5), sel ? kCyan : alpha(kCyan, 0.25), sel ? 0.8 : 0.35,
                      flashOn(it) ? alpha(kCyan, 0.25) : kGlassDeep);
            drawText(p, rc.adjusted(12, 0, -60, 0), Qt::AlignLeft | Qt::AlignVCenter, q(c.name), monoFont(12), kText);
            const QString sel_txt = c.id ? QStringLiteral("%1%2").arg(*c.id).arg(c.sub ? QStringLiteral(".%1").arg(*c.sub) : QString())
                                         : QStringLiteral("#%1").arg(c.command);
            drawText(p, rc.adjusted(0, 0, -12, 0), Qt::AlignRight | Qt::AlignVCenter, sel_txt, monoFont(11), kMuted);
            break;
        }
        case kMinus:
        case kPlus:
            tile(rc, it.kind == kMinus ? QStringLiteral("−") : QStringLiteral("+"), kCyan, false, flashOn(it));
            if (it.kind == kMinus && cur) {
                const auto& prm = cur->params[static_cast<size_t>(it.index)];
                drawText(p, QRectF(kPad + 2, rc.top(), rc.left() - kPad - 6, rc.height() / 2 + 6),
                         Qt::AlignLeft | Qt::AlignBottom, q(prm.name).toUpper(), labelFont(10), kText);
                const QString hint = prm.min && prm.max ? QStringLiteral("%1 … %2").arg(*prm.min).arg(*prm.max) : q(prm.desc);
                drawText(p, QRectF(kPad + 2, rc.center().y() + 4, rc.left() - kPad - 6, rc.height() / 2 - 4),
                         Qt::AlignLeft | Qt::AlignTop, hint, labelFont(8.5, false), kMuted);
            }
            break;
        case kValue:
            if (cur) {
                const auto& prm = cur->params[static_cast<size_t>(it.index)];
                drawGlass(p, chamferPath(rc, 5), alpha(kCyan, 0.4), 0.4, kGlassDeep);
                drawText(p, rc, Qt::AlignCenter, paramText(prm, args_[static_cast<size_t>(it.index)]), monoFont(13, true),
                         kCyan);
            }
            break;
        case kSend: {
            const QString ack = cur && cur->ack == payload::AckPolicy::None ? QStringLiteral("  (no ACK)") : QString();
            tile(rc, QStringLiteral("SEND ▶") + ack, kGreen, true, flashOn(it));
            break;
        }
        case kResults: {
            label(rc.top(), QStringLiteral("RESULTS"));
            double ly = rc.top() + kLabelH;
            for (const auto& l : log_) {
                drawText(p, QRectF(kPad + 4, ly, inner, kFieldH), Qt::AlignLeft | Qt::AlignVCenter,
                         (l.second ? QStringLiteral("✓ ") : QStringLiteral("✗ ")) + l.first, monoFont(11),
                         l.second ? kGreen : kRed);
                ly += kFieldH;
            }
            break;
        }
        case kReportsLabel:
            label(rc.top(), d->reports.empty() ? QStringLiteral("REPORTS") : QStringLiteral("REPORTS · tap to open"));
            if (d->reports.empty()) {
                drawText(p, QRectF(kPad + 4, rc.bottom(), inner, 30), Qt::AlignLeft | Qt::AlignVCenter,
                         QStringLiteral("nothing received from this payload yet"), labelFont(10, false), kMuted);
            }
            break;
        case kReport: {
            const auto& slot = d->reports.at(it.index);
            const bool open = open_reports_.count(it.index) && open_reports_.at(it.index);
            const bool fresh = std::chrono::duration<double>(payload::Clock::now() - slot.rx).count() < 2.0;
            drawGlass(p, chamferPath(rc, 5), alpha(fresh ? kCyan : kMuted, 0.35), 0.35, kGlassDeep);
            const QString name = slot.report.def ? q(slot.report.def->name) : QStringLiteral("?");
            drawText(p, rc.adjusted(12, 0, -12, 0), Qt::AlignLeft | Qt::AlignVCenter,
                     QStringLiteral("%1 %2  %3").arg(it.index, 2).arg(name.toUpper(), open ? QStringLiteral("▴") : QStringLiteral("▾")),
                     labelFont(11), fresh ? kText : kMuted);
            drawText(p, rc.adjusted(12, 0, -12, 0), Qt::AlignRight | Qt::AlignVCenter,
                     slot.hz > 0 ? QStringLiteral("%1 Hz").arg(slot.hz, 0, 'f', 1) : QStringLiteral("event"),
                     monoFont(11), fresh ? kCyan : kMuted);
            if (open && slot.report.def) {
                const auto& rep = slot.report;
                double fy = rc.bottom() + 2;
                const auto& icd = *d->icd;
                for (size_t k = 0; k < rep.def->header.fields.size(); ++k) {
                    const auto& f = rep.def->header.fields[k];
                    drawText(p, QRectF(kPad + 14, fy, inner * 0.45, kFieldH), Qt::AlignLeft | Qt::AlignVCenter,
                             q(f.name), monoFont(11), kMuted);
                    drawText(p, QRectF(kPad + inner * 0.45, fy, inner * 0.55, kFieldH), Qt::AlignRight | Qt::AlignVCenter,
                             q(payload::formatValue(icd, f, rep.fields[k])), monoFont(11), kText);
                    fy += kFieldH;
                }
                if (rep.def->has_records) {
                    drawText(p, QRectF(kPad + 14, fy, inner, kFieldH), Qt::AlignLeft | Qt::AlignVCenter,
                             QStringLiteral("%1: %2").arg(q(rep.def->records_name).toUpper()).arg(rep.records.size()),
                             labelFont(9.5), kMuted);
                    fy += kFieldH;
                    for (size_t n = 0; n < rep.records.size(); ++n) {
                        const auto& fs = rep.def->records.fields;
                        for (size_t k = 0; k < fs.size(); k += 2) {
                            QString line = QStringLiteral("%1 %2=%3").arg(k == 0 ? QStringLiteral("#%1").arg(n) : QStringLiteral("  "))
                                               .arg(q(fs[k].name), q(payload::formatValue(icd, fs[k], rep.records[n][k])));
                            if (k + 1 < fs.size()) {
                                line += QStringLiteral("   %1=%2").arg(q(fs[k + 1].name),
                                                                       q(payload::formatValue(icd, fs[k + 1], rep.records[n][k + 1])));
                            }
                            drawText(p, QRectF(kPad + 14, fy, inner - 14, kFieldH), Qt::AlignLeft | Qt::AlignVCenter, line,
                                     monoFont(10.5), kText);
                            fy += kFieldH;
                        }
                    }
                }
            }
            break;
        }
        default:
            break;
        }
    }
    p.restore();
}

}  // namespace gcs::ui
