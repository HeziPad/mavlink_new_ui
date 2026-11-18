#include "preflight_screen.hpp"

#include "anim.hpp"
#include "attitude_ball.hpp"
#include "core/geo.hpp"
#include "core/mission.hpp"
#include "core/plane_modes.hpp"
#include "preflight_art.hpp"
#include "theme.hpp"

#include <QFontMetricsF>
#include <QMouseEvent>
#include <QPainter>
#include <QWheelEvent>

#include <algorithm>
#include <cmath>

namespace gcs::ui {

using namespace theme;
using namespace gcs::preflight;
using Style = PreflightRunner::Style;

namespace {

QColor statusColor(Status s) {
    switch (s) {
    case Status::Pass: return kGreen;
    case Status::Warn:
    case Status::Skipped: return kAmber;
    case Status::Fail: return kRed;
    case Status::Waiting: return kCyan;
    case Status::NotApplicable:
    case Status::Pending: break;
    }
    return kMuted;
}

QFont body(double px, bool bold = false) {
    QFont f(QStringLiteral("DejaVu Sans"));
    f.setPixelSize(static_cast<int>(std::round(px)));
    f.setBold(bold);
    return f;
}

double wrapH(const QFont& f, double w, const QString& t) {
    return QFontMetricsF(f).boundingRect(QRectF(0, 0, w, 1e6), Qt::TextWordWrap, t).height();
}

void wrapText(QPainter& p, const QRectF& r, const QString& t, const QFont& f, const QColor& c) {
    p.setFont(f);
    p.setPen(c);
    p.drawText(r, Qt::TextWordWrap | Qt::AlignLeft | Qt::AlignTop, t);
}

art::Part partFor(const std::string& id) {
    if (id == "air.structure") return art::Part::Structure;
    if (id == "air.props") return art::Part::Props;
    if (id == "air.linkages") return art::Part::Linkages;
    if (id == "air.pitot") return art::Part::Pitot;
    if (id == "air.cg") return art::Part::CG;
    if (id == "air.battery") return art::Part::Battery;
    if (id == "air.antennas") return art::Part::Antennas;
    if (id == "mot.gate") return art::Part::Props;
    return art::Part::None;
}

}  // namespace

PreflightScreen::PreflightScreen(QWidget* parent, PreflightRunner* runner) : QWidget(parent), runner_(runner) {
    hide();
}

void PreflightScreen::open(bool turnaround) {
    runner_->open(runner_->vehicle(), turnaround);
    if (parentWidget()) {
        setGeometry(parentWidget()->rect());
    }
    open_target_ = 1.0;
    if (reducedMotion()) {
        open_ = 1.0;
    }
    ready_shown_ = runner_->session().passed();
    ready_dismissed_ = ready_shown_;
    if (step_.empty() || !runner_->session().includes(step_)) {
        // first step that isn't done
        for (const auto* d : runner_->session().activeSteps()) {
            if (!isDone(runner_->status(d->id))) {
                selectStep(d->id);
                break;
            }
        }
        if (step_.empty() && !runner_->session().activeSteps().empty()) {
            selectStep(runner_->session().activeSteps().front()->id);
        }
    }
    show();
    raise();
    setFocus();
}

void PreflightScreen::close() {
    if (open_target_ == 0.0) {
        return;
    }
    runner_->close();
    open_target_ = 0.0;
    if (reducedMotion()) {
        open_ = 0.0;
        hide();
    }
    emit closed();
}

void PreflightScreen::tick(double dt) {
    if (!isVisible()) {
        return;
    }
    time_ += dt;
    open_ += std::clamp(open_target_ - open_, -dt * 5.0, dt * 5.0);
    if (open_ <= 0.0 && open_target_ == 0.0) {
        hide();
        return;
    }
    if (pressed_ >= 0 && pressed_ < static_cast<int>(hits_.size()) && hits_[pressed_].hold) {
        hold_t_ += dt;
        if (hold_t_ >= hits_[pressed_].hold_secs) {
            const Hit h = hits_[pressed_];
            pressed_ = -1;
            hold_t_ = 0.0;
            fire(h);
        }
    }
    toast_t_ = std::max(0.0, toast_t_ - dt);
    if (runner_->session().passed() && !ready_shown_) {
        ready_shown_ = true;
        ready_dismissed_ = false;
    }
    update();
}

std::vector<const StepDef*> PreflightScreen::stageSteps(int stage) const {
    std::vector<const StepDef*> v;
    for (const auto* d : runner_->session().activeSteps()) {
        if (static_cast<int>(d->stage) == stage) {
            v.push_back(d);
        }
    }
    return v;
}

void PreflightScreen::selectStage(int stage) {
    const auto list = stageSteps(stage);
    if (list.empty()) {
        return;
    }
    stage_ = stage;
    list_scroll_ = 0.0;
    for (const auto* d : list) {
        if (!isDone(runner_->status(d->id))) {
            step_ = d->id;
            return;
        }
    }
    step_ = list.front()->id;
}

void PreflightScreen::selectStep(const std::string& id) {
    const StepDef* d = findStep(id);
    if (!d) {
        return;
    }
    step_ = id;
    stage_ = static_cast<int>(d->stage);
}

void PreflightScreen::stepBy(int delta) {
    const auto all = runner_->session().activeSteps();
    for (size_t i = 0; i < all.size(); ++i) {
        if (all[i]->id == step_) {
            const int j = std::clamp(static_cast<int>(i) + delta, 0, static_cast<int>(all.size()) - 1);
            selectStep(all[j]->id);
            return;
        }
    }
}

// ---- input ------------------------------------------------------------------------------------

void PreflightScreen::mousePressEvent(QMouseEvent* e) {
    const QPointF pos = e->position();
    press_pos_ = pos;
    press_scroll_ = list_scroll_;
    dragging_ = false;
    pressed_ = -1;
    hold_t_ = 0.0;
    for (int i = static_cast<int>(hits_.size()) - 1; i >= 0; --i) {
        if (hits_[i].rect.contains(pos)) {
            if (!hits_[i].enabled) {
                toast_ = hits_[i].reason.isEmpty() ? QStringLiteral("Not available now") : hits_[i].reason;
                toast_t_ = 2.5;
                return;
            }
            pressed_ = i;
            return;
        }
    }
}

void PreflightScreen::mouseMoveEvent(QMouseEvent* e) {
    if (!(e->buttons() & Qt::LeftButton)) {
        return;
    }
    const double dy = e->position().y() - press_pos_.y();
    if (list_rect_.contains(press_pos_) && std::abs(dy) > 8) {
        dragging_ = true;
        pressed_ = -1;
        list_scroll_ = std::clamp(press_scroll_ - dy, 0.0, std::max(0.0, list_content_h_ - list_rect_.height()));
    }
}

void PreflightScreen::mouseReleaseEvent(QMouseEvent* e) {
    if (pressed_ >= 0 && pressed_ < static_cast<int>(hits_.size()) && !dragging_) {
        const Hit h = hits_[pressed_];
        if (h.rect.contains(e->position())) {
            if (h.hold) {
                if (hold_t_ < h.hold_secs) {
                    toast_ = QStringLiteral("Hold it for %1 s").arg(h.hold_secs, 0, 'f', 0);
                    toast_t_ = 2.0;
                }
            } else {
                fire(h);
            }
        }
    }
    pressed_ = -1;
    hold_t_ = 0.0;
    dragging_ = false;
}

void PreflightScreen::wheelEvent(QWheelEvent* e) {
    if (list_rect_.contains(e->position())) {
        list_scroll_ = std::clamp(list_scroll_ - e->angleDelta().y() / 2.0, 0.0,
                                  std::max(0.0, list_content_h_ - list_rect_.height()));
    }
}

void PreflightScreen::fire(const Hit& h) {
    if (h.kind == "close") {
        close();
    } else if (h.kind == "stage") {
        selectStage(std::stoi(h.value));
    } else if (h.kind == "step") {
        selectStep(h.value);
    } else if (h.kind == "btn") {
        runner_->act(step_, h.value);
        // a confirmed step moves on by itself
        if (h.value == "yes" || h.value == "save" || h.value.rfind("props:", 0) == 0 || h.value == "already" ||
            h.value == "na" || h.value == "skip") {
            if (isDone(runner_->status(step_))) {
                stepBy(1);
            }
        }
    } else if (h.kind == "prev") {
        stepBy(-1);
    } else if (h.kind == "next") {
        stepBy(1);
    } else if (h.kind == "stop") {
        runner_->stopAll();
    } else if (h.kind == "override") {
        emit launchOverride();
        close();
    } else if (h.kind == "full" || h.kind == "turn") {
        runner_->session().begin(runner_->vehicle(), Clock::now(), h.kind == "turn");
        step_.clear();
        open(h.kind == "turn");
    } else if (h.kind == "ready") {
        ready_dismissed_ = true;
        close();
    }
}

// ---- painting --------------------------------------------------------------------------------------

void PreflightScreen::paintButton(QPainter& p, const QRectF& r, const QString& label, Style style, bool enabled,
                                  bool hold, double charge) {
    QColor c = kCyan;
    switch (style) {
    case Style::Good: c = kGreen; break;
    case Style::Bad: c = kRed; break;
    case Style::Danger: c = kRed; break;
    case Style::Neutral: c = kMuted; break;
    case Style::Primary: break;
    }
    if (!enabled) {
        c = alpha(kMuted, 0.5);
    }
    drawGlass(p, chamferPath(r, 7), c, enabled ? 0.9 : 0.4, alpha(c, style == Style::Danger ? 0.16 : 0.08));
    if (charge > 0.0) {
        p.setPen(Qt::NoPen);
        p.setBrush(alpha(c, 0.35));
        p.drawRect(QRectF(r.left(), r.top(), r.width() * std::clamp(charge, 0.0, 1.0), r.height()));
    }
    drawText(p, r.adjusted(8, 0, -8, hold ? -10 : 0), Qt::AlignCenter, label, labelFont(12), enabled ? c : alpha(kMuted, 0.7));
    if (hold) {
        drawText(p, QRectF(r.left(), r.bottom() - 16, r.width(), 12), Qt::AlignCenter, QStringLiteral("HOLD"),
                 labelFont(8), alpha(c, 0.8));
    }
}

void PreflightScreen::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    hits_.clear();
    const double e = easeOutCubic(open_);
    const QRectF full = rect();
    p.fillRect(full, alpha(kBg, 0.99 * e));  // opaque: banners behind must not show through
    if (!reducedMotion()) {
        drawScanlines(p, full, 0.4 * e);
    }
    p.setOpacity(e);
    p.translate(0, (1.0 - e) * 30.0);

    const QRectF r = full.adjusted(16, 12, -16, -12);
    // ---- top bar
    const auto& ses = runner_->session();
    drawText(p, QRectF(r.left() + 6, r.top() + 2, 300, 34), Qt::AlignLeft | Qt::AlignVCenter, QStringLiteral("PREFLIGHT"),
             titleFont(26), kText);
    const int done = ses.doneCount(), total = ses.totalCount();
    {
        const QRectF bar(r.left() + 280, r.top() + 14, std::min(320.0, r.width() * 0.25), 10);
        p.setPen(Qt::NoPen);
        p.setBrush(alpha(kMuted, 0.25));
        p.drawRect(bar);
        p.setBrush(ses.failCount() ? kAmber : kGreen);
        p.drawRect(QRectF(bar.left(), bar.top(), bar.width() * (total ? double(done) / total : 0.0), bar.height()));
        drawText(p, QRectF(bar.right() + 12, r.top() + 2, 260, 34), Qt::AlignLeft | Qt::AlignVCenter,
                 QStringLiteral("%1 / %2 · %3%4")
                     .arg(done)
                     .arg(total)
                     .arg(ses.turnaround() ? QStringLiteral("TURNAROUND") : QStringLiteral("FULL"))
                     .arg(ses.failCount() ? QStringLiteral(" · %1 FAILED").arg(ses.failCount()) : QString()),
                 labelFont(12), ses.failCount() ? kAmber : kText);
    }
    {
        const QRectF cl(r.right() - 52, r.top(), 52, 48);
        drawGlass(p, chamferPath(cl, 8), kCyan, 0.6, kGlassDeep);
        drawText(p, cl, Qt::AlignCenter, QStringLiteral("✕"), monoFont(20, true), kText);
        hits_.push_back({cl, "close", "", false, 1.0, true, {}});
        const QRectF turn(cl.left() - 150, r.top(), 140, 48), fullb(turn.left() - 110, r.top(), 100, 48);
        paintButton(p, fullb, QStringLiteral("FULL"), ses.turnaround() ? Style::Neutral : Style::Good, true, false, 0);
        paintButton(p, turn, QStringLiteral("TURNAROUND"), ses.turnaround() ? Style::Good : Style::Neutral, true, false, 0);
        hits_.push_back({fullb, "full", "", false, 1.0, !ses.turnaround() ? false : true, QStringLiteral("Already the full preflight")});
        hits_.push_back({turn, "turn", "", false, 1.0, ses.turnaround() ? false : true, QStringLiteral("Already the turnaround preflight")});
    }

    const double top = r.top() + 60, bottom = r.bottom() - 64;
    const bool wide = r.width() >= 1250;
    const double rail_w = wide ? 210 : 170, list_w = wide ? 270 : 220, data_w = wide ? 230 : 0;
    const QRectF rail(r.left(), top, rail_w, bottom - top);
    const QRectF list(rail.right() + 12, top, list_w, bottom - top);
    const QRectF data_col(r.right() - data_w, top, data_w, bottom - top);
    const QRectF card(list.right() + 12, top, (wide ? data_col.left() - 12 : r.right()) - list.right() - 12, bottom - top);
    paintRail(p, rail);
    paintSteps(p, list);
    paintCard(p, card);
    if (wide) {
        paintData(p, data_col);
    }

    // ---- bottom bar
    {
        const QRectF prev(r.left(), r.bottom() - 52, 140, 52), next(prev.right() + 10, prev.top(), 140, 52);
        paintButton(p, prev, QStringLiteral("◀ BACK"), Style::Neutral, true, false, 0);
        paintButton(p, next, QStringLiteral("NEXT ▶"), Style::Primary, true, false, 0);
        hits_.push_back({prev, "prev", "", false, 1.0, true, {}});
        hits_.push_back({next, "next", "", false, 1.0, true, {}});
        if (runner_->testRunning()) {
            const QRectF stop(r.center().x() - 110, r.bottom() - 52, 220, 52);
            paintButton(p, stop, QStringLiteral("■ STOP ALL"), Style::Danger, true, false, 0);
            hits_.push_back({stop, "stop", "", false, 1.0, true, {}});
        }
        if (!ses.passed()) {
            // the logged way around a preflight that can't be finished here
            const QRectF ov(r.right() - 330, r.bottom() - 52, 330, 52);
            paintButton(p, ov, QStringLiteral("LAUNCH WITHOUT PREFLIGHT · 3 s"), Style::Neutral, true, true, 0);
            hits_.push_back({ov, "override", "", true, 3.0, true, {}});
        }
    }
    // charge overlay for the button being held
    if (pressed_ >= 0 && pressed_ < static_cast<int>(hits_.size()) && hits_[pressed_].hold) {
        const Hit& h = hits_[pressed_];
        p.setPen(Qt::NoPen);
        p.setBrush(alpha(kAmber, 0.35));
        p.drawRect(QRectF(h.rect.left(), h.rect.top(), h.rect.width() * std::clamp(hold_t_ / h.hold_secs, 0.0, 1.0),
                          h.rect.height()));
    }

    if (ses.passed() && !ready_dismissed_) {
        paintReady(p, r);
    }
    if (toast_t_ > 0.0) {
        const QFont f = labelFont(12);
        const double w = QFontMetricsF(f).horizontalAdvance(toast_) + 40;
        const QRectF box(r.center().x() - w / 2, r.bottom() - 120, w, 40);
        p.setOpacity(e * std::min(1.0, toast_t_ / 0.4));
        drawGlass(p, chamferPath(box, 8), kAmber, 0.9, kGlassDeep);
        drawText(p, box, Qt::AlignCenter, toast_, f, kAmber);
    }
}

void PreflightScreen::paintRail(QPainter& p, const QRectF& r) {
    drawGlass(p, chamferPath(r, 12), kCyan, 0.35, alpha(kGlassDeep, 0.9));
    const int n = static_cast<int>(Stage::Count);
    const double row = std::min(54.0, (r.height() - 16) / n);
    for (int i = 0; i < n; ++i) {
        const auto list = stageSteps(i);
        if (list.empty()) continue;
        int done = 0, fail = 0, wait = 0;
        for (const auto* d : list) {
            const Status st = runner_->status(d->id);
            done += isDone(st);
            fail += st == Status::Fail;
            wait += st == Status::Waiting;
        }
        const QRectF rr(r.left() + 8, r.top() + 8 + i * row, r.width() - 16, row - 4);
        const bool sel = i == stage_;
        if (sel) {
            drawGlass(p, chamferPath(rr, 6), kCyan, 0.9, alpha(kCyan, 0.12));
        }
        const Status agg = fail ? Status::Fail : (done == static_cast<int>(list.size()) ? Status::Pass : (wait ? Status::Waiting : Status::Pending));
        art::statusGlyph(p, QPointF(rr.left() + 18, rr.center().y()), 11, agg, time_);
        drawText(p, QRectF(rr.left() + 38, rr.top(), rr.width() - 80, rr.height()), Qt::AlignLeft | Qt::AlignVCenter,
                 QString::fromLatin1(stageName(static_cast<Stage>(i))).toUpper(), labelFont(11.5), sel ? kCyan : kText);
        drawText(p, QRectF(rr.right() - 48, rr.top(), 44, rr.height()), Qt::AlignRight | Qt::AlignVCenter,
                 QStringLiteral("%1/%2").arg(done).arg(list.size()), monoFont(11), alpha(kMuted, 0.95));
        hits_.push_back({rr, "stage", std::to_string(i), false, 1.0, true, {}});
    }
}

void PreflightScreen::paintSteps(QPainter& p, const QRectF& r) {
    drawGlass(p, chamferPath(r, 12), kCyan, 0.35, alpha(kGlassDeep, 0.9));
    drawText(p, QRectF(r.left() + 14, r.top() + 8, r.width() - 28, 18), Qt::AlignLeft | Qt::AlignVCenter,
             QString::fromLatin1(stageName(static_cast<Stage>(stage_))).toUpper(), labelFont(11), kMuted);
    list_rect_ = r.adjusted(6, 32, -6, -6);
    const auto list = stageSteps(stage_);
    const double row = 52;
    list_content_h_ = list.size() * row;
    p.save();
    p.setClipRect(list_rect_);
    for (size_t i = 0; i < list.size(); ++i) {
        const QRectF rr(list_rect_.left(), list_rect_.top() + i * row - list_scroll_, list_rect_.width(), row - 4);
        if (rr.bottom() < list_rect_.top() || rr.top() > list_rect_.bottom()) continue;
        const bool sel = list[i]->id == step_;
        if (sel) drawGlass(p, chamferPath(rr, 6), kCyan, 0.9, alpha(kCyan, 0.12));
        const Status st = runner_->status(list[i]->id);
        art::statusGlyph(p, QPointF(rr.left() + 18, rr.center().y()), 10, st, time_);
        drawText(p, QRectF(rr.left() + 36, rr.top() + 5, rr.width() - 40, 20), Qt::AlignLeft | Qt::AlignVCenter,
                 QString::fromStdString(list[i]->title), body(13, sel), sel ? kText : alpha(kText, 0.85));
        drawText(p, QRectF(rr.left() + 36, rr.top() + 26, rr.width() - 40, 16), Qt::AlignLeft | Qt::AlignVCenter,
                 QString::fromLatin1(kindName(list[i]->kind)) + (list[i]->critical ? QStringLiteral(" · REQUIRED") : QString()),
                 labelFont(8.5), alpha(statusColor(st), 0.9));
        const QRectF vis = rr.intersected(list_rect_);
        if (!vis.isEmpty()) hits_.push_back({vis, "step", list[i]->id, false, 1.0, true, {}});
    }
    p.restore();
}

void PreflightScreen::paintCard(QPainter& p, const QRectF& r) {
    drawGlass(p, chamferPath(r, 14), kCyan, 0.5, alpha(kGlassDeep, 0.92));
    const StepDef* d = findStep(step_);
    if (!d) return;
    const Status st = runner_->status(step_);
    const QColor sc = statusColor(st);
    const double x = r.left() + 22, w = r.width() - 44;
    double y = r.top() + 16;
    // header
    const QString kind = QString::fromLatin1(kindName(d->kind)) + (d->critical ? QStringLiteral(" · REQUIRED") : QString());
    drawText(p, QRectF(x, y, w, 16), Qt::AlignLeft | Qt::AlignVCenter, kind, labelFont(10), kCyan);
    art::statusGlyph(p, QPointF(r.right() - 40, r.top() + 36), 20, st, time_);
    y += 20;
    const QString title = QString::fromStdString(d->title);
    drawText(p, QRectF(x, y, w - 60, 32), Qt::AlignLeft | Qt::AlignVCenter, title, body(24, true), kText);
    y += 36;
    const QString why = QString::fromStdString(d->why);
    const double wh = wrapH(body(13), w - 60, why);
    wrapText(p, QRectF(x, y, w - 60, wh + 2), why, body(13), alpha(kText, 0.7));
    y += wh + 10;

    // buttons (bottom), measured first so the picture gets the rest
    const auto btns = runner_->buttons(step_);
    std::vector<QRectF> brects;
    {
        double bx = x, by = 0;
        const double bh = 54, gap = 10;
        std::vector<double> widths;
        for (const auto& b : btns) {
            widths.push_back(std::max(110.0, QFontMetricsF(labelFont(12)).horizontalAdvance(b.label) + 40));
        }
        // lay out in rows from the bottom
        std::vector<std::vector<int>> rows(1);
        double rowx = 0;
        for (size_t i = 0; i < btns.size(); ++i) {
            if (rowx + widths[i] > w && !rows.back().empty()) {
                rows.emplace_back();
                rowx = 0;
            }
            rows.back().push_back(static_cast<int>(i));
            rowx += widths[i] + gap;
        }
        brects.resize(btns.size());
        by = r.bottom() - 16 - rows.size() * (bh + gap) + gap;
        for (const auto& row : rows) {
            bx = x;
            for (int i : row) {
                brects[i] = QRectF(bx, by, widths[i], bh);
                bx += widths[i] + gap;
            }
            by += bh + gap;
        }
    }
    const double buttons_top = btns.empty() ? r.bottom() - 16 : brects.front().top();

    // instruction + live detail above the buttons
    const QString how = runner_->instruction(step_);
    const QString det = runner_->detail(step_);
    const double hh = wrapH(body(15, true), w, how);
    const double dh = det.isEmpty() ? 0 : wrapH(body(13), w, det);
    const double text_top = buttons_top - 12 - hh - (dh > 0 ? dh + 8 : 0);
    // the picture
    const QRectF artr(x, y, w, std::max(60.0, text_top - y - 10));
    paintArt(p, artr);
    wrapText(p, QRectF(x, text_top, w, hh + 2), how, body(15, true), kCyan);
    if (dh > 0) {
        wrapText(p, QRectF(x, text_top + hh + 8, w, dh + 2), det, body(13), sc);
    }

    for (size_t i = 0; i < btns.size(); ++i) {
        paintButton(p, brects[i], btns[i].label, btns[i].style, btns[i].enabled, btns[i].hold, 0);
        hits_.push_back({brects[i], "btn", btns[i].action, btns[i].hold, 1.0, btns[i].enabled, btns[i].reason});
    }
}

void PreflightScreen::paintArt(QPainter& p, const QRectF& r) {
    const std::string& id = step_;
    const auto& s = runner_->vehicle();
    const auto& a = runner_->art();
    p.save();
    p.setClipRect(r);
    const QRectF sq(r.center().x() - std::min(r.width(), r.height()) / 2, r.top(), std::min(r.width(), r.height()),
                    std::min(r.width(), r.height()));
    auto bigGlyph = [&](const QString& g, const QColor& c) {
        drawText(p, r, Qt::AlignCenter, g, body(std::min(r.height() * 0.5, 110.0), true), alpha(c, 0.85));
    };

    if (id == "profile.vehicle" || id == "mot.gate" || id.rfind("air.", 0) == 0) {
        art::TopView v;
        v.frame = runner_->frame();
        v.show_dirs = id == "profile.vehicle" || id == "air.props" || id == "mot.gate";
        v.show_letters = id == "profile.vehicle";
        v.highlight = partFor(id);
        art::quadplaneTop(p, sq.adjusted(30, 20, -30, -10), v, time_);
    } else if (id == "mot.lift" || id == "mot.pusher") {
        art::TopView v;
        v.frame = runner_->frame();
        v.show_letters = true;
        v.lit_motor = a.lit_motor;
        v.pusher_lit = a.pusher_lit;
        v.highlight = id == "mot.pusher" ? art::Part::Pusher : art::Part::Motors;
        art::quadplaneTop(p, sq.adjusted(30, 20, -30, -10), v, time_);
    } else if (id == "profile.battery" || id == "power.voltage" || id == "power.balance") {
        const auto& pr = runner_->profile();
        const QRectF br(r.left(), r.top(), r.width(), std::min(r.height(), 200.0));
        art::battery(p, br.adjusted(r.width() * 0.15, 0, -r.width() * 0.15, 0), s.cell_v, s.battery_v,
                     pr.cells, fullCellVolts(pr.chemistry), storageCellVolts(pr.chemistry));
        if (id == "profile.battery") {
            const char* chem = pr.chemistry == Chemistry::LiPo ? "LiPo" : pr.chemistry == Chemistry::LiHV ? "LiHV" : "Li-ion";
            drawText(p, QRectF(r.left(), br.bottom() + 6, r.width(), 30), Qt::AlignCenter,
                     QStringLiteral("%1 · %2 cells · %3 mAh · flight time %4 min")
                         .arg(QString::fromLatin1(chem))
                         .arg(pr.cells)
                         .arg(pr.capacity_mah)
                         .arg(pr.endurance_min, 0, 'f', 0),
                     monoFont(16, true), kText);
        }
    } else if (id == "power.current") {
        art::gauge(p, QRectF(r.left() + 40, r.center().y() - 20, r.width() - 80, 30), std::max(0.0, s.battery_a), 0, 20, 0.05, 8,
                   QStringLiteral("CURRENT AT REST  %1 A").arg(s.battery_a, 0, 'f', 1));
    } else if (id == "power.board") {
        art::gauge(p, QRectF(r.left() + 40, r.center().y() - 20, r.width() - 80, 30), s.board_vcc, 4.0, 6.0, 4.8, 5.4,
                   QStringLiteral("BOARD SUPPLY  %1 V").arg(s.board_vcc, 0, 'f', 2));
    } else if (id == "link.telemetry") {
        art::gauge(p, QRectF(r.left() + 40, r.center().y() - 50, r.width() - 80, 30), s.attitude_hz, 0, 30, 8, 30,
                   QStringLiteral("ATTITUDE RATE  %1 Hz").arg(s.attitude_hz, 0, 'f', 0));
        if (s.has_radio) {
            const int margin = std::min(s.radio_rssi - s.radio_noise, s.radio_remrssi - s.radio_remnoise);
            art::gauge(p, QRectF(r.left() + 40, r.center().y() + 10, r.width() - 80, 30), margin, 0, 120, 20, 120,
                       QStringLiteral("RADIO MARGIN  %1").arg(margin));
        }
    } else if (id == "link.sticks") {
        const auto norm = [&](const char* m, int def) {
            const auto it = s.params.find(m);
            const int ch = it != s.params.end() ? static_cast<int>(it->second) : def;
            return ch >= 1 && ch <= 18 && s.rc[ch - 1] ? std::clamp((s.rc[ch - 1] - 1500.0) / 400.0, -1.0, 1.0) : 0.0;
        };
        art::sticks(p, r.adjusted(10, 10, -10, -10), norm("RCMAP_ROLL", 1), norm("RCMAP_PITCH", 2) * -1.0,
                    (norm("RCMAP_THROTTLE", 3) + 1.0) / 2.0, norm("RCMAP_YAW", 4), a.stick_reached);
    } else if (id == "link.modes") {
        double yy = r.top() + 10;
        for (size_t i = 0; i < a.mode_list.size(); ++i) {
            const bool live = static_cast<int>(i) == a.mode_live;
            const QRectF row(r.left() + r.width() * 0.2, yy, r.width() * 0.6, 34);
            if (live) drawGlass(p, chamferPath(row, 6), kGreen, 0.9, alpha(kGreen, 0.12));
            drawText(p, row.adjusted(14, 0, -10, 0), Qt::AlignLeft | Qt::AlignVCenter, a.mode_list[i], monoFont(15, live),
                     live ? kGreen : kText);
            yy += 38;
        }
        drawText(p, QRectF(r.left(), yy + 4, r.width(), 22), Qt::AlignCenter,
                 QStringLiteral("vehicle mode now: %1").arg(QString::fromStdString(s.mode_name)), labelFont(11), kMuted);
    } else if (id == "link.rcfailsafe") {
        const bool rc_bad = (s.sensors_present & 0x10000) && !(s.sensors_health & 0x10000);
        bigGlyph(rc_bad ? QStringLiteral("RC LOST") : QStringLiteral("RC OK"), rc_bad ? kAmber : kGreen);
    } else if (id == "sens.gyro" || id == "sens.level" || id == "sens.accel") {
        art::pose(p, sq.adjusted(20, 10, -20, -10), a.pose >= 1 && a.pose <= 6 ? a.pose : 1, time_);
        paintAttitudeBall(p, QPointF(r.right() - 70, r.top() + 70), 50, s.roll_deg, s.pitch_deg, s.yaw_deg);
    } else if (id == "sens.compass" || id == "sens.heading") {
        // magcal sphere while rotating, else a compass rose with the aircraft heading
        bool rotating = false;
        for (const auto& c : s.magcal) rotating |= c.active;
        if (rotating) {
            std::array<uint8_t, 10> mask{};
            int pct = 0;
            for (const auto& c : s.magcal) {
                if (!c.active) continue;
                for (int k = 0; k < 10; ++k) mask[k] |= c.mask[k];
                pct = std::max(pct, c.pct);
            }
            art::compassSphere(p, sq.adjusted(20, 10, -20, -10), mask, pct, time_);
        } else {
            const QPointF c = sq.center();
            const double rad = sq.width() * 0.36;
            p.setPen(QPen(alpha(kCyan, 0.6), 1.5));
            p.setBrush(alpha(kBg, 0.6));
            p.drawEllipse(c, rad, rad);
            for (int k = 0; k < 36; ++k) {
                p.save();
                p.translate(c);
                p.rotate(k * 10.0);
                p.setPen(QPen(alpha(kCyan, k % 9 == 0 ? 0.95 : 0.4), k % 9 == 0 ? 2 : 1));
                p.drawLine(QPointF(0, -rad), QPointF(0, -rad + (k % 9 == 0 ? 12 : 6)));
                p.restore();
            }
            const char* card[] = {"N", "E", "S", "W"};
            for (int k = 0; k < 4; ++k) {
                const double ang = k * geo::kPi / 2;
                drawText(p, QRectF(c.x() + std::sin(ang) * (rad - 26) - 12, c.y() - std::cos(ang) * (rad - 26) - 10, 24, 20),
                         Qt::AlignCenter, QString::fromLatin1(card[k]), labelFont(12), kCyan);
            }
            p.save();
            p.translate(c);
            p.rotate(s.yaw_deg);
            p.setPen(QPen(kAmber, 3));
            p.drawLine(QPointF(0, 10), QPointF(0, -rad + 20));
            p.setBrush(kAmber);
            p.setPen(Qt::NoPen);
            p.drawPolygon(QPolygonF({QPointF(0, -rad + 12), QPointF(-8, -rad + 28), QPointF(8, -rad + 28)}));
            p.restore();
            drawText(p, QRectF(c.x() - 80, c.y() + rad + 6, 160, 24), Qt::AlignCenter,
                     QStringLiteral("NOSE %1°").arg(static_cast<int>(std::lround(geo::wrap360(s.yaw_deg))) % 360, 3, 10, QLatin1Char('0')),
                     monoFont(16, true), kAmber);
        }
    } else if (id == "sens.baro" || id == "sens.pitot") {
        art::pitot(p, sq.adjusted(10, 10, -10, -10), s.airspeed, time_);
    } else if (id == "sens.orientation") {
        paintAttitudeBall(p, QPointF(r.center().x(), r.top() + std::min(r.height() * 0.4, 110.0)),
                          std::min(r.height() * 0.32, 90.0), s.roll_deg, s.pitch_deg, s.yaw_deg);
        const double yy = r.top() + std::min(r.height() * 0.4, 110.0) + std::min(r.height() * 0.32, 90.0) + 34;
        const QString row = QStringLiteral("%1 TILT RIGHT     %2 NOSE UP     %3 TURN 45°")
                                .arg(a.orient_roll ? QStringLiteral("✓") : QStringLiteral("·"))
                                .arg(a.orient_pitch ? QStringLiteral("✓") : QStringLiteral("·"))
                                .arg(a.orient_yaw ? QStringLiteral("✓") : QStringLiteral("·"));
        drawText(p, QRectF(r.left(), yy, r.width(), 26), Qt::AlignCenter, row, labelFont(13), kGreen);
    } else if (id.rfind("surf.", 0) == 0 && id != "surf.trim") {
        art::surface(p, r.adjusted(r.width() * 0.1, 0, -r.width() * 0.1, 0),
                     a.axis == 0 ? art::Axis::Roll : a.axis == 1 ? art::Axis::Pitch : art::Axis::Yaw, a.surf_cmd, a.surf_out, time_);
    } else if (id == "pay.gimbal") {
        art::gimbal(p, sq, a.gimbal_pitch, a.gimbal_yaw);
    } else if (id == "site.area" || id == "site.people") {
        art::site(p, sq, time_);
    } else if (id.rfind("mis.", 0) == 0) {
        // route preview: waypoints, L and H, fitted into the box
        if (s.mission_known && s.has_home) {
            const MissionPlan plan = extractPlan(s.mission);
            std::vector<geo::NE> pts;
            for (const auto& wp : plan.wps) pts.push_back(geo::delta(s.home_lat, s.home_lon, wp.lat, wp.lon));
            std::optional<geo::NE> land;
            if (plan.land) land = geo::delta(s.home_lat, s.home_lon, plan.land->lat, plan.land->lon);
            double ext = 50;
            for (const auto& q : pts) ext = std::max({ext, std::abs(q.north), std::abs(q.east)});
            if (land) ext = std::max({ext, std::abs(land->north), std::abs(land->east)});
            const double sc = std::min(r.width(), r.height()) * 0.42 / ext;
            const QPointF c = r.center();
            auto at = [&](const geo::NE& q) { return QPointF(c.x() + q.east * sc, c.y() - q.north * sc); };
            QPolygonF poly;
            for (const auto& q : pts) poly << at(q);
            if (!poly.isEmpty()) poly << poly.front();
            p.setPen(QPen(kCyan, 2, Qt::DashLine));
            p.setBrush(Qt::NoBrush);
            p.drawPolyline(poly);
            for (size_t k = 0; k < pts.size(); ++k) {
                p.setPen(QPen(kCyan, 2));
                p.setBrush(alpha(kBg, 0.9));
                p.drawEllipse(at(pts[k]), 11, 11);
                drawText(p, QRectF(at(pts[k]).x() - 11, at(pts[k]).y() - 11, 22, 22), Qt::AlignCenter,
                         QString::number(k + 1), monoFont(11, true), kCyan);
            }
            p.setPen(QPen(kGreen, 2));
            p.drawEllipse(c, 12, 12);
            drawText(p, QRectF(c.x() - 12, c.y() - 12, 24, 24), Qt::AlignCenter, QStringLiteral("H"), monoFont(12, true), kGreen);
            if (land) {
                const QPointF l = at(*land);
                p.setPen(QPen(kGreen, 2));
                p.drawRect(QRectF(l.x() - 11, l.y() - 11, 22, 22));
                drawText(p, QRectF(l.x() - 11, l.y() - 11, 22, 22), Qt::AlignCenter, QStringLiteral("L"), monoFont(12, true), kGreen);
            }
        } else {
            bigGlyph(QStringLiteral("…"), kMuted);
        }
    } else if (id == "fin.ready" || id == "fin.safety") {
        struct Row {
            QString t;
            bool ok;
        };
        const Row rows[] = {
            {QStringLiteral("Disarmed"), !s.armed},
            {QStringLiteral("AUTO selected"), s.custom_mode == plane::AUTO},
            {QStringLiteral("Mission at its takeoff"), s.mission_known && s.mission_current == 1},
            {QStringLiteral("Pre-arm checks pass"), prearmHealthy(s)},
            {QStringLiteral("GPS and EKF good"), gpsHealthy(s) && ekfHealthy(s)},
            {QStringLiteral("Safety OFF (motors live)"), safetyState(s) == SafetyState::Live},
        };
        double yy = r.top() + 6;
        for (const auto& row : rows) {
            art::statusGlyph(p, QPointF(r.left() + r.width() * 0.25, yy + 16), 13, row.ok ? Status::Pass : Status::Pending, time_);
            drawText(p, QRectF(r.left() + r.width() * 0.25 + 26, yy, r.width() * 0.6, 32), Qt::AlignLeft | Qt::AlignVCenter,
                     row.t, body(16, true), row.ok ? kGreen : kText);
            yy += 38;
        }
    } else if (id.rfind("param.", 0) == 0) {
        bigGlyph(QStringLiteral("⚙"), kCyan);
    } else if (id == "sens.gps" || id == "sens.gps2") {
        const int sats = id == "sens.gps" ? s.gps_sats : s.gps2_sats;
        const double hdop = id == "sens.gps" ? s.gps_hdop : s.gps2_hdop;
        art::gauge(p, QRectF(r.left() + 40, r.center().y() - 50, r.width() - 80, 30), sats, 0, 25, 10, 25,
                   QStringLiteral("SATELLITES  %1").arg(sats));
        art::gauge(p, QRectF(r.left() + 40, r.center().y() + 10, r.width() - 80, 30), 3.0 - std::min(3.0, hdop), 0, 3, 1.8, 3,
                   QStringLiteral("HDOP  %1  (lower is better)").arg(hdop, 0, 'f', 1));
    } else if (id == "sens.ekf") {
        const double v[4] = {s.ekf_vel_var, s.ekf_pos_var, s.ekf_hgt_var, s.ekf_mag_var};
        const char* n[4] = {"VELOCITY", "POSITION", "HEIGHT", "COMPASS"};
        for (int k = 0; k < 4; ++k) {
            art::gauge(p, QRectF(r.left() + 40, r.top() + 20 + k * 52, r.width() - 80, 30), v[k], 0, 1.2, 0, 0.5,
                       QStringLiteral("%1  %2").arg(QString::fromLatin1(n[k])).arg(v[k], 0, 'f', 2));
        }
    } else if (id == "sens.vibe") {
        const double v[3] = {s.vibe_x, s.vibe_y, s.vibe_z};
        const char* n[3] = {"X", "Y", "Z"};
        for (int k = 0; k < 3; ++k) {
            art::gauge(p, QRectF(r.left() + 40, r.top() + 20 + k * 52, r.width() - 80, 30), v[k], 0, 60, 0, 15,
                       QStringLiteral("VIBRATION %1  %2 m/s²").arg(QString::fromLatin1(n[k])).arg(v[k], 0, 'f', 1));
        }
    } else {
        const QString g = id == "site.airspace" ? QStringLiteral("✈") : id == "site.weather" ? QStringLiteral("☁")
                        : id == "site.landing"  ? QStringLiteral("L")  : id == "site.gcs"    ? QStringLiteral("▣")
                        : id == "pay.video"     ? QStringLiteral("▶") : id == "sens.home"   ? QStringLiteral("H")
                        : id == "sens.health"   ? QStringLiteral("♥") : id == "sens.logging" ? QStringLiteral("SD")
                        : id == "surf.trim"     ? QStringLiteral("═") : QStringLiteral("•");
        bigGlyph(g, statusColor(runner_->status(id)));
    }
    p.restore();
}

void PreflightScreen::paintData(QPainter& p, const QRectF& r) {
    drawGlass(p, chamferPath(r, 12), kCyan, 0.35, alpha(kGlassDeep, 0.9));
    const auto& s = runner_->vehicle();
    double y = r.top() + 12;
    auto row = [&](const QString& k, const QString& v, const QColor& c) {
        drawText(p, QRectF(r.left() + 14, y, r.width() - 28, 16), Qt::AlignLeft | Qt::AlignVCenter, k, labelFont(9), kMuted);
        drawText(p, QRectF(r.left() + 14, y + 15, r.width() - 28, 22), Qt::AlignLeft | Qt::AlignVCenter, v, monoFont(14, true), c);
        y += 44;
    };
    drawText(p, QRectF(r.left() + 14, y, r.width() - 28, 18), Qt::AlignLeft | Qt::AlignVCenter, QStringLiteral("LIVE"),
             labelFont(11), kCyan);
    y += 26;
    row(QStringLiteral("MODE · STATE"),
        QStringLiteral("%1 · %2").arg(QString::fromStdString(s.mode_name.empty() ? std::string("—") : s.mode_name),
                                      s.armed ? QStringLiteral("ARMED") : QStringLiteral("DISARMED")),
        s.armed ? kRed : kText);
    const SafetyState sf = safetyState(s);
    row(QStringLiteral("SAFETY"), sf == SafetyState::Safe ? QStringLiteral("ON (motors cut)") : sf == SafetyState::Live ? QStringLiteral("OFF (live)") : QStringLiteral("—"),
        sf == SafetyState::Safe ? kAmber : sf == SafetyState::Live ? kRed : kMuted);
    row(QStringLiteral("BATTERY"), s.has_battery ? QStringLiteral("%1 V  %2 A").arg(s.battery_v, 0, 'f', 2).arg(std::max(0.0, s.battery_a), 0, 'f', 1) : QStringLiteral("—"), kText);
    row(QStringLiteral("GPS"), s.has_gps ? QStringLiteral("fix %1 · %2 sats · %3").arg(s.gps_fix).arg(s.gps_sats).arg(s.gps_hdop, 0, 'f', 1) : QStringLiteral("—"),
        gpsHealthy(s) ? kGreen : kAmber);
    row(QStringLiteral("EKF (worst ratio)"), s.has_ekf ? QString::number(std::max({s.ekf_vel_var, s.ekf_pos_var, s.ekf_hgt_var, s.ekf_mag_var}), 'f', 2) : QStringLiteral("—"),
        ekfHealthy(s) ? kGreen : kAmber);
    row(QStringLiteral("ATTITUDE"), QStringLiteral("R %1° P %2° Y %3°").arg(s.roll_deg, 0, 'f', 0).arg(s.pitch_deg, 0, 'f', 0).arg(geo::wrap360(s.yaw_deg), 0, 'f', 0), kText);
    row(QStringLiteral("AIRSPEED"), QStringLiteral("%1 m/s").arg(s.airspeed, 0, 'f', 1), kText);
    row(QStringLiteral("RC"), s.has_rc ? QStringLiteral("%1 ch · RSSI %2").arg(s.rc_count).arg(s.rc_rssi == 255 ? QStringLiteral("—") : QString::number(s.rc_rssi)) : QStringLiteral("—"),
        s.has_rc && s.rc_count > 0 ? kText : kAmber);
    row(QStringLiteral("LINK"), QStringLiteral("%1 Hz%2").arg(s.attitude_hz, 0, 'f', 0).arg(s.has_radio ? QStringLiteral(" · rssi %1/%2").arg(s.radio_rssi).arg(s.radio_remrssi) : QString()), kText);
    if (y + 44 < r.bottom()) {
        row(QStringLiteral("VIBRATION"), s.has_vibe ? QStringLiteral("%1 / %2 / %3").arg(s.vibe_x, 0, 'f', 0).arg(s.vibe_y, 0, 'f', 0).arg(s.vibe_z, 0, 'f', 0) : QStringLiteral("—"), kText);
    }
}

void PreflightScreen::paintReady(QPainter& p, const QRectF& r) {
    p.fillRect(r, alpha(kBg, 0.75));
    const QRectF box(r.center().x() - 300, r.center().y() - 170, 600, 340);
    drawGlass(p, chamferPath(box, 18), kGreen, 1.0, alpha(kGlassDeep, 0.97));
    drawBrackets(p, box.adjusted(-8, -8, 8, 8), 30, kGreen, 2.0);
    const double pulse = reducedMotion() ? 1.0 : 0.75 + 0.25 * std::sin(time_ * 3.0);
    drawText(p, QRectF(box.left(), box.top() + 30, box.width(), 50), Qt::AlignCenter, QStringLiteral("READY TO LAUNCH"),
             titleFont(34), alpha(kGreen, pulse));
    const auto& ses = runner_->session();
    int warn = 0, skip = 0;
    for (const auto* d : ses.activeSteps()) {
        const Status st = ses.result(d->id).status;
        warn += st == Status::Warn;
        skip += st == Status::Skipped;
    }
    drawText(p, QRectF(box.left(), box.top() + 96, box.width(), 30), Qt::AlignCenter,
             QStringLiteral("%1 checks · %2 warnings · %3 skipped").arg(ses.totalCount()).arg(warn).arg(skip),
             labelFont(13), kText);
    drawText(p, QRectF(box.left() + 40, box.top() + 136, box.width() - 80, 60), Qt::AlignCenter | Qt::TextWordWrap,
             QStringLiteral("AUTO is selected, the mission starts at its takeoff and the pre-arm checks pass. "
                            "The props are live. Launch from the dock (hold)."),
             body(14), alpha(kText, 0.85));
    const QRectF ok(box.center().x() - 140, box.bottom() - 80, 280, 56);
    paintButton(p, ok, QStringLiteral("CLOSE · GO TO LAUNCH"), Style::Good, true, false, 0);
    hits_.push_back({ok, "ready", "", false, 1.0, true, {}});
}

}  // namespace gcs::ui
