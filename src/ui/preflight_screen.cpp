#include "preflight_screen.hpp"

#include "anim.hpp"
#include "core/geo.hpp"
#include "core/mission.hpp"
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
    QFont f(QString::fromLatin1(kSansFamily));
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

QString q(const std::string& s) {
    return QString::fromStdString(s);
}

art::Part partFor(const std::string& id) {
    if (id == "air.props") return art::Part::Props;
    if (id == "air.pitot" || id == "sens.pitot") return art::Part::Pitot;
    if (id == "air.battery") return art::Part::Battery;
    return art::Part::None;
}

constexpr double kGateHold = 1.0;
constexpr double kRebootHold = 1.0;

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
    const Phase ph = runner_->flow().phase();
    if (ph != last_phase_) {
        last_phase_ = ph;
        list_scroll_ = 0.0;
        last_row_ = -1;
    }
    update();
}

// ---- input ------------------------------------------------------------------------------------

void PreflightScreen::mousePressEvent(QMouseEvent* e) {
    const QPointF pos = toLogical(e->position());  // painted scaled by the text size
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
    const double dy = toLogical(e->position()).y() - press_pos_.y();
    if (list_rect_.contains(press_pos_) && std::abs(dy) > 8) {
        dragging_ = true;
        pressed_ = -1;
        list_scroll_ = press_scroll_ - dy;
        user_scrolled_t_ = time_;
        clampScroll();
    }
}

void PreflightScreen::mouseReleaseEvent(QMouseEvent* e) {
    if (pressed_ >= 0 && pressed_ < static_cast<int>(hits_.size()) && !dragging_) {
        const Hit h = hits_[pressed_];
        if (h.rect.contains(toLogical(e->position()))) {
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
    if (list_rect_.contains(toLogical(e->position()))) {
        list_scroll_ -= e->angleDelta().y() / 2.0;
        user_scrolled_t_ = time_;
        clampScroll();
    }
}

void PreflightScreen::clampScroll() {
    list_scroll_ = std::clamp(list_scroll_, 0.0, std::max(0.0, list_content_h_ - list_rect_.height()));
}

void PreflightScreen::fire(const Hit& h) {
    if (h.kind == "close" || h.kind == "ready") {
        close();
    } else if (h.kind == "look") {
        runner_->setLook(h.value, !isDone(runner_->status(h.value)));
    } else if (h.kind == "continue") {
        runner_->lookContinue();
    } else if (h.kind == "gate") {
        runner_->goMoving();
    } else if (h.kind == "stop") {
        runner_->stopAll();
    } else if (h.kind == "override") {
        emit launchOverride();
        close();
    } else if (h.kind == "rerun") {
        runner_->rerun();
    } else if (h.kind == "restart") {
        runner_->restart();
    } else if (h.kind == "setup") {
        runner_->fixSetup();
    } else if (h.kind == "cal") {
        runner_->startCal(static_cast<PreflightRunner::Cal>(std::stoi(h.value)));
    } else if (h.kind == "calcancel") {
        runner_->cancelCal();
    } else if (h.kind == "reboot") {
        runner_->reboot();
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

void PreflightScreen::addButton(QPainter& p, const QRectF& r, const QString& label, Style style, Hit h) {
    h.rect = r;
    paintButton(p, r, label, style, h.enabled, h.hold, 0.0);
    hits_.push_back(std::move(h));
}

void PreflightScreen::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    hits_.clear();
    const double e = easeOutCubic(open_);
    p.fillRect(rect(), alpha(kBg, 0.99 * e));  // opaque: banners behind must not show through
    if (!reducedMotion()) {
        drawScanlines(p, rect(), 0.4 * e);
    }
    // text size: everything on the screen grows; layout in logical pixels
    p.scale(uiScale(), uiScale());
    const QRectF full(QPointF(0, 0), logicalSize(size()));
    p.setOpacity(e);
    p.translate(0, (1.0 - e) * 30.0);

    const QRectF r = full.adjusted(16, 12, -16, -12);
    paintTop(p, r);
    const QRectF bodyr(r.left(), r.top() + 64, r.width(), r.height() - 64 - 66);
    const Phase ph = runner_->flow().phase();
    if (ph == Phase::Look || ph == Phase::Idle) {
        paintLook(p, bodyr);
    } else if (ph == Phase::Result) {
        paintResult(p, bodyr);
    } else {
        paintChecks(p, bodyr);
    }

    // ---- bottom bar
    const auto& ses = runner_->session();
    if (runner_->testRunning()) {
        addButton(p, QRectF(r.center().x() - 120, r.bottom() - 54, 240, 54), QStringLiteral("■ STOP"), Style::Danger,
                  {{}, "stop", "", false, 1.0, true, {}});
    }
    if (ph == Phase::Result) {
        addButton(p, QRectF(r.left(), r.bottom() - 54, 200, 54), QStringLiteral("↻ START OVER"), Style::Neutral,
                  {{}, "restart", "", false, 1.0, !runner_->testRunning(), QStringLiteral("A test is running")});
    }
    if (!ses.passed()) {
        // the logged way around a preflight that can't be finished here
        addButton(p, QRectF(r.right() - 330, r.bottom() - 54, 330, 54), QStringLiteral("LAUNCH WITHOUT PREFLIGHT · 3 s"),
                  Style::Neutral, {{}, "override", "", true, 3.0, true, {}});
    }
    // charge overlay for the button being held
    if (pressed_ >= 0 && pressed_ < static_cast<int>(hits_.size()) && hits_[pressed_].hold) {
        const Hit& h = hits_[pressed_];
        p.setPen(Qt::NoPen);
        p.setBrush(alpha(kAmber, 0.35));
        p.drawRect(QRectF(h.rect.left(), h.rect.top(), h.rect.width() * std::clamp(hold_t_ / h.hold_secs, 0.0, 1.0),
                          h.rect.height()));
    }
    if (runner_->calRunning() != PreflightRunner::Cal::None) {
        paintCal(p, r);
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

void PreflightScreen::paintTop(QPainter& p, const QRectF& r) {
    const auto& ses = runner_->session();
    const auto& fl = runner_->flow();
    drawText(p, QRectF(r.left() + 6, r.top() + 2, 260, 40), Qt::AlignLeft | Qt::AlignVCenter, QStringLiteral("PREFLIGHT"),
             titleFont(26), kText);
    QString sub;
    switch (fl.phase()) {
    case Phase::Idle:
    case Phase::Look: sub = QStringLiteral("STEP 1 OF 2 · LOOK AT THE AIRCRAFT"); break;
    case Phase::Scan:
    case Phase::Gate:
        sub = QStringLiteral("STEP 2 OF 2 · CHECKING %1 OF %2")
                  .arg(std::min(fl.current() + 1, static_cast<int>(fl.rows().size())))
                  .arg(fl.rows().size());
        break;
    case Phase::Result: sub = QStringLiteral("RESULT"); break;
    }
    if (ses.turnaround()) {
        sub += QStringLiteral(" · SHORT CHECK AFTER A FLIGHT");
    }
    drawText(p, QRectF(r.left() + 250, r.top() + 2, r.width() - 330, 40), Qt::AlignLeft | Qt::AlignVCenter, sub,
             labelFont(12), kCyan);
    // progress over the whole run
    if (fl.phase() == Phase::Scan || fl.phase() == Phase::Gate || fl.phase() == Phase::Result) {
        const QRectF bar(r.left() + 6, r.top() + 50, r.width() - 12, 4);
        p.setPen(Qt::NoPen);
        p.setBrush(alpha(kMuted, 0.25));
        p.drawRect(bar);
        const double f = fl.phase() == Phase::Result ? 1.0 : static_cast<double>(fl.current()) / std::max<size_t>(1, fl.rows().size());
        p.setBrush(kCyan);
        p.drawRect(QRectF(bar.left(), bar.top(), bar.width() * f, bar.height()));
    }
    const QRectF cl(r.right() - 52, r.top(), 52, 48);
    drawGlass(p, chamferPath(cl, 8), kCyan, 0.6, kGlassDeep);
    drawText(p, cl, Qt::AlignCenter, QStringLiteral("✕"), monoFont(20, true), kText);
    hits_.push_back({cl, "close", "", false, 1.0, true, {}});
}

// ---- look at the aircraft ----------------------------------------------------------------------------

void PreflightScreen::paintLook(QPainter& p, const QRectF& r) {
    const auto& ses = runner_->session();
    const double list_w = std::min(680.0, r.width() * 0.58);
    const QRectF col(r.left(), r.top(), list_w, r.height());
    drawText(p, QRectF(col.left() + 4, col.top(), col.width(), 36), Qt::AlignLeft | Qt::AlignVCenter,
             QStringLiteral("Look at the aircraft"), body(26, true), kText);
    drawText(p, QRectF(col.left() + 4, col.top() + 36, col.width(), 24), Qt::AlignLeft | Qt::AlignVCenter,
             QStringLiteral("Tap each one once you've checked it. The rest checks itself."), body(14), alpha(kText, 0.7));
    double y = col.top() + 76;
    const double row_h = std::clamp((r.bottom() - y) / 5.0, 52.0, 72.0);
    std::string focus;
    for (const auto* d : ses.activeSteps()) {
        if (d->stage != Stage::Look) continue;
        const Status st = runner_->status(d->id);
        const bool done = isDone(st);
        if (!done && focus.empty()) focus = d->id;
        const QRectF rr(col.left(), y, col.width(), row_h - 8);
        const bool tappable = d->kind == Kind::You;
        drawGlass(p, chamferPath(rr, 8), done ? kGreen : kCyan, done ? 0.8 : 0.45,
                  alpha(done ? kGreen : kGlassDeep, done ? 0.10 : 0.9));
        // tick box (the pitot row ticks itself)
        const QRectF box(rr.left() + 16, rr.center().y() - 16, 32, 32);
        if (tappable) {
            p.setPen(QPen(done ? kGreen : kCyan, 2));
            p.setBrush(done ? alpha(kGreen, 0.25) : Qt::NoBrush);
            p.drawRect(box);
            if (done) {
                drawText(p, box, Qt::AlignCenter, QStringLiteral("✓"), monoFont(22, true), kGreen);
            }
        } else {
            art::statusGlyph(p, box.center(), 16, st == Status::Pending ? Status::Waiting : st, time_);
        }
        const QString title = q(d->title);
        const QString live = tappable ? QString() : runner_->detail(d->id);
        drawText(p, QRectF(box.right() + 18, rr.top() + (live.isEmpty() ? 0 : 6), rr.width() - 90,
                           live.isEmpty() ? rr.height() : 26),
                 Qt::AlignLeft | Qt::AlignVCenter, title, body(18, true), done ? kGreen : kText);
        if (!live.isEmpty()) {
            drawText(p, QRectF(box.right() + 18, rr.top() + 30, rr.width() - 90, 20), Qt::AlignLeft | Qt::AlignVCenter,
                     live + (st != Status::NotApplicable && !done
                                 ? QStringLiteral(" · %1 m/s").arg(runner_->vehicle().airspeed, 0, 'f', 1)
                                 : QString()),
                     body(13), alpha(statusColor(st), 0.95));
        }
        if (tappable) {
            hits_.push_back({rr, "look", d->id, false, 1.0, true, {}});
        }
        y += row_h;
    }
    const bool all = lookComplete(ses);
    // in the bottom bar, where the result has START OVER: always on screen
    addButton(p, QRectF(col.left(), r.bottom() + 66 - 54, 320, 54), QStringLiteral("CONTINUE ▶"),
              all ? Style::Good : Style::Primary,
              {{}, "continue", "", false, 1.0, all,
               runner_->status("sens.pitot") != Status::Pass && runner_->status("sens.pitot") != Status::NotApplicable &&
                       ses.includes("sens.pitot")
                   ? QStringLiteral("Blow gently into the pitot tube first")
                   : QStringLiteral("Tap each item once you've checked it")});

    // the picture: the item to look at next
    const QRectF pic(col.right() + 30, r.top() + 60, r.right() - col.right() - 30, r.height() - 60);
    if (pic.width() > 160) {
        const double side = std::min(pic.width(), pic.height());
        const QRectF sq(pic.center().x() - side / 2, pic.top(), side, side);
        if (focus == "site.area") {
            art::site(p, sq, time_);
        } else {
            art::TopView v;
            v.frame = runner_->frame();
            v.show_dirs = focus == "air.props";
            v.highlight = partFor(focus);
            art::quadplaneTop(p, sq.adjusted(20, 10, -20, -10), v, time_);
        }
    }
}

// ---- the checks ------------------------------------------------------------------------------------------

void PreflightScreen::paintChecks(QPainter& p, const QRectF& r) {
    const auto& fl = runner_->flow();
    const auto& rows = fl.rows();
    const double list_w = std::clamp(r.width() * 0.42, 340.0, 520.0);
    const QRectF col(r.left(), r.top(), list_w, r.height());
    drawGlass(p, chamferPath(col, 12), kCyan, 0.35, alpha(kGlassDeep, 0.9));
    list_rect_ = col.adjusted(8, 8, -8, -8);

    // content: a header per section, a row per step
    const double row_h = 40, head_h = 30;
    struct Line {
        double y;
        int row;  // -1 = header
        Stage stage;
    };
    std::vector<Line> lines;
    double y = 0;
    Stage last = Stage::Count;
    int cur_y = 0;
    for (size_t i = 0; i < rows.size(); ++i) {
        const StepDef* d = findStep(rows[i]);
        if (!d) continue;
        if (d->stage != last) {
            lines.push_back({y, -1, d->stage});
            y += head_h;
            last = d->stage;
        }
        if (static_cast<int>(i) == fl.current()) cur_y = static_cast<int>(y);
        lines.push_back({y, static_cast<int>(i), d->stage});
        y += row_h;
    }
    list_content_h_ = y;
    // keep the running row in view unless the operator is looking around
    if (fl.current() != last_row_ && time_ - user_scrolled_t_ > 4.0) {
        last_row_ = fl.current();
        list_scroll_ = cur_y - list_rect_.height() * 0.45;
    }
    clampScroll();

    p.save();
    p.setClipRect(list_rect_);
    for (const auto& ln : lines) {
        const QRectF rr(list_rect_.left(), list_rect_.top() + ln.y - list_scroll_, list_rect_.width(),
                        ln.row < 0 ? head_h : row_h - 4);
        if (rr.bottom() < list_rect_.top() || rr.top() > list_rect_.bottom()) continue;
        if (ln.row < 0) {
            drawText(p, rr.adjusted(10, 6, -10, 0), Qt::AlignLeft | Qt::AlignVCenter,
                     QString::fromLatin1(stageName(ln.stage)).toUpper(), labelFont(10), kMuted);
            continue;
        }
        const std::string& id = rows[static_cast<size_t>(ln.row)];
        const StepDef* d = findStep(id);
        const bool now = ln.row == fl.current() && fl.phase() != Phase::Result;
        const bool reached = fl.reached(ln.row);
        const Status st = runner_->status(id);
        if (now) drawGlass(p, chamferPath(rr, 6), kCyan, 0.9, alpha(kCyan, 0.12));
        const Status shown = now ? Status::Waiting : reached ? st : Status::Pending;
        art::statusGlyph(p, QPointF(rr.left() + 20, rr.center().y()), 11, shown, time_);
        const QColor tc = now ? kText : reached ? alpha(kText, 0.9) : alpha(kMuted, 0.7);
        drawText(p, QRectF(rr.left() + 42, rr.top(), rr.width() - 140, rr.height()), Qt::AlignLeft | Qt::AlignVCenter,
                 q(d->title), body(14, now), tc);
        if (reached) {
            const QString word = st == Status::Pass ? QStringLiteral("OK")
                                 : st == Status::NotApplicable ? QStringLiteral("—")
                                 : st == Status::Fail && d->critical ? QStringLiteral("PROBLEM")
                                 : isDone(st) || st == Status::Fail ? QStringLiteral("NOTE")
                                                                    : QStringLiteral("…");
            drawText(p, QRectF(rr.right() - 96, rr.top(), 88, rr.height()), Qt::AlignRight | Qt::AlignVCenter, word,
                     labelFont(10), statusColor(st == Status::Fail && !d->critical ? Status::Warn : st));
        }
    }
    p.restore();

    const QRectF right(col.right() + 16, r.top(), r.right() - col.right() - 16, r.height());
    if (fl.phase() == Phase::Gate) {
        paintGate(p, right);
    } else {
        paintNow(p, right);
    }
}

void PreflightScreen::paintNow(QPainter& p, const QRectF& r) {
    drawGlass(p, chamferPath(r, 14), kCyan, 0.5, alpha(kGlassDeep, 0.92));
    const std::string id = runner_->flow().currentId();
    const StepDef* d = findStep(id);
    if (!d) return;
    const double x = r.left() + 26, w = r.width() - 52;
    double y = r.top() + 20;
    drawText(p, QRectF(x, y, w, 18), Qt::AlignLeft | Qt::AlignVCenter, QString::fromLatin1(stageName(d->stage)).toUpper(),
             labelFont(11), kCyan);
    y += 26;
    const int dots = static_cast<int>(time_ * 2.5) % 4;
    const QString sentence = q(d->sentence) + QString(dots, QLatin1Char('.'));
    const double sh = wrapH(body(26, true), w, sentence);
    wrapText(p, QRectF(x, y, w, sh + 4), sentence, body(26, true), kText);
    y += sh + 12;
    const QString det = runner_->detail(id);
    if (!det.isEmpty()) {
        const double dh = std::min(90.0, wrapH(body(15), w, det));
        wrapText(p, QRectF(x, y, w, dh + 2), det, body(15), alpha(statusColor(runner_->status(id)), 0.95));
        y += dh + 12;
    }
    paintArt(p, QRectF(x, y, w, r.bottom() - 20 - y), id);
}

void PreflightScreen::paintGate(QPainter& p, const QRectF& r) {
    drawGlass(p, chamferPath(r, 14), kAmber, 0.8, alpha(kGlassDeep, 0.94));
    const double x = r.left() + 30, w = r.width() - 60;
    double y = r.top() + 26;
    drawText(p, QRectF(x, y, w, 18), Qt::AlignLeft | Qt::AlignVCenter, QStringLiteral("MOVING PARTS NEXT"), labelFont(12),
             kAmber);
    y += 30;
    const QString t1 = QStringLiteral("The control surfaces will move, then each motor spins slowly for 2 seconds.");
    const double h1 = wrapH(body(24, true), w, t1);
    wrapText(p, QRectF(x, y, w, h1 + 4), t1, body(24, true), kText);
    y += h1 + 14;
    const QString t2 = QStringLiteral("It stays on the ground. Keep hands, people and loose things clear of every "
                                      "propeller, then hold the button.");
    const double h2 = wrapH(body(16), w, t2);
    wrapText(p, QRectF(x, y, w, h2 + 4), t2, body(16), alpha(kText, 0.8));
    y += h2 + 20;
    const double side = std::min(w, r.bottom() - y - 110);
    if (side > 80) {
        art::TopView v;
        v.frame = runner_->frame();
        v.show_letters = true;
        v.highlight = art::Part::Motors;
        art::quadplaneTop(p, QRectF(r.center().x() - side / 2, y, side, side), v, time_);
    }
    const bool armed = runner_->vehicle().armed;
    addButton(p, QRectF(r.center().x() - 190, r.bottom() - 92, 380, 70), QStringLiteral("ALL CLEAR · MOVE AND SPIN"),
              Style::Danger, {{}, "gate", "", true, kGateHold, !armed, QStringLiteral("Disarm first")});
}

// ---- result ----------------------------------------------------------------------------------------------

void PreflightScreen::paintResult(QPainter& p, const QRectF& r) {
    const auto& ses = runner_->session();
    const auto probs = problems(ses);
    const auto nts = notes(ses);
    const bool ready = probs.empty() && ses.complete();
    const double x = r.left(), w = std::min(r.width(), 1100.0);
    const double cx = r.center().x() - w / 2;
    double y = r.top();
    if (ready) {
        const double pulse = reducedMotion() ? 1.0 : 0.8 + 0.2 * std::sin(time_ * 3.0);
        drawText(p, QRectF(cx, y, w, 56), Qt::AlignLeft | Qt::AlignVCenter, QStringLiteral("READY TO FLY"), titleFont(40),
                 alpha(kGreen, pulse));
        drawText(p, QRectF(cx, y + 56, w, 26), Qt::AlignLeft | Qt::AlignVCenter,
                 nts.empty() ? QStringLiteral("Everything checked out. Launch from the dock.")
                             : QStringLiteral("Everything needed checked out. %1 note%2 below, none of them stops the flight.")
                                   .arg(nts.size())
                                   .arg(nts.size() == 1 ? "" : "s"),
                 body(16), alpha(kText, 0.85));
    } else {
        const size_t n = probs.size();
        drawText(p, QRectF(cx, y, w, 56), Qt::AlignLeft | Qt::AlignVCenter,
                 n ? QStringLiteral("%1 PROBLEM%2 TO FIX").arg(n).arg(n == 1 ? "" : "S") : QStringLiteral("STILL CHECKING"),
                 titleFont(36), n ? kRed : kCyan);
        drawText(p, QRectF(cx, y + 56, w, 26), Qt::AlignLeft | Qt::AlignVCenter,
                 n == 1 ? QStringLiteral("Launch stays locked until it's fixed. It says what to do.")
                        : QStringLiteral("Launch stays locked until they're fixed. Each one says what to do."),
                 body(16),
                 alpha(kText, 0.85));
    }
    y += 96;
    if (!ready) {
        addButton(p, QRectF(cx + w - 220, r.top() + 6, 220, 56), QStringLiteral("↻ CHECK AGAIN"), Style::Primary,
                  {{}, "rerun", "", false, 1.0, !runner_->testRunning(), QStringLiteral("A test is running")});
    } else {
        addButton(p, QRectF(cx + w - 220, r.top() + 6, 220, 56), QStringLiteral("CLOSE"), Style::Good,
                  {{}, "ready", "", false, 1.0, true, {}});
    }

    // the cards: problems, the restart, then the notes
    struct Card {
        const StepDef* def = nullptr;
        Result res;
        bool problem = false;
        bool reboot = false;
    };
    std::vector<Card> cards;
    for (const auto& it : probs) cards.push_back({it.def, it.result, true, false});
    if (runner_->rebootPending()) cards.push_back({nullptr, {}, false, true});
    for (const auto& it : nts) cards.push_back({it.def, it.result, false, false});

    list_rect_ = QRectF(cx, y, w, r.bottom() - y);
    const double card_w = w;
    std::vector<double> heights;
    double total = 0;
    for (const auto& c : cards) {
        double h = 0;
        if (c.reboot) {
            h = 92;
        } else {
            const QString how = q(c.def->how), det = q(c.res.detail);
            h = 16 + 26 + wrapH(body(15), card_w - 330, how) + 6 + (det.isEmpty() ? 0 : wrapH(body(12.5), card_w - 330, det) + 6) + 14;
            h = std::max(h, 92.0);
        }
        heights.push_back(h);
        total += h + 10;
    }
    list_content_h_ = total;
    clampScroll();
    p.save();
    p.setClipRect(list_rect_);
    double cy = list_rect_.top() - list_scroll_;
    for (size_t i = 0; i < cards.size(); ++i) {
        const Card& c = cards[i];
        const QRectF cr(cx, cy, card_w, heights[i]);
        cy += heights[i] + 10;
        if (cr.bottom() < list_rect_.top() || cr.top() > list_rect_.bottom()) continue;
        const QColor edge = c.problem ? kRed : c.reboot ? kCyan : kAmber;
        drawGlass(p, chamferPath(cr, 10), edge, 0.7, alpha(kGlassDeep, 0.92));
        const QRectF btn(cr.right() - 270, cr.center().y() - 30, 250, 60);
        const bool btn_visible = cr.intersects(list_rect_);
        if (c.reboot) {
            art::statusGlyph(p, QPointF(cr.left() + 30, cr.top() + 30), 14, Status::Waiting, 0.0);
            drawText(p, QRectF(cr.left() + 58, cr.top() + 14, cr.width() - 340, 28), Qt::AlignLeft | Qt::AlignVCenter,
                     QStringLiteral("Restart the autopilot"), body(18, true), kText);
            drawText(p, QRectF(cr.left() + 58, cr.top() + 44, cr.width() - 340, 36), Qt::AlignLeft | Qt::AlignTop | Qt::TextWordWrap,
                     QStringLiteral("A new setting only takes effect after a restart. The link comes back in a few seconds."),
                     body(14), alpha(kText, 0.8));
            if (btn_visible) {
                addButton(p, btn, QStringLiteral("RESTART AUTOPILOT"), Style::Primary,
                          {{}, "reboot", "", true, kRebootHold, !runner_->vehicle().armed, QStringLiteral("Disarm first")});
            }
            continue;
        }
        const Status shown = c.problem ? (c.res.status == Status::Waiting || c.res.status == Status::Pending ? Status::Fail : c.res.status)
                                       : Status::Warn;
        art::statusGlyph(p, QPointF(cr.left() + 30, cr.top() + 30), 14, shown, time_);
        const double tx = cr.left() + 58, tw = cr.width() - 340;
        double ty = cr.top() + 14;
        drawText(p, QRectF(tx, ty, tw, 26), Qt::AlignLeft | Qt::AlignVCenter, q(c.def->title), body(18, true), kText);
        ty += 30;
        const QString how = q(c.def->how);
        const double hh = wrapH(body(15), tw, how);
        wrapText(p, QRectF(tx, ty, tw, hh + 2), how, body(15), alpha(kText, 0.85));
        ty += hh + 6;
        QString det = q(c.res.detail);
        if (c.res.note == "timed out") det = QStringLiteral("Still waiting: ") + det;
        if (!det.isEmpty()) {
            wrapText(p, QRectF(tx, ty, tw, wrapH(body(12.5), tw, det) + 2), det, body(12.5), alpha(kMuted, 0.95));
        }
        if (!btn_visible) continue;
        // at most one action
        const std::string& id = c.def->id;
        const PreflightRunner::Cal cal = runner_->calFor(id);
        if (id == "setup.app" && runner_->setupFixable()) {
            addButton(p, btn, runner_->setupWriting() ? QStringLiteral("WRITING…") : QStringLiteral("SET IT UP"),
                      Style::Good, {{}, "setup", "", false, 1.0, !runner_->setupWriting(), QStringLiteral("Writing the settings")});
        } else if (cal != PreflightRunner::Cal::None) {
            addButton(p, btn, cal == PreflightRunner::Cal::Compass ? QStringLiteral("CALIBRATE COMPASS")
                                                                   : QStringLiteral("CALIBRATE LEVEL"),
                      Style::Primary,
                      {{}, "cal", std::to_string(static_cast<int>(cal)), false, 1.0,
                       !runner_->vehicle().armed && !runner_->testRunning(), QStringLiteral("Disarm first")});
        } else if (c.def->kind == Kind::Test && c.def->stage != Stage::Look) {
            addButton(p, btn, QStringLiteral("↻ TRY AGAIN"), Style::Primary,
                      {{}, "rerun", "", false, 1.0, !runner_->testRunning(), QStringLiteral("A test is running")});
        }
    }
    p.restore();
    (void)x;
}

void PreflightScreen::paintCal(QPainter& p, const QRectF& r) {
    p.fillRect(r, alpha(kBg, 0.8));
    const QRectF box(r.center().x() - 360, r.center().y() - 260, 720, 520);
    drawGlass(p, chamferPath(box, 18), kCyan, 1.0, alpha(kGlassDeep, 0.97));
    const bool compass = runner_->calRunning() == PreflightRunner::Cal::Compass;
    drawText(p, QRectF(box.left() + 30, box.top() + 20, box.width() - 60, 40), Qt::AlignLeft | Qt::AlignVCenter,
             compass ? QStringLiteral("Calibrating the compass") : QStringLiteral("Calibrating level"), body(24, true), kText);
    const QString t = runner_->calText();
    const double th = wrapH(body(16), box.width() - 60, t);
    wrapText(p, QRectF(box.left() + 30, box.top() + 70, box.width() - 60, th + 4), t, body(16), alpha(kText, 0.85));
    const QRectF pic(box.center().x() - 140, box.top() + 90 + th, 280, 280);
    if (compass) {
        std::array<uint8_t, 10> mask{};
        for (const auto& c : runner_->vehicle().magcal) {
            if (!c.active) continue;
            for (int k = 0; k < 10; ++k) mask[k] |= c.mask[k];
        }
        art::compassSphere(p, pic, mask, runner_->calPercent(), time_);
    } else {
        art::statusGlyph(p, pic.center(), 60, Status::Waiting, time_);
    }
    addButton(p, QRectF(box.center().x() - 110, box.bottom() - 76, 220, 56), QStringLiteral("CANCEL"), Style::Neutral,
              {{}, "calcancel", "", false, 1.0, true, {}});
}

// ---- pictures --------------------------------------------------------------------------------------------

void PreflightScreen::paintArt(QPainter& p, const QRectF& r, const std::string& id) {
    if (r.height() < 60) return;
    const auto& s = runner_->vehicle();
    const auto& a = runner_->art();
    p.save();
    p.setClipRect(r);
    const double side = std::min(r.width(), r.height());
    const QRectF sq(r.center().x() - side / 2, r.top(), side, side);
    auto bigGlyph = [&](const QString& g, const QColor& c) {
        drawText(p, r, Qt::AlignCenter, g, body(std::min(r.height() * 0.5, 110.0), true), alpha(c, 0.85));
    };
    auto gaugeAt = [&](int k, double v, double lo, double hi, double glo, double ghi, const QString& label) {
        art::gauge(p, QRectF(r.left() + 40, r.top() + 20 + k * 56, r.width() - 80, 30), v, lo, hi, glo, ghi, label);
    };

    if (id == "profile.vehicle") {
        art::TopView v;
        v.frame = runner_->frame();
        v.show_dirs = true;
        v.show_letters = true;
        art::quadplaneTop(p, sq.adjusted(30, 10, -30, -10), v, time_);
    } else if (id == "mot.lift" || id == "mot.pusher") {
        art::TopView v;
        v.frame = runner_->frame();
        v.show_letters = true;
        v.lit_motor = a.lit_motor;
        v.pusher_lit = a.pusher_lit;
        v.highlight = id == "mot.pusher" ? art::Part::Pusher : art::Part::Motors;
        art::quadplaneTop(p, sq.adjusted(30, 10, -30, -10), v, time_);
    } else if (id == "power.voltage" || id == "power.balance") {
        const auto& pr = runner_->profile();
        art::battery(p, QRectF(r.left() + r.width() * 0.15, r.top(), r.width() * 0.7, std::min(r.height(), 200.0)), s.cell_v,
                     s.battery_v, pr.cells, fullCellVolts(pr.chemistry), storageCellVolts(pr.chemistry));
    } else if (id == "power.current") {
        gaugeAt(0, std::max(0.0, s.battery_a), 0, 20, 0.05, 8, QStringLiteral("CURRENT AT REST  %1 A").arg(s.battery_a, 0, 'f', 1));
    } else if (id == "power.board") {
        gaugeAt(0, s.board_vcc, 4.0, 6.0, 4.8, 5.4, QStringLiteral("AUTOPILOT SUPPLY  %1 V").arg(s.board_vcc, 0, 'f', 2));
    } else if (id == "link.telemetry") {
        gaugeAt(0, s.attitude_hz, 0, 30, 8, 30, QStringLiteral("MESSAGES  %1 / s").arg(s.attitude_hz, 0, 'f', 0));
        if (s.has_radio) {
            const int margin = std::min(s.radio_rssi - s.radio_noise, s.radio_remrssi - s.radio_remnoise);
            gaugeAt(1, margin, 0, 120, 20, 120, QStringLiteral("RADIO SIGNAL  %1").arg(margin));
        }
    } else if (id == "sens.gps") {
        gaugeAt(0, s.gps_sats, 0, 25, 10, 25, QStringLiteral("SATELLITES  %1  (10 needed)").arg(s.gps_sats));
        gaugeAt(1, 3.0 - std::min(3.0, s.gps_hdop), 0, 3, 1.8, 3,
                QStringLiteral("PRECISION  %1  (lower is better)").arg(s.gps_hdop, 0, 'f', 1));
    } else if (id == "sens.ekf") {
        const double v[4] = {s.ekf_vel_var, s.ekf_pos_var, s.ekf_hgt_var, s.ekf_mag_var};
        const char* n[4] = {"SPEED", "POSITION", "HEIGHT", "COMPASS"};
        for (int k = 0; k < 4; ++k) {
            gaugeAt(k, v[k], 0, 1.2, 0, 0.5, QStringLiteral("%1  %2").arg(QString::fromLatin1(n[k])).arg(v[k], 0, 'f', 2));
        }
    } else if (id == "sens.vibe") {
        const double v[3] = {s.vibe_x, s.vibe_y, s.vibe_z};
        const char* n[3] = {"X", "Y", "Z"};
        for (int k = 0; k < 3; ++k) {
            gaugeAt(k, v[k], 0, 60, 0, 15, QStringLiteral("VIBRATION %1  %2").arg(QString::fromLatin1(n[k])).arg(v[k], 0, 'f', 1));
        }
    } else if (id.rfind("surf.", 0) == 0) {
        art::surface(p, r.adjusted(r.width() * 0.1, 0, -r.width() * 0.1, 0),
                     a.axis == 0 ? art::Axis::Roll : a.axis == 1 ? art::Axis::Pitch : art::Axis::Yaw, a.surf_cmd, a.surf_out,
                     time_);
    } else if (id == "pay.gimbal") {
        art::gimbal(p, sq, a.gimbal_pitch, a.gimbal_yaw);
    } else if (id.rfind("mis.", 0) == 0) {
        // route preview: waypoints, L and H, fitted into the box
        if (s.mission_known && s.has_home) {
            const MissionPlan plan = extractPlan(s.mission);
            std::vector<geo::NE> pts;
            for (const auto& wp : plan.wps) pts.push_back(geo::delta(s.home_lat, s.home_lon, wp.lat, wp.lon));
            std::optional<geo::NE> land;
            if (plan.land) land = geo::delta(s.home_lat, s.home_lon, plan.land->lat, plan.land->lon);
            double ext = 50;
            for (const auto& pt : pts) ext = std::max({ext, std::abs(pt.north), std::abs(pt.east)});
            if (land) ext = std::max({ext, std::abs(land->north), std::abs(land->east)});
            const double sc = side * 0.42 / ext;
            const QPointF c = r.center();
            auto at = [&](const geo::NE& pt) { return QPointF(c.x() + pt.east * sc, c.y() - pt.north * sc); };
            QPolygonF poly;
            for (const auto& pt : pts) poly << at(pt);
            if (!poly.isEmpty()) poly << poly.front();
            p.setPen(QPen(kCyan, 2, Qt::DashLine));
            p.setBrush(Qt::NoBrush);
            p.drawPolyline(poly);
            for (size_t k = 0; k < pts.size(); ++k) {
                p.setPen(QPen(kCyan, 2));
                p.setBrush(alpha(kBg, 0.9));
                p.drawEllipse(at(pts[k]), 11, 11);
                drawText(p, QRectF(at(pts[k]).x() - 11, at(pts[k]).y() - 11, 22, 22), Qt::AlignCenter, QString::number(k + 1),
                         monoFont(11, true), kCyan);
            }
            p.setPen(QPen(kText, 2));
            p.drawEllipse(c, 12, 12);
            drawText(p, QRectF(c.x() - 12, c.y() - 12, 24, 24), Qt::AlignCenter, QStringLiteral("H"), monoFont(12, true), kText);
            if (land) {
                const QPointF l = at(*land);
                p.setPen(QPen(kCyan, 2));
                p.drawRect(QRectF(l.x() - 11, l.y() - 11, 22, 22));
                drawText(p, QRectF(l.x() - 11, l.y() - 11, 22, 22), Qt::AlignCenter, QStringLiteral("L"), monoFont(12, true), kCyan);
            }
        } else {
            bigGlyph(QStringLiteral("…"), kMuted);
        }
    } else {
        const QString g = id == "pay.video" ? QStringLiteral("▶")
                        : id == "sens.home" ? QStringLiteral("H")
                        : id == "sens.health" ? QStringLiteral("♥")
                        : id == "sens.logging" ? QStringLiteral("SD")
                        : id.rfind("setup.", 0) == 0 ? QStringLiteral("⚙")
                        : id == "fin.ready" ? QStringLiteral("✓")
                                            : QStringLiteral("•");
        bigGlyph(g, statusColor(runner_->status(id)));
    }
    p.restore();
}

}  // namespace gcs::ui
