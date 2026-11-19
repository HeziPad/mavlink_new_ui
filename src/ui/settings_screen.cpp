#include "settings_screen.hpp"

#include "anim.hpp"
#include "core/endurance.hpp"
#include "theme.hpp"

#include <QFontMetricsF>
#include <QMouseEvent>
#include <QPainter>

#include <algorithm>
#include <cmath>

namespace gcs::ui {

using namespace theme;

const std::vector<int>& SettingsScreen::trailChoices() {
    static const std::vector<int> c = {30, 60, 120, 300, 600, 0};
    return c;
}

const std::vector<int>& SettingsScreen::reserveChoices() {
    static const std::vector<int> c = {15, 20, 25, 30, 35};
    return c;
}

namespace {
QString sizeText(qint64 b) {
    if (b < 0) {
        return QStringLiteral("…");
    }
    if (b >= 1000LL * 1000 * 1000) {
        return QStringLiteral("%1 GB").arg(b / 1e9, 0, 'f', 1);
    }
    if (b >= 1000LL * 1000) {
        return QStringLiteral("%1 MB").arg(b / 1e6, 0, 'f', b >= 100e6 ? 0 : 1);
    }
    return QStringLiteral("%1 kB").arg(b / 1e3, 0, 'f', 0);
}
}  // namespace

SettingsScreen::SettingsScreen(QWidget* parent) : QWidget(parent) {
    labels_.labels = {QStringLiteral("עברית"), QStringLiteral("ENGLISH")};
    online_.labels = {QStringLiteral("OFF"), QStringLiteral("ON")};
    alt_.labels = {QStringLiteral("METERS"), QStringLiteral("FEET")};
    speed_.labels = {QStringLiteral("M/S"), QStringLiteral("KNOTS")};
    coord_.labels = {QStringLiteral("DEGREES"), QStringLiteral("D° M′ S″"), QStringLiteral("UTM")};
    trail_on_.labels = {QStringLiteral("OFF"), QStringLiteral("ON")};
    for (int pct : reserveChoices()) {
        reserve_.labels << QStringLiteral("%1 %").arg(pct);
    }
    text_size_.labels = {QStringLiteral("SMALL"), QStringLiteral("MEDIUM"), QStringLiteral("LARGE")};
    link_type_.labels = {QStringLiteral("UDP · NETWORK"), QStringLiteral("SERIAL · USB / RADIO")};
    for (int b : link::serialBauds()) {
        baud_.labels << QString::number(b);
    }
    trail_len_.labels = {QStringLiteral("30 S"), QStringLiteral("1 MIN"), QStringLiteral("2 MIN"),
                         QStringLiteral("5 MIN"), QStringLiteral("10 MIN"), QStringLiteral("ALL")};
    hide();
}

void SettingsScreen::open(const Values& v) {
    v_ = v;
    alt_.selected = v.alt_unit;
    speed_.selected = v.speed_unit;
    coord_.selected = v.coord_format;
    trail_on_.selected = v.trail_on ? 1 : 0;
    const auto& c = trailChoices();
    const auto it = std::find(c.begin(), c.end(), v.trail_secs);
    trail_len_.selected = it != c.end() ? static_cast<int>(it - c.begin()) : 2;
    labels_.selected = v.labels_english ? 1 : 0;
    online_.selected = off_.online ? 1 : 0;
    const auto& rc = reserveChoices();
    // the nearest choice, so a value saved from an older choice list still shows
    const auto rit = std::min_element(rc.begin(), rc.end(), [&](int a, int b) {
        return std::abs(a - v.reserve_pct) < std::abs(b - v.reserve_pct);
    });
    reserve_.selected = static_cast<int>(rit - rc.begin());
    text_size_.selected = std::clamp(v.text_size, 0, 2);
    setLink(v.link);
    applied_ = v.link;
    refreshPorts();
    for (Segmented* s : {&alt_, &speed_, &coord_, &trail_on_, &trail_len_, &labels_, &online_, &reserve_, &text_size_, &link_type_, &baud_}) {
        s->knob = s->selected;
    }
    confirm_ = Confirm::None;
    trail_len_.enabled = v.trail_on;

    // freeze a blurred copy of what's behind: cheap down/up-scale blur
    hide();
    if (parentWidget()) {
        const QPixmap shot = parentWidget()->grab();
        const QSize small = (QSizeF(shot.size()) / 10.0).toSize().expandedTo(QSize(1, 1));
        backdrop_ = shot.scaled(small, Qt::IgnoreAspectRatio, Qt::SmoothTransformation)
                        .scaled(shot.size(), Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
        backdrop_.setDevicePixelRatio(shot.devicePixelRatio());
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

void SettingsScreen::close() {
    if (open_target_ == 0.0) {
        return;
    }
    open_target_ = 0.0;
    if (reducedMotion()) {
        open_ = 0.0;
        hide();
    }
    emit closed();
}

void SettingsScreen::tick(double dt) {
    if (!isVisible()) {
        return;
    }
    time_ += dt;
    open_ += std::clamp(open_target_ - open_, -dt * 4.5, dt * 4.5);
    if (open_ <= 0.0 && open_target_ == 0.0) {
        hide();
        backdrop_ = QPixmap();
        return;
    }
    const double k = reducedMotion() ? 1.0 : 1.0 - std::exp(-16.0 * dt);
    for (Segmented* s : {&alt_, &speed_, &coord_, &trail_on_, &trail_len_, &labels_, &online_, &reserve_, &text_size_, &link_type_, &baud_}) {
        s->knob += (s->selected - s->knob) * k;
    }
    // the serial port list follows what is plugged in
    ports_age_ += dt;
    if (category_ == kLink && link_.serial && ports_age_ > 2.0 && confirm_ == Confirm::None) {
        refreshPorts();
    }
    cat_knob_ += (category_ - cat_knob_) * k;
    flash_ = std::max(0.0, flash_ - dt * 2.0);
    update();
}

QSizeF SettingsScreen::lsize() const {
    return logicalSize(size());
}

QRectF SettingsScreen::panelRect() const {
    const QSizeF s = lsize();
    const double w = std::min(860.0, s.width() - 48.0);
    const double want = category_ == kLink ? 640.0 : (category_ == kOffline || category_ == kUnits ? 560.0 : 480.0);
    const double h = std::min(want, s.height() - 80.0);
    return {(s.width() - w) / 2.0, (s.height() - h) / 2.0, w, h};
}

bool SettingsScreen::pressSegmented(Segmented& seg, const QPointF& pos) {
    if (!seg.enabled) {
        return false;
    }
    for (int i = 0; i < static_cast<int>(seg.cells.size()); ++i) {
        if (seg.cells[i].contains(pos) && i != seg.selected) {
            seg.selected = i;
            flash_ = 1.0;
            return true;
        }
    }
    return false;
}

void SettingsScreen::mousePressEvent(QMouseEvent* e) {
    const QPointF pos = toLogical(e->position());  // the panel is painted scaled by the text size
    if (open_target_ < 0.5) {
        return;
    }
    if (confirm_ != Confirm::None) {
        // the modal owns every tap until it's answered
        if (confirm_yes_.contains(pos)) {
            if (confirm_ == Confirm::Relink) {
                applied_ = link_;
                emit linkConnect(link_);
            } else {
                confirm_ == Confirm::ClearDetail ? emit offlineClearDetail() : emit offlineDeleteAll();
            }
            confirm_ = Confirm::None;
        } else if (confirm_no_.contains(pos)) {
            confirm_ = Confirm::None;
        }
        update();
        return;
    }
    if (close_rect_.contains(pos) || !panelRect().contains(pos)) {
        close();
        return;
    }
    for (int i = 0; i < static_cast<int>(cat_rects_.size()); ++i) {
        if (cat_rects_[i].contains(pos)) {
            category_ = i;
            update();
            return;
        }
    }
    if (category_ == kLink) {
        pressLink(pos);
    } else if (category_ == kUnits) {
        if (pressSegmented(alt_, pos)) {
            emit altUnitChanged(alt_.selected);
        } else if (pressSegmented(speed_, pos)) {
            emit speedUnitChanged(speed_.selected);
        } else if (pressSegmented(coord_, pos)) {
            emit coordFormatChanged(coord_.selected);
        }
    } else if (category_ == kDisplay) {
        if (pressSegmented(text_size_, pos)) {
            emit textSizeChanged(text_size_.selected);
        }
    } else if (category_ == kBattery) {
        if (pressSegmented(reserve_, pos)) {
            emit reserveChanged(reserveChoices()[reserve_.selected]);
        }
    } else if (category_ == kOffline) {
        if (country_rect_.contains(pos)) {
            off_.running ? emit offlineCancel() : emit offlineCountry();
        } else if (area_rect_.contains(pos) && !off_.running) {
            emit offlineSelectArea();
        } else if (clear_rect_.contains(pos) && !off_.running && off_.area_bytes > 0) {
            confirm_ = Confirm::ClearDetail;
        } else if (delete_rect_.contains(pos) && !off_.running && off_.disk_bytes > 0) {
            confirm_ = Confirm::DeleteAll;
        } else if (folder_rect_.contains(pos) && !off_.running) {
            emit offlineChooseFolder();
        } else if (pressSegmented(online_, pos)) {
            off_.online = online_.selected == 1;
            emit onlineChanged(off_.online);
        }
    } else if (category_ == kMap) {
        bool changed = false;
        if (pressSegmented(trail_on_, pos)) {
            trail_len_.enabled = trail_on_.selected == 1;
            changed = true;
        } else if (pressSegmented(trail_len_, pos)) {
            changed = true;
        }
        if (changed) {
            emit trailChanged(trail_on_.selected == 1, trailChoices()[trail_len_.selected]);
        } else if (pressSegmented(labels_, pos)) {
            emit labelsEnglishChanged(labels_.selected == 1);
        }
    }
    update();
}

void SettingsScreen::paintSegmented(QPainter& p, const QRectF& track, Segmented& seg) {
    const int n = static_cast<int>(seg.labels.size());
    const double a = seg.enabled ? 1.0 : 0.35;
    p.save();
    p.setOpacity(p.opacity() * a);
    drawGlass(p, chamferPath(track, 10), kCyan, 0.5, alpha(kBg, 0.7));
    const double cw = track.width() / n;
    seg.cells.clear();
    for (int i = 0; i < n; ++i) {
        seg.cells.push_back(QRectF(track.left() + i * cw, track.top(), cw, track.height()));
    }
    const QRectF knob(track.left() + 4 + seg.knob * cw, track.top() + 4, cw - 8, track.height() - 8);
    const QColor kc = mix(kCyan, kText, 0.15 * flash_);
    drawGlass(p, chamferPath(knob, 8), kc, 1.0, alpha(kCyan, 0.22 + 0.25 * flash_));
    const QFont f = labelFont(n > 3 ? 12.5 : 15);
    for (int i = 0; i < n; ++i) {
        const double sel = 1.0 - std::min(1.0, std::abs(seg.knob - i));
        drawText(p, seg.cells[i], Qt::AlignCenter, seg.labels[i], f, mix(kMuted, kCyan, sel));
    }
    p.restore();
}

void SettingsScreen::paintRow(QPainter& p, const QRectF& r, const QString& title, const QString& sub) {
    drawText(p, QRectF(r.left(), r.top(), r.width(), 24), Qt::AlignLeft | Qt::AlignVCenter, title, labelFont(14),
             kText);
    drawText(p, QRectF(r.left(), r.top() + 24, r.width(), 18), Qt::AlignLeft | Qt::AlignVCenter, sub,
             labelFont(10, false), kMuted);
}

void SettingsScreen::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    const double e = easeOutCubic(open_);
    const QRectF r = rect();

    // ---- backdrop ---------------------------------------------------------------
    if (!backdrop_.isNull()) {
        p.setOpacity(e);
        p.drawPixmap(r.toRect(), backdrop_);
        p.setOpacity(1.0);
    }
    p.fillRect(r, alpha(kBg, 0.62 * e));
    if (!reducedMotion()) {
        drawScanlines(p, r, 0.5 * e);
    }
    // the panel itself grows with the text size (taps go through toLogical)
    p.scale(uiScale(), uiScale());

    // ---- panel: slides up as it opens ----------------------------------------------
    const QRectF pr = panelRect();
    p.save();
    p.setOpacity(e);
    p.translate(0, (1.0 - e) * 40.0);
    const QPainterPath shape = chamferPath(pr, 22);
    drawGlass(p, shape, kCyan, 0.75, alpha(kGlassDeep, 0.96));
    drawBrackets(p, pr.adjusted(-6, -6, 6, 6), 26, alpha(kCyan, 0.8), 2.0);
    p.setPen(QPen(kCyan, 2));
    p.drawLine(QPointF(pr.left() + 1, pr.top() + 22), QPointF(pr.left() + 22, pr.top() + 1));

    drawText(p, QRectF(pr.left() + 34, pr.top() + 18, 400, 34), Qt::AlignLeft | Qt::AlignVCenter,
             QStringLiteral("SETTINGS"), titleFont(26), kText);
    drawText(p, QRectF(pr.left() + 36, pr.top() + 50, 400, 16), Qt::AlignLeft | Qt::AlignVCenter,
             category_ == kLink ? QStringLiteral("link changes apply when you press connect")
                                : QStringLiteral("changes apply immediately"),
             labelFont(10), kMuted);

    close_rect_ = QRectF(pr.right() - 72, pr.top() + 16, 52, 52);
    drawGlass(p, chamferPath(close_rect_, 8), kCyan, 0.6, kGlassDeep);
    drawText(p, close_rect_, Qt::AlignCenter, QStringLiteral("✕"), monoFont(20, true), kText);

    // ---- categories ----------------------------------------------------------------
    const double top = pr.top() + 92;
    const QRectF cat_col(pr.left() + 24, top, 190, pr.bottom() - top - 24);
    p.setPen(QPen(alpha(kCyan, 0.18), 1));
    p.drawLine(QPointF(cat_col.right() + 16, cat_col.top()), QPointF(cat_col.right() + 16, cat_col.bottom()));
    const char* names[kCategories] = {"CONNECTION", "UNITS", "DISPLAY", "MAP", "BATTERY", "OFFLINE"};
    cat_rects_.clear();
    for (int i = 0; i < kCategories; ++i) {
        const QRectF item(cat_col.left(), cat_col.top() + i * 60, cat_col.width(), 52);
        cat_rects_.push_back(item);
        const double sel = 1.0 - std::min(1.0, std::abs(cat_knob_ - i));
        drawGlass(p, chamferPath(item, 8), mix(kMuted, kCyan, sel), 0.4 + 0.5 * sel, alpha(kCyan, 0.12 * sel));
        if (sel > 0.01) {
            p.setPen(Qt::NoPen);
            p.setBrush(alpha(kCyan, sel));
            p.drawRect(QRectF(item.left(), item.top() + 10, 3, item.height() - 20));
        }
        drawText(p, item.adjusted(18, 0, 0, 0), Qt::AlignLeft | Qt::AlignVCenter, QString::fromLatin1(names[i]),
                 labelFont(14), mix(kMuted, kCyan, sel));
    }

    // ---- rows of the selected category ----------------------------------------------------
    const QRectF body(cat_col.right() + 40, top, pr.right() - cat_col.right() - 64, cat_col.height());
    if (category_ == kLink) {
        paintLink(p, body);
    } else if (category_ == kUnits) {
        paintRow(p, body, QStringLiteral("ALTITUDE"),
                 QStringLiteral("HUD tape, telemetry bar, map labels, panels and steppers"));
        paintSegmented(p, QRectF(body.left(), body.top() + 50, std::min(body.width(), 360.0), 54), alt_);
        paintRow(p, body.adjusted(0, 122, 0, 0), QStringLiteral("SPEED"),
                 QStringLiteral("airspeed, ground speed and wind"));
        paintSegmented(p, QRectF(body.left(), body.top() + 172, std::min(body.width(), 360.0), 54), speed_);
        paintRow(p, body.adjusted(0, 244, 0, 0), QStringLiteral("COORDINATES"),
                 QStringLiteral("aircraft position, map cursor, camera target"));
        paintSegmented(p, QRectF(body.left(), body.top() + 294, std::min(body.width(), 480.0), 54), coord_);
        drawText(p, QRectF(body.left(), body.bottom() - 18, body.width(), 18), Qt::AlignLeft | Qt::AlignVCenter,
                 QStringLiteral("distances stay metric · UTM on WGS84"), labelFont(10, false), alpha(kMuted, 0.8));
    } else if (category_ == kOffline) {
        paintOffline(p, body);
    } else if (category_ == kDisplay) {
        paintRow(p, body, QStringLiteral("TEXT SIZE"),
                 QStringLiteral("labels, numbers and the boxes around them · map and camera keep their size"));
        paintSegmented(p, QRectF(body.left(), body.top() + 54, std::min(body.width(), 480.0), 58), text_size_);
        const QStringList notes = {
            QStringLiteral("SMALL fits the most on screen · LARGE reads at arm's length in sunlight"),
            QStringLiteral("The top bar grows only as far as it still fits the screen width")};
        for (int i = 0; i < notes.size(); ++i) {
            drawText(p, QRectF(body.left(), body.top() + 136 + i * 22, body.width(), 20),
                     Qt::AlignLeft | Qt::AlignVCenter, notes[i], labelFont(10.5, false), i == 0 ? kText : kMuted);
        }
    } else if (category_ == kBattery) {
        paintRow(p, body, QStringLiteral("LANDING RESERVE"),
                 QStringLiteral("battery that \"time left\" never counts on, on top of the return and the landing"));
        paintSegmented(p, QRectF(body.left(), body.top() + 54, body.width(), 58), reserve_);
        const QStringList notes = {
            QStringLiteral("TIME LEFT (battery chip, in flight) = how long it can keep flying the mission and still:"),
            QStringLiteral("  · fly back to L (or home) into a wind %1 % stronger than measured")
                .arg(std::lround((endurance::kWindFactor - 1.0) * 100.0)),
            QStringLiteral("  · circle down the approach, go around once, and land on the lift rotors"),
            QStringLiteral("  · with all of that x %1, and this reserve left untouched").arg(endurance::kReturnMargin),
            QStringLiteral("Warns at 5 min, says LAND NOW at 0. It never lands by itself.")};
        for (int i = 0; i < notes.size(); ++i) {
            drawText(p, QRectF(body.left(), body.top() + 136 + i * 22, body.width(), 20),
                     Qt::AlignLeft | Qt::AlignVCenter, notes[i], labelFont(10.5, false), i == 0 ? kText : kMuted);
        }
    } else {
        paintRow(p, body, QStringLiteral("FLIGHT TRAIL"), QStringLiteral("the line the aircraft leaves behind on the map"));
        paintSegmented(p, QRectF(body.left(), body.top() + 54, std::min(body.width(), 220.0), 58), trail_on_);
        const QRectF lr(body.left(), body.top() + 134, body.width(), 42);
        drawText(p, QRectF(lr.left(), lr.top(), lr.width(), 18), Qt::AlignLeft | Qt::AlignVCenter,
                 QStringLiteral("LENGTH"), labelFont(11), trail_len_.enabled ? kText : alpha(kMuted, 0.6));
        drawText(p, QRectF(lr.left(), lr.top() + 18, lr.width(), 18), Qt::AlignLeft | Qt::AlignVCenter,
                 QStringLiteral("how far back in time it reaches"), labelFont(10, false),
                 alpha(kMuted, trail_len_.enabled ? 1.0 : 0.6));
        paintSegmented(p, QRectF(body.left(), body.top() + 182, body.width(), 58), trail_len_);
        drawText(p, QRectF(body.left(), body.top() + 258, body.width(), 18), Qt::AlignLeft | Qt::AlignVCenter,
                 QStringLiteral("MAP LABELS"), labelFont(11), kText);
        drawText(p, QRectF(body.left(), body.top() + 276, body.width(), 18), Qt::AlignLeft | Qt::AlignVCenter,
                 QStringLiteral("place and street names on the offline street map"), labelFont(10, false), kMuted);
        paintSegmented(p, QRectF(body.left(), body.top() + 300, std::min(body.width(), 300.0), 52), labels_);
    }
    paintConfirm(p, pr);
    p.restore();
}

void SettingsScreen::setOffline(const Offline& o) {
    off_ = o;
    online_.selected = o.online ? 1 : 0;
    update();
}

void SettingsScreen::paintAction(QPainter& p, const QRectF& r, const QString& label, const QColor& c, bool enabled) {
    p.save();
    p.setOpacity(p.opacity() * (enabled ? 1.0 : 0.35));
    drawGlass(p, chamferPath(r, 9), c, 0.85, alpha(c, 0.16));
    drawText(p, r, Qt::AlignCenter, label, labelFont(13), c);
    p.restore();
}

void SettingsScreen::paintOffline(QPainter& p, const QRectF& body) {
    double y = body.top();
    // ---- country pack ------------------------------------------------------------------
    paintRow(p, QRectF(body.left(), y, body.width(), 44), QStringLiteral("COUNTRY PACK · ISRAEL + 50 KM"),
             QStringLiteral("street map (all zooms) · satellite to ~10 m/px · ground height · "
                            "the button only fetches what's new or missing"));
    drawText(p, QRectF(body.left(), y + 44, body.width(), 18), Qt::AlignLeft | Qt::AlignVCenter, off_.country,
             monoFont(11), off_.street_map && !off_.country.contains(QStringLiteral("missing")) ? kGreen : kAmber);
    y += 70;
    country_rect_ = QRectF(body.left(), y, 250, 50);
    const bool country_running = off_.running;
    paintAction(p, country_rect_,
                country_running ? QStringLiteral("CANCEL")
                                : (off_.street_map ? QStringLiteral("⟳ CHECK & COMPLETE") : QStringLiteral("⬇ DOWNLOAD COUNTRY")),
                country_running ? kAmber : kCyan, true);
    if (country_running) {
        const QRectF bar(country_rect_.right() + 16, y + 10, body.right() - country_rect_.right() - 16, 10);
        p.setPen(QPen(alpha(kCyan, 0.5), 1));
        p.setBrush(alpha(kBg, 0.6));
        p.drawRect(bar);
        p.setPen(Qt::NoPen);
        p.setBrush(kCyan);
        p.drawRect(QRectF(bar.left(), bar.top(), bar.width() * std::clamp(off_.fraction, 0.0, 1.0), bar.height()));
        drawText(p, QRectF(bar.left(), bar.bottom() + 4, bar.width(), 18), Qt::AlignLeft | Qt::AlignVCenter,
                 QStringLiteral("%1 · %2%").arg(off_.stage).arg(static_cast<int>(off_.fraction * 100)), monoFont(10.5),
                 kText);
    } else if (!off_.status.isEmpty()) {
        drawText(p, QRectF(country_rect_.right() + 16, y, body.right() - country_rect_.right() - 16, 50),
                 Qt::AlignLeft | Qt::AlignVCenter | Qt::TextWordWrap, off_.status, labelFont(10, false), kMuted);
    }
    y += 74;
    // ---- full-detail areas -------------------------------------------------------------------
    paintRow(p, QRectF(body.left(), y, body.width(), 44), QStringLiteral("FULL-DETAIL AREAS"),
             off_.areas > 0 ? QStringLiteral("%1 area%2 · %3 · overlapping areas share their tiles")
                                  .arg(off_.areas)
                                  .arg(off_.areas == 1 ? "" : "s")
                                  .arg(sizeText(off_.area_bytes))
                            : QStringLiteral("draw a rectangle on the map: satellite to ~0.5 m/px there"));
    y += 52;
    area_rect_ = QRectF(body.left(), y, 250, 50);
    paintAction(p, area_rect_, QStringLiteral("▭ SELECT AREA ON MAP"), kCyan, !off_.running);
    clear_rect_ = QRectF(area_rect_.right() + 12, y, 220, 50);
    paintAction(p, clear_rect_, QStringLiteral("CLEAR FULL DETAIL"), kRed, !off_.running && off_.area_bytes > 0);
    y += 76;
    // ---- online + folder ----------------------------------------------------------------------
    drawText(p, QRectF(body.left(), y, 260, 18), Qt::AlignLeft | Qt::AlignVCenter, QStringLiteral("ONLINE TILES"),
             labelFont(11), kText);
    drawText(p, QRectF(body.left(), y + 18, 300, 16), Qt::AlignLeft | Qt::AlignVCenter,
             QStringLiteral("fetch missing satellite while browsing"), labelFont(10, false), kMuted);
    paintSegmented(p, QRectF(body.left(), y + 38, 200, 46), online_);
    delete_rect_ = QRectF(body.right() - 170, y + 38, 170, 46);
    paintAction(p, delete_rect_, QStringLiteral("DELETE ALL"), kRed, !off_.running && off_.disk_bytes > 0);
    y += 100;
    drawText(p, QRectF(body.left(), y, body.width(), 18), Qt::AlignLeft | Qt::AlignVCenter,
             QStringLiteral("FOLDER · %1 on disk · copy it to use the maps on another computer")
                 .arg(sizeText(off_.disk_bytes)),
             labelFont(11), kText);
    folder_rect_ = QRectF(body.left(), y + 22, body.width(), 44);
    drawGlass(p, chamferPath(folder_rect_, 7), kMuted, 0.4, alpha(kBg, 0.6));
    const QFont ff = monoFont(10.5);
    drawText(p, folder_rect_.adjusted(12, 0, -90, 0), Qt::AlignLeft | Qt::AlignVCenter,
             QFontMetricsF(ff).elidedText(off_.folder, Qt::ElideMiddle, folder_rect_.width() - 110), ff, kMuted);
    drawText(p, folder_rect_.adjusted(0, 0, -12, 0), Qt::AlignRight | Qt::AlignVCenter, QStringLiteral("CHANGE"),
             labelFont(11), off_.running ? alpha(kMuted, 0.4) : kCyan);
}

void SettingsScreen::paintConfirm(QPainter& p, const QRectF& panel) {
    if (confirm_ == Confirm::None) {
        confirm_yes_ = confirm_no_ = QRectF();
        return;
    }
    p.fillRect(panel, alpha(kBg, 0.75));
    const QRectF box(panel.center().x() - 260, panel.center().y() - 110, 520, 220);
    const bool relink = confirm_ == Confirm::Relink;
    const QColor tone = relink ? kAmber : kRed;
    drawGlass(p, chamferPath(box, 14), tone, 0.9, alpha(kGlassDeep, 0.98));
    drawText(p, QRectF(box.left() + 28, box.top() + 20, box.width() - 56, 30), Qt::AlignLeft | Qt::AlignVCenter,
             relink ? QStringLiteral("SWITCH LINK IN FLIGHT?") : QStringLiteral("ARE YOU SURE?"), titleFont(20), tone);
    const QString text =
        relink ? QStringLiteral("The aircraft is armed. Telemetry and control stop until it answers on the new link. "
                                "It keeps flying what it is doing; a long silence triggers its GCS failsafe.")
        : confirm_ == Confirm::ClearDetail
            ? QStringLiteral("Delete all full-detail satellite and ground height (%1)? The country map, the street map "
                             "and the country satellite stay.")
                  .arg(sizeText(off_.area_bytes))
            : QStringLiteral("Delete everything in the offline folder (%1): the street map, all satellite and ground "
                             "height? Without internet the map will be empty until you download again.")
                  .arg(sizeText(off_.disk_bytes));
    drawText(p, QRectF(box.left() + 28, box.top() + 58, box.width() - 56, 70), Qt::AlignLeft | Qt::AlignTop | Qt::TextWordWrap,
             text, labelFont(11.5, false), kText);
    confirm_no_ = QRectF(box.left() + 28, box.bottom() - 74, 200, 52);
    confirm_yes_ = QRectF(box.right() - 228, box.bottom() - 74, 200, 52);
    paintAction(p, confirm_no_, QStringLiteral("CANCEL"), kCyan, true);
    paintAction(p, confirm_yes_, relink ? QStringLiteral("SWITCH") : QStringLiteral("DELETE"), tone, true);
}

// ---- CONNECTION ------------------------------------------------------------------------------

void SettingsScreen::setLink(const Link& l) {
    link_ = l;
    link_type_.selected = l.serial ? 1 : 0;
    const auto& bauds = link::serialBauds();
    const auto it = std::find(bauds.begin(), bauds.end(), l.baud);
    baud_.selected = it != bauds.end() ? static_cast<int>(it - bauds.begin()) : 0;
    update();
}

void SettingsScreen::setLinkStatus(const QString& text, const QColor& color, bool armed) {
    if (text != link_status_ || color != link_status_color_ || armed != armed_) {
        link_status_ = text;
        link_status_color_ = color;
        armed_ = armed;
        if (category_ == kLink && isVisible()) {
            update();
        }
    }
}

void SettingsScreen::refreshPorts() {
    ports_ = link::listSerialPorts();
    ports_age_ = 0.0;
    // nothing chosen yet: take the likeliest autopilot
    if (link_.serial_port.isEmpty() && !ports_.empty()) {
        link_.serial_port = QString::fromStdString(ports_.front().path);
    }
}

bool SettingsScreen::pressLink(const QPointF& pos) {
    if (pressSegmented(link_type_, pos)) {
        link_.serial = link_type_.selected == 1;
        if (link_.serial) {
            refreshPorts();
        }
        return true;
    }
    if (connect_rect_.contains(pos)) {
        if (armed_) {
            confirm_ = Confirm::Relink;
        } else {
            applied_ = link_;
            emit linkConnect(link_);
        }
        return true;
    }
    if (video_rect_.contains(pos)) {
        emit linkEdit(kVideoField);
        return true;
    }
    if (!link_.serial) {
        if (listen_rect_.contains(pos)) {
            emit linkEdit(kListenField);
            return true;
        }
        if (sendto_rect_.contains(pos)) {
            emit linkEdit(kSendToField);
            return true;
        }
        return false;
    }
    if (refresh_rect_.contains(pos)) {
        refreshPorts();
        flash_ = 1.0;
        return true;
    }
    for (size_t i = 0; i < port_rects_.size() && i < ports_.size(); ++i) {
        if (port_rects_[i].contains(pos)) {
            link_.serial_port = QString::fromStdString(ports_[i].path);
            flash_ = 1.0;
            return true;
        }
    }
    if (pressSegmented(baud_, pos)) {
        link_.baud = link::serialBauds()[baud_.selected];
        return true;
    }
    return false;
}

void SettingsScreen::paintField(QPainter& p, const QRectF& r, const QString& name, const QString& value,
                                const QString& note) {
    drawGlass(p, chamferPath(r, 8), kCyan, 0.45, alpha(kBg, 0.6));
    drawText(p, QRectF(r.left() + 16, r.top(), 130, r.height()), Qt::AlignLeft | Qt::AlignVCenter, name,
             labelFont(12), kMuted);
    const QFont vf = monoFont(15, true);
    const double vx = r.left() + 150;
    drawText(p, QRectF(vx, r.top() + 4, r.width() - 230, 26), Qt::AlignLeft | Qt::AlignVCenter, value, vf, kText);
    drawText(p, QRectF(vx, r.top() + 28, r.width() - 230, 18), Qt::AlignLeft | Qt::AlignVCenter, note,
             labelFont(9.5, false), kMuted);
    drawText(p, r.adjusted(0, 0, -16, 0), Qt::AlignRight | Qt::AlignVCenter, QStringLiteral("EDIT"), labelFont(12),
             kCyan);
}

void SettingsScreen::paintLink(QPainter& p, const QRectF& body) {
    paintRow(p, body, QStringLiteral("LINK"),
             QStringLiteral("how this computer reaches the aircraft · applied when you press CONNECT"));
    paintSegmented(p, QRectF(body.left(), body.top() + 48, std::min(body.width(), 540.0), 52), link_type_);
    double y = body.top() + 116;
    const double fh = 54;
    listen_rect_ = sendto_rect_ = refresh_rect_ = QRectF();
    port_rects_.clear();
    if (!link_.serial) {
        listen_rect_ = QRectF(body.left(), y, body.width(), fh);
        const QString where = link_.host == QStringLiteral("0.0.0.0")      ? QStringLiteral("all networks of this computer")
                              : link_.host.startsWith(QStringLiteral("127.")) ? QStringLiteral("this computer only (simulator)")
                                                                              : QStringLiteral("this address only");
        paintField(p, listen_rect_, QStringLiteral("LISTEN ON"), QStringLiteral("%1 : %2").arg(link_.host).arg(link_.port),
                   where + QStringLiteral(" · telemetry port"));
        y += fh + 10;
        sendto_rect_ = QRectF(body.left(), y, body.width(), fh);
        paintField(p, sendto_rect_, QStringLiteral("SEND TO"),
                   link_.remote_host.isEmpty() ? QStringLiteral("—")
                                               : QStringLiteral("%1 : %2").arg(link_.remote_host).arg(link_.remote_port),
                   link_.remote_host.isEmpty() ? QStringLiteral("off: answers whoever sends telemetry (usual)")
                                               : QStringLiteral("for radios that wait for the GCS to talk first"));
        y += fh + 10;
    } else {
        drawText(p, QRectF(body.left(), y, 200, 20), Qt::AlignLeft | Qt::AlignVCenter, QStringLiteral("PORT"),
                 labelFont(12), kText);
        refresh_rect_ = QRectF(body.right() - 130, y - 8, 130, 36);
        paintAction(p, refresh_rect_, QStringLiteral("⟳ REFRESH"), kCyan, true);
        y += 34;
        const int rows = 3;
        const double rh = 40;
        if (ports_.empty()) {
            const QRectF r(body.left(), y, body.width(), rh * 2);
            drawGlass(p, chamferPath(r, 8), kMuted, 0.35, alpha(kBg, 0.5));
            drawText(p, r.adjusted(16, 0, -16, 0), Qt::AlignLeft | Qt::AlignVCenter | Qt::TextWordWrap,
                     QStringLiteral("No serial ports found. Plug in the Pixhawk (USB) or the telemetry radio; "
                                    "the list updates by itself."),
                     labelFont(11, false), kMuted);
        }
        for (int i = 0; i < rows && i < static_cast<int>(ports_.size()); ++i) {
            const QRectF r(body.left(), y + i * (rh + 6), body.width(), rh);
            port_rects_.push_back(r);
            const auto& port = ports_[static_cast<size_t>(i)];
            const bool sel = QString::fromStdString(port.path) == link_.serial_port;
            drawGlass(p, chamferPath(r, 8), sel ? kCyan : kMuted, sel ? 0.9 : 0.35,
                      alpha(sel ? kCyan : kBg, sel ? 0.16 : 0.5));
            drawText(p, r.adjusted(16, 0, -16, 0), Qt::AlignLeft | Qt::AlignVCenter,
                     QString::fromStdString(port.label), labelFont(12.5, sel), sel ? kCyan : kText);
            drawText(p, r.adjusted(16, 0, -16, 0), Qt::AlignRight | Qt::AlignVCenter,
                     port.likely_autopilot ? QStringLiteral("AUTOPILOT") : QString::fromStdString(port.path),
                     monoFont(10), port.likely_autopilot ? kGreen : kMuted);
        }
        // the chosen port while it is unplugged
        const bool chosen_listed =
            std::any_of(ports_.begin(), ports_.end(),
                        [&](const link::SerialPortInfo& pi) { return QString::fromStdString(pi.path) == link_.serial_port; });
        y += rows * (rh + 6);
        if (!chosen_listed && !link_.serial_port.isEmpty()) {
            drawText(p, QRectF(body.left(), y - 4, body.width(), 18), Qt::AlignLeft | Qt::AlignVCenter,
                     QStringLiteral("chosen: %1 (not plugged in)").arg(link_.serial_port), labelFont(10, false), kAmber);
        }
        y += 16;
        drawText(p, QRectF(body.left(), y, 200, 18), Qt::AlignLeft | Qt::AlignVCenter, QStringLiteral("SPEED (BAUD)"),
                 labelFont(12), kText);
        drawText(p, QRectF(body.left() + 140, y, body.width() - 140, 18), Qt::AlignLeft | Qt::AlignVCenter,
                 QStringLiteral("SiK radio: 57600 · USB cable: any (ignored)"), labelFont(10, false), kMuted);
        paintSegmented(p, QRectF(body.left(), y + 24, body.width(), 46), baud_);
        y += 82;
    }
    video_rect_ = QRectF(body.left(), y, body.width(), fh);
    paintField(p, video_rect_, QStringLiteral("VIDEO PORT"), QString::number(link_.video_port),
               QStringLiteral("UDP, RTP / H.264 from the camera"));

    // ---- connect + status ----------------------------------------------------------------
    connect_rect_ = QRectF(body.left(), body.bottom() - 54, 220, 52);
    const bool dirty = link_ != applied_;
    const QColor cc = dirty ? kAmber : kCyan;
    paintAction(p, connect_rect_, dirty ? QStringLiteral("● CONNECT") : QStringLiteral("RECONNECT"), cc, true);
    const QRectF st(connect_rect_.right() + 18, connect_rect_.top(), body.right() - connect_rect_.right() - 18, 52);
    drawText(p, st, Qt::AlignLeft | Qt::AlignVCenter | Qt::TextWordWrap,
             dirty ? QStringLiteral("changed: press CONNECT to use it") : link_status_, labelFont(11, false),
             dirty ? kAmber : link_status_color_);
}

}  // namespace gcs::ui
