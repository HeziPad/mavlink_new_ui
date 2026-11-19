#include "hud_panel.hpp"

#include "camera_view.hpp"
#include "theme.hpp"

#include <QFontMetricsF>
#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QWheelEvent>

#include <algorithm>
#include <cmath>

namespace gcs::ui {

using namespace theme;

namespace {

constexpr double kWidth = 440.0;   // logical px
constexpr double kPad = 16.0;
constexpr double kHeader = 68.0;   // title + close, not scrolled
constexpr double kGap = 8.0;
constexpr double kLabelH = 20.0;   // a section label row

constexpr double kPresetY = 0.0, kPresetH = 50.0;
constexpr double kShowY = kPresetY + kLabelH + kPresetH + 12.0;
constexpr int kShowCols = 3;
constexpr double kTileH = 54.0;
constexpr int kShowRows = (hud::kElements + kShowCols - 1) / kShowCols;
constexpr double kCrossY = kShowY + kLabelH + kShowRows * (kTileH + kGap) + 4.0;
constexpr int kCrossCols = 4;
constexpr double kCrossH = 74.0;
constexpr int kCrossRows = (hud::kCrosshairs + kCrossCols - 1) / kCrossCols;
constexpr double kLineY = kCrossY + kLabelH + kCrossRows * (kCrossH + kGap) + 4.0;
constexpr double kLineH = 48.0;
constexpr double kColorY = kLineY + kLabelH + kLineH + 12.0;
constexpr double kColorH = 52.0;
constexpr double kContentH = kColorY + kLabelH + kColorH + kPad;

const char* elementName(hud::Element e) {
    switch (e) {
    case hud::Element::Ladder: return "PITCH LADDER";
    case hud::Element::RollScale: return "ROLL ARC";
    case hud::Element::HeadingTape: return "HEADING";
    case hud::Element::SpeedTape: return "SPEED TAPE";
    case hud::Element::AltTape: return "ALT TAPE";
    case hud::Element::PathMarker: return "FLIGHT PATH";
    case hud::Element::StatusLine: return "STATUS LINE";
    case hud::Element::TargetBox: return "TARGET";
    case hud::Element::AircraftBox: return "AIRCRAFT";
    case hud::Element::LosBox: return "CAMERA LOS";
    case hud::Element::WindBox: return "WIND";
    case hud::Element::RefChip: return "RULER REF";
    case hud::Element::SensorFx: return "SENSOR FX";
    case hud::Element::Count: break;
    }
    return "";
}

const char* crossName(hud::Crosshair c) {
    switch (c) {
    case hud::Crosshair::Cross: return "CROSS";
    case hud::Crosshair::Dot: return "DOT";
    case hud::Crosshair::CircleDot: return "RING";
    case hud::Crosshair::Duplex: return "SCOPE";
    case hud::Crosshair::Brackets: return "BOX";
    case hud::Crosshair::Fine: return "FINE";
    case hud::Crosshair::Measure: return "MEASURE";
    case hud::Crosshair::None: return "NONE";
    case hud::Crosshair::Count: break;
    }
    return "";
}

// a small line drawing of each element, in `r` (about 28 x 22)
void elementIcon(QPainter& p, const QRectF& r, hud::Element e, const QColor& c) {
    p.save();
    p.setPen(QPen(c, 1.4, Qt::SolidLine, Qt::RoundCap));
    p.setBrush(Qt::NoBrush);
    const QPointF m = r.center();
    const double w = r.width(), h = r.height();
    switch (e) {
    case hud::Element::Ladder:
        p.drawLine(QPointF(r.left(), m.y()), QPointF(r.right(), m.y()));
        p.setPen(QPen(c, 1.2, Qt::DashLine));
        p.drawLine(QPointF(m.x() - w * 0.3, r.top() + 2), QPointF(m.x() + w * 0.3, r.top() + 2));
        p.drawLine(QPointF(m.x() - w * 0.3, r.bottom() - 2), QPointF(m.x() + w * 0.3, r.bottom() - 2));
        break;
    case hud::Element::RollScale:
        p.drawArc(QRectF(m.x() - w * 0.45, r.top() + 2, w * 0.9, h * 1.6), 30 * 16, 120 * 16);
        p.drawLine(QPointF(m.x(), r.top() + 2), QPointF(m.x(), r.top() + 7));
        break;
    case hud::Element::HeadingTape:
        p.drawLine(QPointF(r.left(), r.bottom() - 3), QPointF(r.right(), r.bottom() - 3));
        for (int i = 0; i < 5; ++i) {
            const double x = r.left() + 2 + i * (w - 4) / 4.0;
            p.drawLine(QPointF(x, r.bottom() - 3), QPointF(x, r.bottom() - (i % 2 ? 7 : 11)));
        }
        break;
    case hud::Element::SpeedTape:
    case hud::Element::AltTape: {
        const bool left = e == hud::Element::SpeedTape;
        const double x = left ? r.right() - 6 : r.left() + 6;
        p.drawLine(QPointF(x, r.top()), QPointF(x, r.bottom()));
        for (int i = 0; i < 4; ++i) {
            const double y = r.top() + 2 + i * (h - 4) / 3.0;
            p.drawLine(QPointF(x, y), QPointF(x + (left ? -1 : 1) * (i % 2 ? 4 : 8), y));
        }
        break;
    }
    case hud::Element::PathMarker:
        p.drawEllipse(m, 4, 4);
        p.drawLine(m + QPointF(-12, 0), m + QPointF(-4, 0));
        p.drawLine(m + QPointF(4, 0), m + QPointF(12, 0));
        p.drawLine(m + QPointF(0, -4), m + QPointF(0, -9));
        break;
    case hud::Element::StatusLine:
        for (int i = 0; i < 3; ++i) {
            const double x = r.left() + i * w / 3.0;
            p.drawLine(QPointF(x + 1, m.y()), QPointF(x + w / 3.0 - 3, m.y()));
        }
        break;
    case hud::Element::TargetBox:
    case hud::Element::AircraftBox:
    case hud::Element::LosBox:
        p.drawRect(r.adjusted(1, 2, -1, -2));
        if (e == hud::Element::TargetBox) {
            p.drawLine(QPointF(m.x() - 5, m.y()), QPointF(m.x() + 5, m.y()));
            p.drawLine(QPointF(m.x(), m.y() - 5), QPointF(m.x(), m.y() + 5));
        } else if (e == hud::Element::AircraftBox) {
            p.drawEllipse(m, 6, 6);
            p.drawLine(m + QPointF(-6, 0), m + QPointF(6, 0));
        } else {
            p.drawRect(QRectF(m.x() - 7, m.y() - 4, 10, 8));
            p.drawLine(QPointF(m.x() + 3, m.y()), QPointF(m.x() + 8, m.y() - 3));
            p.drawLine(QPointF(m.x() + 3, m.y()), QPointF(m.x() + 8, m.y() + 3));
        }
        break;
    case hud::Element::WindBox:
        p.drawEllipse(m, 9, 9);
        p.drawLine(m + QPointF(-5, -5), m + QPointF(5, 5));
        p.drawLine(m + QPointF(5, 5), m + QPointF(5, 0));
        p.drawLine(m + QPointF(5, 5), m + QPointF(0, 5));
        break;
    case hud::Element::RefChip:
        p.drawRoundedRect(QRectF(m.x() - 11, m.y() - 5, 22, 10), 5, 5);
        p.setBrush(c);
        p.drawEllipse(QPointF(m.x() + 6, m.y()), 3, 3);
        break;
    case hud::Element::SensorFx:
        for (int i = 0; i < 5; ++i) {
            const double y = r.top() + 2 + i * (h - 4) / 4.0;
            p.setPen(QPen(alpha(c, 0.4 + 0.15 * i), 1.0));
            p.drawLine(QPointF(r.left(), y), QPointF(r.right(), y));
        }
        break;
    case hud::Element::Count:
        break;
    }
    p.restore();
}

}  // namespace

HudPanel::HudPanel(QWidget* parent) : QWidget(parent) {
    hide();
}

void HudPanel::setConfig(const hud::Config& c) {
    cfg_ = c;
    update();
}

QRectF HudPanel::logicalRect() const {
    return {QPointF(0, 0), logicalSize(size())};
}

double HudPanel::viewTop() const {
    return kHeader;
}

double HudPanel::contentHeight() const {
    return kContentH;
}

void HudPanel::place(const QRect& area) {
    area_ = area;
    const double k = uiScale();
    const double w = std::min(kWidth, area.width() / k - 16.0);
    const double h = std::min(kHeader + kContentH + 6.0, area.height() / k);
    setGeometry(static_cast<int>(area.right() - w * k), area.top(), static_cast<int>(w * k), static_cast<int>(h * k));
    clampScroll();
}

void HudPanel::clampScroll() {
    const double view = logicalRect().height() - viewTop() - 6.0;
    scroll_ = std::clamp(scroll_, 0.0, std::max(0.0, contentHeight() - view));
}

void HudPanel::tick(double dt) {
    if (!isVisible()) {
        return;
    }
    time_ += dt;
    flash_ = std::max(0.0, flash_ - dt * 2.5);
    update();
}

void HudPanel::layoutHits(double width) {
    hits_.clear();
    const double inner = width - 2 * kPad;
    // presets
    const double pw = (inner - 2 * kGap) / 3.0;
    for (int i = 0; i < 3; ++i) {
        hits_.push_back({QRectF(kPad + i * (pw + kGap), kPresetY + kLabelH, pw, kPresetH), kPreset, i});
    }
    // elements
    const double tw = (inner - (kShowCols - 1) * kGap) / kShowCols;
    for (int i = 0; i < hud::kElements; ++i) {
        const int row = i / kShowCols, col = i % kShowCols;
        hits_.push_back({QRectF(kPad + col * (tw + kGap), kShowY + kLabelH + row * (kTileH + kGap), tw, kTileH),
                         kElement, i});
    }
    // crosshairs
    const double cw = (inner - (kCrossCols - 1) * kGap) / kCrossCols;
    for (int i = 0; i < hud::kCrosshairs; ++i) {
        const int row = i / kCrossCols, col = i % kCrossCols;
        hits_.push_back({QRectF(kPad + col * (cw + kGap), kCrossY + kLabelH + row * (kCrossH + kGap), cw, kCrossH),
                         kCross, i});
    }
    // line width: one segmented control
    const double lwid = inner / hud::kLineWidths;
    for (int i = 0; i < hud::kLineWidths; ++i) {
        hits_.push_back({QRectF(kPad + i * lwid, kLineY + kLabelH, lwid, kLineH), kLine, i});
    }
    // colour swatches
    const double sw = inner / hud::kColors;
    for (int i = 0; i < hud::kColors; ++i) {
        hits_.push_back({QRectF(kPad + i * sw, kColorY + kLabelH, sw, kColorH), kColor, i});
    }
}

void HudPanel::press(const Hit& h) {
    switch (h.kind) {
    case kPreset:
        cfg_ = hud::presetConfig(static_cast<hud::Preset>(h.index), cfg_);
        break;
    case kElement: {
        const auto e = static_cast<hud::Element>(h.index);
        cfg_.setShown(e, !cfg_.shows(e));
        break;
    }
    case kCross:
        cfg_.crosshair = static_cast<hud::Crosshair>(h.index);
        break;
    case kLine:
        cfg_.line = h.index;
        break;
    case kColor:
        cfg_.color = h.index;
        break;
    default:
        return;
    }
    flash_kind_ = h.kind;
    flash_index_ = h.index;
    flash_ = 1.0;
    emit configChanged();
    update();
}

void HudPanel::mousePressEvent(QMouseEvent* e) {
    pressing_ = true;
    dragged_ = false;
    press_pos_ = toLogical(e->position());
    press_scroll_ = scroll_;
}

void HudPanel::mouseMoveEvent(QMouseEvent* e) {
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

void HudPanel::mouseReleaseEvent(QMouseEvent* e) {
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
    if (pos.y() < viewTop()) {
        return;
    }
    const QPointF content(pos.x(), pos.y() - viewTop() + scroll_);
    layoutHits(lr.width());
    for (const Hit& h : hits_) {
        if (h.rect.contains(content)) {
            press(h);
            return;
        }
    }
}

void HudPanel::wheelEvent(QWheelEvent* e) {
    scroll_ -= e->angleDelta().y() / 2.0;
    clampScroll();
    update();
}

void HudPanel::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    p.scale(uiScale(), uiScale());  // text size: the whole panel grows
    const QRectF lr = logicalRect();
    const QRectF r = lr.adjusted(1, 1, -1, -1);
    const QColor hud = CameraView::swatch(cfg_.color);
    drawGlass(p, chamferPath(r, 16), kCyan, 0.6, alpha(kGlassDeep, 0.94));
    p.setPen(QPen(kCyan, 2));
    p.drawLine(QPointF(r.left() + 1, r.top() + 16), QPointF(r.left() + 16, r.top() + 1));

    // ---- header -----------------------------------------------------------------
    drawText(p, QRectF(r.left() + 22, r.top() + 12, 300, 28), Qt::AlignLeft | Qt::AlignVCenter,
             QStringLiteral("HUD LOADOUT"), titleFont(18), kText);
    drawText(p, QRectF(r.left() + 24, r.top() + 40, 320, 16), Qt::AlignLeft | Qt::AlignVCenter,
             QStringLiteral("changes show live on the video"), labelFont(9.5, false), kMuted);
    const QRectF close(lr.right() - 62, 10, 50, 50);
    drawGlass(p, chamferPath(close.adjusted(3, 3, -3, -3), 7), kCyan, 0.55, kGlassDeep);
    drawText(p, close, Qt::AlignCenter, QStringLiteral("✕"), monoFont(18, true), kText);

    // ---- scrolling content ----------------------------------------------------------
    layoutHits(lr.width());
    p.save();
    p.setClipRect(QRectF(0, viewTop(), lr.width(), lr.height() - viewTop() - 4));
    p.translate(0, viewTop() - scroll_);

    const hud::Preset preset = hud::matchingPreset(cfg_);
    auto label = [&](double y, const QString& text, const QString& right = {}, const QColor& rc = kMuted) {
        drawText(p, QRectF(kPad + 2, y, lr.width() - 2 * kPad, kLabelH - 4), Qt::AlignLeft | Qt::AlignVCenter, text,
                 labelFont(10.5), alpha(kMuted, 0.95));
        if (!right.isEmpty()) {
            drawText(p, QRectF(kPad, y, lr.width() - 2 * kPad - 2, kLabelH - 4), Qt::AlignRight | Qt::AlignVCenter,
                     right, labelFont(10.5), rc);
        }
    };
    auto flashOf = [&](int kind, int index) { return flash_kind_ == kind && flash_index_ == index ? flash_ : 0.0; };
    label(kPresetY, QStringLiteral("PRESET"),
          preset == hud::Preset::Custom ? QStringLiteral("CUSTOM · your own mix") : QString(), kAmber);
    label(kShowY, QStringLiteral("SHOW ON THE VIDEO"));
    label(kCrossY, QStringLiteral("CROSSHAIR"),
          cfg_.crosshair == hud::Crosshair::Measure ? QStringLiteral("drag the corners to fit the target") : QString(),
          alpha(hud, 0.9));
    label(kLineY, QStringLiteral("LINE WIDTH"));
    label(kColorY, QStringLiteral("COLOUR"));

    for (const Hit& h : hits_) {
        const QRectF& t = h.rect;
        const double fl = flashOf(h.kind, h.index);
        switch (h.kind) {
        case kPreset: {
            const bool on = static_cast<int>(preset) == h.index;
            drawGlass(p, chamferPath(t, 9), on ? kCyan : kMuted, on ? 0.95 : 0.4,
                      on ? alpha(kCyan, 0.18 + 0.2 * fl) : alpha(kBg, 0.55));
            drawText(p, t, Qt::AlignCenter, QString::fromLatin1(hud::presetName(static_cast<hud::Preset>(h.index))),
                     labelFont(12.5), on ? kCyan : kText);
            break;
        }
        case kElement: {
            const auto e = static_cast<hud::Element>(h.index);
            const bool on = cfg_.shows(e);
            const QColor c = on ? hud : alpha(kMuted, 0.8);
            drawGlass(p, chamferPath(t, 7), c, on ? 0.85 : 0.3, on ? alpha(hud, 0.12 + 0.25 * fl) : alpha(kBg, 0.5));
            elementIcon(p, QRectF(t.left() + 8, t.center().y() - 11, 28, 22), e, c);
            drawText(p, QRectF(t.left() + 42, t.top(), t.width() - 46, t.height()), Qt::AlignLeft | Qt::AlignVCenter |
                     Qt::TextWordWrap, QString::fromLatin1(elementName(e)), labelFont(9.5), on ? kText : alpha(kMuted, 0.9));
            // the on light
            p.setPen(Qt::NoPen);
            p.setBrush(on ? hud : alpha(kMuted, 0.35));
            p.drawEllipse(QPointF(t.right() - 9, t.top() + 9), 3, 3);
            break;
        }
        case kCross: {
            const auto c = static_cast<hud::Crosshair>(h.index);
            const bool on = cfg_.crosshair == c;
            drawGlass(p, chamferPath(t, 8), on ? hud : kMuted, on ? 0.95 : 0.35,
                      on ? alpha(hud, 0.14 + 0.25 * fl) : alpha(kBg, 0.6));
            const QRectF pv(t.left() + 4, t.top() + 4, t.width() - 8, t.height() - 24);
            p.save();
            p.setClipRect(pv);
            CameraView::drawReticle(p, pv.center(), c, on ? hud : alpha(hud, 0.7), cfg_.lineFactor(),
                                    c == hud::Crosshair::Duplex ? 0.33 : 0.62, pv);
            p.restore();
            drawText(p, QRectF(t.left(), t.bottom() - 20, t.width(), 16), Qt::AlignCenter,
                     QString::fromLatin1(crossName(c)), labelFont(9), on ? hud : alpha(kMuted, 0.95));
            break;
        }
        case kLine: {
            const bool on = cfg_.line == h.index;
            drawGlass(p, chamferPath(t.adjusted(1, 0, -1, 0), 4), on ? hud : kMuted, on ? 0.9 : 0.35, on ? alpha(hud, 0.16 + 0.25 * fl) : alpha(kBg, 0.55));
            const double f = h.index == 0 ? 0.7 : (h.index == 2 ? 1.5 : 1.0);
            p.setPen(QPen(on ? hud : alpha(kMuted, 0.9), 2.0 * f, Qt::SolidLine, Qt::RoundCap));
            p.drawLine(QPointF(t.left() + 16, t.center().y() - 6), QPointF(t.right() - 16, t.center().y() - 6));
            drawText(p, QRectF(t.left(), t.center().y() + 2, t.width(), 16), Qt::AlignCenter,
                     h.index == 0 ? QStringLiteral("THIN") : (h.index == 2 ? QStringLiteral("BOLD") : QStringLiteral("NORMAL")),
                     labelFont(9.5), on ? hud : alpha(kMuted, 0.95));
            break;
        }
        case kColor: {
            const QColor sc = CameraView::swatch(h.index);
            const bool on = cfg_.color == h.index;
            const QPointF m = t.center();
            const double rad = on ? 17.0 : 13.0;
            if (on) {
                // a pulsing halo around the chosen colour
                const double pulse = reducedMotion() ? 0.6 : 0.45 + 0.25 * std::sin(time_ * 4.0);
                p.setPen(QPen(alpha(sc, pulse), 6.0));
                p.setBrush(Qt::NoBrush);
                p.drawEllipse(m, rad + 4, rad + 4);
            }
            p.setPen(QPen(on ? kText : alpha(sc, 0.6), on ? 2.0 : 1.0));
            p.setBrush(alpha(sc, on ? 0.95 : 0.75));
            p.drawEllipse(m, rad + 2 * fl, rad + 2 * fl);
            break;
        }
        default:
            break;
        }
    }
    p.restore();

    // scroll indicator
    const double view = lr.height() - viewTop() - 6.0;
    if (contentHeight() > view + 1) {
        const double frac = view / contentHeight();
        const double th = std::max(30.0, view * frac);
        const double ty = viewTop() + (view - th) * (scroll_ / std::max(1.0, contentHeight() - view));
        p.setPen(Qt::NoPen);
        p.setBrush(alpha(kCyan, 0.45));
        p.drawRoundedRect(QRectF(lr.right() - 6, ty, 3, th), 1.5, 1.5);
    }
}

}  // namespace gcs::ui
