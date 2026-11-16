#include "map_view.hpp"

#include "core/geo.hpp"
#include "theme.hpp"
#include "tile_cache.hpp"

#include <QFontMetricsF>
#include <QMouseEvent>
#include <QPainter>
#include <QWheelEvent>

#include <algorithm>
#include <cmath>

namespace gcs::ui {

using namespace theme;

namespace {

constexpr double kTile = 256.0;
constexpr double kMaxZoom = 19.5;
constexpr double kMinZoom = 2.0;
constexpr double kHitRadius = 15.0;
constexpr size_t kUndoDepth = 64;

QPointF worldPx(double lat, double lon, double zoom) {
    const double n = kTile * std::exp2(zoom);
    const double x = (lon + 180.0) / 360.0 * n;
    const double s = std::sin(std::clamp(lat, -85.05, 85.05) * geo::kDegToRad);
    const double y = (0.5 - std::log((1.0 + s) / (1.0 - s)) / (4.0 * geo::kPi)) * n;
    return {x, y};
}

void worldToLatLon(const QPointF& p, double zoom, double& lat, double& lon) {
    const double n = kTile * std::exp2(zoom);
    lon = p.x() / n * 360.0 - 180.0;
    const double yy = geo::kPi * (1.0 - 2.0 * p.y() / n);
    lat = std::atan(std::sinh(yy)) * geo::kRadToDeg;
}

QString formatDistance(double m) {
    if (m >= 1000.0) {
        return QStringLiteral("%1 km").arg(m / 1000.0, 0, 'f', m >= 10000.0 ? 1 : 2);
    }
    return QStringLiteral("%1 m").arg(m, 0, 'f', 0);
}

QColor vtolColor(VtolState v) {
    switch (v) {
    case VtolState::Cruise: return kGreen;
    case VtolState::TransitionToCruise:
    case VtolState::TransitionToHover: return kAmber;
    default: return kCyan;
    }
}

}  // namespace

MapView::MapView(TileCache* tiles, QWidget* parent) : HudView(parent), tiles_(tiles) {
    connect(tiles_, &TileCache::tileReady, this, qOverload<>(&QWidget::update));
    heading_.rate = 10.0;
    updateCursor();
}

QString MapView::pipTag() const {
    return QStringLiteral("Map · %1").arg(TileCache::layerName(layer_));
}

void MapView::setState(const VehicleState& s) {
    s_ = s;
    vehicle_wps_ = s.mission_known ? extractUserWaypoints(s.mission) : std::vector<UserWaypoint>{};
    if (s.has_attitude) {
        heading_.set(geo::wrap360(s.yaw_deg));
    } else if (s.has_vfr_hud) {
        heading_.set(s.heading_deg);
    }
    if (!centred_) {
        if (s.has_position && (s.lat != 0.0 || s.lon != 0.0)) {
            clat_ = s.lat;
            clon_ = s.lon;
            zoom_ = zoom_target_ = 16.0;
            centred_ = true;
        } else if (s.has_home) {
            clat_ = s.home_lat;
            clon_ = s.home_lon;
            zoom_ = zoom_target_ = 16.0;
        }
    }
}

void MapView::setMode(Mode m) {
    mode_ = m;
    drag_wp_ = -1;
    updateCursor();
    update();
}

void MapView::setLayer(int layer) {
    if (layer_ != layer) {
        layer_ = layer;
        emit layerChanged(layer);
        update();
    }
}

void MapView::setFollow(bool on) {
    if (follow_ != on) {
        follow_ = on;
        emit followChanged(on);
        update();
    }
}

void MapView::setPlan(std::vector<UserWaypoint> plan) {
    plan_ = std::move(plan);
    undo_.clear();
    emit planChanged();
    update();
}

void MapView::clearPlan() {
    if (plan_.empty()) {
        return;
    }
    pushUndo();
    plan_.clear();
    emit planChanged();
    update();
}

void MapView::undo() {
    if (undo_.empty()) {
        return;
    }
    plan_ = std::move(undo_.back());
    undo_.pop_back();
    emit planChanged();
    update();
}

void MapView::pushUndo() {
    undo_.push_back(plan_);
    if (undo_.size() > kUndoDepth) {
        undo_.erase(undo_.begin());
    }
}

void MapView::setGuidedTarget(bool has, double lat, double lon) {
    has_target_ = has;
    target_lat_ = lat;
    target_lon_ = lon;
    update();
}

void MapView::ping(double lat, double lon, const QColor& color) {
    if (reducedMotion()) {
        return;
    }
    pings_.push_back({lat, lon, color, 0.0});
}

void MapView::tick(double dt) {
    time_ += dt;
    heading_.step(dt);

    // smooth zoom, anchored under the cursor unless following
    if (zoom_ != zoom_target_) {
        const double k = reducedMotion() ? 1.0 : 1.0 - std::exp(-14.0 * dt);
        zoom_ += (zoom_target_ - zoom_) * k;
        if (std::abs(zoom_target_ - zoom_) < 1e-3) {
            zoom_ = zoom_target_;
        }
        if (anchor_valid_ && !follow_) {
            const QPointF w = worldPx(anchor_lat_, anchor_lon_, zoom_) -
                              (anchor_px_ - QPointF(width() / 2.0, height() / 2.0));
            worldToLatLon(w, zoom_, clat_, clon_);
        }
        if (zoom_ == zoom_target_) {
            anchor_valid_ = false;
        }
    }

    if ((follow_ || compact_) && s_.has_position && centred_ && !pressing_) {
        const double k = 1.0 - std::exp(-6.0 * dt);
        clat_ += (s_.lat - clat_) * k;
        clon_ += (s_.lon - clon_) * k;
    }

    for (auto& pg : pings_) {
        pg.t += dt / 0.9;
    }
    pings_.erase(std::remove_if(pings_.begin(), pings_.end(), [](const Ping& pg) { return pg.t >= 1.0; }),
                 pings_.end());

    if (isVisible()) {
        update();
    }
}

QPointF MapView::toScreen(double lat, double lon) const {
    return worldPx(lat, lon, zoom_) - worldPx(clat_, clon_, zoom_) + QPointF(width() / 2.0, height() / 2.0);
}

void MapView::toLatLon(const QPointF& pt, double& lat, double& lon) const {
    const QPointF w = pt - QPointF(width() / 2.0, height() / 2.0) + worldPx(clat_, clon_, zoom_);
    worldToLatLon(w, zoom_, lat, lon);
}

int MapView::planPointAt(const QPointF& pos) const {
    for (int i = static_cast<int>(plan_.size()) - 1; i >= 0; --i) {
        const QPointF sp = toScreen(plan_[i].lat, plan_[i].lon);
        if (QLineF(sp, pos).length() <= kHitRadius) {
            return i;
        }
    }
    return -1;
}

void MapView::updateCursor() {
    if (compact_) {
        setCursor(Qt::PointingHandCursor);
    } else if (mode_ == Mode::Plan) {
        setCursor(hover_wp_ >= 0 ? Qt::SizeAllCursor : Qt::CrossCursor);
    } else if (mode_ == Mode::Guide) {
        setCursor(Qt::CrossCursor);
    } else {
        setCursor(pressing_ && dragged_ ? Qt::ClosedHandCursor : Qt::OpenHandCursor);
    }
}

// ---- input -----------------------------------------------------------------------

void MapView::mousePressEvent(QMouseEvent* e) {
    if (compact_) {
        return;
    }
    const QPointF pos = e->position();
    if (e->button() == Qt::LeftButton) {
        if (follow_chip_.contains(pos)) {
            setFollow(!follow_);
            return;
        }
        if (layer_chip_.contains(pos)) {
            setLayer(layer_ == TileCache::Streets ? TileCache::Satellite : TileCache::Streets);
            return;
        }
    }
    if (mode_ == Mode::Plan && e->button() == Qt::RightButton) {
        const int i = planPointAt(pos);
        if (i >= 0) {
            pushUndo();
            plan_.erase(plan_.begin() + i);
            hover_wp_ = -1;
            emit planChanged();
            update();
        }
        return;
    }
    if (e->button() != Qt::LeftButton) {
        return;
    }
    pressing_ = true;
    dragged_ = false;
    press_pos_ = last_pos_ = pos;
    drag_wp_ = -1;
    if (mode_ == Mode::Plan) {
        drag_wp_ = planPointAt(pos);
        if (drag_wp_ >= 0) {
            pushUndo();
        }
    }
    updateCursor();
}

void MapView::mouseMoveEvent(QMouseEvent* e) {
    if (compact_) {
        return;
    }
    const QPointF pos = e->position();
    if (pressing_) {
        if (!dragged_ && QLineF(pos, press_pos_).length() > 4.0) {
            dragged_ = true;
            updateCursor();
        }
        if (dragged_) {
            if (drag_wp_ >= 0) {
                toLatLon(pos, plan_[drag_wp_].lat, plan_[drag_wp_].lon);
                emit planChanged();
            } else {
                const QPointF d = pos - last_pos_;
                const QPointF w = worldPx(clat_, clon_, zoom_) - d;
                worldToLatLon(w, zoom_, clat_, clon_);
                anchor_valid_ = false;
                setFollow(false);
            }
            update();
        }
        last_pos_ = pos;
        return;
    }
    if (mode_ == Mode::Plan) {
        const int h = planPointAt(pos);
        if (h != hover_wp_) {
            hover_wp_ = h;
            updateCursor();
            update();
        }
    }
}

void MapView::mouseReleaseEvent(QMouseEvent* e) {
    if (compact_ || e->button() != Qt::LeftButton || !pressing_) {
        return;
    }
    pressing_ = false;
    const QPointF pos = e->position();
    if (!dragged_) {
        double lat = 0.0, lon = 0.0;
        toLatLon(pos, lat, lon);
        if (mode_ == Mode::Plan && drag_wp_ < 0) {
            pushUndo();
            const float alt = plan_.empty() ? default_alt_ : plan_.back().alt_rel;
            plan_.push_back({lat, lon, alt});
            ping(lat, lon, kAmber);
            emit planChanged();
        } else if (mode_ == Mode::Plan && drag_wp_ >= 0) {
            undo_.pop_back();  // a click on a point is not an edit
        } else if (mode_ == Mode::Guide) {
            emit guideClicked(lat, lon);
        }
    }
    drag_wp_ = -1;
    dragged_ = false;
    updateCursor();
    update();
}

void MapView::wheelEvent(QWheelEvent* e) {
    if (compact_) {
        return;
    }
    const double steps = e->angleDelta().y() / 120.0;
    if (mode_ == Mode::Plan) {
        const int i = planPointAt(e->position());
        if (i >= 0) {
            pushUndo();
            plan_[i].alt_rel = std::clamp(plan_[i].alt_rel + static_cast<float>(steps * 5.0), 10.0f, 500.0f);
            emit planChanged();
            update();
            return;
        }
    }
    zoom_target_ = std::clamp(zoom_target_ + steps * 0.5, kMinZoom, kMaxZoom);
    if (!follow_) {
        anchor_px_ = e->position();
        toLatLon(anchor_px_, anchor_lat_, anchor_lon_);
        anchor_valid_ = true;
    }
}

// ---- painting --------------------------------------------------------------------

void MapView::paintEvent(QPaintEvent*) {
    QPainter p(this);
    beginView(p);
    const QRectF r = rect();

    // background + tactical grid (visible where tiles are missing / offline)
    p.fillRect(r, kBg);
    p.setPen(QPen(alpha(kCyan, 0.05), 1));
    const double grid = 64.0;
    const QPointF c0 = worldPx(clat_, clon_, zoom_);
    const double ox = std::fmod(-c0.x() + width() / 2.0, grid);
    const double oy = std::fmod(-c0.y() + height() / 2.0, grid);
    for (double x = ox; x < r.right(); x += grid) {
        p.drawLine(QPointF(x, 0), QPointF(x, r.bottom()));
    }
    for (double y = oy; y < r.bottom(); y += grid) {
        p.drawLine(QPointF(0, y), QPointF(r.right(), y));
    }

    paintTiles(p);

    // home
    if (s_.has_home) {
        const QPointF h = toScreen(s_.home_lat, s_.home_lon);
        const QPainterPath hex = hexPath(h, 12);
        p.setPen(Qt::NoPen);
        p.setBrush(alpha(kBg, 0.8));
        p.drawPath(hex);
        drawGlowPath(p, hex, kGreen, 1.5, 5);
        drawText(p, QRectF(h.x() - 10, h.y() - 10, 20, 20), Qt::AlignCenter, QStringLiteral("H"),
                 monoFont(12, true), kGreen);
    }

    // routes: the vehicle's mission underneath, the plan being edited on top
    int active = -1;
    if (s_.mission_current >= 2) {
        active = s_.mission_current - 2;
    }
    if (mode_ == Mode::Plan) {
        if (!vehicle_wps_.empty()) {
            p.setOpacity(0.35);
            paintRoute(p, vehicle_wps_, kCyan, false, -1);
            p.setOpacity(1.0);
        }
        paintRoute(p, plan_, kAmber, true, -1);
    } else if (!vehicle_wps_.empty()) {
        paintRoute(p, vehicle_wps_, kCyan, false, active);
    }

    // trail: fades along its length
    if (s_.trail.size() > 1) {
        const size_t n = s_.trail.size();
        const size_t chunk = 12;
        for (size_t i = 1; i < n; i += chunk) {
            QPolygonF poly;
            const size_t end = std::min(n, i + chunk);
            for (size_t k = i - 1; k < end; ++k) {
                poly << toScreen(s_.trail[k].lat, s_.trail[k].lon);
            }
            const double a = 0.08 + 0.75 * static_cast<double>(end) / n;
            p.setPen(QPen(alpha(kCyan, a), compact_ ? 1.5 : 2.2, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
            p.setBrush(Qt::NoBrush);
            p.drawPolyline(poly);
        }
    }

    // guided target
    if (has_target_) {
        const QPointF t = toScreen(target_lat_, target_lon_);
        if (s_.has_position) {
            QPen dash(alpha(kCyan, 0.6), 1.2, Qt::DashLine);
            dash.setDashOffset(-time_ * 8.0);
            p.setPen(dash);
            p.drawLine(toScreen(s_.lat, s_.lon), t);
        }
        p.save();
        p.translate(t);
        p.rotate(reducedMotion() ? 45.0 : std::fmod(time_ * 30.0, 90.0));
        drawBrackets(p, QRectF(-14, -14, 28, 28), 7, kCyan, 2.0);
        p.restore();
        p.setPen(Qt::NoPen);
        p.setBrush(kCyan);
        p.drawEllipse(t, 2.5, 2.5);
    }

    // ping ripples (RTS move order)
    for (const auto& pg : pings_) {
        const QPointF c = toScreen(pg.lat, pg.lon);
        const double e = easeOutCubic(pg.t);
        p.setBrush(Qt::NoBrush);
        p.setPen(QPen(alpha(pg.color, 1.0 - pg.t), 2.0));
        p.drawEllipse(c, 6 + 34 * e, 6 + 34 * e);
        p.setPen(QPen(alpha(pg.color, 0.6 * (1.0 - pg.t)), 1.0));
        p.drawEllipse(c, 4 + 18 * e, 4 + 18 * e);
    }

    paintAircraft(p);
    paintOverlayChrome(p);
    finishView(p);
}

void MapView::paintTiles(QPainter& p) {
    const int maxz = TileCache::maxZoom(layer_);
    const int z = std::clamp(static_cast<int>(std::floor(zoom_ + 0.25)), TileCache::kMinZoom, maxz);
    const double scale = std::exp2(zoom_ - z);
    const double ts = kTile * scale;
    const QPointF cw = worldPx(clat_, clon_, z) * scale;
    const QPointF origin = QPointF(width() / 2.0, height() / 2.0) - cw;
    const int n = 1 << z;

    const int x0 = static_cast<int>(std::floor(-origin.x() / ts));
    const int x1 = static_cast<int>(std::floor((width() - origin.x()) / ts));
    const int y0 = std::max(0, static_cast<int>(std::floor(-origin.y() / ts)));
    const int y1 = std::min(n - 1, static_cast<int>(std::floor((height() - origin.y()) / ts)));

    p.save();
    p.setRenderHint(QPainter::Antialiasing, false);
    p.setRenderHint(QPainter::SmoothPixmapTransform, std::abs(scale - 1.0) > 1e-3);
    // request the centre tiles first so the area under the aircraft fills in first
    const double cx = (width() / 2.0 - origin.x()) / ts;
    const double cy = (height() / 2.0 - origin.y()) / ts;
    std::vector<std::pair<int, int>> order;
    for (int ty = y0; ty <= y1; ++ty) {
        for (int tx = x0; tx <= x1; ++tx) {
            order.emplace_back(tx, ty);
        }
    }
    std::sort(order.begin(), order.end(), [cx, cy](const auto& a, const auto& b) {
        return std::hypot(a.first + 0.5 - cx, a.second + 0.5 - cy) < std::hypot(b.first + 0.5 - cx, b.second + 0.5 - cy);
    });
    for (const auto& [tx, ty] : order) {
        const QRectF dst(origin.x() + tx * ts, origin.y() + ty * ts, ts + 0.5, ts + 0.5);
        const int wx = ((tx % n) + n) % n;
        if (const QPixmap* pm = tiles_->tile(layer_, z, wx, ty)) {
            p.drawPixmap(dst, *pm, QRectF(pm->rect()));
            continue;
        }
        // fall back to a scaled-up ancestor already in memory
        for (int dz = 1; dz <= 5 && z - dz >= 0; ++dz) {
            const int px = wx >> dz;
            const int py = ty >> dz;
            if (const QPixmap* pm = tiles_->peek(layer_, z - dz, px, py)) {
                const double sub = pm->width() / static_cast<double>(1 << dz);
                const QRectF src((wx - (px << dz)) * sub, (ty - (py << dz)) * sub, sub, sub);
                p.drawPixmap(dst, *pm, src);
                break;
            }
        }
    }
    p.restore();
}

void MapView::paintRoute(QPainter& p, const std::vector<UserWaypoint>& pts, const QColor& c, bool editable,
                         int active_index) {
    if (pts.empty()) {
        return;
    }
    QPolygonF line;
    if (s_.has_home) {
        line << toScreen(s_.home_lat, s_.home_lon);
    }
    for (const auto& wp : pts) {
        line << toScreen(wp.lat, wp.lon);
    }
    // marching ants show direction of travel
    p.setBrush(Qt::NoBrush);
    p.setPen(QPen(alpha(c, 0.22), 6, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    p.drawPolyline(line);
    QPen ants(c, 2.0, Qt::CustomDashLine, Qt::FlatCap, Qt::RoundJoin);
    ants.setDashPattern({5, 4});
    ants.setDashOffset(reducedMotion() ? 0.0 : -std::fmod(time_ * 14.0, 9.0));
    p.setPen(ants);
    p.drawPolyline(line);

    if (compact_) {
        for (const auto& wp : pts) {
            p.setPen(Qt::NoPen);
            p.setBrush(c);
            p.drawEllipse(toScreen(wp.lat, wp.lon), 3, 3);
        }
        return;
    }

    const QFont nf = monoFont(11, true);
    const QFont af = monoFont(10);
    for (size_t i = 0; i < pts.size(); ++i) {
        const QPointF sp = toScreen(pts[i].lat, pts[i].lon);
        const bool last = i + 1 == pts.size();
        const bool is_active = static_cast<int>(i) == active_index;
        const bool hot = editable && (static_cast<int>(i) == hover_wp_ || static_cast<int>(i) == drag_wp_);
        double rad = 12.0 + (hot ? 2.0 : 0.0);
        QColor mc = c;
        if (is_active) {
            mc = kGreen;
            if (!reducedMotion()) {
                const double ph = std::fmod(time_ * 1.2, 1.0);
                p.setPen(QPen(alpha(kGreen, 1.0 - ph), 1.5));
                p.setBrush(Qt::NoBrush);
                p.drawPath(hexPath(sp, rad + 4 + 14 * ph));
            }
        }
        const QPainterPath hex = hexPath(sp, rad);
        p.setPen(Qt::NoPen);
        p.setBrush(alpha(kBg, 0.85));
        p.drawPath(hex);
        drawGlowPath(p, hex, mc, hot ? 2.0 : 1.4, hot ? 8 : 4);
        drawText(p, QRectF(sp.x() - 12, sp.y() - 12, 24, 24), Qt::AlignCenter,
                 last ? QStringLiteral("L") : QString::number(i + 1), nf, mc);
        QString caption = editable ? QStringLiteral("%1 m").arg(pts[i].alt_rel, 0, 'f', 0) : QString();
        if (last) {
            caption = caption.isEmpty() ? QStringLiteral("VTOL LAND") : caption + QStringLiteral(" · LAND");
        }
        if (!caption.isEmpty()) {
            // beside the marker, so route legs (which leave from the centre) rarely cross it
            const double cw = QFontMetricsF(af).horizontalAdvance(caption) + 10;
            const QRectF cr(sp.x() + rad + 4, sp.y() - 8, cw, 16);
            p.setPen(Qt::NoPen);
            p.setBrush(alpha(kBg, 0.82));
            p.drawRect(cr);
            drawText(p, cr, Qt::AlignCenter, caption, af, alpha(mc, 0.95));
        }
    }
}

void MapView::paintAircraft(QPainter& p) {
    if (!s_.has_position) {
        return;
    }
    const QPointF c = toScreen(s_.lat, s_.lon);
    const VtolState v = effectiveVtolState(s_);
    const QColor col = vtolColor(v);
    const double scale = compact_ ? 0.75 : 1.0;

    if (!reducedMotion()) {
        const double ph = std::fmod(time_ * 0.8, 1.0);
        p.setBrush(Qt::NoBrush);
        p.setPen(QPen(alpha(col, 0.5 * (1.0 - ph)), 1.5));
        p.drawEllipse(c, (18 + 22 * ph) * scale, (18 + 22 * ph) * scale);
    }

    p.save();
    p.translate(c);
    p.rotate(heading_.value);
    p.scale(scale, scale);

    // heading vector
    p.setPen(QPen(alpha(col, 0.5), 1.2, Qt::DashLine));
    p.drawLine(QPointF(0, -18), QPointF(0, -48));

    // QuadPlane silhouette: fuselage, wing, tail, booms, four lift rotors
    QPainterPath body;
    body.addRoundedRect(QRectF(-2.6, -15, 5.2, 29), 2.6, 2.6);
    QPainterPath wing;
    wing.moveTo(-17, -1);
    wing.lineTo(17, -1);
    wing.lineTo(16, 4);
    wing.lineTo(-16, 4);
    wing.closeSubpath();
    QPainterPath tail;
    tail.addRect(QRectF(-7, 11, 14, 3));
    QPainterPath shape = body.united(wing).united(tail);

    p.setPen(Qt::NoPen);
    p.setBrush(alpha(kBg, 0.85));
    p.drawPath(shape);
    drawGlowPath(p, shape, col, 1.4, 5);

    const bool rotors_on = v == VtolState::Hover || v == VtolState::TransitionToCruise ||
                           v == VtolState::TransitionToHover || v == VtolState::Unknown;
    p.setPen(QPen(alpha(col, 0.8), 1.2));
    p.drawLine(QPointF(-8, -10), QPointF(-8, 9));
    p.drawLine(QPointF(8, -10), QPointF(8, 9));
    for (QPointF rp : {QPointF(-8, -10), QPointF(8, -10), QPointF(-8, 9), QPointF(8, 9)}) {
        p.setBrush(rotors_on ? alpha(col, 0.25) : Qt::NoBrush);
        p.setPen(QPen(alpha(col, rotors_on ? 0.95 : 0.4), 1.2));
        p.drawEllipse(rp, 4.5, 4.5);
        if (rotors_on && s_.armed && !reducedMotion()) {
            const double a = std::fmod(time_ * 1400.0, 360.0);
            p.save();
            p.translate(rp);
            p.rotate(a);
            p.drawLine(QPointF(-4, 0), QPointF(4, 0));
            p.restore();
        }
    }
    p.restore();
}

void MapView::paintOverlayChrome(QPainter& p) {
    if (compact_) {
        follow_chip_ = layer_chip_ = QRectF();
        return;
    }
    const double right = width() - 16.0;
    double y = height() - 24.0;

    // scale bar
    const double mpp = std::cos(clat_ * geo::kDegToRad) * 2.0 * geo::kPi * geo::kEarthRadius /
                       (kTile * std::exp2(zoom_));
    const double target = mpp * 110.0;
    const double mag = std::pow(10.0, std::floor(std::log10(target)));
    double nice = mag;
    for (double m : {1.0, 2.0, 5.0}) {
        if (m * mag <= target) {
            nice = m * mag;
        }
    }
    const double px = nice / mpp;
    p.setPen(QPen(kText, 1.5));
    p.drawLine(QPointF(right - px, y), QPointF(right, y));
    p.drawLine(QPointF(right - px, y - 5), QPointF(right - px, y));
    p.drawLine(QPointF(right, y - 5), QPointF(right, y));
    drawText(p, QRectF(right - px - 80, y - 18, px + 80, 14), Qt::AlignRight | Qt::AlignVCenter,
             formatDistance(nice), monoFont(11), kText);
    y -= 30;

    // distances
    QStringList parts;
    if (s_.has_home && s_.has_position) {
        parts << QStringLiteral("HOME %1").arg(formatDistance(geo::distance(s_.lat, s_.lon, s_.home_lat, s_.home_lon)));
    }
    if (s_.has_position && s_.mission_current >= 2) {
        const int idx = s_.mission_current - 2;
        if (idx >= 0 && idx < static_cast<int>(vehicle_wps_.size())) {
            const auto& wp = vehicle_wps_[idx];
            parts << QStringLiteral("WP%1 %2").arg(idx + 1).arg(formatDistance(geo::distance(s_.lat, s_.lon, wp.lat, wp.lon)));
        }
    }
    if (has_target_ && s_.has_position) {
        parts << QStringLiteral("TGT %1").arg(formatDistance(geo::distance(s_.lat, s_.lon, target_lat_, target_lon_)));
    }
    if (!parts.isEmpty()) {
        const QString text = parts.join(QStringLiteral("  ·  "));
        const QFont f = monoFont(12, true);
        const double w = QFontMetricsF(f).horizontalAdvance(text) + 24;
        const QRectF box(right - w, y - 26, w, 26);
        drawGlass(p, chamferPath(box, 7), kCyan, 0.3, kGlassDeep);
        drawText(p, box, Qt::AlignCenter, text, f, kText);
        y -= 34;
    }

    // chips: follow / layer
    auto chip = [&](QRectF& out, double x_right, const QString& text, bool on) {
        const QFont f = labelFont(10.5);
        const double w = QFontMetricsF(f).horizontalAdvance(text) + 22;
        out = QRectF(x_right - w, y - 24, w, 24);
        drawGlass(p, chamferPath(out, 6), on ? kCyan : kMuted, on ? 0.8 : 0.4, kGlassDeep);
        drawText(p, out, Qt::AlignCenter, text, f, on ? kCyan : kMuted);
        return out.left() - 8;
    };
    double xr = right;
    xr = chip(layer_chip_, xr, layer_ == TileCache::Satellite ? QStringLiteral("▣ Sat") : QStringLiteral("▣ Map"), true);
    chip(follow_chip_, xr, QStringLiteral("◎ Follow"), follow_);

    // mode hint
    if (mode_ == Mode::Guide) {
        const QString text = QStringLiteral("GUIDE · click the map to send the aircraft");
        const QFont f = labelFont(11);
        const double w = QFontMetricsF(f).horizontalAdvance(text) + 30;
        const QRectF box(width() / 2.0 - w / 2.0, 70, w, 26);
        drawGlass(p, chamferPath(box, 7), kCyan, 0.8, kGlassDeep);
        drawText(p, box, Qt::AlignCenter, text, f, kCyan);
    }
    if (mode_ != Mode::Normal) {
        drawBrackets(p, QRectF(rect()).adjusted(8, 8, -8, -8), 28, alpha(mode_ == Mode::Plan ? kAmber : kCyan, 0.7), 2.0);
    }
}

}  // namespace gcs::ui
