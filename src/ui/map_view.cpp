#include "map_view.hpp"

#include "attitude_ball.hpp"
#include "elevation_cache.hpp"
#include "wind_gauge.hpp"
#include "core/geo.hpp"
#include "theme.hpp"
#include "tile_cache.hpp"
#include "units.hpp"

#include <QDateTime>
#include <QFontMetricsF>
#include <QMouseEvent>
#include <QPainter>
#include <QPointingDevice>
#include <QRadialGradient>
#include <QTouchEvent>
#include <QWheelEvent>

#include <algorithm>
#include <cmath>

namespace gcs::ui {

using namespace theme;

namespace {

constexpr double kTile = 256.0;
constexpr double kMaxZoom = 19.5;
constexpr double kMinZoom = 2.0;
constexpr double kHitRadiusMouse = 15.0;
constexpr double kHitRadiusTouch = 28.0;  // a fingertip is ~7 mm; keep points easy to grab
constexpr double kDragSlopMouse = 4.0;
constexpr double kDragSlopTouch = 12.0;
constexpr float kMinAlt = 10.0f;
constexpr float kMaxAlt = 5000.0f;
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
    // Touch is handled here (pinch zoom + one-finger tap/drag); in PiP mode it
    // is ignored so Qt synthesises the mouse click the ViewHost swaps on.
    setAttribute(Qt::WA_AcceptTouchEvents);
    setMouseTracking(true);  // the lat / lon / ground readout follows the mouse
    connect(tiles_, &TileCache::tileReady, this, [this] {
        ++tiles_gen_;
        update();
    });
    heading_.rate = 25.0;  // light smoothing on top of the per-frame prediction
    updateCursor();
}

QString MapView::pipTag() const {
    return QStringLiteral("Map · %1").arg(TileCache::layerName(layer_));
}

void MapView::setState(const VehicleState& s) {
    s_ = s;
    vehicle_plan_ = s.mission_known ? extractPlan(s.mission) : MissionPlan{};
    approach_ = s.mission_known ? extractApproach(s.mission) : std::nullopt;
    if (!s.has_attitude && s.has_vfr_hud) {
        heading_.set(s.heading_deg);  // otherwise set per frame from the prediction
    }
    if (!s.has_position) {
        has_ac_ = false;
    } else if (!has_ac_) {
        setAircraftPosition(s.lat, s.lon);
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
    drag_wp_ = kNone;
    if (m != Mode::Plan) {
        select(kNone);
        setAddTool(AddTool::Waypoint);
    }
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

void MapView::setPlan(MissionPlan plan) {
    plan_ = std::move(plan);
    undo_.clear();
    alt_edit_wp_ = kNone;
    select(kNone);
    emit planChanged();
    update();
}

void MapView::clearPlan() {
    if (plan_.empty()) {
        return;
    }
    editPlan();
    plan_ = MissionPlan{};
    select(kNone);
    emit planChanged();
    update();
}

void MapView::undo() {
    if (undo_.empty()) {
        return;
    }
    plan_ = std::move(undo_.back());
    undo_.pop_back();
    alt_edit_wp_ = kNone;
    if (sel_ >= static_cast<int>(plan_.wps.size()) || (sel_ == kLandId && !plan_.land)) {
        select(kNone);
    }
    emit planChanged();
    update();
}

void MapView::pushUndo() {
    undo_.push_back(plan_);
    if (undo_.size() > kUndoDepth) {
        undo_.erase(undo_.begin());
    }
}

void MapView::editPlan() {
    pushUndo();
    alt_edit_wp_ = kNone;
}

void MapView::setAddTool(AddTool t) {
    if (tool_ != t) {
        tool_ = t;
        emit addToolChanged(t);
        updateCursor();
    }
}

void MapView::select(int id) {
    if (sel_ != id) {
        sel_ = id;
        emit selectionChanged(id);
        update();
    }
}

void MapView::adjustAltitude(int id, float delta) {
    UserWaypoint* wp = nullptr;
    if (id == kLandId && plan_.land) {
        wp = &*plan_.land;
    } else if (id >= 0 && id < static_cast<int>(plan_.wps.size())) {
        wp = &plan_.wps[id];
    } else {
        return;
    }
    if (alt_edit_wp_ != id) {
        pushUndo();  // a run of +/- presses on one point undoes in one step
        alt_edit_wp_ = id;
    }
    auto& alt = wp->alt_rel;
    alt = std::clamp(alt + delta, kMinAlt, kMaxAlt);
    emit planChanged();
    update();
}

void MapView::deletePoint(int id) {
    if (id == kLandId && plan_.land) {
        editPlan();
        plan_.land.reset();
    } else if (id >= 0 && id < static_cast<int>(plan_.wps.size())) {
        editPlan();
        plan_.wps.erase(plan_.wps.begin() + id);
    } else {
        return;
    }
    hover_wp_ = kNone;
    select(kNone);
    emit planChanged();
    update();
}

void MapView::insertAfter(int index) {
    const int n = static_cast<int>(plan_.wps.size());
    if (index < 0 || index >= n) {
        return;
    }
    editPlan();
    const auto& a = plan_.wps[index];
    UserWaypoint mid = a;
    if (n > 1) {
        const auto& b = plan_.wps[(index + 1) % n];  // the loop closes back to WP1
        mid = {(a.lat + b.lat) / 2.0, (a.lon + b.lon) / 2.0, (a.alt_rel + b.alt_rel) / 2.0f};
    } else {
        geo::offset(a.lat, a.lon, 0.0, 150.0, mid.lat, mid.lon);  // 150 m east
    }
    plan_.wps.insert(plan_.wps.begin() + index + 1, mid);
    ping(mid.lat, mid.lon, kAmber);
    emit planChanged();
    select(index + 1);
}

void MapView::setAircraftPosition(double lat, double lon) {
    has_ac_ = true;
    ac_lat_ = lat;
    ac_lon_ = lon;
}

void MapView::setHeading(double deg) {
    heading_.set(geo::wrap360(deg));
}

void MapView::setAttitude(double roll_deg, double pitch_deg) {
    if (!has_att_) {
        roll_.rate = pitch_.rate = 25.0;
    }
    roll_.set(roll_deg);
    pitch_.set(pitch_deg);
    has_att_ = true;
}

void MapView::setFootprint(std::optional<FootprintDraw> fp) {
    footprint_ = std::move(fp);
}

void MapView::setBallCollapsed(bool on) {
    ball_collapsed_ = on;
    if (reducedMotion()) {
        ball_open_ = on ? 0.0 : 1.0;
    }
    update();
}

void MapView::setTrail(bool on, int secs) {
    trail_on_ = on;
    trail_secs_ = std::max(0, secs);
    update();
}

void MapView::setOrbit(std::optional<OrbitDraw> orbit) {
    orbit_ = orbit;
    update();
}

double MapView::metersPerPixel() const {
    return std::cos(clat_ * geo::kDegToRad) * 2.0 * geo::kPi * geo::kEarthRadius / (kTile * std::exp2(zoom_));
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
    roll_.step(dt);
    pitch_.step(dt);
    {
        const double target = ball_collapsed_ ? 0.0 : 1.0;
        ball_open_ = reducedMotion() ? target : ball_open_ + std::clamp(target - ball_open_, -dt * 5.0, dt * 5.0);
    }

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

    if ((follow_ || compact_) && has_ac_ && centred_ && !pressing_) {
        const double k = 1.0 - std::exp(-6.0 * dt);
        clat_ += (ac_lat_ - clat_) * k;
        clon_ += (ac_lon_ - clon_) * k;
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
    const double radius = touch_input_ ? kHitRadiusTouch : kHitRadiusMouse;
    int best = kNone;
    double best_d = radius;
    auto consider = [&](int id, const UserWaypoint& wp) {
        const double d = QLineF(toScreen(wp.lat, wp.lon), pos).length();
        if (d <= best_d) {  // nearest wins, later points on ties
            best_d = d;
            best = id;
        }
    };
    for (int i = 0; i < static_cast<int>(plan_.wps.size()); ++i) {
        consider(i, plan_.wps[i]);
    }
    if (plan_.land) {
        consider(kLandId, *plan_.land);
    }
    return best;
}

void MapView::updateCursor() {
    if (compact_) {
        setCursor(Qt::PointingHandCursor);
    } else if (mode_ == Mode::Plan) {
        setCursor(hover_wp_ != kNone ? Qt::SizeAllCursor : Qt::CrossCursor);
    } else if (mode_ == Mode::Guide) {
        setCursor(Qt::CrossCursor);
    } else {
        setCursor(pressing_ && dragged_ ? Qt::ClosedHandCursor : Qt::OpenHandCursor);
    }
}

// ---- input -----------------------------------------------------------------------
//
// Mouse and one-finger touch share pointerPress/Move/Release. Two fingers pinch
// to zoom (and pan with the midpoint). Mouse-only extras: right-click deletes a
// point, the wheel over a point changes its altitude.

bool MapView::event(QEvent* e) {
    switch (e->type()) {
    case QEvent::TouchBegin:
    case QEvent::TouchUpdate:
    case QEvent::TouchEnd:
    case QEvent::TouchCancel:
        if (compact_) {
            e->ignore();  // let Qt synthesise a click for the PiP swap
            return false;
        }
        handleTouch(static_cast<QTouchEvent*>(e));
        return true;
    default:
        return HudView::event(e);
    }
}

void MapView::handleTouch(QTouchEvent* e) {
    touch_input_ = true;
    const auto& pts = e->points();
    if (e->type() == QEvent::TouchCancel) {
        pressing_ = dragged_ = pinching_ = false;
        drag_wp_ = kNone;
        update();
        return;
    }
    if (pts.size() >= 2) {
        const QPointF a = pts[0].position();
        const QPointF b = pts[1].position();
        const QPointF mid = (a + b) / 2.0;
        const double dist = std::max(1.0, QLineF(a, b).length());
        if (!pinching_) {
            // a second finger turns whatever the first one started into a pinch
            if (pressing_ && drag_wp_ != kNone && dragged_) {
                undo();  // put a half-dragged point back
            }
            pressing_ = dragged_ = false;
            drag_wp_ = kNone;
            pinching_ = true;
            pinch_dist0_ = dist;
            pinch_zoom0_ = zoom_;
            toLatLon(mid, pinch_lat_, pinch_lon_);
            setFollow(false);
        }
        zoom_ = zoom_target_ = std::clamp(pinch_zoom0_ + std::log2(dist / pinch_dist0_), kMinZoom, kMaxZoom);
        // keep the point that started under the fingers under them
        const QPointF w = worldPx(pinch_lat_, pinch_lon_, zoom_) - (mid - QPointF(width() / 2.0, height() / 2.0));
        worldToLatLon(w, zoom_, clat_, clon_);
        anchor_valid_ = false;
        update();
        if (e->type() == QEvent::TouchEnd) {
            pinching_ = false;
        }
        return;
    }
    if (pts.isEmpty()) {
        return;
    }
    const QPointF pos = pts[0].position();
    if (pinching_) {
        // one finger left after a pinch: wait for all fingers to lift
        if (e->type() == QEvent::TouchEnd) {
            pinching_ = false;
        }
        return;
    }
    switch (e->type()) {
    case QEvent::TouchBegin:
        pointerPress(pos);
        break;
    case QEvent::TouchUpdate:
        pointerMove(pos);
        break;
    case QEvent::TouchEnd:
        pointerRelease(pos);
        break;
    default:
        break;
    }
}

void MapView::mousePressEvent(QMouseEvent* e) {
    if (compact_) {
        return;
    }
    touch_input_ = e->pointingDevice() && e->pointingDevice()->type() == QInputDevice::DeviceType::TouchScreen;
    const QPointF pos = e->position();
    if (mode_ == Mode::Plan && e->button() == Qt::RightButton) {
        deletePoint(planPointAt(pos));
        return;
    }
    if (e->button() == Qt::LeftButton) {
        pointerPress(pos);
    }
}

void MapView::mouseMoveEvent(QMouseEvent* e) {
    if (compact_) {
        return;
    }
    const QPointF pos = e->position();
    if (pressing_) {
        pointerMove(pos);
        return;
    }
    touch_input_ = false;  // hovering only happens with a mouse
    if (e->source() == Qt::MouseEventNotSynthesized) {
        hover_pos_ = pos;
        update();
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

void MapView::leaveEvent(QEvent* e) {
    hover_pos_.reset();
    update();
    HudView::leaveEvent(e);
}

void MapView::setElevation(ElevationCache* elev) {
    elev_ = elev;
    if (elev_) {
        connect(elev_, &ElevationCache::ready, this, [this] {
            ++elev_gen_;
            update();
        });
    }
}

void MapView::paintHover(QPainter& p, double right, double bottom) {
    if (!hover_pos_ || compact_ || touch_input_ || pressing_ || !rect().contains(hover_pos_->toPoint())) {
        return;
    }
    double lat = 0.0, lon = 0.0;
    toLatLon(*hover_pos_, lat, lon);
    const QString pos = QStringLiteral("%1° %2   %3° %4")
                            .arg(std::abs(lat), 0, 'f', 5)
                            .arg(lat >= 0 ? QLatin1Char('N') : QLatin1Char('S'))
                            .arg(std::abs(lon), 0, 'f', 5)
                            .arg(lon >= 0 ? QLatin1Char('E') : QLatin1Char('W'));
    QString ground = QStringLiteral("GROUND  —");
    if (elev_) {
        if (hover_elev_.lat != lat || hover_elev_.lon != lon || hover_elev_.gen != elev_gen_) {
            hover_elev_ = {lat, lon, elev_gen_, elev_->elevation(lat, lon)};
        }
        if (const auto h = hover_elev_.h) {
            ground = QStringLiteral("GROUND  %1 MSL").arg(units::altText(*h));
            if (s_.has_home) {
                const double rel = *h - s_.home_alt_amsl;
                ground += QStringLiteral("  ·  %1%2 vs HOME")
                              .arg(rel >= 0 ? QStringLiteral("+") : QString())
                              .arg(units::altText(rel));
            }
        } else {
            ground = QStringLiteral("GROUND  loading…");
        }
    }
    QString from;
    if (has_ac_) {
        const double d = geo::distance(ac_lat_, ac_lon_, lat, lon);
        from = QStringLiteral("%1  ·  %2° from AIRCRAFT")
                   .arg(formatDistance(d))
                   .arg(static_cast<int>(std::lround(geo::wrap360(geo::bearing(ac_lat_, ac_lon_, lat, lon)))) % 360, 3,
                        10, QLatin1Char('0'));
    }
    const QFont mf = monoFont(12, true);
    const QFont sf = monoFont(11);
    const QFontMetricsF fm(mf), fs(sf);
    double w = std::max({fm.horizontalAdvance(pos), fs.horizontalAdvance(ground), fs.horizontalAdvance(from)}) + 28;
    const int lines = from.isEmpty() ? 2 : 3;
    const QRectF box(right - w, bottom - (12 + lines * 18), w, 12 + lines * 18);
    drawGlass(p, chamferPath(box, 7), kCyan, 0.35, alpha(kGlassDeep, 0.9));
    double y = box.top() + 6;
    drawText(p, QRectF(box.left() + 14, y, w - 28, 18), Qt::AlignLeft | Qt::AlignVCenter, pos, mf, kText);
    y += 18;
    drawText(p, QRectF(box.left() + 14, y, w - 28, 18), Qt::AlignLeft | Qt::AlignVCenter, ground, sf, kGreen);
    y += 18;
    if (!from.isEmpty()) {
        drawText(p, QRectF(box.left() + 14, y, w - 28, 18), Qt::AlignLeft | Qt::AlignVCenter, from, sf, kMuted);
    }
}

void MapView::mouseReleaseEvent(QMouseEvent* e) {
    if (compact_ || e->button() != Qt::LeftButton) {
        return;
    }
    pointerRelease(e->position());
}

void MapView::pointerPress(const QPointF& pos) {
    if (ball_rect_.contains(pos)) {
        setBallCollapsed(!ball_collapsed_);
        emit ballCollapsedChanged(ball_collapsed_);
        return;
    }
    if (follow_chip_.contains(pos)) {
        setFollow(!follow_);
        return;
    }
    if (layer_chip_.contains(pos)) {
        setLayer(layer_ == TileCache::Streets ? TileCache::Satellite : TileCache::Streets);
        return;
    }
    pressing_ = true;
    dragged_ = false;
    press_pos_ = last_pos_ = pos;
    drag_wp_ = mode_ == Mode::Plan ? planPointAt(pos) : kNone;
    updateCursor();
}

void MapView::pointerMove(const QPointF& pos) {
    if (!pressing_) {
        return;
    }
    const double slop = touch_input_ ? kDragSlopTouch : kDragSlopMouse;
    if (!dragged_ && QLineF(pos, press_pos_).length() > slop) {
        dragged_ = true;
        if (drag_wp_ != kNone) {
            editPlan();
            select(drag_wp_);
        }
        updateCursor();
    }
    if (dragged_) {
        if (drag_wp_ != kNone) {
            UserWaypoint& wp = drag_wp_ == kLandId ? *plan_.land : plan_.wps[drag_wp_];
            toLatLon(pos, wp.lat, wp.lon);
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
}

void MapView::pointerRelease(const QPointF& pos) {
    if (!pressing_) {
        return;
    }
    pressing_ = false;
    if (!dragged_) {
        double lat = 0.0, lon = 0.0;
        toLatLon(pos, lat, lon);
        if (mode_ == Mode::Plan) {
            if (drag_wp_ != kNone) {
                select(drag_wp_ == sel_ ? kNone : drag_wp_);  // tap a point: select / deselect
            } else if (tool_ == AddTool::Landing) {
                editPlan();
                const float approach = plan_.land ? plan_.land->alt_rel : default_land_alt_;
                plan_.land = UserWaypoint{lat, lon, approach};
                ping(lat, lon, kGreen);
                setAddTool(AddTool::Waypoint);
                emit planChanged();
                select(kLandId);
            } else if (sel_ != kNone) {
                select(kNone);  // tap empty map: first deselect, next tap adds
            } else {
                editPlan();
                const float alt = plan_.wps.empty() ? default_alt_ : plan_.wps.back().alt_rel;
                plan_.wps.push_back({lat, lon, alt});
                ping(lat, lon, kAmber);
                emit planChanged();
            }
        } else if (mode_ == Mode::Guide) {
            emit guideClicked(lat, lon);
        }
    }
    drag_wp_ = kNone;
    dragged_ = false;
    updateCursor();
    update();
}

void MapView::wheelEvent(QWheelEvent* e) {
    if (compact_) {
        return;
    }
    touch_input_ = false;
    const double steps = e->angleDelta().y() / 120.0;
    if (mode_ == Mode::Plan) {
        const int i = planPointAt(e->position());
        if (i >= 0) {
            adjustAltitude(i, static_cast<float>(steps * 5.0));
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
    paintBackdrop(p);

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
    const int land_seq = s_.mission_known ? landSeq(s_.mission) : -1;
    int active = s_.mission_known ? seqToWaypointIndex(s_.mission, s_.mission_current) : kNone;
    if (land_seq >= 0 && s_.mission_current == land_seq) {
        active = kLandId;
    }
    if (mode_ == Mode::Plan) {
        if (!vehicle_plan_.empty()) {
            p.setOpacity(0.35);
            paintRoute(p, vehicle_plan_, kCyan, false, kNone);
            p.setOpacity(1.0);
        }
        paintRoute(p, plan_, kAmber, true, kNone);
    } else if (!vehicle_plan_.empty()) {
        paintRoute(p, vehicle_plan_, kCyan, false, active);
    }

    // trail head: the points that arrived after the backdrop was drawn
    if (trail_on_ && bd_.trail_on && bd_.trail_n > 0 && s_.trail.size() > bd_.trail_n) {
        const QPointF world_tl = worldPx(clat_, clon_, zoom_) - QPointF(width() / 2.0, height() / 2.0);
        const size_t first = trailFirst();
        paintTrail(p, world_tl, bd_.trail_n - 1, s_.trail.size(), first, s_.trail.size() - first);
    }

    paintApproach(p);
    paintFootprint(p);
    paintOrbit(p);

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

size_t MapView::trailFirst() const {
    if (trail_secs_ <= 0) {
        return 0;
    }
    const auto cutoff = Clock::now() - std::chrono::seconds(trail_secs_);
    return static_cast<size_t>(std::lower_bound(s_.trail.begin(), s_.trail.end(), cutoff,
                                                [](const TrailPoint& tp, Clock::time_point c) { return tp.t < c; }) -
                               s_.trail.begin());
}

void MapView::paintTrail(QPainter& p, const QPointF& world_tl, size_t from, size_t to, size_t first, size_t span) {
    // fading toward the oldest end; points closer than ~2 px on screen are
    // skipped, so a long trail costs what it covers on screen, not its length
    from = std::max(from, first);
    if (to <= from + 1 || span < 2) {
        return;
    }
    p.save();
    p.setBrush(Qt::NoBrush);
    constexpr int kBuckets = 10;  // alpha steps along the trail
    QPolygonF poly;
    int bucket = -1;
    QPointF last;
    auto flush = [&] {
        if (poly.size() > 1) {
            const double a = 0.08 + 0.75 * (bucket + 1.0) / kBuckets;
            p.setPen(QPen(alpha(kCyan, a), compact_ ? 1.5 : 2.2, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
            p.drawPolyline(poly);
        }
    };
    for (size_t k = from; k < to; ++k) {
        const QPointF pt = worldPx(s_.trail[k].lat, s_.trail[k].lon, zoom_) - world_tl;
        const int b = std::min(kBuckets - 1, static_cast<int>(kBuckets * static_cast<double>(k - first) / span));
        if (b != bucket) {
            flush();
            const QPointF join = poly.isEmpty() ? pt : poly.back();
            poly.clear();
            poly << join;
            bucket = b;
            last = join;
        }
        const QPointF d = pt - last;
        if (k + 1 == to || d.x() * d.x() + d.y() * d.y() >= 4.0) {
            poly << pt;
            last = pt;
        }
    }
    flush();
    p.restore();
}

void MapView::buildBackdrop(const QPointF& world_tl, const QSize& size) {
    const qreal dpr = devicePixelRatioF();
    if (bd_.pm.size() != size * dpr) {
        bd_.pm = QPixmap(size * dpr);
    }
    bd_.pm.setDevicePixelRatio(dpr);
    QPainter p(&bd_.pm);
    p.fillRect(QRectF(QPointF(0, 0), QSizeF(size)), kBg);
    // tactical grid (visible where tiles are missing / offline)
    const QColor grid_c = alpha(kCyan, 0.05);
    const double grid = 64.0;
    // first grid line at or after the pixmap's left / top edge, fixed to the world
    auto first = [grid](double w) { return std::fmod(grid - std::fmod(w, grid), grid); };
    for (double x = first(world_tl.x()); x < size.width(); x += grid) {
        p.fillRect(QRectF(std::floor(x), 0, 1, size.height()), grid_c);
    }
    for (double y = first(world_tl.y()); y < size.height(); y += grid) {
        p.fillRect(QRectF(0, std::floor(y), size.width(), 1), grid_c);
    }
    paintTiles(p, world_tl, QSizeF(size));
    p.setRenderHint(QPainter::Antialiasing, true);
    bd_.trail_on = trail_on_;
    bd_.trail_secs = trail_secs_;
    bd_.trail_n = s_.trail.size();
    bd_.trail_first = 0;
    if (trail_on_ && s_.trail.size() > 1) {
        const size_t first = trailFirst();
        bd_.trail_first = first;
        paintTrail(p, world_tl, first, s_.trail.size(), first, s_.trail.size() - first);
    }
    bd_.world_tl = world_tl;
    bd_.zoom = zoom_;
    bd_.layer = layer_;
    bd_.tiles_gen = tiles_gen_;
    bd_.built_ms = QDateTime::currentMSecsSinceEpoch();
}

void MapView::paintBackdrop(QPainter& p) {
    const QPointF centre = worldPx(clat_, clon_, zoom_);
    const QPointF view_tl = centre - QPointF(width() / 2.0, height() / 2.0);
    const QSize want(width() + 2 * kBackdropMargin, height() + 2 * kBackdropMargin);
    const qint64 now = QDateTime::currentMSecsSinceEpoch();
    const QPointF off = view_tl - bd_.world_tl;  // where the view sits inside the pixmap
    const bool inside = off.x() >= 0 && off.y() >= 0 && off.x() + width() <= bd_.pm.width() / bd_.pm.devicePixelRatio() &&
                        off.y() + height() <= bd_.pm.height() / bd_.pm.devicePixelRatio();
    const bool stale =
        bd_.pm.isNull() || bd_.zoom != zoom_ || bd_.layer != layer_ || !inside ||
        bd_.pm.size() != want * devicePixelRatioF() || bd_.trail_on != trail_on_ || bd_.trail_secs != trail_secs_ ||
        (bd_.tiles_gen != tiles_gen_) ||
        (bd_.missing > 0 && now - bd_.built_ms > 2000) ||                      // retry tiles that failed
        (trail_on_ && s_.trail.size() != bd_.trail_n && now - bd_.built_ms > 1000) ||  // fold the trail head in
        (trail_on_ && s_.trail.size() < bd_.trail_n);                           // trail restarted
    if (stale) {
        // snap to whole pixels so the shifted copy stays crisp
        const QPointF tl(std::floor(view_tl.x()) - kBackdropMargin, std::floor(view_tl.y()) - kBackdropMargin);
        buildBackdrop(tl, want);
    }
    const QPointF at = bd_.world_tl - view_tl;
    p.save();
    p.setRenderHint(QPainter::SmoothPixmapTransform, false);
    p.drawPixmap(QPointF(std::round(at.x()), std::round(at.y())), bd_.pm);
    p.restore();
}

void MapView::paintTiles(QPainter& p, const QPointF& world_tl, const QSizeF& size) {
    const int maxz = TileCache::maxZoom(layer_);
    const int z = std::clamp(static_cast<int>(std::floor(zoom_ + 0.25)), TileCache::kMinZoom, maxz);
    const double scale = std::exp2(zoom_ - z);
    const double ts = kTile * scale;
    const QPointF origin = -world_tl;  // world px at zoom_ == worldPx(z) * scale
    const int n = 1 << z;

    const int x0 = static_cast<int>(std::floor(-origin.x() / ts));
    const int x1 = static_cast<int>(std::floor((size.width() - origin.x()) / ts));
    const int y0 = std::max(0, static_cast<int>(std::floor(-origin.y() / ts)));
    const int y1 = std::min(n - 1, static_cast<int>(std::floor((size.height() - origin.y()) / ts)));
    bd_.missing = 0;

    p.save();
    p.setRenderHint(QPainter::Antialiasing, false);
    p.setRenderHint(QPainter::SmoothPixmapTransform, std::abs(scale - 1.0) > 1e-3);
    // request the centre tiles first so the area under the aircraft fills in first
    const double cx = (size.width() / 2.0 - origin.x()) / ts;
    const double cy = (size.height() / 2.0 - origin.y()) / ts;
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
        ++bd_.missing;
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

void MapView::paintRoute(QPainter& p, const MissionPlan& plan, const QColor& c, bool editable, int active_index) {
    const auto& pts = plan.wps;
    auto stroke = [&](const QPolygonF& line, double opacity, bool closed) {
        // glow underlay + marching ants showing the direction of travel
        p.setBrush(Qt::NoBrush);
        p.setPen(QPen(alpha(c, 0.22 * opacity), 6, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
        closed ? p.drawPolygon(line) : p.drawPolyline(line);
        QPen ants(alpha(c, opacity), 2.0, Qt::CustomDashLine, Qt::FlatCap, Qt::RoundJoin);
        ants.setDashPattern({5, 4});
        ants.setDashOffset(reducedMotion() ? 0.0 : -std::fmod(time_ * 14.0, 9.0));
        p.setPen(ants);
        closed ? p.drawPolygon(line) : p.drawPolyline(line);
    };

    // takeoff leg: home -> WP1, fainter (flown once)
    if (s_.has_home && !pts.empty()) {
        stroke(QPolygonF{toScreen(s_.home_lat, s_.home_lon), toScreen(pts[0].lat, pts[0].lon)}, 0.45, false);
    }
    // the loop itself, closed back to WP1
    if (pts.size() > 1) {
        QPolygonF loop;
        for (const auto& wp : pts) {
            loop << toScreen(wp.lat, wp.lon);
        }
        stroke(loop, 1.0, true);
    } else if (pts.size() == 1 && !compact_) {
        // one point: the aircraft circles it
        const QPointF sp = toScreen(pts[0].lat, pts[0].lon);
        QPen ring(alpha(c, 0.7), 1.5, Qt::DashLine);
        p.setPen(ring);
        p.setBrush(Qt::NoBrush);
        p.drawEllipse(sp, 30, 30);
    }

    if (compact_) {
        p.setPen(Qt::NoPen);
        p.setBrush(c);
        for (const auto& wp : pts) {
            p.drawEllipse(toScreen(wp.lat, wp.lon), 3, 3);
        }
        if (plan.land) {
            p.setBrush(kGreen);
            p.drawEllipse(toScreen(plan.land->lat, plan.land->lon), 3, 3);
        }
        return;
    }

    const QFont nf = monoFont(11, true);
    const QFont af = monoFont(10);
    auto marker = [&](const QPointF& sp, int id, const QString& label, const QColor& base, const QString& caption) {
        const bool is_active = id == active_index && active_index != kNone;
        const bool is_sel = editable && id == sel_;
        const bool hot = editable && (id == hover_wp_ || id == drag_wp_ || is_sel);
        const double rad = 12.0 + (hot ? 2.0 : 0.0) + (is_sel ? 2.0 : 0.0);
        QColor mc = base;
        if (is_active) {
            mc = kGreen;
            if (!reducedMotion()) {
                const double ph = std::fmod(time_ * 1.2, 1.0);
                p.setPen(QPen(alpha(kGreen, 1.0 - ph), 1.5));
                p.setBrush(Qt::NoBrush);
                p.drawPath(hexPath(sp, rad + 4 + 14 * ph));
            }
        }
        if (is_sel) {
            // selection: corner brackets, so it reads at a glance under a finger
            drawBrackets(p, QRectF(sp.x() - rad - 7, sp.y() - rad - 7, 2 * rad + 14, 2 * rad + 14), 6, kText, 1.6);
        }
        const QPainterPath hex = hexPath(sp, rad);
        p.setPen(Qt::NoPen);
        p.setBrush(alpha(kBg, 0.85));
        p.drawPath(hex);
        drawGlowPath(p, hex, mc, hot ? 2.0 : 1.4, hot ? 8 : 4);
        drawText(p, QRectF(sp.x() - 12, sp.y() - 12, 24, 24), Qt::AlignCenter, label, nf, mc);
        if (!caption.isEmpty()) {
            // beside the marker, so route legs (which leave from the centre) rarely cross it
            const double cw = QFontMetricsF(af).horizontalAdvance(caption) + 10;
            const QRectF cr(sp.x() + rad + 4, sp.y() - 8, cw, 16);
            p.setPen(Qt::NoPen);
            p.setBrush(alpha(kBg, 0.82));
            p.drawRect(cr);
            drawText(p, cr, Qt::AlignCenter, caption, af, alpha(mc, 0.95));
        }
    };

    for (size_t i = 0; i < pts.size(); ++i) {
        marker(toScreen(pts[i].lat, pts[i].lon), static_cast<int>(i), QString::number(i + 1), c,
               units::altText(pts[i].alt_rel));
    }
    if (plan.land) {
        const QPointF lp = toScreen(plan.land->lat, plan.land->lon);
        // spiral glyph: the approach is flown on the wing, circling down
        QPainterPath spiral;
        for (int k = 0; k <= 90; ++k) {
            const double t = k / 90.0;
            const double ang = t * 4.0 * geo::kPi + time_ * (reducedMotion() ? 0.0 : 0.6);
            const double rr = 34.0 - 16.0 * t;
            const QPointF q = lp + QPointF(std::cos(ang) * rr, std::sin(ang) * rr);
            k == 0 ? spiral.moveTo(q) : spiral.lineTo(q);
        }
        p.setBrush(Qt::NoBrush);
        p.setPen(QPen(alpha(kGreen, 0.45), 1.3, Qt::DashLine));
        p.drawPath(spiral);
        marker(lp, kLandId, QStringLiteral("L"), kGreen,
               QStringLiteral("LAND · approach %1").arg(units::altText(plan.land->alt_rel)));
    }
}

void MapView::paintOrbit(QPainter& p) {
    if (!orbit_) {
        return;
    }
    const QPointF c = toScreen(orbit_->lat, orbit_->lon);
    const double rpx = orbit_->radius / metersPerPixel();
    const QColor col = orbit_->hold ? kCyan : kAmber;
    // line from the aircraft while it's still on its way to the circle
    if (has_ac_) {
        const QPointF a = toScreen(ac_lat_, ac_lon_);
        if (QLineF(a, c).length() > rpx * 1.3) {
            QPen dash(alpha(col, 0.5), 1.2, Qt::DashLine);
            dash.setDashOffset(-time_ * 8.0);
            p.setPen(dash);
            p.drawLine(a, c);
        }
    }
    if (rpx > 4.0) {
        QPen ring(alpha(col, 0.75), compact_ ? 1.2 : 1.8, Qt::CustomDashLine);
        ring.setDashPattern({6, 5});
        // the dashes march in the direction of the turn
        const double phase = reducedMotion() ? 0.0 : std::fmod(time_ * 10.0, 11.0);
        ring.setDashOffset(orbit_->ccw ? -phase : phase);
        p.setPen(ring);
        p.setBrush(alpha(col, 0.05));
        p.drawEllipse(c, rpx, rpx);
        // the circle actually flown while the radius trim is still closing in
        if (orbit_->flown > 0.0 && !orbit_->trimmed && !compact_) {
            const double fpx = orbit_->flown / metersPerPixel();
            QPen flown(alpha(kMuted, 0.6), 1.0, Qt::DotLine);
            p.setPen(flown);
            p.setBrush(Qt::NoBrush);
            p.drawEllipse(c, fpx, fpx);
            drawText(p, QRectF(c.x() - 80, c.y() + fpx + 2, 160, 14), Qt::AlignCenter,
                     QStringLiteral("flown %1 · trimming").arg(formatDistance(orbit_->flown)), labelFont(9),
                     alpha(kMuted, 0.9));
        }
        if (!compact_) {
            // direction chevrons at N / E / S / W
            for (int k = 0; k < 4; ++k) {
                const double ang = k * geo::kPi / 2.0;  // screen angle from +x
                const QPointF at = c + QPointF(std::cos(ang) * rpx, std::sin(ang) * rpx);
                // tangent: clockwise on screen (y down) is +90 deg from the radius
                const double t = ang + (orbit_->ccw ? -geo::kPi / 2.0 : geo::kPi / 2.0);
                const QPointF dir(std::cos(t), std::sin(t));
                const QPointF n(-dir.y(), dir.x());
                QPolygonF chev({at + dir * 6.0, at - dir * 4.0 + n * 5.0, at - dir * 1.0, at - dir * 4.0 - n * 5.0});
                p.setPen(Qt::NoPen);
                p.setBrush(alpha(col, 0.9));
                p.drawPolygon(chev);
            }
        }
    }
    p.save();
    p.translate(c);
    p.rotate(reducedMotion() ? 45.0 : std::fmod(time_ * 30.0, 90.0));
    drawBrackets(p, QRectF(-12, -12, 24, 24), 6, col, 2.0);
    p.restore();
    p.setPen(Qt::NoPen);
    p.setBrush(col);
    p.drawEllipse(c, 2.5, 2.5);
}

void MapView::paintApproach(QPainter& p) {
    if (!approach_ || compact_) {
        return;
    }
    const auto& a = *approach_;
    const int lseq = landSeq(s_.mission);
    const bool active = lseq >= 0 && s_.mission_current >= lseq;
    const double op = active ? 1.0 : 0.35;
    const QPointF c = toScreen(a.c_lat, a.c_lon);
    const QPointF e = toScreen(a.e_lat, a.e_lon);
    const QPointF l = toScreen(a.l_lat, a.l_lon);
    const double rpx = a.radius / metersPerPixel();
    // descent circle beside L, the tangent exit, and the straight leg into the wind
    QPen ring(alpha(kGreen, 0.6 * op), 1.4, Qt::DashLine);
    p.setPen(ring);
    p.setBrush(Qt::NoBrush);
    p.drawEllipse(c, rpx, rpx);
    QPen leg(alpha(kGreen, 0.9 * op), 2.0, Qt::CustomDashLine, Qt::FlatCap);
    leg.setDashPattern({5, 4});
    leg.setDashOffset(reducedMotion() ? 0.0 : -std::fmod(time_ * 14.0, 9.0));
    p.setPen(leg);
    p.drawLine(e, l);
    p.setPen(Qt::NoPen);
    p.setBrush(alpha(kGreen, op));
    p.drawEllipse(e, 3.5, 3.5);
    // wind arrow beside L: blows toward the downwind side
    if (s_.has_wind && s_.wind_speed >= 0.5) {
        const double to = (s_.wind_from_deg + 180.0) * geo::kDegToRad;  // compass, 0 = up
        const QPointF dir(std::sin(to), -std::cos(to));
        const QPointF base = l + QPointF(-dir.y(), dir.x()) * 34.0 - dir * 18.0;
        const QPointF tip = base + dir * 36.0;
        p.setPen(QPen(alpha(kText, 0.8 * op + 0.2), 1.8));
        p.drawLine(base, tip);
        const QPointF n(-dir.y(), dir.x());
        p.drawLine(tip, tip - dir * 8.0 + n * 5.0);
        p.drawLine(tip, tip - dir * 8.0 - n * 5.0);
        drawText(p, QRectF(base.x() - 30, base.y() - 22, 60, 14), Qt::AlignCenter,
                 QStringLiteral("%1 m/s").arg(s_.wind_speed, 0, 'f', 1), monoFont(10), alpha(kText, 0.9));
    }
}

void MapView::paintAircraft(QPainter& p) {
    if (!s_.has_position || !has_ac_) {
        return;
    }
    const QPointF c = toScreen(ac_lat_, ac_lon_);
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
        follow_chip_ = layer_chip_ = ball_rect_ = QRectF();
        return;
    }
    paintBall(p);
    const double right = width() - 16.0;
    double y = height() - 24.0;

    // scale bar
    const double mpp = metersPerPixel();
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
    if (s_.has_position && s_.mission_known) {
        const int idx = seqToWaypointIndex(s_.mission, s_.mission_current);
        if (idx >= 0 && idx < static_cast<int>(vehicle_plan_.wps.size())) {
            const auto& wp = vehicle_plan_.wps[idx];
            parts << QStringLiteral("WP%1 %2").arg(idx + 1).arg(formatDistance(geo::distance(s_.lat, s_.lon, wp.lat, wp.lon)));
        } else if (vehicle_plan_.land && s_.mission_current == landSeq(s_.mission)) {
            const auto& l = *vehicle_plan_.land;
            parts << QStringLiteral("LAND %1").arg(formatDistance(geo::distance(s_.lat, s_.lon, l.lat, l.lon)));
        }
    }
    if (orbit_ && s_.has_position) {
        parts << QStringLiteral("%1 %2")
                     .arg(orbit_->hold ? QStringLiteral("HOLD") : QStringLiteral("TGT"))
                     .arg(formatDistance(geo::distance(s_.lat, s_.lon, orbit_->lat, orbit_->lon)));
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
        const QFont f = labelFont(12);
        const double w = QFontMetricsF(f).horizontalAdvance(text) + 28;
        out = QRectF(x_right - std::max(w, 72.0), y - 44, std::max(w, 72.0), 44);  // >= 44 px: finger-sized
        drawGlass(p, chamferPath(out, 6), on ? kCyan : kMuted, on ? 0.8 : 0.4, kGlassDeep);
        drawText(p, out, Qt::AlignCenter, text, f, on ? kCyan : kMuted);
        return out.left() - 8;
    };
    double xr = right;
    xr = chip(layer_chip_, xr, layer_ == TileCache::Satellite ? QStringLiteral("▣ Sat") : QStringLiteral("▣ Map"), true);
    chip(follow_chip_, xr, QStringLiteral("◎ Follow"), follow_);
    paintHover(p, right, y - 52);

    // mode hint
    if (mode_ == Mode::Guide) {
        const QString text = QStringLiteral("GUIDE · tap the map to send the aircraft");
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

void MapView::paintFootprint(QPainter& p) {
    if (!footprint_ || !has_ac_ || footprint_->outline.size() < 3) {
        return;
    }
    const auto& fp = *footprint_;
    auto at = [&](const geo::NE& ne) {
        double lat = 0.0, lon = 0.0;
        geo::offset(ac_lat_, ac_lon_, ne.north, ne.east, lat, lon);
        return toScreen(lat, lon);
    };
    QPolygonF poly;
    for (const auto& ne : fp.outline) {
        poly << at(ne);
    }
    const QPointF ac = toScreen(ac_lat_, ac_lon_);
    const QColor col = kGreen;

    // fill: brighter toward the aircraft, so the near edge reads as solid ground
    QPainterPath path;
    path.addPolygon(poly);
    path.closeSubpath();
    QRadialGradient fill(ac, std::max(40.0, QLineF(ac, at(fp.center)).length() * 2.2));
    fill.setColorAt(0.0, alpha(col, compact_ ? 0.16 : 0.22));
    fill.setColorAt(1.0, alpha(col, 0.05));
    p.setPen(Qt::NoPen);
    p.setBrush(fill);
    p.drawPath(path);

    // scan line sweeping from the near edge to the far edge
    if (!reducedMotion() && !compact_) {
        const double ph = std::fmod(time_ * 0.45, 1.0);
        p.save();
        p.setClipPath(path);
        const QPointF far = at(fp.center);
        const QPointF dir = far - ac;
        const double len = std::hypot(dir.x(), dir.y());
        if (len > 4.0) {
            const QPointF u = dir / len, n(-u.y(), u.x());
            const QPointF m = ac + dir * (0.15 + 1.9 * ph);
            p.setPen(QPen(alpha(col, 0.55 * (1.0 - ph)), 2.0));
            p.drawLine(m - n * 4000.0, m + n * 4000.0);
        }
        p.restore();
    }

    // edges: solid where the camera sees the ground, dashed where it runs off
    // past the horizon / max range
    const int n = static_cast<int>(poly.size());
    for (int i = 0; i < n; ++i) {
        const int j = (i + 1) % n;
        const bool open = fp.open[i] || fp.open[j];
        QPen pen(alpha(col, open ? 0.35 : 0.85), open ? 1.2 : 1.6);
        if (open) {
            pen.setStyle(Qt::DashLine);
        }
        p.setPen(pen);
        p.drawLine(poly[i], poly[j]);
    }

    // line of sight to the boresight point
    const QPointF bc = at(fp.center);
    QPen los(alpha(col, 0.7), 1.2, Qt::DashLine);
    p.setPen(los);
    p.drawLine(ac, bc);
    if (fp.center_hit) {
        p.setPen(QPen(col, 1.6));
        p.setBrush(Qt::NoBrush);
        const double s = compact_ ? 5 : 8;
        p.drawEllipse(bc, s, s);
        p.drawLine(bc + QPointF(-s - 6, 0), bc + QPointF(-s + 2, 0));
        p.drawLine(bc + QPointF(s - 2, 0), bc + QPointF(s + 6, 0));
        p.drawLine(bc + QPointF(0, -s - 6), bc + QPointF(0, -s + 2));
        p.drawLine(bc + QPointF(0, s - 2), bc + QPointF(0, s + 6));
        if (!compact_) {
            const QString text = QStringLiteral("CAM %1").arg(formatDistance(std::hypot(fp.center.north, fp.center.east)));
            const QFont f = labelFont(10);
            const double w = QFontMetricsF(f).horizontalAdvance(text) + 14;
            const QRectF box(bc.x() + 14, bc.y() - 22, w, 18);
            drawGlass(p, chamferPath(box, 4), col, 0.6, kGlassDeep);
            drawText(p, box, Qt::AlignCenter, text, f, col);
        }
    }
}

void MapView::paintBall(QPainter& p) {
    if (!has_att_) {
        ball_rect_ = QRectF();
        return;
    }
    // top-left, under the telemetry bar; toasts stack on the right
    const double e = easeInOutCubic(ball_open_);
    const double r = lerp(22.0, 58.0, e);
    const double pad = lerp(8.0, 30.0, e);
    const double box_h = 2 * r + pad * 2 + (e > 0.5 ? 26.0 : 0.0);
    const QRectF box(16, 72, 2 * r + pad * 2, box_h);
    ball_rect_ = box.adjusted(-4, -4, 4, 4);  // >= 48 px even collapsed
    drawGlass(p, chamferPath(box, 10), kCyan, 0.45, alpha(kGlassDeep, 0.85));
    const QPointF c(box.center().x(), box.top() + pad + r);
    paintAttitudeBall(p, c, r, roll_.value, pitch_.value, heading_.value, e < 0.5);
    if (e > 0.5) {
        drawText(p, QRectF(box.left() + 10, box.top() + 4, 80, 14), Qt::AlignLeft | Qt::AlignVCenter,
                 QStringLiteral("ATTITUDE"), labelFont(9), alpha(kMuted, e));
    }

    // wind, under the ball (north-up like the map); collapses with it
    WindInfo w;
    w.valid = s_.has_wind;
    w.from_deg = s_.wind_from_deg;
    w.speed = s_.wind_speed;
    const double wh = lerp(box.width(), 132.0, e);
    const QRectF wbox(box.left(), box.bottom() + 8, box.width(), wh);
    drawGlass(p, chamferPath(wbox, 10), kCyan, 0.45, alpha(kGlassDeep, 0.85));
    if (e > 0.5) {
        drawText(p, QRectF(wbox.left() + 10, wbox.top() + 4, 80, 14), Qt::AlignLeft | Qt::AlignVCenter,
                 QStringLiteral("WIND"), labelFont(9), alpha(kMuted, e));
    }
    paintWindGauge(p, wbox.adjusted(8, e > 0.5 ? 20 : 6, -8, -6), w, heading_.value, time_, true, e < 0.5);
    ball_rect_ = ball_rect_.united(wbox.adjusted(-4, -4, 4, 4));
}

}  // namespace gcs::ui
