#pragma once

#include "anim.hpp"
#include "core/flight_status.hpp"
#include "core/mission.hpp"
#include "core/vehicle_state.hpp"
#include "view_host.hpp"

#include <QColor>

#include <array>
#include <functional>
#include <optional>
#include <vector>

class QTouchEvent;

namespace gcs::ui {

class ElevationCache;

class TileCache;


class MapView : public HudView {
    Q_OBJECT

public:
    enum class Mode { Normal, Plan, Guide, Area };
    enum class AddTool { Waypoint, Landing };
    // selection ids: >= 0 a loop waypoint index, kLandId the landing point
    static constexpr int kNone = -1;
    static constexpr int kLandId = -2;

    explicit MapView(TileCache* tiles, QWidget* parent = nullptr);

    QString pipTag() const override;

    // applied to the plan read from the vehicle (hold times it can't store)
    void setPlanFixup(std::function<void(MissionPlan&)> fn) { plan_fixup_ = std::move(fn); }
    void setState(const VehicleState& s);
    void setMode(Mode m);
    Mode mode() const { return mode_; }
    void setLayer(int layer);
    int layer() const { return layer_; }
    void setFollow(bool on);
    bool follow() const { return follow_; }
    void setDefaultAltitude(float alt) { default_alt_ = alt; }
    void setDefaultLandAltitude(float alt) { default_land_alt_ = alt; }
    // the autopilot's WP_LOITER_RAD: a hold with radius 0 circles at this
    void setDefaultLoiterRadius(double m) { default_loiter_radius_ = m; }
    // the nav box (bottom right): the ground under the aircraft, and where it is going
    struct NavInfo {
        QString aircraft_ground;  // "GROUND 512 m MSL · AGL 80 m (rangefinder)"
        NavTarget target;
    };
    void setNav(NavInfo nav) { nav_ = std::move(nav); }
    double defaultLoiterRadius() const { return default_loiter_radius_; }

    // ---- plan editing ------------------------------------------------------
    const MissionPlan& plan() const { return plan_; }
    void setPlan(MissionPlan plan);
    void clearPlan();
    void undo();
    bool canUndo() const { return !undo_.empty(); }
    void setAddTool(AddTool t);
    AddTool addTool() const { return tool_; }
    int selected() const { return sel_; }
    void select(int id);
    void adjustAltitude(int id, float delta);  // waypoint or kLandId; one undo step per point
    void addPoint(double lat, double lon, bool landing);  // as a tap would (landing: sets L)
    std::optional<UserWaypoint> point(int id) const;      // waypoint or kLandId
    // Edit one point with undo. coalesce_key >= 0: consecutive edits with the
    // same key on the same point undo in one step (steppers); kNone: own step.
    void editPoint(int id, const std::function<void(UserWaypoint&)>& fn, int coalesce_key = kNone);
    void deletePoint(int id);
    void insertAfter(int index);                  // midpoint to the next point (past the end of an open route)
    void setPattern(Pattern pattern);             // how the route is flown (undoable)
    void setHold(float radius, bool ccw);         // the final hold circle (undoable; radius 0 = default)

    // ---- motion (predicted, set every frame) ---------------------------------
    void setAircraftPosition(double lat, double lon);
    void setHeading(double deg);
    void setAttitude(double roll_deg, double pitch_deg);  // attitude ball

    // ---- camera footprint: the ground the camera sees ------------------------
    struct FootprintDraw {
        std::vector<geo::NE> outline;  // metres from the aircraft
        std::vector<bool> open;        // beyond the horizon / max range
        geo::NE center;
        bool center_hit = false;
    };
    void setFootprint(std::optional<FootprintDraw> fp);
    void setBallCollapsed(bool on);
    // flight trail: off, or the last `secs` seconds (0 = everything kept)
    void setTrail(bool on, int secs);
    // ground height for the mouse readout (lat, lon, ground, distance from the aircraft)
    void setElevation(ElevationCache* elev);

    // ---- full-detail area selection (Mode::Area) ----------------------------------
    struct GeoBox {
        double min_lon = 0, min_lat = 0, max_lon = 0, max_lat = 0;
    };
    void setAreas(std::vector<GeoBox> areas);  // already downloaded (outlines)
    std::optional<GeoBox> areaSelection() const { return area_sel_; }
    void clearAreaSelection();
    // the panel under the selection: two lines and whether DOWNLOAD is allowed
    void setAreaInfo(const QString& line1, const QString& line2, bool can_download, bool warn);
    // a chip while an offline download runs ("" hides it)
    void setDownloadStatus(const QString& text);
    void tileSourcesChanged();  // offline data changed: redraw the backdrop
    double centerLat() const { return clat_; }
    double centerLon() const { return clon_; }

    // ---- the circle commanded in GUIDED (Hold / Guide) --------------------------
    struct OrbitDraw {
        double lat = 0.0;
        double lon = 0.0;
        double radius = 80.0;
        bool ccw = false;
        bool hold = true;
        double flown = -1.0;    // measured flown radius, < 0 unknown
        bool trimmed = false;   // the flown circle matches the request
    };
    void setOrbit(std::optional<OrbitDraw> orbit);
    void ping(double lat, double lon, const QColor& color);

    void tick(double dt);

signals:
    void planChanged();
    void guideClicked(double lat, double lon);
    void selectionChanged(int id);
    void addToolChanged(AddTool t);
    void followChanged(bool on);
    void layerChanged(int layer);
    void ballCollapsedChanged(bool collapsed);
    void areaChanged(GeoBox box, bool final);  // final: the finger / button was released
    void areaDownload();
    void areaCancel();

protected:
    bool event(QEvent* e) override;
    void paintEvent(QPaintEvent* e) override;

public:
    int paints_ = 0;  // paint events since the last GCS_FPS report

protected:
    void mousePressEvent(QMouseEvent* e) override;
    void mouseMoveEvent(QMouseEvent* e) override;
    void mouseReleaseEvent(QMouseEvent* e) override;
    void wheelEvent(QWheelEvent* e) override;
    void leaveEvent(QEvent* e) override;
    void compactChanged() override { updateCursor(); }

private:
    QPointF toScreen(double lat, double lon) const;
    void toLatLon(const QPointF& pt, double& lat, double& lon) const;
    int planPointAt(const QPointF& pos) const;  // waypoint index, kLandId or kNone
    void pushUndo();
    void editPlan();  // pushUndo + ends alt-edit coalescing
    UserWaypoint* pointPtr(int id);
    void pointerPress(const QPointF& pos);
    void pointerMove(const QPointF& pos);
    void pointerRelease(const QPointF& pos);
    void handleTouch(QTouchEvent* e);
    void updateCursor();

    // the slow-changing background (grid, tiles, the trail but its newest
    // points) is drawn into bd_ with a margin and only shifted per frame
    void paintBackdrop(QPainter& p);
    void buildBackdrop(const QPointF& world_tl, const QSize& size);
    void paintTiles(QPainter& p, const QPointF& world_tl, const QSizeF& size);
    void paintTrail(QPainter& p, const QPointF& world_tl, size_t from, size_t to, size_t first, size_t span);
    size_t trailFirst() const;  // first trail point inside the time window
    void paintRoute(QPainter& p, const MissionPlan& plan, const QColor& c, bool editable, int active_index);
    void paintAircraft(QPainter& p);
    void paintOrbit(QPainter& p);
    void paintApproach(QPainter& p);
    void paintFootprint(QPainter& p);
    void paintBall(QPainter& p);
    // text size: the on-screen chrome (paintOverlayChrome) is painted scaled
    // by chrome_k_ in a logical rect; taps are divided by it
    double chrome_k_ = 1.0;
    QRectF lrect() const;
    double lw() const;
    double lh() const;
    double metersPerPixel() const;
    void paintOverlayChrome(QPainter& p);
    void paintArea(QPainter& p);
    bool areaPress(const QPointF& pos);  // true: handled
    std::array<QPointF, 4> areaCorners() const;  // NW NE SE SW on screen
    void paintNavBox(QPainter& p, double right, double bottom);

    TileCache* tiles_;
    ElevationCache* elev_ = nullptr;
    std::optional<QPointF> hover_pos_;  // mouse over the map (never set by touch)
    // last ground-height lookup, redone only when the point or the data changes
    struct HoverElev {
        double lat = 1e9, lon = 1e9;
        quint64 gen = ~0ull;
        std::optional<double> h;
    } hover_elev_;
    quint64 elev_gen_ = 0;
    VehicleState s_;
    std::function<void(MissionPlan&)> plan_fixup_;
    MissionPlan vehicle_plan_;
    bool centred_ = false;

    double clat_ = 20.0;
    double clon_ = 0.0;
    double zoom_ = 3.0;
    double zoom_target_ = 3.0;
    bool anchor_valid_ = false;
    double anchor_lat_ = 0.0, anchor_lon_ = 0.0;
    QPointF anchor_px_;

    Mode mode_ = Mode::Normal;
    int layer_ = 0;
    bool follow_ = true;
    float default_alt_ = 80.0f;
    float default_land_alt_ = kDefaultLandApproachAlt;
    double default_loiter_radius_ = 80.0;
    NavInfo nav_;

    MissionPlan plan_;
    std::vector<MissionPlan> undo_;
    AddTool tool_ = AddTool::Waypoint;
    int sel_ = kNone;
    int alt_edit_wp_ = kNone;  // point whose altitude edits share one undo step

    std::optional<OrbitDraw> orbit_;
    std::optional<LandingApproach> approach_;  // from the vehicle's mission
    bool has_ac_ = false;
    double ac_lat_ = 0.0, ac_lon_ = 0.0;       // predicted aircraft position

    struct Ping {
        double lat, lon;
        QColor color;
        double t;
    };
    std::vector<Ping> pings_;

    // interaction
    bool pressing_ = false;
    bool dragged_ = false;
    QPointF press_pos_;
    QPointF last_pos_;
    int drag_wp_ = kNone;
    int hover_wp_ = kNone;
    bool touch_input_ = false;  // last pointer was a finger: bigger hit radius, drag slop
    bool pinching_ = false;
    double pinch_dist0_ = 0.0;
    double pinch_zoom0_ = 0.0;
    double pinch_lat_ = 0.0, pinch_lon_ = 0.0;
    QRectF follow_chip_, layer_chip_, ball_rect_;

    std::optional<FootprintDraw> footprint_;
    Tween roll_, pitch_;
    bool has_att_ = false;
    bool ball_collapsed_ = false;
    bool trail_on_ = true;
    int trail_secs_ = 120;
    double ball_open_ = 1.0;  // 0 collapsed .. 1 open, animated
    Tween as_, gs_;

    AngleTween heading_;

    struct Backdrop {
        QPixmap pm;
        QPointF world_tl;          // world px (at `zoom`) of the pixmap's top-left
        double zoom = -1.0;
        int layer = -1;
        quint64 tiles_gen = 0;
        int missing = 0;           // tiles not available when it was drawn
        int drawn = 0;             // tiles (or enlarged parents) drawn
        qint64 built_ms = 0;
        size_t trail_n = 0;        // trail points baked in
        size_t trail_first = 0;
        bool trail_on = false;
        int trail_secs = -1;
    } bd_;
    quint64 tiles_gen_ = 0;        // bumped when a tile arrives
    // area selection
    std::vector<GeoBox> areas_;
    std::optional<GeoBox> area_sel_;
    int area_handle_ = -1;         // corner being dragged (0 NW 1 NE 2 SE 3 SW)
    double area_anchor_lat_ = 0, area_anchor_lon_ = 0;  // the fixed corner while drawing a new one
    bool area_new_ = false;
    QString area_info1_, area_info2_;
    bool area_can_dl_ = false, area_warn_ = false;
    QRectF area_dl_rect_, area_cancel_rect_;
    QString dl_status_;
    static constexpr int kBackdropMargin = 320;
};

}  // namespace gcs::ui
