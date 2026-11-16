#pragma once

#include "anim.hpp"
#include "core/mission.hpp"
#include "core/vehicle_state.hpp"
#include "view_host.hpp"

#include <QColor>

#include <optional>
#include <vector>

class QTouchEvent;

namespace gcs::ui {

class TileCache;


class MapView : public HudView {
    Q_OBJECT

public:
    enum class Mode { Normal, Plan, Guide };
    enum class AddTool { Waypoint, Landing };
    // selection ids: >= 0 a loop waypoint index, kLandId the landing point
    static constexpr int kNone = -1;
    static constexpr int kLandId = -2;

    explicit MapView(TileCache* tiles, QWidget* parent = nullptr);

    QString pipTag() const override;

    void setState(const VehicleState& s);
    void setMode(Mode m);
    Mode mode() const { return mode_; }
    void setLayer(int layer);
    int layer() const { return layer_; }
    void setFollow(bool on);
    bool follow() const { return follow_; }
    void setDefaultAltitude(float alt) { default_alt_ = alt; }
    void setDefaultLandAltitude(float alt) { default_land_alt_ = alt; }

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
    void deletePoint(int id);
    void insertAfter(int index);                  // midpoint to the next loop point

    // ---- motion (predicted, set every frame) ---------------------------------
    void setAircraftPosition(double lat, double lon);
    void setHeading(double deg);

    // ---- the circle commanded in GUIDED (Hold / Guide) --------------------------
    struct OrbitDraw {
        double lat = 0.0;
        double lon = 0.0;
        double radius = 80.0;
        bool ccw = false;
        bool hold = true;
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

protected:
    bool event(QEvent* e) override;
    void paintEvent(QPaintEvent* e) override;
    void mousePressEvent(QMouseEvent* e) override;
    void mouseMoveEvent(QMouseEvent* e) override;
    void mouseReleaseEvent(QMouseEvent* e) override;
    void wheelEvent(QWheelEvent* e) override;
    void compactChanged() override { updateCursor(); }

private:
    QPointF toScreen(double lat, double lon) const;
    void toLatLon(const QPointF& pt, double& lat, double& lon) const;
    int planPointAt(const QPointF& pos) const;  // waypoint index, kLandId or kNone
    void pushUndo();
    void editPlan();  // pushUndo + ends alt-edit coalescing
    void pointerPress(const QPointF& pos);
    void pointerMove(const QPointF& pos);
    void pointerRelease(const QPointF& pos);
    void handleTouch(QTouchEvent* e);
    void updateCursor();

    void paintTiles(QPainter& p);
    void paintRoute(QPainter& p, const MissionPlan& plan, const QColor& c, bool editable, int active_index);
    void paintAircraft(QPainter& p);
    void paintOrbit(QPainter& p);
    void paintApproach(QPainter& p);
    double metersPerPixel() const;
    void paintOverlayChrome(QPainter& p);

    TileCache* tiles_;
    VehicleState s_;
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
    QRectF follow_chip_, layer_chip_;

    AngleTween heading_;
};

}  // namespace gcs::ui
