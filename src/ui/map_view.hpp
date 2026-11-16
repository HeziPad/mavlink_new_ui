#pragma once

#include "anim.hpp"
#include "core/mission.hpp"
#include "core/vehicle_state.hpp"
#include "view_host.hpp"

#include <QColor>

#include <vector>

namespace gcs::ui {

class TileCache;

class MapView : public HudView {
    Q_OBJECT

public:
    enum class Mode { Normal, Plan, Guide };

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

    // ---- plan editing ------------------------------------------------------
    const std::vector<UserWaypoint>& plan() const { return plan_; }
    void setPlan(std::vector<UserWaypoint> plan);
    void clearPlan();
    void undo();
    bool canUndo() const { return !undo_.empty(); }

    // ---- guided ------------------------------------------------------------
    void setGuidedTarget(bool has, double lat = 0.0, double lon = 0.0);
    void ping(double lat, double lon, const QColor& color);

    void tick(double dt);

signals:
    void planChanged();
    void guideClicked(double lat, double lon);
    void followChanged(bool on);
    void layerChanged(int layer);

protected:
    void paintEvent(QPaintEvent* e) override;
    void mousePressEvent(QMouseEvent* e) override;
    void mouseMoveEvent(QMouseEvent* e) override;
    void mouseReleaseEvent(QMouseEvent* e) override;
    void wheelEvent(QWheelEvent* e) override;
    void compactChanged() override { updateCursor(); }

private:
    QPointF toScreen(double lat, double lon) const;
    void toLatLon(const QPointF& pt, double& lat, double& lon) const;
    int planPointAt(const QPointF& pos) const;
    void pushUndo();
    void updateCursor();

    void paintTiles(QPainter& p);
    void paintRoute(QPainter& p, const std::vector<UserWaypoint>& pts, const QColor& c, bool editable,
                    int active_index);
    void paintAircraft(QPainter& p);
    void paintOverlayChrome(QPainter& p);

    TileCache* tiles_;
    VehicleState s_;
    std::vector<UserWaypoint> vehicle_wps_;
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

    std::vector<UserWaypoint> plan_;
    std::vector<std::vector<UserWaypoint>> undo_;

    bool has_target_ = false;
    double target_lat_ = 0.0, target_lon_ = 0.0;

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
    int drag_wp_ = -1;
    int hover_wp_ = -1;
    QRectF follow_chip_, layer_chip_;

    AngleTween heading_;
};

}  // namespace gcs::ui
