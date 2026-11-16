#pragma once

#include "core/vehicle_state.hpp"
#include "link/link_events.hpp"

#include <QElapsedTimer>
#include <QString>
#include <QWidget>

#include <memory>
#include <optional>
#include <string>

class QTimer;

namespace gcs::link {
class MavlinkClient;
}

namespace gcs::ui {

class CameraView;
class ControlDock;
class FxOverlay;
class MapView;
class OrbitPanel;
class PlanStrip;
class TelemetryBar;
class TileCache;
class VideoReceiver;
class ViewHost;
class WaypointPanel;

struct AppSettings {
    QString host = QStringLiteral("127.0.0.1");
    int port = 14551;
    int video_port = 5600;
    float takeoff_alt = 40.0f;
    float waypoint_alt = 80.0f;
    float land_approach_alt = 30.0f;  // fixed-wing spiral descends to this, then VTOL
    double camera_hfov = 70.0;
    double camera_tilt = -30.0;
    bool gimbal_body_frame = false;  // ArduPilot servo mounts report body-frame angles
    int map_layer = 0;
    bool follow = true;
    bool reduced_motion = false;
    double cruise_speed = 18.0;

    static AppSettings load();
    void save() const;
};

// Composes the views, HUD and link, and owns the meaning of the six dock
// slots (Plan, Launch/Auto, Guide, Hold, Land, QRTL). Everything runs off one 60 Hz timer: state is pulled from the link
// at 30 Hz, animations step every tick.
class MainWindow : public QWidget {
    Q_OBJECT

public:
    explicit MainWindow(const AppSettings& settings, QWidget* parent = nullptr);
    ~MainWindow() override;

protected:
    void resizeEvent(QResizeEvent* e) override;
    void closeEvent(QCloseEvent* e) override;
    bool eventFilter(QObject* obj, QEvent* e) override;

private:
    // Map/camera swap has no slot: tap the picture-in-picture.
    enum Slot { kPlan = 0, kLaunch = 1, kGuide = 2, kHold = 3, kLand = 4, kQrtl = 5 };

    void frame();
    void pullState();
    void handleEvent(const link::LinkEvent& ev);
    void updateAlerts();
    void refreshDock();
    void refreshPlanStrip();
    void refreshPanel();
    void refreshOrbitPanel();
    void updateMotion();
    void checkTakeoffHandoff();
    void layoutOverlays();

    void onTriggered(int slot);
    void onLongPressed(int slot);
    void enterPlan();
    void exitPlan();
    void syncPlan();
    void setGuide(bool on);
    void sendGuided(double lat, double lon);
    void sendOrbit();
    void adjustOrbit(float d_alt, float d_radius, bool flip);
    double defaultRadius() const;
    void startLandingApproach();
    void onCameraClick(double px, double py, double w, double h);
    void launchOrAuto();
    void hold();
    void land();
    void qrtl();
    int slotForTag(const std::string& tag) const;

    AppSettings settings_;
    std::unique_ptr<link::MavlinkClient> client_;
    std::unique_ptr<VideoReceiver> video_;
    TileCache* tiles_ = nullptr;

    MapView* map_ = nullptr;
    CameraView* camera_ = nullptr;
    ViewHost* host_ = nullptr;
    TelemetryBar* bar_ = nullptr;
    ControlDock* dock_ = nullptr;
    PlanStrip* strip_ = nullptr;
    WaypointPanel* panel_ = nullptr;
    OrbitPanel* orbit_panel_ = nullptr;
    FxOverlay* fx_ = nullptr;

    QTimer* timer_ = nullptr;
    QElapsedTimer clock_;
    qint64 last_ms_ = 0;
    int frame_count_ = 0;

    VehicleState s_;
    bool planning_ = false;
    bool guiding_ = false;
    bool uploading_ = false;
    bool ever_connected_ = false;
    bool link_was_alive_ = false;
    std::string last_mode_;
    int last_vtol_ = -1;
    // GUIDED takeoff ends in a rotor hover; hand over to LOITER on the wing
    // once it reaches altitude (rotors are for takeoff and landing only)
    bool takeoff_handoff_ = false;
    bool handoff_seen_guided_ = false;
    LiftRotors last_lift_ = LiftRotors::Unknown;

    // The circle we commanded in GUIDED (Hold or a Guide target) via
    // DO_REPOSITION. Edited live from the orbit panel; changes are coalesced
    // and re-sent at most every kOrbitResendMs.
    struct Orbit {
        double lat = 0.0;
        double lon = 0.0;
        float alt = 80.0f;
        float radius = 80.0f;
        bool ccw = false;
        bool hold = true;  // false: a Guide target
    };
    std::optional<Orbit> orbit_;
    bool orbit_dirty_ = false;
    qint64 orbit_sent_ms_ = 0;
    static constexpr qint64 kOrbitResendMs = 300;

    // Land: the approach is recomputed with the current wind and re-uploaded
    // first; the jump into it happens when the upload is acknowledged.
    bool land_pending_ = false;
    int resume_seq_ = 2;  // loop item to resume after an aborted landing
};

}  // namespace gcs::ui
