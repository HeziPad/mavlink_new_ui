#pragma once

#include "core/health.hpp"
#include "core/orbit_trim.hpp"
#include "core/vehicle_state.hpp"
#include "link/link_events.hpp"
#include "map_view.hpp"

#include <QElapsedTimer>
#include <QString>
#include <QWidget>

#include <memory>
#include <optional>
#include <set>
#include <string>

class QTimer;

namespace gcs::link {
class MavlinkClient;
}

namespace gcs::ui {

class AlertsPanel;
class ElevationCache;
class OfflineDownloader;
class CameraView;
class ControlDock;
class FxOverlay;
class OrbitPanel;
class PlanStrip;
class PreflightRunner;
class PreflightScreen;
class SettingsScreen;
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
    int alt_unit = 0;            // units::Alt
    int hud_reference = 0;       // CameraView::HudRef
    bool wind_north_up = false;  // camera HUD wind gauge
    bool ball_collapsed = false; // map attitude ball
    bool trail_on = true;
    int trail_secs = 120;        // 0 = all

    static AppSettings load();
    void save() const;
};

// Composes the views, HUD and link, and owns the meaning of the five dock
// slots (Plan, Launch/Auto, Guide, Hold, Land). Everything runs off one 60 Hz timer: state is pulled from the link
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
    enum Slot { kPlan = 0, kLaunch = 1, kGuide = 2, kHold = 3, kLand = 4 };

    void frame();
    void pullState();
    void handleEvent(const link::LinkEvent& ev);
    void updateAlerts();
    void refreshDock();
    void refreshPlanStrip();
    void refreshPanel();
    void refreshOrbitPanel();
    void updateMotion();
    void updateFootprint(const CameraLos& los);
    void checkTakeoffHandoff();
    void layoutOverlays();
    void applyAltUnit(int unit);
    void showAlerts(bool on);
    QRect alertsArea() const;
    int orbitPanelX() const;

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
    MissionPlan withLanding(MissionPlan plan) const;  // L = home when the plan has none
    LandingApproach approachFor(const MissionPlan& plan, double from_lat, double from_lon) const;
    void refreshApproachForWind(qint64 now_ms);
    void onCameraClick(double px, double py, double w, double h);
    void launchOrAuto();
    void hold();
    void land();
    int slotForTag(const std::string& tag) const;

    AppSettings settings_;
    std::unique_ptr<link::MavlinkClient> client_;
    std::unique_ptr<VideoReceiver> video_;
    TileCache* tiles_ = nullptr;
    ElevationCache* elev_ = nullptr;
    OfflineDownloader* offline_ = nullptr;
    int offline_radius_km_ = 25;
    qint64 offline_disk_ = -1;
    QString offline_status_;
    void refreshOffline();

    MapView* map_ = nullptr;
    CameraView* camera_ = nullptr;
    ViewHost* host_ = nullptr;
    TelemetryBar* bar_ = nullptr;
    ControlDock* dock_ = nullptr;
    PlanStrip* strip_ = nullptr;
    WaypointPanel* panel_ = nullptr;
    OrbitPanel* orbit_panel_ = nullptr;
    FxOverlay* fx_ = nullptr;
    SettingsScreen* settings_screen_ = nullptr;
    AlertsPanel* alerts_ = nullptr;
    std::unique_ptr<PreflightRunner> pf_;
    PreflightScreen* preflight_ = nullptr;
    bool flew_ = false;  // airborne since the last preflight (offers the turnaround)
    void openPreflight();

    QTimer* timer_ = nullptr;
    QElapsedTimer clock_;
    qint64 last_ms_ = 0;
    int frame_count_ = 0;
    // GCS_FPS=1 diagnostics
    bool diag_ = false;
    int diag_ticks_ = 0;
    qint64 diag_t0_ = 0, diag_last_tick_ = 0, diag_gap_max_ = 0, diag_work_max_ = 0;
    int diag_att_ = 0, diag_pos_ = 0;
    Clock::time_point diag_att_rx_{}, diag_pos_rx_{};
    void diagFrame(qint64 now_ms);

    VehicleState s_;
    bool planning_ = false;
    bool guiding_ = false;
    bool uploading_ = false;
    bool ever_connected_ = false;
    bool link_was_alive_ = false;
    // plain-language health: state-derived advisories plus translated
    // STATUSTEXTs; ids already announced with a toast
    AdvisoryBook book_;
    std::vector<Advisory> advisories_;
    std::set<std::string> announced_;
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
    // ArduPlane flies the loiter bigger than asked; this trims the commanded
    // radius from measured laps so the flown circle matches the drawn one
    OrbitTrim trim_;
    Clock::time_point trim_pos_rx_{};
    TrackPredictor track_;  // ground track for the HUD's flight-path marker
    Clock::time_point track_rx_{};
    void updateOrbitTrim();
    MapView::OrbitDraw orbitDraw() const;
    bool orbit_dirty_ = false;
    bool orbit_trim_only_ = false;  // pending re-send is a silent trim correction
    qint64 orbit_sent_ms_ = 0;
    static constexpr qint64 kOrbitResendMs = 300;
    // the on-board approach (flown by failsafe RTLs) follows the wind
    std::optional<double> approach_wind_;  // wind direction it was laid out for
    qint64 approach_sent_ms_ = -1000000;
    bool approach_refresh_ = false;        // the upload in flight is a silent refresh
    static constexpr qint64 kApproachRefreshMs = 60000;
    static constexpr double kApproachRefreshDeg = 30.0;

    // Land: the approach is recomputed with the current wind and re-uploaded
    // first; the jump into it happens when the upload is acknowledged.
    bool land_pending_ = false;
};

}  // namespace gcs::ui
