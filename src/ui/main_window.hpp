#pragma once

#include "core/altitude.hpp"
#include "core/coords.hpp"
#include "core/endurance.hpp"
#include "core/descent_speed.hpp"
#include "core/speed_mode.hpp"
#include "core/flight_status.hpp"
#include "core/health.hpp"
#include "core/orbit_trim.hpp"
#include "core/vehicle_state.hpp"
#include "link/link_events.hpp"
#include "link/mavlink_client.hpp"
#include "payload/adapter.hpp"
#include "map_view.hpp"
#include "plan_library.hpp"
#include "settings_screen.hpp"

#include <QElapsedTimer>
#include <QString>
#include <QWidget>

#include <map>
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
class HudPanel;
class ElevationCache;
class VectorMap;
class OfflineDownloader;
class CameraView;
class ControlDock;
class FxOverlay;
class OrbitPanel;
class PayloadPanel;
class PlanStrip;
class PlanLibraryPanel;
class PreflightRunner;
class PreflightScreen;
class SettingsScreen;
class EntryPad;
class SpeedPanel;
class TelemetryBar;
class TileCache;
class VideoReceiver;
class ViewHost;
class WaypointPanel;

struct AppSettings {
    // link (Settings > CONNECTION; --udp / --serial override for one run)
    bool link_serial = false;
    QString host = QStringLiteral("127.0.0.1");
    int port = 14551;
    QString remote_host;  // UDP: also send here from the start (empty = only reply)
    int remote_port = 0;
    QString serial_port;
    int serial_baud = 57600;
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
    int speed_unit = 0;          // units::Speed
    int coord_format = 0;        // coords::Fmt
    int alt_ref = 0;             // altitude chip: 0 ASL, 1 AGL, 2 AFL (above the landing point)
    int speed_mode = 0;          // speed::Mode: ECO / NORMAL / FAST
    int hud_reference = 0;       // CameraView::HudRef
    bool wind_north_up = false;  // camera HUD wind gauge
    QString hud;                 // camera HUD: hud::encode (empty: the default)
    bool ball_collapsed = false; // map attitude ball
    bool trail_on = true;
    int trail_secs = 120;        // 0 = all
    int battery_reserve_pct = 20;  // battery time left never plans into this much of the pack
    int text_size = 0;           // theme::TextSize: Small / Medium / Large

    static AppSettings load();
    void save() const;
    void saveLink() const;  // only when the operator presses CONNECT
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
    void applySpeedUnit(int unit);
    void updateHeights(qint64 now_ms);
    // typed input
    altitude::Grounds groundsAt(double lat, double lon) const;
    double lapSeconds(double radius) const;  // one hold turn at the expected loiter speed
    void syncHoldTurns(UserWaypoint& p) const;  // an own-radius hold's turns from its time
    void openCoordPad(const QString& title, std::optional<coords::LatLon> initial,
                      std::function<void(double, double)> done);
    void openAltPad(const QString& title, double lat, double lon, double rel, double min_agl,
                    std::function<void(double)> done);
    EntryPad* pad_ = nullptr;
    // airspeed by situation (speed mode)
    void updateSpeedTarget(qint64 now_ms);
    void refreshSpeedPanel();
    SpeedPanel* speed_panel_ = nullptr;
    qint64 speed_eval_ms_ = -1000000;
    qint64 speed_want_ms_ = 0;
    std::optional<speed::Command> speed_want_;
    std::optional<speed::Command> speed_sent_;
    speed::Target speed_chip_ = speed::Target::Default;  // LOITER / CRUISE / MAX for the bar
    speed::DescentTuner descent_;      // the best descent speed, learned (QSettings speed/descent)
    bool descent_ready_ = false;       // started from the saved value / AIRSPEED_CRUISE
    uint32_t speed_mode_seen_ = 0xffffffff;
    QString speed_label_;
    // safety on the Launch button: a sent "safety off" waits for the state (ms)
    qint64 safety_pending_until_ms_ = 0;
    bool was_armed_ = false;
    void autoSafetyOnLanded();
    // the altitude the GCS commanded (above home), for the bar's "→"
    std::optional<double> commandedAltRel() const;
    int wp_hold_seq_ = -1;          // the waypoint hold being flown
    qint64 wp_hold_since_ms_ = -1;  // when its circle was reached
    bool wp_hold_cut_ = false;      // its end (SET_MISSION_CURRENT) was sent
    void updateWaypointHold(FlightStatus& fs);
    // Hold times as uploaded: LOITER_TURNS keeps only turns on board, so the
    // time the operator set is remembered here (and in QSettings).
    struct SyncedHold {
        double lat = 0.0, lon = 0.0;
        float radius = 0.0f;
        float hold_s = 0.0f;
    };
    std::vector<SyncedHold> synced_holds_;
    void rememberHolds(const MissionPlan& plan);
    std::optional<float> syncedHold(double lat, double lon, float radius) const;
    void fillHoldTimes(MissionPlan& plan) const;  // times for holds that came back as turns
    qint64 heights_ms_ = -1000000;
    qint64 nav_ms_ = -1000000;
    void updateLosTarget(const CameraLos& los);
    qint64 los_ms_ = -1000000;
    std::optional<double> terrain_here_;  // ground (AMSL) under the aircraft, from the elevation map
    std::optional<double> land_ground_;   // ground (AMSL) at the landing point
    void applyCoordFormat(int fmt);
    void applyTextSize(int size);  // theme::TextSize, live
    int barBottom() const;         // y just under the telemetry bar
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
    void launchNow();  // the GUIDED takeoff sequence (the mission on board is ours)

    // ---- plans ----
    // The GCS only uses a mission it put on the vehicle (or adopted): any
    // other one (left over from an earlier session) is masked out of s_ every
    // frame, so the map, Land, failsafe checks and preflight see "no plan".
    bool vehicleMissionOurs() const;
    // uploads the no-plan mission (takeoff + landing at home)
    void uploadNoPlanMission();
    bool landsAtHome(const MissionPlan& plan) const;  // L missing or at home
    bool editorEdited() const;
    void refreshPlanName();     // strip + library panel
    void showLibrary(bool on);
    void savePlan(bool ask_name);
    void saveEditorAs(const QString& id, const QString& name);
    void openSavedPlan(const QString& id);
    void renameSavedPlan(const QString& id);
    void deleteSavedPlan(const QString& id);
    void clearEditorToNoPlan();
    QString defaultPlanName() const;
    // the seq Resume Plan / a new plan rejoins at: the nearest route point,
    // keeping the direction of flight; -1 if the mission has no route
    int rejoinSeq(const std::vector<MissionItem>& items) const;
    void announceRejoin(const std::vector<MissionItem>& items, int seq, const QString& what);
    // airborne in AUTO on the route (not landing)
    bool flyingThePlan() const;
    static int holdSeq(const std::vector<MissionItem>& items);  // the final hold item, -1 none
    void hold();
    void land();
    int slotForTag(const std::string& tag) const;

    AppSettings settings_;
    std::unique_ptr<link::MavlinkClient> client_;
    std::unique_ptr<VideoReceiver> video_;
    TileCache* tiles_ = nullptr;
    ElevationCache* elev_ = nullptr;
    VectorMap* vector_map_ = nullptr;
    OfflineDownloader* offline_ = nullptr;
    qint64 offline_disk_ = -1;
    qint64 offline_area_bytes_ = 0;
    int offline_areas_ = 0;
    QString offline_country_;
    QString offline_status_;
    void refreshOffline();
    void refreshOfflineCounts();
    // the map's area panel: sizes per detail level and what the chosen one fetches
    // (final: the finger is up, so the disk can be checked)
    void refreshAreaPanel(bool final);
    int area_detail_ = -1;  // the level the operator tapped (-1: the preferred one)
    link::MavlinkClient::Config linkConfig() const;
    void applyLink(const SettingsScreen::Link& l);
    SettingsScreen::Link linkValues() const;
    void editLinkField(int field);
    void updateLinkStatus();

    MapView* map_ = nullptr;
    CameraView* camera_ = nullptr;
    ViewHost* host_ = nullptr;
    TelemetryBar* bar_ = nullptr;
    ControlDock* dock_ = nullptr;
    PlanStrip* strip_ = nullptr;
    PlanLibraryPanel* library_panel_ = nullptr;
    PlanLibrary library_;
    // the plan in the editor: its saved name / file (empty = not saved) and
    // the saved copy, to tell an edit
    QString editor_name_, editor_id_;
    std::optional<MissionPlan> editor_saved_;
    // the plan on the vehicle (set by a Sync; "Vehicle plan" when adopted)
    QString active_name_, active_id_;
    std::vector<MissionItem> last_uploaded_;  // what this GCS put on board (or adopted)
    std::vector<MissionItem> raw_mission_;    // the vehicle's mission, before masking
    bool sync_upload_ = false;          // the upload in flight is a Sync from the editor
    bool launch_pending_ = false;       // Launch: the no-plan mission goes up first
    bool adopt_on_download_ = false;    // long press on Plan: take what the vehicle has
    WaypointPanel* panel_ = nullptr;
    OrbitPanel* orbit_panel_ = nullptr;
    FxOverlay* fx_ = nullptr;
    SettingsScreen* settings_screen_ = nullptr;
    AlertsPanel* alerts_ = nullptr;
    HudPanel* hud_panel_ = nullptr;  // HUD LOADOUT over the video
    void showHudPanel(bool on);
    // ---- payloads (icds/*.json): the generic panel + adapters (payload/adapter.hpp)
    PayloadPanel* payload_panel_ = nullptr;
    void showPayloadPanel(bool on);
    void updatePayload();
    // the camera LOS from a payload adapter, when one reports it (else the gimbal / fixed mount)
    std::optional<CameraLos> payloadLos(const geo::Attitude& aircraft) const;
    double cameraHfov() const;  // the payload's FOV when it reports one, else the setting
    std::map<std::string, std::unique_ptr<payload::PayloadAdapter>> adapters_;
    payload::PayloadSnapshot payload_snap_;
    std::optional<payload::PayloadView> payload_view_;  // the camera payload's view
    std::string payload_cam_;                            // its ICD id
    double payload_hfov_sent_ = 0.0;
    void saveHudConfig();
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
    // after the vertical climb: a straight leg for the transition, then the plan
    void startClimbOut();
    void checkClimbOut();
    static int firstRouteSeq(const std::vector<MissionItem>& items);
    bool climb_out_ = false;
    double climb_out_lat_ = 0.0, climb_out_lon_ = 0.0;
    qint64 climb_out_ms_ = 0;
    static constexpr double kClimbOutMin = 300.0;      // m
    static constexpr double kClimbOutSeconds = 12.0;   // x cruise airspeed
    static constexpr qint64 kClimbOutTimeoutMs = 40000;
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
    // Fly To pressed with no circle of ours yet: the panel's values for the first target
    std::optional<Orbit> guide_draft_;
    bool hold_panel_hidden_ = false;  // Hold pressed again while holding: panel tucked away
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
    // orbit / hold limits: the radius can't usefully go below what the
    // aircraft can turn at its bank limit
    static constexpr float kMinOrbitRadius = 40.0f;
    static constexpr float kMaxOrbitRadius = 5000.0f;
    static constexpr float kMinOrbitAlt = 20.0f;
    static constexpr float kMaxOrbitAlt = 5000.0f;
    // the on-board approach (flown by failsafe RTLs) follows the wind
    std::optional<double> approach_wind_;  // wind direction it was laid out for
    qint64 approach_sent_ms_ = -1000000;
    bool approach_refresh_ = false;        // the upload in flight is a silent refresh
    static constexpr qint64 kApproachRefreshMs = 60000;
    static constexpr double kApproachRefreshDeg = 30.0;

    // Land: the approach is recomputed with the current wind and re-uploaded
    // first; the jump into it happens when the upload is acknowledged.
    bool land_pending_ = false;
    // a Sync while flying the plan: jump to the nearest new waypoint once it is on board
    bool rejoin_after_upload_ = false;
    bool end_hold_upload_ = false;  // a live edit of the one-way end hold (silent)

    // In AUTO on the final hold (end of a one-way route): the orbit panel
    // edits its radius / direction / altitude; edits are coalesced and
    // uploaded (same item count), then the hold item is re-loaded.
    bool onEndHold() const;
    void adjustEndHold(float d_alt, float d_radius, bool flip);
    void sendEndHold(qint64 now_ms);
    std::optional<MissionPlan> end_hold_plan_;  // the edited plan, until it is on board
    bool end_hold_dirty_ = false;
    qint64 end_hold_sent_ms_ = -1000000;
    static constexpr qint64 kEndHoldResendMs = 1000;

    // battery time left (core/endurance), recomputed every kEnduranceMs in flight
    void updateEndurance(qint64 now_ms);
    endurance::CurrentTracker endurance_tracker_;
    endurance::LevelLatch endurance_latch_;
    endurance::TimeLeftFilter endurance_shown_;  // what the pilot sees: smoothed
    bool endurance_learned_ = false;             // cruise current learned (restart the smoothing once)
    std::vector<Advisory> endurance_cards_;
    qint64 endurance_ms_ = -1000000;
    qint64 endurance_sample_ms_ = -1000000;
    static constexpr qint64 kEnduranceMs = 2000;       // estimate
    static constexpr qint64 kEnduranceSampleMs = 500;  // feed the current learner (battery data is ~1 Hz)
};

}  // namespace gcs::ui
