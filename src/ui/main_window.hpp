#pragma once

#include "core/vehicle_state.hpp"
#include "link/link_events.hpp"

#include <QElapsedTimer>
#include <QString>
#include <QWidget>

#include <memory>
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
class PlanStrip;
class TelemetryBar;
class TileCache;
class VideoReceiver;
class ViewHost;

struct AppSettings {
    QString host = QStringLiteral("127.0.0.1");
    int port = 14551;
    int video_port = 5600;
    float takeoff_alt = 40.0f;
    float waypoint_alt = 80.0f;
    double camera_hfov = 70.0;
    double camera_tilt = -30.0;
    int map_layer = 0;
    bool follow = true;
    bool reduced_motion = false;
    double min_land_approach = 120.0;
    double cruise_speed = 18.0;

    static AppSettings load();
    void save() const;
};

// Composes the views, HUD and link, and owns the meaning of the six dock
// slots. Everything runs off one 60 Hz timer: state is pulled from the link
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
    enum Slot { kSwap = 0, kPlan = 1, kLaunch = 2, kGuide = 3, kHold = 4, kQrtl = 5 };

    void frame();
    void pullState();
    void handleEvent(const link::LinkEvent& ev);
    void updateAlerts();
    void refreshDock();
    void refreshPlanStrip();
    void layoutOverlays();

    void onTriggered(int slot);
    void onLongPressed(int slot);
    void enterPlan();
    void exitPlan();
    void syncPlan();
    void setGuide(bool on);
    void sendGuided(double lat, double lon);
    void onCameraClick(double px, double py, double w, double h);
    void launchOrAuto();
    void hold();
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
};

}  // namespace gcs::ui
