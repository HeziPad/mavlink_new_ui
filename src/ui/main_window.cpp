#include "main_window.hpp"

#include "camera_view.hpp"
#include "control_dock.hpp"
#include "core/geo.hpp"
#include "core/plane_modes.hpp"
#include "fx_overlay.hpp"
#include "link/mavlink_client.hpp"
#include "map_view.hpp"
#include "plan_strip.hpp"
#include "telemetry_bar.hpp"
#include "theme.hpp"
#include "tile_cache.hpp"
#include "video_receiver.hpp"
#include "view_host.hpp"

#include <QApplication>
#include <QCloseEvent>
#include <QKeyEvent>
#include <QSettings>
#include <QTimer>

#include <algorithm>

namespace gcs::ui {

using namespace theme;

// ---- settings -------------------------------------------------------------------

AppSettings AppSettings::load() {
    QSettings q;
    AppSettings s;
    s.host = q.value(QStringLiteral("link/host"), s.host).toString();
    s.port = q.value(QStringLiteral("link/port"), s.port).toInt();
    s.video_port = q.value(QStringLiteral("video/port"), s.video_port).toInt();
    s.takeoff_alt = q.value(QStringLiteral("flight/takeoff_alt"), s.takeoff_alt).toFloat();
    s.waypoint_alt = q.value(QStringLiteral("flight/waypoint_alt"), s.waypoint_alt).toFloat();
    s.cruise_speed = q.value(QStringLiteral("flight/cruise_speed"), s.cruise_speed).toDouble();
    s.min_land_approach = q.value(QStringLiteral("flight/min_land_approach"), s.min_land_approach).toDouble();
    s.camera_hfov = q.value(QStringLiteral("camera/hfov"), s.camera_hfov).toDouble();
    s.camera_tilt = q.value(QStringLiteral("camera/tilt"), s.camera_tilt).toDouble();
    s.map_layer = q.value(QStringLiteral("map/layer"), s.map_layer).toInt();
    s.follow = q.value(QStringLiteral("map/follow"), s.follow).toBool();
    s.reduced_motion = q.value(QStringLiteral("ui/reduced_motion"), s.reduced_motion).toBool();

    // seed missing keys so the settings file is discoverable and hand-editable
    const std::pair<const char*, QVariant> defaults[] = {
        {"link/host", s.host}, {"link/port", s.port}, {"video/port", s.video_port},
        {"flight/takeoff_alt", s.takeoff_alt}, {"flight/waypoint_alt", s.waypoint_alt},
        {"flight/cruise_speed", s.cruise_speed}, {"flight/min_land_approach", s.min_land_approach},
        {"camera/hfov", s.camera_hfov}, {"camera/tilt", s.camera_tilt},
        {"ui/reduced_motion", s.reduced_motion}};
    for (const auto& [k, v] : defaults) {
        if (!q.contains(QLatin1String(k))) {
            q.setValue(QLatin1String(k), v);
        }
    }
    return s;
}

void AppSettings::save() const {
    // only UI state the user changes from inside the app; command-line
    // overrides (--udp, --reduced-motion, ...) must not stick
    QSettings q;
    q.setValue(QStringLiteral("map/layer"), map_layer);
    q.setValue(QStringLiteral("map/follow"), follow);
}

// ---- construction ---------------------------------------------------------------

MainWindow::MainWindow(const AppSettings& settings, QWidget* parent) : QWidget(parent), settings_(settings) {
    setWindowTitle(QStringLiteral("QuadPlane GCS"));
    setMinimumSize(960, 600);
    resize(1440, 900);
    setAutoFillBackground(true);
    QPalette pal = palette();
    pal.setColor(QPalette::Window, kBg);
    setPalette(pal);
    setReducedMotion(settings_.reduced_motion);

    link::MavlinkClient::Config cfg;
    cfg.bind_host = settings_.host.toStdString();
    cfg.port = static_cast<uint16_t>(settings_.port);
    client_ = std::make_unique<link::MavlinkClient>(cfg);
    video_ = std::make_unique<VideoReceiver>(settings_.video_port);
    tiles_ = new TileCache(this);

    map_ = new MapView(tiles_);
    map_->setLayer(settings_.map_layer);
    map_->setFollow(settings_.follow);
    map_->setDefaultAltitude(settings_.waypoint_alt);
    camera_ = new CameraView(video_.get());
    camera_->setCamera(settings_.camera_hfov, settings_.camera_tilt);

    host_ = new ViewHost(map_, camera_, this);
    strip_ = new PlanStrip(this);
    strip_->hide();
    bar_ = new TelemetryBar(this);
    dock_ = new ControlDock(this);
    fx_ = new FxOverlay(this);
    fx_->setTopInset(58);

    connect(dock_, &ControlDock::triggered, this, &MainWindow::onTriggered);
    connect(dock_, &ControlDock::longPressed, this, &MainWindow::onLongPressed);
    connect(dock_, &ControlDock::blocked, this, [this](int, const QString& reason) {
        if (!reason.isEmpty()) {
            fx_->toast(reason, kMuted, QStringLiteral("Locked"));
        }
    });
    connect(dock_, &ControlDock::holdCancelled, this, [this](int) {
        fx_->toast(QStringLiteral("Hold the button for 1 s to confirm"), kAmber, QStringLiteral("Hold"));
    });

    connect(map_, &MapView::planChanged, this, &MainWindow::refreshPlanStrip);
    connect(map_, &MapView::guideClicked, this, &MainWindow::sendGuided);
    connect(map_, &MapView::followChanged, this, [this](bool on) { settings_.follow = on; });
    connect(map_, &MapView::layerChanged, this, [this](int layer) { settings_.map_layer = layer; });
    connect(camera_, &CameraView::groundClicked, this, &MainWindow::onCameraClick);
    connect(strip_, &PlanStrip::undoRequested, map_, &MapView::undo);
    connect(strip_, &PlanStrip::clearRequested, map_, &MapView::clearPlan);
    connect(strip_, &PlanStrip::cancelRequested, this, [this] {
        exitPlan();
        fx_->toast(QStringLiteral("Plan discarded"), kMuted);
    });

    qApp->installEventFilter(this);

    client_->start();
    video_->start();

    timer_ = new QTimer(this);
    timer_->setTimerType(Qt::PreciseTimer);
    connect(timer_, &QTimer::timeout, this, &MainWindow::frame);
    timer_->start(16);
    clock_.start();

    fx_->toast(QStringLiteral("Listening on udp %1:%2").arg(settings_.host).arg(settings_.port), kCyan,
               QStringLiteral("Link"));
    refreshDock();
    layoutOverlays();
}

MainWindow::~MainWindow() {
    qApp->removeEventFilter(this);
    timer_->stop();
    video_->stop();
    client_->stop();
}

void MainWindow::closeEvent(QCloseEvent* e) {
    settings_.save();
    QWidget::closeEvent(e);
}

void MainWindow::resizeEvent(QResizeEvent* e) {
    QWidget::resizeEvent(e);
    layoutOverlays();
}

void MainWindow::layoutOverlays() {
    host_->setGeometry(rect());
    bar_->setGeometry(8, 8, width() - 16, 52);
    const QSize ds = dock_->sizeHint();
    dock_->setGeometry((width() - ds.width()) / 2, height() - ds.height() - 10, ds.width(), ds.height());
    host_->setPipMaxWidth((width() - ds.width()) / 2.0 - 32.0);
    strip_->adjustSize();
    strip_->move((width() - strip_->width()) / 2, 66);
    fx_->setGeometry(rect());
    strip_->raise();
    bar_->raise();
    dock_->raise();
    fx_->raise();
}

// ---- main loop --------------------------------------------------------------------

void MainWindow::frame() {
    const qint64 now_ms = clock_.elapsed();
    const double dt = std::clamp((now_ms - last_ms_) / 1000.0, 0.0, 0.1);
    last_ms_ = now_ms;

    if ((frame_count_++ % 2) == 0) {
        pullState();
    }
    host_->tick(dt);
    map_->tick(dt);
    camera_->tick(dt);
    bar_->tick(dt);
    dock_->tick(dt);
    fx_->tick(dt);
}

void MainWindow::pullState() {
    s_ = client_->snapshot();
    const auto now = Clock::now();
    for (const auto& ev : client_->pollEvents()) {
        handleEvent(ev);
    }

    // mode / VTOL change banners
    if (s_.has_heartbeat && s_.mode_name != last_mode_) {
        if (s_.custom_mode != plane::GUIDED) {
            map_->setGuidedTarget(false);  // a target only means something in GUIDED
        }
        if (!last_mode_.empty()) {
            fx_->banner(QStringLiteral("Mode · %1").arg(QString::fromStdString(s_.mode_name)), kCyan);
        }
        last_mode_ = s_.mode_name;
    }
    const VtolState v = effectiveVtolState(s_);
    if (static_cast<int>(v) != last_vtol_) {
        if (last_vtol_ >= 0 && v != VtolState::Unknown) {
            const QColor c = v == VtolState::Cruise ? kGreen : (v == VtolState::Hover ? kCyan : kAmber);
            fx_->banner(QString::fromUtf8(vtolStateName(v)), c);
        }
        last_vtol_ = static_cast<int>(v);
    }

    bar_->setState(s_, now);
    map_->setState(s_);
    camera_->setState(s_);
    updateAlerts();
    refreshDock();
}

void MainWindow::updateAlerts() {
    const bool alive = linkAlive(s_, Clock::now());
    if (alive && !link_was_alive_ && ever_connected_) {
        fx_->banner(QStringLiteral("Link restored"), kGreen);
    }
    if (!alive && link_was_alive_) {
        fx_->toast(QStringLiteral("No heartbeat for 3 s"), kRed, QStringLiteral("Link"));
    }
    link_was_alive_ = alive;

    QString alert;
    if (ever_connected_ && !alive) {
        alert = QStringLiteral("Link lost");
    } else if (alive && s_.battery_pct >= 0 && s_.battery_pct < 15) {
        alert = QStringLiteral("Battery critical %1%").arg(s_.battery_pct);
    } else if (!s_.error.empty()) {
        alert = QString::fromStdString(s_.error);
    }
    fx_->setAlert(alert);
}

void MainWindow::handleEvent(const link::LinkEvent& ev) {
    using T = link::LinkEvent::Type;
    const QString text = QString::fromStdString(ev.text);
    switch (ev.type) {
    case T::StatusText: {
        if (ev.severity >= 7) {
            break;  // debug
        }
        QColor c = kMuted;
        QString tag;
        if (ev.severity <= 3) {
            c = kRed;
            tag = QStringLiteral("Alert");
        } else if (ev.severity == 4) {
            c = kAmber;
            tag = QStringLiteral("Warn");
        } else if (ev.severity == 5) {
            c = kCyan;
        }
        if (text.startsWith(QLatin1String("PreArm")) || text.startsWith(QLatin1String("Arm:"))) {
            c = kAmber;
            tag = QStringLiteral("PreArm");
        }
        fx_->toast(text, c, tag);
        break;
    }
    case T::CommandStep:
        break;
    case T::CommandResult: {
        const int slot = slotForTag(ev.tag);
        if (slot >= 0) {
            dock_->flash(slot, ev.ok);
        }
        if (ev.ok) {
            fx_->toast(QStringLiteral("%1 accepted").arg(text), kGreen,
                       QString::fromStdString(ev.tag).toUpper());
        } else {
            fx_->toast(text, kRed, QString::fromStdString(ev.tag).toUpper());
        }
        break;
    }
    case T::MissionProgress:
        if (ev.total > 0) {
            dock_->setProgress(kPlan, static_cast<double>(ev.done) / ev.total);
        }
        break;
    case T::MissionUploaded:
        uploading_ = false;
        dock_->setProgress(kPlan, -1.0);
        dock_->flash(kPlan, true);
        fx_->toast(QStringLiteral("Mission synced · %1 items").arg(ev.items.size()), kGreen, QStringLiteral("Plan"));
        if (planning_) {
            exitPlan();
        }
        break;
    case T::MissionDownloaded:
        dock_->setProgress(kPlan, -1.0);
        if (ev.items.size() > 1) {
            fx_->toast(QStringLiteral("Vehicle mission loaded · %1 items").arg(ev.items.size()), kCyan,
                       QStringLiteral("Plan"));
        }
        break;
    case T::MissionFailed:
        uploading_ = false;
        dock_->setProgress(kPlan, -1.0);
        dock_->flash(kPlan, false);
        fx_->toast(QStringLiteral("Mission %1 failed: %2").arg(ev.upload ? QStringLiteral("upload") : QStringLiteral("download"), text),
                   kRed, QStringLiteral("Plan"));
        break;
    case T::GuidedResult:
        dock_->flash(kGuide, ev.ok);
        fx_->toast(ev.ok ? QStringLiteral("Guided target accepted") : text, ev.ok ? kGreen : kRed, QStringLiteral("Guide"));
        if (!ev.ok) {
            map_->setGuidedTarget(false);
        }
        break;
    case T::VehicleConnected:
        ever_connected_ = true;
        fx_->banner(QStringLiteral("Link established"), kGreen);
        fx_->toast(QStringLiteral("Vehicle sysid %1 connected").arg(ev.done), kGreen, QStringLiteral("Link"));
        break;
    case T::Error:
        fx_->toast(text, kRed, QStringLiteral("Error"));
        break;
    }
}

int MainWindow::slotForTag(const std::string& tag) const {
    if (tag == "launch" || tag == "auto") {
        return kLaunch;
    }
    if (tag == "hold") {
        return kHold;
    }
    if (tag == "qrtl") {
        return kQrtl;
    }
    return -1;
}

// ---- dock -----------------------------------------------------------------------

void MainWindow::refreshDock() {
    const bool alive = linkAlive(s_, Clock::now());
    const bool airborne = isAirborne(s_);
    const QString no_link = QStringLiteral("No vehicle link");

    // 1 swap
    {
        DockSlotSpec spec;
        spec.label = QStringLiteral("Swap");
        spec.icon = DockIcon::Swap;
        spec.hint = QStringLiteral("Swap map and camera");
        dock_->setSlot(kSwap, spec);
    }

    // 2 plan / sync
    {
        DockSlotSpec spec;
        spec.accent = kAmber;
        if (planning_) {
            spec.label = uploading_ ? QStringLiteral("Syncing") : QStringLiteral("Sync ↑");
            spec.icon = DockIcon::Upload;
            spec.active = true;
            spec.hint = QStringLiteral("Upload the plan to the vehicle");
            if (!alive) {
                spec.enabled = false;
                spec.reason = no_link;
            } else if (!s_.has_home) {
                spec.enabled = false;
                spec.reason = QStringLiteral("Vehicle has no home position yet");
            } else if (map_->plan().empty()) {
                spec.enabled = false;
                spec.reason = QStringLiteral("Click the map to add waypoints");
            } else if (uploading_ || s_.mission_transfer_active) {
                spec.enabled = false;
                spec.reason = QStringLiteral("Mission transfer in progress");
            }
        } else {
            spec.label = QStringLiteral("Plan");
            spec.icon = DockIcon::Plan;
            spec.long_press = true;
            spec.hint = QStringLiteral("Edit the mission · hold to re-download it from the vehicle");
        }
        dock_->setSlot(kPlan, spec);
    }

    // 3 launch / takeoff / auto
    {
        DockSlotSpec spec;
        spec.accent = kGreen;
        if (!airborne) {
            const bool mission = hasFlyableMission(s_);
            spec.label = mission ? QStringLiteral("Launch")
                                 : QStringLiteral("Takeoff %1m").arg(static_cast<int>(settings_.takeoff_alt));
            spec.icon = mission ? DockIcon::Launch : DockIcon::Takeoff;
            spec.hold_to_confirm = true;
            spec.hint = mission ? QStringLiteral("Hold: AUTO + arm, flies the uploaded mission")
                                : QStringLiteral("Hold: GUIDED + arm + VTOL takeoff");
            // Not a lock: the pre-arm bit is evaluated in the *current* mode, and
            // the sequence changes mode first (e.g. "QRTL mode not armable"
            // clears once in AUTO). The vehicle has the final say via the ARM ack.
            if (s_.has_sys_status && !prearmHealthy(s_) && !s_.armed) {
                spec.hint += QStringLiteral("\n⚠ PreArm currently failing in %1 — the vehicle may refuse")
                                 .arg(QString::fromStdString(s_.mode_name));
            }
            if (!alive) {
                spec.enabled = false;
                spec.reason = no_link;
            } else if (!gpsHealthy(s_) || !s_.has_home) {
                spec.enabled = false;
                spec.reason = QStringLiteral("Waiting for GPS fix and home position");
            } else if (planning_) {
                spec.enabled = false;
                spec.reason = QStringLiteral("Sync or cancel the plan first");
            } else if (s_.mission_transfer_active) {
                spec.enabled = false;
                spec.reason = QStringLiteral("Mission transfer in progress");
            }
        } else {
            spec.label = QStringLiteral("Auto");
            spec.icon = DockIcon::Auto;
            spec.active = s_.custom_mode == plane::AUTO;
            spec.hint = QStringLiteral("Resume the mission (AUTO)");
            if (!alive) {
                spec.enabled = false;
                spec.reason = no_link;
            } else if (!hasFlyableMission(s_)) {
                spec.enabled = false;
                spec.reason = QStringLiteral("No mission on the vehicle");
            } else if (spec.active) {
                spec.enabled = false;
                spec.reason = QStringLiteral("Already flying the mission");
            }
        }
        dock_->setSlot(kLaunch, spec);
    }

    // 4 guide
    {
        DockSlotSpec spec;
        spec.label = QStringLiteral("Guide");
        spec.icon = DockIcon::Guide;
        spec.active = guiding_;
        spec.hint = QStringLiteral("Click the map or camera to send the aircraft there");
        if (!alive) {
            spec.enabled = false;
            spec.reason = no_link;
        } else if (!s_.has_position) {
            spec.enabled = false;
            spec.reason = QStringLiteral("No position yet");
        } else if (!airborne) {
            spec.enabled = false;
            spec.reason = QStringLiteral("Take off first");
        }
        if (!spec.enabled && guiding_) {
            setGuide(false);
            spec.active = false;
        }
        dock_->setSlot(kGuide, spec);
    }

    // 5 hold
    {
        DockSlotSpec spec;
        const bool hover = effectiveVtolState(s_) == VtolState::Hover || s_.airspeed < kCruiseAirspeed;
        spec.label = QStringLiteral("Hold");
        spec.icon = DockIcon::Hold;
        spec.active = s_.custom_mode == plane::QLOITER || s_.custom_mode == plane::LOITER;
        spec.hint = hover ? QStringLiteral("QLOITER: hover in place") : QStringLiteral("LOITER: circle here");
        if (!alive) {
            spec.enabled = false;
            spec.reason = no_link;
        }
        dock_->setSlot(kHold, spec);
    }

    // 6 qrtl
    {
        DockSlotSpec spec;
        spec.label = QStringLiteral("QRTL");
        spec.icon = DockIcon::Qrtl;
        spec.accent = kAmber;
        spec.active = s_.custom_mode == plane::QRTL;
        spec.hint = QStringLiteral("Return home and land vertically");
        if (!alive) {
            spec.enabled = false;
            spec.reason = no_link;
        }
        dock_->setSlot(kQrtl, spec);
    }
}

void MainWindow::onTriggered(int slot) {
    switch (slot) {
    case kSwap:
        host_->swap();
        break;
    case kPlan:
        if (planning_) {
            syncPlan();
        } else {
            enterPlan();
        }
        break;
    case kLaunch:
        launchOrAuto();
        break;
    case kGuide:
        setGuide(!guiding_);
        break;
    case kHold:
        hold();
        break;
    case kQrtl:
        qrtl();
        break;
    default:
        break;
    }
    refreshDock();
}

void MainWindow::onLongPressed(int slot) {
    if (slot != kPlan || planning_) {
        return;
    }
    if (!linkAlive(s_, Clock::now())) {
        fx_->toast(QStringLiteral("No vehicle link"), kMuted, QStringLiteral("Locked"));
        return;
    }
    client_->downloadMission();
    dock_->setProgress(kPlan, 0.0);
    fx_->toast(QStringLiteral("Downloading mission from vehicle"), kCyan, QStringLiteral("Plan"));
}

// ---- actions --------------------------------------------------------------------

void MainWindow::enterPlan() {
    setGuide(false);
    planning_ = true;
    map_->setPlan(s_.mission_known ? extractUserWaypoints(s_.mission) : std::vector<UserWaypoint>{});
    map_->setMode(MapView::Mode::Plan);
    if (host_->mainView() != map_) {
        host_->swap();
    }
    strip_->show();
    refreshPlanStrip();
    fx_->banner(QStringLiteral("Plan mode"), kAmber);
}

void MainWindow::exitPlan() {
    planning_ = false;
    map_->setMode(guiding_ ? MapView::Mode::Guide : MapView::Mode::Normal);
    strip_->hide();
    refreshDock();
}

void MainWindow::refreshPlanStrip() {
    const auto& plan = map_->plan();
    const double hl = s_.has_home ? s_.home_lat : (plan.empty() ? 0.0 : plan.front().lat);
    const double ho = s_.has_home ? s_.home_lon : (plan.empty() ? 0.0 : plan.front().lon);
    const double route = routeLength(hl, ho, plan);
    const double approach = plan.empty() ? -1.0 : finalApproachLength(hl, ho, plan);
    strip_->setInfo(static_cast<int>(plan.size()), route, route / std::max(1.0, settings_.cruise_speed), approach,
                    settings_.min_land_approach, map_->canUndo());
    strip_->move((width() - strip_->width()) / 2, 66);
    refreshDock();
}

void MainWindow::syncPlan() {
    const auto& plan = map_->plan();
    if (plan.empty() || !s_.has_home) {
        return;
    }
    auto items = buildVtolMission(s_.home_lat, s_.home_lon, s_.home_alt_amsl, plan, settings_.takeoff_alt);
    uploading_ = true;
    dock_->setProgress(kPlan, 0.0);
    client_->uploadMission(std::move(items));
    fx_->toast(QStringLiteral("Uploading %1 waypoints").arg(plan.size()), kAmber, QStringLiteral("Plan"));
}

void MainWindow::setGuide(bool on) {
    if (guiding_ == on) {
        return;
    }
    guiding_ = on;
    if (on && planning_) {
        exitPlan();
    }
    if (!planning_) {
        map_->setMode(on ? MapView::Mode::Guide : MapView::Mode::Normal);
    }
    camera_->setGuideMode(on);
    if (!on) {
        map_->setGuidedTarget(false);
    }
}

void MainWindow::sendGuided(double lat, double lon) {
    const float alt = static_cast<float>(std::max(30.0, s_.alt_rel));
    client_->guidedGoto(lat, lon, alt);
    map_->setGuidedTarget(true, lat, lon);
    map_->ping(lat, lon, kCyan);
    dock_->setPending(kGuide, true);
}

void MainWindow::onCameraClick(double px, double py, double w, double h) {
    if (!guiding_ || !s_.has_position) {
        return;
    }
    geo::CameraModel cam;
    cam.hfov_deg = settings_.camera_hfov;
    cam.mount_pitch_deg = settings_.camera_tilt;
    cam.width = w;
    cam.height = h;
    geo::Attitude att{s_.roll_deg, s_.pitch_deg, s_.yaw_deg};
    const auto t = geo::cameraRayToGround(cam, att, px, py, std::max(1.0, s_.alt_rel));
    double lat = 0.0, lon = 0.0;
    geo::offset(s_.lat, s_.lon, t.north, t.east, lat, lon);
    if (!t.hit_ground) {
        fx_->toast(QStringLiteral("Above the horizon — target %1 m out on bearing %2°")
                       .arg(t.range, 0, 'f', 0)
                       .arg(t.bearing_deg, 0, 'f', 0),
                   kAmber, QStringLiteral("Guide"));
    }
    sendGuided(lat, lon);
}

void MainWindow::launchOrAuto() {
    setGuide(false);
    if (isAirborne(s_)) {
        client_->setMode(plane::AUTO, "auto");
        dock_->setPending(kLaunch, true);
        return;
    }
    std::vector<link::CommandRequest> steps;
    if (hasFlyableMission(s_)) {
        // ArduPlane starts the mission when it arms in AUTO
        steps = {link::makeSetMode(plane::AUTO, "AUTO"), link::makeArm(true)};
    } else {
        steps = {link::makeSetMode(plane::GUIDED, "GUIDED"), link::makeArm(true),
                 link::makeVtolTakeoff(settings_.takeoff_alt)};
    }
    client_->runSequence("launch", std::move(steps));
    dock_->setPending(kLaunch, true);
}

void MainWindow::hold() {
    // instant: no confirmation, effects play afterwards
    const bool hover = effectiveVtolState(s_) == VtolState::Hover || s_.airspeed < kCruiseAirspeed;
    client_->setMode(hover ? plane::QLOITER : plane::LOITER, "hold");
    setGuide(false);
    dock_->setPending(kHold, true);
}

void MainWindow::qrtl() {
    client_->setMode(plane::QRTL, "qrtl");
    setGuide(false);
    dock_->setPending(kQrtl, true);
}

// ---- keyboard -------------------------------------------------------------------

bool MainWindow::eventFilter(QObject* obj, QEvent* e) {
    if ((e->type() == QEvent::KeyPress || e->type() == QEvent::KeyRelease) && isActiveWindow()) {
        auto* ke = static_cast<QKeyEvent*>(e);
        const int key = ke->key();
        if (key >= Qt::Key_1 && key <= Qt::Key_6) {
            if (!ke->isAutoRepeat()) {
                const int slot = key - Qt::Key_1;
                if (e->type() == QEvent::KeyPress) {
                    dock_->pressSlot(slot);
                } else {
                    dock_->releaseSlot(slot);
                }
            }
            return true;
        }
        if (key == Qt::Key_Escape && e->type() == QEvent::KeyPress) {
            if (planning_) {
                exitPlan();
                fx_->toast(QStringLiteral("Plan discarded"), kMuted);
            } else if (guiding_) {
                setGuide(false);
            }
            refreshDock();
            return true;
        }
        if (planning_ && e->type() == QEvent::KeyPress && key == Qt::Key_Z &&
            (ke->modifiers() & Qt::ControlModifier)) {
            map_->undo();
            return true;
        }
    }
    return QWidget::eventFilter(obj, e);
}

}  // namespace gcs::ui
