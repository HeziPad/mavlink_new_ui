#include "main_window.hpp"

#include "alerts_panel.hpp"
#include "hud_panel.hpp"
#include "camera_view.hpp"
#include "control_dock.hpp"
#include "core/geo.hpp"
#include "core/plane_modes.hpp"
#include "fx_overlay.hpp"
#include "link/mavlink_client.hpp"
#include "map_view.hpp"
#include "orbit_panel.hpp"
#include "elevation_cache.hpp"
#include "entry_pad.hpp"
#include "speed_panel.hpp"
#include "core/altitude.hpp"
#include "offline_downloader.hpp"
#include "offline_store.hpp"
#include "plan_strip.hpp"
#include "vector_map.hpp"
#include "preflight_runner.hpp"
#include "preflight_screen.hpp"
#include "settings_screen.hpp"
#include "telemetry_bar.hpp"
#include "theme.hpp"
#include "tile_cache.hpp"
#include "units.hpp"
#include "video_receiver.hpp"
#include "view_host.hpp"
#include "waypoint_panel.hpp"

#include <QApplication>
#include <QCloseEvent>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QTouchEvent>
#include <QFileDialog>
#include <QSettings>
#include <QTimer>

#include <algorithm>
#include <cstdio>

namespace gcs::ui {

using namespace theme;

// ---- settings -------------------------------------------------------------------

AppSettings AppSettings::load() {
    QSettings q;
    AppSettings s;
    s.link_serial = q.value(QStringLiteral("link/type")).toString() == QStringLiteral("serial");
    s.host = q.value(QStringLiteral("link/host"), s.host).toString();
    s.port = q.value(QStringLiteral("link/port"), s.port).toInt();
    s.remote_host = q.value(QStringLiteral("link/remote_host"), s.remote_host).toString();
    s.remote_port = q.value(QStringLiteral("link/remote_port"), s.remote_port).toInt();
    s.serial_port = q.value(QStringLiteral("link/serial_port"), s.serial_port).toString();
    s.serial_baud = q.value(QStringLiteral("link/serial_baud"), s.serial_baud).toInt();
    s.video_port = q.value(QStringLiteral("video/port"), s.video_port).toInt();
    s.takeoff_alt = q.value(QStringLiteral("flight/takeoff_alt"), s.takeoff_alt).toFloat();
    s.waypoint_alt = q.value(QStringLiteral("flight/waypoint_alt"), s.waypoint_alt).toFloat();
    s.land_approach_alt = q.value(QStringLiteral("flight/land_approach_alt"), s.land_approach_alt).toFloat();
    s.cruise_speed = q.value(QStringLiteral("flight/cruise_speed"), s.cruise_speed).toDouble();
    s.camera_hfov = q.value(QStringLiteral("camera/hfov"), s.camera_hfov).toDouble();
    s.camera_tilt = q.value(QStringLiteral("camera/tilt"), s.camera_tilt).toDouble();
    s.gimbal_body_frame = q.value(QStringLiteral("camera/gimbal_body_frame"), s.gimbal_body_frame).toBool();
    s.map_layer = q.value(QStringLiteral("map/layer"), s.map_layer).toInt();
    s.follow = q.value(QStringLiteral("map/follow"), s.follow).toBool();
    s.reduced_motion = q.value(QStringLiteral("ui/reduced_motion"), s.reduced_motion).toBool();
    s.alt_unit = q.value(QStringLiteral("ui/alt_unit"), s.alt_unit).toInt();
    s.speed_unit = std::clamp(q.value(QStringLiteral("ui/speed_unit"), s.speed_unit).toInt(), 0, 1);
    s.coord_format = std::clamp(q.value(QStringLiteral("ui/coord_format"), s.coord_format).toInt(), 0, 2);
    s.alt_ref = std::clamp(q.value(QStringLiteral("ui/alt_ref"), s.alt_ref).toInt(), 0, 2);
    s.speed_mode = std::clamp(q.value(QStringLiteral("flight/speed_mode"), s.speed_mode).toInt(), 0, 2);
    s.hud_reference = q.value(QStringLiteral("camera/hud_reference"), s.hud_reference).toInt();
    s.wind_north_up = q.value(QStringLiteral("camera/wind_north_up"), s.wind_north_up).toBool();
    s.hud = q.value(QStringLiteral("camera/hud"), s.hud).toString();
    s.ball_collapsed = q.value(QStringLiteral("map/ball_collapsed"), s.ball_collapsed).toBool();
    s.trail_on = q.value(QStringLiteral("map/trail_on"), s.trail_on).toBool();
    s.trail_secs = q.value(QStringLiteral("map/trail_secs"), s.trail_secs).toInt();
    s.battery_reserve_pct =
        std::clamp(q.value(QStringLiteral("flight/battery_reserve_pct"), s.battery_reserve_pct).toInt(), 15, 50);
    s.text_size = std::clamp(q.value(QStringLiteral("display/text_size"), s.text_size).toInt(), 0, 2);

    // seed missing keys so the settings file is discoverable and hand-editable
    const std::pair<const char*, QVariant> defaults[] = {
        {"link/host", s.host}, {"link/port", s.port}, {"video/port", s.video_port},
        {"flight/takeoff_alt", s.takeoff_alt}, {"flight/waypoint_alt", s.waypoint_alt},
        {"flight/land_approach_alt", s.land_approach_alt},
        {"flight/cruise_speed", s.cruise_speed},
        {"camera/hfov", s.camera_hfov}, {"camera/tilt", s.camera_tilt},
        {"camera/gimbal_body_frame", s.gimbal_body_frame},
        {"ui/reduced_motion", s.reduced_motion}};
    for (const auto& [k, v] : defaults) {
        if (!q.contains(QLatin1String(k))) {
            q.setValue(QLatin1String(k), v);
        }
    }
    return s;
}

void AppSettings::saveLink() const {
    QSettings q;
    q.setValue(QStringLiteral("link/type"), link_serial ? QStringLiteral("serial") : QStringLiteral("udp"));
    q.setValue(QStringLiteral("link/host"), host);
    q.setValue(QStringLiteral("link/port"), port);
    q.setValue(QStringLiteral("link/remote_host"), remote_host);
    q.setValue(QStringLiteral("link/remote_port"), remote_port);
    q.setValue(QStringLiteral("link/serial_port"), serial_port);
    q.setValue(QStringLiteral("link/serial_baud"), serial_baud);
    q.setValue(QStringLiteral("video/port"), video_port);
}

void AppSettings::save() const {
    // only UI state the user changes from inside the app; command-line
    // overrides (--udp, --reduced-motion, ...) must not stick
    QSettings q;
    q.setValue(QStringLiteral("map/layer"), map_layer);
    q.setValue(QStringLiteral("map/follow"), follow);
    q.setValue(QStringLiteral("ui/alt_unit"), alt_unit);
    q.setValue(QStringLiteral("ui/speed_unit"), speed_unit);
    q.setValue(QStringLiteral("ui/coord_format"), coord_format);
    q.setValue(QStringLiteral("ui/alt_ref"), alt_ref);
    q.setValue(QStringLiteral("flight/speed_mode"), speed_mode);
    q.setValue(QStringLiteral("camera/hud_reference"), hud_reference);
    q.setValue(QStringLiteral("camera/wind_north_up"), wind_north_up);
    q.setValue(QStringLiteral("camera/hud"), hud);
    q.setValue(QStringLiteral("map/ball_collapsed"), ball_collapsed);
    q.setValue(QStringLiteral("map/trail_on"), trail_on);
    q.setValue(QStringLiteral("map/trail_secs"), trail_secs);
    q.setValue(QStringLiteral("flight/battery_reserve_pct"), battery_reserve_pct);
    q.setValue(QStringLiteral("display/text_size"), text_size);
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
    setUiScale(scaleFor(static_cast<TextSize>(settings_.text_size)));  // before any widget is built
    units::setAltUnit(settings_.alt_unit == 1 ? units::Alt::Feet : units::Alt::Meters);
    units::setSpeedUnit(static_cast<units::Speed>(settings_.speed_unit));
    units::setCoordFormat(static_cast<coords::Fmt>(settings_.coord_format));

    client_ = std::make_unique<link::MavlinkClient>(linkConfig());
    video_ = std::make_unique<VideoReceiver>(settings_.video_port);
    tiles_ = new TileCache(this);
    vector_map_ = new VectorMap(this);
    vector_map_->setEnglish(QSettings().value(QStringLiteral("map/labels_english"), false).toBool());
    tiles_->setVectorMap(vector_map_);
    elev_ = new ElevationCache(this);
    offline_ = new OfflineDownloader(this);

    map_ = new MapView(tiles_);
    for (const auto& line : QSettings().value(QStringLiteral("plan/hold_times")).toStringList()) {
        const auto f = line.split(QLatin1Char(','));
        if (f.size() == 4) {
            synced_holds_.push_back({f[0].toDouble(), f[1].toDouble(), f[2].toFloat(), f[3].toFloat()});
        }
    }
    map_->setPlanFixup([this](MissionPlan& p) { fillHoldTimes(p); });
    map_->setElevation(elev_);
    map_->setLayer(settings_.map_layer);
    map_->setFollow(settings_.follow);
    map_->setDefaultAltitude(settings_.waypoint_alt);
    map_->setDefaultLandAltitude(settings_.land_approach_alt);
    map_->setBallCollapsed(settings_.ball_collapsed);
    map_->setTrail(settings_.trail_on, settings_.trail_secs);
    camera_ = new CameraView(video_.get());
    camera_->setCamera(settings_.camera_hfov, settings_.camera_tilt);
    camera_->setHudReference(settings_.hud_reference == 1 ? CameraView::HudRef::Body : CameraView::HudRef::Los);
    camera_->setWindNorthUp(settings_.wind_north_up);
    camera_->setHudConfig(hud::decode(settings_.hud.toStdString()));  // empty: the starting CUSTOM set

    host_ = new ViewHost(map_, camera_, this);
    strip_ = new PlanStrip(this);
    strip_->hide();
    panel_ = new WaypointPanel(this);
    panel_->hide();
    orbit_panel_ = new OrbitPanel(this);
    orbit_panel_->hide();
    bar_ = new TelemetryBar(this);
    dock_ = new ControlDock(this);
    fx_ = new FxOverlay(this);
    fx_->setTopInset(barBottom());
    alerts_ = new AlertsPanel(this);
    hud_panel_ = new HudPanel(this);
    pf_ = std::make_unique<PreflightRunner>(client_.get());
    preflight_ = new PreflightScreen(this, pf_.get());
    pf_->onToast = [this](const QString& t, bool good) { fx_->toast(t, good ? kGreen : kRed, QStringLiteral("Preflight")); };
    pf_->onSafety = [this](bool make_safe) {
        client_->runSequence("safety", {link::makeSetSafety(make_safe)});
        fx_->toast(make_safe ? QStringLiteral("Safety ON: motors cut") : QStringLiteral("Safety OFF: motors live"),
                   make_safe ? kAmber : kRed, QStringLiteral("Safety"));
    };
    pf_->onComplete = [this] { fx_->banner(QStringLiteral("Preflight complete · ready to launch"), kGreen); };
    connect(preflight_, &PreflightScreen::launchOverride, this, [this] {
        pf_->overrideLaunch(Clock::now());
        fx_->toast(QStringLiteral("Launch allowed without a complete preflight (logged)"), kAmber, QStringLiteral("Preflight"));
    });
    settings_screen_ = new SettingsScreen(this);
    speed_panel_ = new SpeedPanel(this);
    pad_ = new EntryPad(this);  // on top of everything
    pad_->setGroundLookup([this](double lat, double lon) {
        return elev_ ? elev_->elevation(lat, lon) : std::optional<double>{};
    });
    connect(speed_panel_, &SpeedPanel::closeRequested, speed_panel_, &QWidget::hide);
    connect(speed_panel_, &SpeedPanel::modeChanged, this, [this](int m) {
        settings_.speed_mode = m;
        settings_.save();
        speed_sent_.reset();  // re-evaluate now
        refreshSpeedPanel();
    });
    connect(bar_, &TelemetryBar::speedTapped, this, [this] {
        if (speed_panel_->isVisible()) {
            speed_panel_->hide();
            return;
        }
        refreshSpeedPanel();
        speed_panel_->move(std::max(8, width() / 2 + static_cast<int>(120 * bar_->effectiveScale())), barBottom() + 4);
        speed_panel_->show();
        speed_panel_->raise();
    });

    connect(bar_, &TelemetryBar::systemsTapped, this, [this] { showAlerts(!alerts_->isVisible()); });

    bar_->setAltRef(settings_.alt_ref);
    connect(bar_, &TelemetryBar::altRefChanged, this, [this](int ref) {
        settings_.alt_ref = ref;
        settings_.save();
        const char* what[] = {"above sea level", "above the ground below (rangefinder, else terrain map)",
                              "above the landing point's ground"};
        fx_->toast(QStringLiteral("Altitude %1").arg(QLatin1String(what[std::clamp(ref, 0, 2)])), kCyan,
                   QStringLiteral("Altitude"));
    });
    connect(bar_, &TelemetryBar::settingsRequested, this, [this] {
        showAlerts(false);
        SettingsScreen::Values v;
        v.alt_unit = settings_.alt_unit;
        v.speed_unit = settings_.speed_unit;
        v.coord_format = settings_.coord_format;
        v.trail_on = settings_.trail_on;
        v.trail_secs = settings_.trail_secs;
        v.labels_english = vector_map_->english();
        v.reserve_pct = settings_.battery_reserve_pct;
        v.text_size = settings_.text_size;
        v.link = linkValues();
        refreshOfflineCounts();
        updateLinkStatus();
        settings_screen_->open(v);
    });
    // ---- offline map data ------------------------------------------------------------
    connect(settings_screen_, &SettingsScreen::offlineCountry, this, [this] {
        offline_status_.clear();
        offline_->startCountry();
    });
    connect(settings_screen_, &SettingsScreen::offlineCancel, offline_, &OfflineDownloader::cancel);
    connect(settings_screen_, &SettingsScreen::offlineSelectArea, this, [this] {
        settings_screen_->close();
        if (planning_) exitPlan();
        setGuide(false);
        if (host_->mainView() != map_) host_->swap();
        std::vector<MapView::GeoBox> boxes;
        for (const auto& b : OfflineDownloader::areas()) boxes.push_back({b.min_lon, b.min_lat, b.max_lon, b.max_lat});
        map_->setAreas(boxes);
        map_->clearAreaSelection();
        map_->setMode(MapView::Mode::Area);
    });
    connect(map_, &MapView::areaChanged, this, [this](MapView::GeoBox g, bool final) {
        if (!g.max_lat && !g.min_lat) return;
        const OfflineDownloader::Box b{g.min_lon, g.min_lat, g.max_lon, g.max_lat};
        const QString dims = QStringLiteral("%1 × %2 km").arg(b.widthKm(), 0, 'f', 1).arg(b.heightKm(), 0, 'f', 1);
        // the full count checks the disk for every tile: only when the finger lifts
        const double km2 = b.widthKm() * b.heightKm();
        const double tiles_guess = km2 * 75.0;  // z15-18 at ~31 N
        if (tiles_guess > 90000) {
            map_->setAreaInfo(dims + QStringLiteral(" · too big for full detail"),
                              QStringLiteral("about %1 tiles (~%2 GB). Draw a smaller area (up to ~1000 km²), or several.")
                                  .arg(static_cast<long long>(tiles_guess))
                                  .arg(tiles_guess * 22000 / 1e9, 0, 'f', 1),
                              false, true);
            return;
        }
        if (!final) {
            map_->setAreaInfo(dims, QStringLiteral("lift to count the tiles…"), false, false);
            return;
        }
        const auto c = OfflineDownloader::areaCount(b);
        map_->setAreaInfo(
            dims + QStringLiteral(" · ~%1 MB new").arg(c.bytes / 1e6, 0, 'f', 0),
            c.fresh == 0 ? QStringLiteral("all %1 tiles already on disk: nothing to download").arg(c.have)
                         : QStringLiteral("satellite to ~0.5 m/px + ground height · %1 new tiles · %2 already on disk "
                                          "(shared with areas you have)")
                               .arg(c.fresh)
                               .arg(c.have),
            c.fresh > 0 && !offline_->running(), false);
    });
    connect(map_, &MapView::areaCancel, this, [this] { map_->setMode(MapView::Mode::Normal); });
    connect(map_, &MapView::areaDownload, this, [this] {
        const auto g = map_->areaSelection();
        if (!g) return;
        offline_status_.clear();
        offline_->startArea({g->min_lon, g->min_lat, g->max_lon, g->max_lat});
        map_->setMode(MapView::Mode::Normal);
        fx_->toast(QStringLiteral("Downloading the full-detail area"), kCyan, QStringLiteral("Offline"));
    });
    connect(settings_screen_, &SettingsScreen::offlineClearDetail, this, [this] {
        offline_status_ = OfflineDownloader::clearFullDetail() ? QStringLiteral("full detail cleared")
                                                               : QStringLiteral("could not delete everything");
        tiles_->clearMemory();
        map_->tileSourcesChanged();
        refreshOfflineCounts();
    });
    connect(settings_screen_, &SettingsScreen::offlineDeleteAll, this, [this] {
        offline_status_ = OfflineDownloader::clearAll() ? QStringLiteral("everything deleted")
                                                        : QStringLiteral("could not delete everything");
        vector_map_->reopen();
        tiles_->vectorChanged();
        tiles_->clearMemory();
        map_->tileSourcesChanged();
        refreshOfflineCounts();
    });
    connect(settings_screen_, &SettingsScreen::offlineChooseFolder, this, [this] {
        const QString dir = QFileDialog::getExistingDirectory(this, QStringLiteral("Offline map folder"),
                                                              offline::folder());
        if (!dir.isEmpty()) {
            offline::setFolder(dir);
            offline_status_ = QStringLiteral("folder changed");
            vector_map_->reopen();
            tiles_->vectorChanged();
            tiles_->clearMemory();
            map_->tileSourcesChanged();
            refreshOfflineCounts();
        }
    });
    connect(settings_screen_, &SettingsScreen::onlineChanged, this, [this](bool on) {
        offline::setOnline(on);
        map_->tileSourcesChanged();
        refreshOffline();
    });
    connect(settings_screen_, &SettingsScreen::labelsEnglishChanged, this, [this](bool en) {
        QSettings().setValue(QStringLiteral("map/labels_english"), en);
        vector_map_->setEnglish(en);
        tiles_->vectorChanged();
    });
    connect(offline_, &OfflineDownloader::progress, this, [this] {
        refreshOffline();
        map_->setDownloadStatus(QStringLiteral("⬇ %1 · %2%").arg(offline_->stage()).arg(static_cast<int>(offline_->fraction() * 100)));
    });
    connect(offline_, &OfflineDownloader::streetMapInstalled, this, [this] {
        vector_map_->reopen();
        tiles_->vectorChanged();
        fx_->toast(QStringLiteral("Street map installed"), kGreen, QStringLiteral("Offline"));
    });
    connect(offline_, &OfflineDownloader::finished, this, [this](bool cancelled, const QString& summary) {
        offline_status_ = summary;
        map_->setDownloadStatus(QString());
        tiles_->clearMemory();
        map_->tileSourcesChanged();
        refreshOfflineCounts();
        fx_->toast(summary, cancelled || offline_->failed() ? kAmber : kGreen, QStringLiteral("Offline"));
    });
    connect(settings_screen_, &SettingsScreen::linkEdit, this, &MainWindow::editLinkField);
    connect(settings_screen_, &SettingsScreen::linkConnect, this, &MainWindow::applyLink);
    connect(settings_screen_, &SettingsScreen::altUnitChanged, this, &MainWindow::applyAltUnit);
    connect(settings_screen_, &SettingsScreen::speedUnitChanged, this, &MainWindow::applySpeedUnit);
    connect(settings_screen_, &SettingsScreen::coordFormatChanged, this, &MainWindow::applyCoordFormat);
    connect(settings_screen_, &SettingsScreen::reserveChanged, this, [this](int pct) {
        settings_.battery_reserve_pct = pct;
        settings_.save();
        endurance_ms_ = -1000000;  // recompute now
    });
    connect(settings_screen_, &SettingsScreen::textSizeChanged, this, &MainWindow::applyTextSize);
    connect(settings_screen_, &SettingsScreen::trailChanged, this, [this](bool on, int secs) {
        settings_.trail_on = on;
        settings_.trail_secs = secs;
        settings_.save();
        map_->setTrail(on, secs);
    });

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
    connect(map_, &MapView::ballCollapsedChanged, this, [this](bool on) {
        settings_.ball_collapsed = on;
        settings_.save();
    });
    connect(camera_, &CameraView::groundClicked, this, &MainWindow::onCameraClick);
    // the HUD: the panel edits it live; box folds and the brackets change it on the video
    connect(camera_, &CameraView::hudPanelRequested, this, [this] { showHudPanel(!hud_panel_->isVisible()); });
    connect(camera_, &CameraView::hudConfigChanged, this, [this] {
        hud_panel_->setConfig(camera_->hudConfig());
        saveHudConfig();
    });
    connect(hud_panel_, &HudPanel::configChanged, this, [this] {
        camera_->setHudConfig(hud_panel_->config());
        saveHudConfig();
    });
    connect(hud_panel_, &HudPanel::closeRequested, this, [this] { showHudPanel(false); });
    connect(host_, &ViewHost::swapped, this, [this] {
        if (host_->mainView() != camera_) {
            showHudPanel(false);  // it edits the video: only while the video is the main view
        }
    });
    connect(camera_, &CameraView::windNorthUpChanged, this, [this](bool on) {
        settings_.wind_north_up = on;
        settings_.save();
    });
    // what the HUD rulers follow: switched in the HUD panel (the panel shows it, no toast)
    connect(hud_panel_, &HudPanel::rulerRefChanged, this, [this](int ref) {
        settings_.hud_reference = ref;
        settings_.save();
        camera_->setHudReference(ref == 1 ? CameraView::HudRef::Body : CameraView::HudRef::Los, true);
    });
    connect(map_, &MapView::selectionChanged, this, &MainWindow::refreshPanel);
    connect(map_, &MapView::addToolChanged, this,
            [this](MapView::AddTool t) { strip_->setLandingTool(t == MapView::AddTool::Landing); });
    connect(strip_, &PlanStrip::landingToolToggled, this, [this](bool on) {
        map_->select(MapView::kNone);
        map_->setAddTool(on ? MapView::AddTool::Landing : MapView::AddTool::Waypoint);
        if (on) {
            fx_->toast(QStringLiteral("Tap the map to place the landing point"), kGreen, QStringLiteral("Plan"));
        }
    });
    connect(panel_, &WaypointPanel::altitudeDelta, this, [this](float d) {
        // the steppers stop at the minimum height above the ground at the point
        const int id = map_->selected();
        const auto wp = map_->point(id);
        if (!wp || !s_.has_home) {
            map_->adjustAltitude(id, d);
            return;
        }
        const double floor = altitude::minRelHome(groundsAt(wp->lat, wp->lon),
                                                  id == MapView::kLandId ? altitude::kMinApproachAgl : altitude::kMinAgl);
        map_->editPoint(id, [&](UserWaypoint& p) {
            p.alt_rel = static_cast<float>(std::clamp<double>(p.alt_rel + d, std::max(floor, 0.0), altitude::kMaxRelHome));
        }, 0);
    });
    connect(panel_, &WaypointPanel::coordRequested, this, [this] {
        const int id = map_->selected();
        const auto wp = map_->point(id);
        if (!wp) {
            return;
        }
        openCoordPad(id == MapView::kLandId ? QStringLiteral("LANDING POINT · POSITION")
                                            : QStringLiteral("WAYPOINT %1 · POSITION").arg(id + 1),
                     coords::LatLon{wp->lat, wp->lon}, [this, id](double lat, double lon) {
                         map_->editPoint(id, [&](UserWaypoint& p) {
                             p.lat = lat;
                             p.lon = lon;
                         });
                         map_->ping(lat, lon, kAmber);
                     });
    });
    connect(panel_, &WaypointPanel::altitudeTapped, this, [this] {
        const int id = map_->selected();
        const auto wp = map_->point(id);
        if (!wp) {
            return;
        }
        const bool land = id == MapView::kLandId;
        openAltPad(land ? QStringLiteral("LANDING POINT · APPROACH ALTITUDE") : QStringLiteral("WAYPOINT %1 · ALTITUDE").arg(id + 1),
                   wp->lat, wp->lon, wp->alt_rel, land ? altitude::kMinApproachAgl : altitude::kMinAgl,
                   [this, id](double rel) {
                       map_->editPoint(id, [&](UserWaypoint& p) { p.alt_rel = static_cast<float>(rel); });
                   });
    });
    connect(panel_, &WaypointPanel::holdTimeDelta, this, [this](float d) {
        map_->editPoint(map_->selected(), [&](UserWaypoint& p) {
            p.hold_s = std::clamp(p.hold_s + d, 0.0f, 3600.0f);
            syncHoldTurns(p);
        }, 1);
    });
    connect(panel_, &WaypointPanel::holdValueTapped, this, [this] {
        const int id = map_->selected();
        const auto wp = map_->point(id);
        if (!wp) {
            return;
        }
        pad_->openDuration(QStringLiteral("WAYPOINT %1 · HOLD TIME").arg(id + 1), wp->hold_s, 3600.0,
                           [this, id](double sec) {
                               map_->editPoint(id, [&](UserWaypoint& p) {
                                   p.hold_s = static_cast<float>(sec);
                                   syncHoldTurns(p);
                               });
                           });
    });
    connect(panel_, &WaypointPanel::insertRequested, this, [this] { map_->insertAfter(map_->selected()); });
    connect(panel_, &WaypointPanel::deleteRequested, this, [this] { map_->deletePoint(map_->selected()); });
    connect(panel_, &WaypointPanel::closeRequested, this, [this] { map_->select(MapView::kNone); });
    connect(panel_, &WaypointPanel::holdRadiusDelta, this, [this](float d) {
        const auto& plan = map_->plan();
        const int id = map_->selected();
        // stepping onto WP_LOITER_RAD's value means "the default" again (stored as 0)
        const float def = static_cast<float>(defaultRadius());
        auto stepped = [&](float now, float max) {
            const float r = std::clamp((now > 0.0f ? now : def) + d, kMinOrbitRadius, max);
            return std::abs(r - def) < 0.5f ? 0.0f : r;
        };
        if (id == holdIndex(plan)) {  // the final hold
            map_->setHold(stepped(plan.hold_radius, kMaxOrbitRadius), plan.hold_ccw);
            return;
        }
        // an own radius is uploaded as whole turns of about the hold time
        // (LOITER_TURNS keeps a radius, LOITER_TIME doesn't)
        map_->editPoint(id, [&](UserWaypoint& p) {
            p.loiter_radius = stepped(p.loiter_radius, kMaxTurnsRadius);
            syncHoldTurns(p);
        }, 2);
    });
    connect(panel_, &WaypointPanel::holdDirectionToggled, this, [this] {
        const auto& plan = map_->plan();
        const int id = map_->selected();
        if (id == holdIndex(plan)) {
            map_->setHold(plan.hold_radius, !plan.hold_ccw);
            return;
        }
        map_->editPoint(id, [&](UserWaypoint& p) { p.loiter_ccw = !p.loiter_ccw; });
    });
    connect(orbit_panel_, &OrbitPanel::coordRequested, this, [this] {
        std::optional<coords::LatLon> init;
        if (orbit_) {
            init = coords::LatLon{orbit_->lat, orbit_->lon};
        }
        openCoordPad(QStringLiteral("FLY TO · POSITION"), init, [this](double lat, double lon) { sendGuided(lat, lon); });
    });
    connect(orbit_panel_, &OrbitPanel::altitudeTapped, this, [this] {
        if (guiding_ && guide_draft_) {
            openAltPad(QStringLiteral("FLY TO · ALTITUDE"), s_.lat, s_.lon, guide_draft_->alt, altitude::kMinAgl,
                       [this](double rel) {
                           if (guide_draft_) {
                               guide_draft_->alt = static_cast<float>(rel);
                               refreshOrbitPanel();
                           }
                       });
        } else if (orbit_) {
            openAltPad(orbit_->hold ? QStringLiteral("HOLD · ALTITUDE") : QStringLiteral("FLY TO · ALTITUDE"), orbit_->lat,
                       orbit_->lon, orbit_->alt, altitude::kMinAgl, [this](double rel) {
                           if (orbit_) {
                               orbit_->alt = static_cast<float>(rel);
                               orbit_trim_only_ = false;
                               orbit_dirty_ = true;
                               refreshOrbitPanel();
                           }
                       });
        } else if (onEndHold()) {
            const MissionPlan plan = end_hold_plan_ ? *end_hold_plan_ : extractPlan(s_.mission);
            const int h = holdIndex(plan);
            if (h < 0) {
                return;
            }
            openAltPad(QStringLiteral("END HOLD · ALTITUDE"), plan.wps[h].lat, plan.wps[h].lon, plan.wps[h].alt_rel,
                       altitude::kMinAgl, [this](double rel) {
                           if (!end_hold_plan_) {
                               end_hold_plan_ = extractPlan(s_.mission);
                           }
                           const int hh = holdIndex(*end_hold_plan_);
                           if (hh >= 0) {
                               end_hold_plan_->wps[hh].alt_rel = static_cast<float>(rel);
                               end_hold_dirty_ = true;
                               refreshOrbitPanel();
                           }
                       });
        }
    });
    connect(strip_, &PlanStrip::addByCoordRequested, this, [this](bool landing) {
        openCoordPad(landing ? QStringLiteral("NEW LANDING POINT · POSITION") : QStringLiteral("NEW WAYPOINT · POSITION"),
                     std::nullopt, [this, landing](double lat, double lon) { map_->addPoint(lat, lon, landing); });
    });
    connect(orbit_panel_, &OrbitPanel::altitudeDelta, this, [this](float d) { adjustOrbit(d, 0.0f, false); });
    connect(orbit_panel_, &OrbitPanel::radiusDelta, this, [this](float d) { adjustOrbit(0.0f, d, false); });
    connect(orbit_panel_, &OrbitPanel::directionToggled, this, [this] { adjustOrbit(0.0f, 0.0f, true); });
    connect(strip_, &PlanStrip::patternChanged, map_, &MapView::setPattern);
    connect(strip_, &PlanStrip::undoRequested, map_, &MapView::undo);
    connect(strip_, &PlanStrip::clearRequested, map_, &MapView::clearPlan);
    connect(strip_, &PlanStrip::cancelRequested, this, [this] {
        exitPlan();
        fx_->toast(QStringLiteral("Plan discarded"), kMuted);
    });

    qApp->installEventFilter(this);

    client_->start();
    video_->start();

    diag_ = qEnvironmentVariableIntValue("GCS_FPS") > 0;
    timer_ = new QTimer(this);
    timer_->setTimerType(Qt::PreciseTimer);
    connect(timer_, &QTimer::timeout, this, &MainWindow::frame);
    timer_->start(16);
    clock_.start();

    fx_->toast(QStringLiteral("Link: %1").arg(QString::fromStdString(client_->snapshot().endpoint)), kCyan,
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
    bar_->setGeometry(8, 8, width() - 16, bar_->heightFor(width() - 16));
    const QSize ds = dock_->sizeHint();
    dock_->setGeometry((width() - ds.width()) / 2, height() - ds.height() - 10, ds.width(), ds.height());
    fx_->setTopInset(strip_->isVisible() ? strip_->geometry().bottom() + 4 : barBottom());
    host_->setPipMaxWidth((width() - ds.width()) / 2.0 - 32.0);
    strip_->adjustSize();
    strip_->move((width() - strip_->width()) / 2, barBottom() + 6);
    panel_->adjustSize();
    panel_->move((width() - panel_->width()) / 2, height() - ds.height() - 18 - panel_->height());
    orbit_panel_->move(orbitPanelX(), height() - ds.height() - 18 - orbit_panel_->height());
    fx_->setGeometry(rect());
    if (settings_screen_->isVisible()) {
        settings_screen_->setGeometry(rect());
    }
    if (preflight_->isVisible()) {
        preflight_->setGeometry(rect());
    }
    if (alerts_->isVisible()) {
        alerts_->place(alertsArea());
        fx_->setRightInset(alerts_->width() + 8.0);  // its width follows the text size
    }
    if (hud_panel_->isVisible()) {
        hud_panel_->place(alertsArea());
        fx_->setRightInset(hud_panel_->width() + 8.0);
    }
    strip_->raise();
    panel_->raise();
    orbit_panel_->raise();
    bar_->raise();
    dock_->raise();
    alerts_->raise();
    hud_panel_->raise();
    fx_->raise();
    if (settings_screen_->isVisible()) {
        settings_screen_->raise();
    }
    if (preflight_->isVisible()) {
        preflight_->raise();
    }
    if (pad_ && pad_->isOpen()) {
        pad_->raise();  // typing a value for a panel underneath
    }
}

// ---- main loop --------------------------------------------------------------------

void MainWindow::frame() {
    const qint64 now_ms = clock_.elapsed();
    const double dt = std::clamp((now_ms - last_ms_) / 1000.0, 0.0, 0.1);
    last_ms_ = now_ms;

    // every frame: the snapshot is a cheap copy, and motion is predicted from
    // timestamped samples so the HUD and map move continuously between them
    ++frame_count_;
    pullState();
    updateMotion();
    refreshApproachForWind(now_ms);
    if (orbit_dirty_ && orbit_ && now_ms - orbit_sent_ms_ >= kOrbitResendMs) {
        sendOrbit();
    }
    if (end_hold_dirty_ && !uploading_ && !s_.mission_transfer_active &&
        now_ms - end_hold_sent_ms_ >= kEndHoldResendMs) {
        sendEndHold(now_ms);
    }
    host_->tick(dt);
    map_->tick(dt);
    camera_->tick(dt);
    bar_->tick(dt);
    dock_->tick(dt);
    fx_->tick(dt);
    settings_screen_->tick(dt);
    if (settings_screen_->isVisible()) {
        updateLinkStatus();
    }
    alerts_->tick(dt);
    if (hud_panel_->isVisible()) {
        hud_panel_->setLiveVideo(camera_->live());
    }
    hud_panel_->tick(dt);
    pf_->update(s_, Clock::now(), video_->frameAge());
    preflight_->tick(dt);
    if (preflight_->isOpen() && isAirborne(s_)) {
        preflight_->close();  // ground only
    }
    if (diag_) {
        diagFrame(now_ms);
    }
}

void MainWindow::diagFrame(qint64 now_ms) {
    // GCS_FPS=1: once a second, how often frame() ran, how often the map and
    // camera actually painted, and the worst stalls. Low paint counts with a
    // normal tick count mean the window system is throttling presentation.
    const qint64 work = clock_.elapsed() - now_ms;
    diag_work_max_ = std::max(diag_work_max_, work);
    if (diag_last_tick_ > 0) {
        diag_gap_max_ = std::max(diag_gap_max_, now_ms - diag_last_tick_);
    }
    diag_last_tick_ = now_ms;
    ++diag_ticks_;
    if (s_.att_rx != diag_att_rx_) {
        diag_att_rx_ = s_.att_rx;
        ++diag_att_;
    }
    if (s_.pos_rx != diag_pos_rx_) {
        diag_pos_rx_ = s_.pos_rx;
        ++diag_pos_;
    }
    if (now_ms - diag_t0_ >= 1000) {
        std::fprintf(stderr,
                     "fps: ticks %d · map paints %d · camera paints %d · longest gap %lld ms · longest frame() %lld ms"
                     " · attitude %d Hz · position %d Hz\n",
                     diag_ticks_, map_->paints_, camera_->paints_, static_cast<long long>(diag_gap_max_),
                     static_cast<long long>(diag_work_max_), diag_att_, diag_pos_);
        diag_att_ = diag_pos_ = 0;
        map_->paints_ = camera_->paints_ = 0;
        diag_ticks_ = 0;
        diag_gap_max_ = diag_work_max_ = 0;
        diag_t0_ = now_ms;
    }
}

int MainWindow::orbitPanelX() const {
    // centred, unless that would cover the picture-in-picture: then just right of it
    const int centred = (width() - orbit_panel_->width()) / 2;
    const int clear = static_cast<int>(host_->pipRect().right()) + 12;
    return std::max(centred, std::min(clear, width() - orbit_panel_->width() - 8));
}

int MainWindow::barBottom() const {
    return bar_->geometry().bottom() + 1;
}

void MainWindow::applyTextSize(int size) {
    settings_.text_size = std::clamp(size, 0, 2);
    settings_.save();
    setUiScale(scaleFor(static_cast<TextSize>(settings_.text_size)));
    // stylesheet panels rebuild their sheets; painted widgets read uiScale() on their next paint
    strip_->applyScale();
    panel_->applyScale();
    orbit_panel_->applyScale();
    speed_panel_->applyScale();
    pad_->applyScale();
    layoutOverlays();
    refreshPlanStrip();
    if (speed_panel_->isVisible()) {
        speed_panel_->move(std::max(8, width() / 2 + static_cast<int>(120 * bar_->effectiveScale())), barBottom() + 4);
    }
    for (QWidget* w : findChildren<QWidget*>()) {
        w->update();
    }
}

QRect MainWindow::alertsArea() const {
    const int top = barBottom() + 6;
    const int bottom = height() - dock_->sizeHint().height() - 20;
    return QRect(8, top, width() - 16, std::max(160, bottom - top));
}

void MainWindow::refreshOfflineCounts() {
    // disk walks: only when Settings opens or a download / delete finishes
    offline_disk_ = OfflineDownloader::diskUsage();
    offline_area_bytes_ = OfflineDownloader::fullDetailUsage();
    offline_areas_ = static_cast<int>(OfflineDownloader::areas().size());
    const auto c = OfflineDownloader::countryCount();
    const auto a = OfflineDownloader::areasMissing();
    const bool street = OfflineDownloader::hasStreetMap();
    const QString build = OfflineDownloader::installedBuild();
    if (!street) {
        offline_country_ = QStringLiteral("not downloaded · about %1 MB").arg(c.bytes / 1e6, 0, 'f', 0);
    } else {
        QStringList parts;
        parts << (build.isEmpty() ? QStringLiteral("street map ✓")
                                  : QStringLiteral("street map ✓ %1-%2-%3").arg(build.left(4), build.mid(4, 2), build.mid(6, 2)));
        parts << (c.fresh == 0 ? QStringLiteral("country ✓")
                               : QStringLiteral("country: %1 tiles missing (~%2 MB)").arg(c.fresh).arg(c.bytes / 1e6, 0, 'f', 0));
        if (offline_areas_ > 0) {
            parts << (a.fresh == 0 ? QStringLiteral("areas ✓") : QStringLiteral("areas: %1 tiles missing").arg(a.fresh));
        }
        offline_country_ = parts.join(QStringLiteral(" · "));
    }
    refreshOffline();
}

void MainWindow::refreshOffline() {
    SettingsScreen::Offline o;
    o.street_map = OfflineDownloader::hasStreetMap();
    o.country = offline_country_;
    o.areas = offline_areas_;
    o.area_bytes = offline_area_bytes_;
    o.running = offline_->running();
    o.stage = offline_->stage();
    o.fraction = offline_->fraction();
    o.disk_bytes = offline_disk_;
    o.folder = offline::folder();
    o.status = offline_status_;
    o.online = offline::online();
    settings_screen_->setOffline(o);
}

// ---- link (Settings > CONNECTION) ----------------------------------------------------------

link::MavlinkClient::Config MainWindow::linkConfig() const {
    link::MavlinkClient::Config cfg;
    cfg.type = settings_.link_serial ? link::MavlinkClient::LinkType::Serial : link::MavlinkClient::LinkType::Udp;
    cfg.bind_host = settings_.host.toStdString();
    cfg.port = static_cast<uint16_t>(settings_.port);
    cfg.remote_host = settings_.remote_host.toStdString();
    cfg.remote_port = static_cast<uint16_t>(settings_.remote_port);
    cfg.serial_port = settings_.serial_port.toStdString();
    cfg.serial_baud = settings_.serial_baud;
    return cfg;
}

SettingsScreen::Link MainWindow::linkValues() const {
    SettingsScreen::Link l;
    l.serial = settings_.link_serial;
    l.host = settings_.host;
    l.port = settings_.port;
    l.remote_host = settings_.remote_host;
    l.remote_port = settings_.remote_port;
    l.serial_port = settings_.serial_port;
    l.baud = settings_.serial_baud;
    l.video_port = settings_.video_port;
    return l;
}

void MainWindow::applyLink(const SettingsScreen::Link& l) {
    const bool video_changed = l.video_port != settings_.video_port;
    settings_.link_serial = l.serial;
    settings_.host = l.host;
    settings_.port = l.port;
    settings_.remote_host = l.remote_host;
    settings_.remote_port = l.remote_port;
    settings_.serial_port = l.serial_port;
    settings_.serial_baud = l.baud;
    settings_.video_port = l.video_port;
    settings_.saveLink();
    const auto cfg = linkConfig();
    client_->reconfigure(cfg);
    if (video_changed) {
        video_->restart(l.video_port);
    }
    fx_->toast(QStringLiteral("Connecting: %1").arg(QString::fromStdString(link::MavlinkClient::makeTransport(cfg)->describe())),
               kCyan, QStringLiteral("Link"));
}

void MainWindow::editLinkField(int field) {
    const SettingsScreen::Link cur = settings_screen_->link();
    if (field == SettingsScreen::kListenField) {
        pad_->openAddress(QStringLiteral("LISTEN FOR TELEMETRY ON"), cur.host, cur.port,
                          {{QStringLiteral("ALL NETWORKS :14550"), QStringLiteral("0.0.0.0"), 14550},
                           {QStringLiteral("THIS COMPUTER :14550"), QStringLiteral("127.0.0.1"), 14550},
                           {QStringLiteral("SIMULATOR :14551"), QStringLiteral("127.0.0.1"), 14551}},
                          [this](const QString& host, int port) {
                              auto l = settings_screen_->link();
                              l.host = host;
                              l.port = port;
                              settings_screen_->setLink(l);
                          });
    } else if (field == SettingsScreen::kSendToField) {
        pad_->openAddress(QStringLiteral("ALSO SEND TO (RADIO / COMPANION)"), cur.remote_host,
                          cur.remote_port > 0 ? cur.remote_port : 14550,
                          {{QStringLiteral("OFF · ONLY REPLY"), QString(), 0}},
                          [this](const QString& host, int port) {
                              auto l = settings_screen_->link();
                              l.remote_host = host;
                              l.remote_port = host.isEmpty() ? 0 : port;
                              settings_screen_->setLink(l);
                          });
    } else if (field == SettingsScreen::kVideoField) {
        pad_->openNumber(QStringLiteral("VIDEO PORT (UDP, RTP / H.264)"), QStringLiteral("PORT"), cur.video_port, 1, 65535,
                         {{QStringLiteral("5600"), 5600}, {QStringLiteral("5601"), 5601}, {QStringLiteral("5000"), 5000}},
                         [this](int port) {
                             auto l = settings_screen_->link();
                             l.video_port = port;
                             settings_screen_->setLink(l);
                         });
    }
}

void MainWindow::updateLinkStatus() {
    const QString where = QString::fromStdString(s_.endpoint);
    QString text;
    QColor c = kMuted;
    const double hb_age =
        s_.has_heartbeat ? std::chrono::duration<double>(Clock::now() - s_.last_heartbeat).count() : 1e9;
    if (!s_.error.empty()) {
        text = QStringLiteral("%1 · retrying").arg(QString::fromStdString(s_.error));
        c = kRed;
    } else if (!s_.socket_open) {
        text = QStringLiteral("opening %1…").arg(where);
    } else if (hb_age < 3.0) {
        text = QStringLiteral("CONNECTED · %1").arg(where);
        c = kGreen;
    } else {
        text = QStringLiteral("open · waiting for the aircraft on %1").arg(where);
        c = kAmber;
    }
    settings_screen_->setLinkStatus(text, c, s_.armed);
}

void MainWindow::showAlerts(bool on) {
    bar_->setSystemsOpen(on);
    if (!on) {
        alerts_->hide();
        fx_->setRightInset(0.0);
        return;
    }
    if (hud_panel_->isVisible()) {
        hud_panel_->hide();  // one panel on the right at a time
        camera_->setHudPanelOpen(false);
    }
    alerts_->setAdvisories(advisories_, linkAlive(s_, Clock::now()));
    alerts_->place(alertsArea());
    alerts_->show();
    alerts_->raise();
    fx_->setRightInset(alerts_->width() + 8.0);
}

void MainWindow::showHudPanel(bool on) {
    if (on) {
        showAlerts(false);
        speed_panel_->hide();
        hud_panel_->setConfig(camera_->hudConfig());
        hud_panel_->setRulerRef(settings_.hud_reference);
        hud_panel_->setLiveVideo(camera_->live());
        hud_panel_->place(alertsArea());
        hud_panel_->show();
        hud_panel_->raise();
        fx_->setRightInset(hud_panel_->width() + 8.0);
    } else if (hud_panel_->isVisible()) {
        hud_panel_->hide();
        fx_->setRightInset(0.0);
    }
    camera_->setHudPanelOpen(on);
}

void MainWindow::saveHudConfig() {
    settings_.hud = QString::fromStdString(hud::encode(camera_->hudConfig()));
    settings_.save();
}

void MainWindow::openPreflight() {
    showAlerts(false);
    preflight_->setGeometry(rect());
    preflight_->open(flew_ && pf_->session().active());
    preflight_->raise();
}

void MainWindow::applyAltUnit(int unit) {
    settings_.alt_unit = unit;
    units::setAltUnit(unit == 1 ? units::Alt::Feet : units::Alt::Meters);
    settings_.save();
    orbit_panel_->unitsChanged();
    panel_->unitsChanged();
    refreshPanel();
    refreshOrbitPanel();
    refreshDock();
    layoutOverlays();
    fx_->toast(unit == 1 ? QStringLiteral("Altitudes in feet") : QStringLiteral("Altitudes in metres"), kCyan,
               QStringLiteral("Settings"));
}

void MainWindow::updateHeights(qint64 now_ms) {
    // AGL: the rangefinder when its reading can be trusted, else the terrain
    // map; AFL: above the landing point's ground. Terrain lookups at 2 Hz.
    const auto rf = rangefinderAgl(s_, Clock::now());
    if (now_ms - heights_ms_ >= 500 && s_.has_position) {
        heights_ms_ = now_ms;
        terrain_here_ = elev_ ? elev_->elevation(s_.lat, s_.lon) : std::nullopt;
        land_ground_.reset();
        const MissionPlan plan = withLanding(s_.mission_known ? extractPlan(s_.mission) : MissionPlan{});
        if (plan.land) {
            const bool at_home = s_.has_home && geo::distance(plan.land->lat, plan.land->lon, s_.home_lat,
                                                              s_.home_lon) < 30.0;
            if (const auto e = elev_ ? elev_->elevation(plan.land->lat, plan.land->lon) : std::nullopt; e && !at_home) {
                land_ground_ = *e;
            } else if (s_.has_home) {
                land_ground_ = s_.home_alt_amsl;  // home (or no terrain tile): the autopilot's home altitude
            }
        }
    }
    std::optional<double> agl;
    if (rf) {
        agl = *rf;
    } else if (terrain_here_ && s_.has_position) {
        agl = s_.alt_amsl - *terrain_here_;
    }
    std::optional<double> afl;
    if (land_ground_ && s_.has_position) {
        afl = s_.alt_amsl - *land_ground_;
    }
    bar_->setAltitudes(agl, afl);

    // the map's nav box: ground under the aircraft, and where it is going (5 Hz)
    if (now_ms - nav_ms_ < 200) {
        return;
    }
    nav_ms_ = now_ms;
    MapView::NavInfo nav;
    if (s_.has_position) {
        QStringList ground;
        if (terrain_here_) {
            ground << QStringLiteral("GROUND %1 MSL").arg(units::altText(*terrain_here_));
        }
        if (agl) {
            ground << QStringLiteral("AGL %1 (%2)").arg(units::altText(*agl), rf ? QStringLiteral("rangefinder")
                                                                                : QStringLiteral("terrain"));
        }
        nav.aircraft_ground = ground.join(QStringLiteral("  ·  "));
    }
    if (climb_out_) {
        nav.target = pointTarget(s_, "CLIMB-OUT", climb_out_lat_, climb_out_lon_);
    } else if (s_.custom_mode == plane::AUTO) {
        nav.target = missionTarget(s_);
    } else if (orbit_ && s_.custom_mode == plane::GUIDED) {
        nav.target = pointTarget(s_, orbit_->hold ? "HOLD" : "FLY TO", orbit_->lat, orbit_->lon, orbit_->radius);
    }
    bar_->setNavTarget(nav.target);
    map_->setNav(std::move(nav));
}

void MainWindow::updateWaypointHold(FlightStatus& fs) {
    // A waypoint hold is a timer kept here: it starts when the aircraft is on
    // the circle, and when it runs out the GCS moves the mission on
    // (SET_MISSION_CURRENT). ArduPlane would otherwise finish its circle (or its
    // spare turn). The on-board item is only the backstop without the link.
    const int cur = s_.mission_current;
    const bool on_item = s_.custom_mode == plane::AUTO && s_.mission_known && s_.has_position && cur >= 1 &&
                         cur < static_cast<int>(s_.mission.size()) &&
                         (s_.mission[cur].command == mav::kCmdNavLoiterTime ||
                          s_.mission[cur].command == mav::kCmdNavLoiterTurns) &&
                         seqToWaypointIndex(s_.mission, cur) >= 0;
    if (!on_item) {
        wp_hold_seq_ = -1;
        return;
    }
    const MissionItem& it = s_.mission[cur];
    if (wp_hold_seq_ != cur) {
        wp_hold_seq_ = cur;
        wp_hold_since_ms_ = -1;
        wp_hold_cut_ = false;
    }
    const bool turns = it.command == mav::kCmdNavLoiterTurns;
    const float radius = turns && std::abs(it.param3) > 1.0f ? std::abs(it.param3) : 0.0f;
    std::optional<float> hold_s;
    if (turns) {
        hold_s = syncedHold(it.lat(), it.lon(), radius);  // unknown for another GCS's mission: its turns rule
    } else {
        hold_s = it.param1;
    }
    const double r = radius > 0.0f ? radius : defaultRadius();
    if (wp_hold_since_ms_ < 0 && onCircle(geo::distance(s_.lat, s_.lon, it.lat(), it.lon()), r)) {
        wp_hold_since_ms_ = clock_.elapsed();
    }
    if (wp_hold_since_ms_ < 0 || !hold_s) {
        return;
    }
    const double left = *hold_s - (clock_.elapsed() - wp_hold_since_ms_) / 1000.0;
    fs.label += " · HOLD " + units::durationText(std::max(0.0, std::ceil(left))).toStdString();
    if (left > 0.0 || wp_hold_cut_ || uploading_ || s_.mission_transfer_active) {
        return;
    }
    // time is up: on to the next point (through a DO_JUMP to its target)
    int next = cur + 1;
    for (int guard = 0; guard < 8 && next < static_cast<int>(s_.mission.size()); ++guard) {
        const MissionItem& n = s_.mission[next];
        if (n.command == mav::kCmdDoJump) {
            next = static_cast<int>(std::lround(n.param1));
        } else {
            break;
        }
    }
    wp_hold_cut_ = true;
    const int land = landSeq(s_.mission);
    if (next < 1 || next >= static_cast<int>(s_.mission.size()) || (land >= 0 && next >= land) ||
        s_.mission[next].command >= 100) {
        return;  // nothing to go on to (landing, or not a point): leave it to the aircraft
    }
    client_->runSequence("hold_done", {link::makeSetMissionCurrent(static_cast<uint16_t>(next))});
    const int wp_from = seqToWaypointIndex(s_.mission, cur);
    const int wp_to = seqToWaypointIndex(s_.mission, next);
    fx_->toast(wp_to >= 0 ? QStringLiteral("WP%1 hold done → WP%2").arg(wp_from + 1).arg(wp_to + 1)
                          : QStringLiteral("WP%1 hold done").arg(wp_from + 1),
               kCyan, QStringLiteral("Plan"));
}

void MainWindow::rememberHolds(const MissionPlan& plan) {
    synced_holds_.clear();
    QStringList saved;
    for (const auto& wp : plan.wps) {
        if (wp.hold_s > 0.0f) {
            synced_holds_.push_back({wp.lat, wp.lon, wp.loiter_radius, wp.hold_s});
            saved << QStringLiteral("%1,%2,%3,%4")
                         .arg(wp.lat, 0, 'f', 8)
                         .arg(wp.lon, 0, 'f', 8)
                         .arg(wp.loiter_radius)
                         .arg(wp.hold_s);
        }
    }
    QSettings().setValue(QStringLiteral("plan/hold_times"), saved);
}

std::optional<float> MainWindow::syncedHold(double lat, double lon, float radius) const {
    for (const auto& h : synced_holds_) {
        // AP_Mission keeps radii above 255 m in 10 m steps
        if (geo::distance(lat, lon, h.lat, h.lon) < 1.0 &&
            std::abs(storedTurnsRadius(h.radius) - radius) < 0.5f) {
            return h.hold_s;
        }
    }
    return std::nullopt;
}

void MainWindow::fillHoldTimes(MissionPlan& plan) const {
    for (auto& wp : plan.wps) {
        if (wp.loiter_radius > 0.0f && wp.hold_turns > 0 && wp.hold_s <= 0.0f) {
            if (const auto t = syncedHold(wp.lat, wp.lon, wp.loiter_radius)) {
                wp.hold_s = *t;
            } else {  // not ours: about as long as its turns
                wp.hold_s = static_cast<float>(std::round(wp.hold_turns * lapSeconds(wp.loiter_radius) / 5.0) * 5.0);
            }
        }
    }
}

void MainWindow::syncHoldTurns(UserWaypoint& p) const {
    p.hold_turns = p.loiter_radius > 0.0f ? holdTurns(p.hold_s, lapSeconds(p.loiter_radius)) : 0;
}

double MainWindow::lapSeconds(double radius) const {
    // one turn of a hold circle at the speed the speed mode will ask for
    const double cruise = s_.airspeed_cruise > 0 ? s_.airspeed_cruise : settings_.cruise_speed;
    double v = cruise;
    if (settings_.speed_mode == static_cast<int>(speed::Mode::Eco) && s_.airspeed_min > 0) {
        v = speed::loiterSpeed(s_.airspeed_min, cruise, s_.q_assist_speed);
    }
    return 2.0 * geo::kPi * std::max(20.0, radius) / std::max(5.0, v);
}

altitude::Grounds MainWindow::groundsAt(double lat, double lon) const {
    altitude::Grounds g;
    g.home_amsl = s_.home_alt_amsl;
    g.point = elev_ ? elev_->elevation(lat, lon) : std::nullopt;
    g.land = land_ground_;
    return g;
}

void MainWindow::openCoordPad(const QString& title, std::optional<coords::LatLon> initial,
                              std::function<void(double, double)> done) {
    pad_->setAircraft(s_.has_position ? std::optional<coords::LatLon>(coords::LatLon{s_.lat, s_.lon}) : std::nullopt);
    pad_->setHome(s_.has_home ? std::optional<coords::LatLon>(coords::LatLon{s_.home_lat, s_.home_lon}) : std::nullopt);
    pad_->openCoordinate(title, initial, std::move(done));
}

void MainWindow::openAltPad(const QString& title, double lat, double lon, double rel, double min_agl,
                            std::function<void(double)> done) {
    if (!s_.has_home) {
        fx_->toast(QStringLiteral("No home position yet: altitudes are relative to it"), kMuted, QStringLiteral("Altitude"));
        return;
    }
    const MissionPlan plan = withLanding(s_.mission_known ? extractPlan(s_.mission) : MissionPlan{});
    std::optional<coords::LatLon> land;
    if (planning_ && map_->plan().land) {
        land = coords::LatLon{map_->plan().land->lat, map_->plan().land->lon};
    } else if (plan.land) {
        land = coords::LatLon{plan.land->lat, plan.land->lon};
    }
    pad_->setHome(coords::LatLon{s_.home_lat, s_.home_lon});
    pad_->openAltitude(title, lat, lon, rel, s_.home_alt_amsl, land, settings_.alt_ref, min_agl,
                       [done = std::move(done)](double r, int) { done(r); });
}

void MainWindow::refreshSpeedPanel() {
    speed_panel_->setValues(s_.airspeed_min, s_.airspeed_cruise, s_.airspeed_max, settings_.speed_mode,
                            speed_label_, s_.q_assist_speed > 0 ? speed::loiterSpeed(s_.airspeed_min, s_.airspeed_cruise,
                                                                              s_.q_assist_speed)
                                                                : s_.airspeed_min,
                            s_.q_assist_speed);
}

void MainWindow::updateSpeedTarget(qint64 now_ms) {
    // which airspeed to ask for, from what the aircraft is doing (2 Hz)
    if (now_ms - speed_eval_ms_ < 500) {
        return;
    }
    speed_eval_ms_ = now_ms;
    using namespace speed;
    if (s_.airspeed_min <= 0 || s_.airspeed_cruise <= 0 || s_.airspeed_max <= 0) {
        return;  // parameters not read yet: leave the autopilot alone
    }
    Situation sit = Situation::None;
    if (s_.armed && isAirborne(s_) && !climb_out_ && !takeoff_handoff_) {
        const bool landing = isLandingInMission(s_);
        if (landing) {
            sit = Situation::Landing;
        } else if (s_.custom_mode == plane::GUIDED && orbit_) {
            const double d = geo::distance(s_.lat, s_.lon, orbit_->lat, orbit_->lon);
            sit = d < orbit_->radius * 1.5 + 30.0 ? Situation::Circling : Situation::EnRoute;
        } else if (s_.custom_mode == plane::AUTO && s_.mission_known && s_.mission_current >= 1 &&
                   s_.mission_current < static_cast<int>(s_.mission.size())) {
            const MissionItem& it = s_.mission[s_.mission_current];
            const bool loiter = it.command == mav::kCmdNavLoiterUnlim || it.command == mav::kCmdNavLoiterTime ||
                                it.command == mav::kCmdNavLoiterTurns;
            const double r = std::abs(it.param3) > 1.0f ? std::abs(it.param3) : defaultRadius();
            sit = loiter && geo::distance(s_.lat, s_.lon, it.lat(), it.lon()) < r * 1.5 + 30.0 ? Situation::Circling
                                                                                                 : Situation::EnRoute;
        }
        // descending: well above where it is going (ArduPlane slews its own
        // altitude target, so NAV_CONTROLLER_OUTPUT's error stays small)
        if (sit == Situation::EnRoute && s_.has_position) {
            std::optional<double> goal;
            if (s_.custom_mode == plane::GUIDED && orbit_) {
                goal = orbit_->alt;
            } else if (s_.custom_mode == plane::AUTO && s_.mission_current >= 1 &&
                       s_.mission_current < static_cast<int>(s_.mission.size())) {
                goal = s_.mission[s_.mission_current].z;
            }
            if (goal && s_.alt_rel - *goal > kDescentForMax) {
                sit = Situation::Descending;
            }
        }
    }
    const Target want = target(static_cast<Mode>(settings_.speed_mode), sit);
    if (s_.custom_mode != speed_mode_seen_) {
        speed_mode_seen_ = s_.custom_mode;
        speed_sent_.reset();  // a mode change may reset ArduPlane's target: send again
    }
    if (!speed_want_ || *speed_want_ != want) {
        speed_want_ = want;
        speed_want_ms_ = now_ms;
    }
    // hold a new target for 3 s before sending (no flapping); landing at once
    const bool settled = now_ms - speed_want_ms_ >= 3000 || sit == Situation::Landing;
    if (!settled || (speed_sent_ && *speed_sent_ == want)) {
        return;
    }
    float v = -2.0f;  // AIRSPEED_CRUISE (ArduPilot's default)
    const double loiter = loiterSpeed(s_.airspeed_min, s_.airspeed_cruise, s_.q_assist_speed);
    if (want == Target::Loiter) {
        v = static_cast<float>(loiter);
    } else if (want == Target::Max) {
        v = static_cast<float>(s_.airspeed_max);
    }
    if (want == Target::Default && (!speed_sent_ || *speed_sent_ == Target::Default || *speed_sent_ == Target::Cruise)) {
        speed_sent_ = want;  // nothing of ours to undo
        speed_label_.clear();
        return;
    }
    client_->runSequence("speed", {link::makeChangeSpeed(v)});
    speed_sent_ = want;
    const double shown = want == Target::Loiter ? loiter : want == Target::Max ? s_.airspeed_max : s_.airspeed_cruise;
    speed_label_ = want == Target::Default ? QString()
                                           : QStringLiteral("%1 %2").arg(QLatin1String(targetName(want)), units::speedText(shown));
    if (speed_panel_->isVisible()) {
        refreshSpeedPanel();
    }
}

void MainWindow::applySpeedUnit(int unit) {
    settings_.speed_unit = unit;
    units::setSpeedUnit(static_cast<units::Speed>(unit));
    settings_.save();
    fx_->toast(unit == 1 ? QStringLiteral("Speeds in knots") : QStringLiteral("Speeds in m/s"), kCyan,
               QStringLiteral("Settings"));
}

void MainWindow::applyCoordFormat(int fmt) {
    settings_.coord_format = fmt;
    units::setCoordFormat(static_cast<coords::Fmt>(fmt));
    settings_.save();
    const char* names[] = {"decimal degrees", "degrees, minutes, seconds", "UTM"};
    fx_->toast(QStringLiteral("Coordinates in %1").arg(QLatin1String(names[std::clamp(fmt, 0, 2)])), kCyan,
               QStringLiteral("Settings"));
}

void MainWindow::pullState() {
    s_ = client_->snapshot();
    const auto now = Clock::now();
    for (const auto& ev : client_->pollEvents()) {
        handleEvent(ev);
    }

    // mode / VTOL change banners
    if (s_.has_heartbeat && s_.mode_name != last_mode_) {
        if (s_.custom_mode != plane::GUIDED && !last_mode_.empty() && orbit_) {
            orbit_.reset();  // an orbit only means something in GUIDED
            map_->setOrbit(std::nullopt);
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

    checkTakeoffHandoff();
    checkClimbOut();
    updateOrbitTrim();
    // a flight happened: the preflight no longer holds; offer the turnaround one
    if (isAirborne(s_) && s_.armed && !flew_) {
        flew_ = true;
        pf_->noteFlight();
    }
    const LiftRotors lift = liftRotorState(s_, now);
    if (lift == LiftRotors::Assist && last_lift_ != LiftRotors::Assist) {
        fx_->toast(QStringLiteral("Lift rotors assisting (Q_ASSIST): low airspeed, steep bank or low altitude"),
                   kAmber, QStringLiteral("Rotors"));
    }
    last_lift_ = lift;

    updateEndurance(clock_.elapsed());
    {
        FlightContext ctx;
        ctx.orbit = orbit_.has_value() && s_.custom_mode == plane::GUIDED;
        ctx.orbit_hold = orbit_ && orbit_->hold;
        ctx.transition_leg = climb_out_ || takeoff_handoff_;
        FlightStatus fs = flightStatus(s_, ctx, now);
        updateWaypointHold(fs);
        bar_->setFlightStatus(fs);
        camera_->setFlightLabel(QString::fromStdString(fs.label));
    }
    updateHeights(clock_.elapsed());
    updateSpeedTarget(clock_.elapsed());
    autoSafetyOnLanded();
    bar_->setSpeed(speed_sent_ ? static_cast<int>(*speed_sent_) : 0, settings_.speed_mode, isAirborne(s_));
    bar_->setCommandedAlt(commandedAltRel());
    bar_->setState(s_, now);
    map_->setState(s_);
    map_->setDefaultLoiterRadius(defaultRadius());
    camera_->setState(s_);
    updateAlerts();
    refreshDock();
    refreshOrbitPanel();
}

void MainWindow::updateMotion() {
    const auto now = Clock::now();
    if (s_.has_attitude) {
        const geo::Attitude att = predictAttitude(s_, now);
        const CameraLos los = cameraLos(s_, att, settings_.camera_tilt, now, settings_.gimbal_body_frame);
        camera_->setMotion(att, los);
        map_->setHeading(att.yaw_deg);
        map_->setAttitude(att.roll_deg, att.pitch_deg);
        updateFootprint(los);
        updateLosTarget(los);
    }
    if (s_.has_position) {
        const PredictedPosition p = predictPosition(s_, now);
        map_->setAircraftPosition(p.lat, p.lon);
        camera_->setAltitude(p.alt_rel);
        if (s_.pos_rx != track_rx_) {
            track_rx_ = s_.pos_rx;
            track_.addSample(s_.vn, s_.ve, s_.vd, s_.pos_rx, s_.yaw_rate);
        }
        const PredictedVelocity v = track_.at(now);
        camera_->setVelocity(v.vn, v.ve, v.vd);
    }
}

void MainWindow::updateLosTarget(const CameraLos& los) {
    // where the camera looks, against the terrain (5 Hz: ~200 elevation lookups)
    const qint64 now_ms = clock_.elapsed();
    if (now_ms - los_ms_ < 200) {
        return;
    }
    los_ms_ = now_ms;
    CameraView::LosTarget t;
    if (s_.has_position && isAirborne(s_)) {
        t.valid = true;
        const auto ground = [this](double lat, double lon) {
            return elev_ ? elev_->elevation(lat, lon) : std::optional<double>{};
        };
        const geo::TerrainHit h = geo::rayToTerrain(s_.lat, s_.lon, s_.alt_amsl, los.earth.yaw_deg, los.earth.pitch_deg,
                                                    ground, s_.has_home ? s_.home_alt_amsl : s_.alt_amsl - s_.alt_rel);
        t.hit = h.hit;
        t.approximate = h.approximate;
        t.lat = h.lat;
        t.lon = h.lon;
        t.ground = h.ground;
        t.slant = h.slant;
    }
    camera_->setLosTarget(t);
}

void MainWindow::updateFootprint(const CameraLos& los) {
    // flat ground at home level; on the ground there is nothing to project
    const double agl = s_.alt_rel;
    if (!s_.has_position || agl < 3.0) {
        map_->setFootprint(std::nullopt);
        return;
    }
    geo::CameraModel cam;
    cam.hfov_deg = settings_.camera_hfov;
    cam.mount_pitch_deg = 0.0;  // los.earth already includes the mount / gimbal
    cam.width = 1600.0;
    cam.height = 1600.0 / camera_->aspect();
    const double max_range = std::min(3000.0, 25.0 * agl);
    const geo::Footprint fp = geo::cameraFootprint(cam, los.earth, agl, max_range);
    if (!fp.valid) {
        map_->setFootprint(std::nullopt);
        return;
    }
    MapView::FootprintDraw d;
    d.outline = fp.outline;
    d.open = fp.open;
    d.center = {fp.center.north, fp.center.east};
    d.center_hit = fp.center.hit_ground;
    map_->setFootprint(std::move(d));
}

void MainWindow::updateAlerts() {
    const auto now = Clock::now();
    const bool alive = linkAlive(s_, now);
    if (alive && !link_was_alive_ && ever_connected_) {
        fx_->banner(QStringLiteral("Link restored"), kGreen);
    }
    link_was_alive_ = alive;

    book_.update(s_, now);
    std::vector<Advisory> state = assess(s_, now, ever_connected_);
    state = mergeAdvisories(std::move(state), endurance_cards_);
    advisories_ = mergeAdvisories(state, book_.current());
    reconcileLandedMode(advisories_, s_);
    {
        // the vehicle's own words for why it won't arm, for the preflight's final step
        std::vector<std::string> reasons;
        for (const auto& adv : advisories_) {
            if (adv.id.rfind("prearm.", 0) == 0 && adv.id != "prearm.mode") {
                reasons.push_back(adv.detail.empty() ? adv.title : adv.detail);
            }
        }
        pf_->setPrearmReasons(std::move(reasons));
    }
    bar_->setAdvisories(advisories_);
    if (alerts_->isVisible()) {
        alerts_->setAdvisories(advisories_, alive);
    }

    // announce new state-derived problems once (text ones toast on arrival):
    // critical always, faults once armed (on the bench they're expected)
    std::set<std::string> present;
    for (const auto& a : state) {
        present.insert(a.id);
        if (announced_.count(a.id)) {
            continue;
        }
        if (a.severity == Severity::Critical || (a.severity == Severity::Warning && s_.armed)) {
            announced_.insert(a.id);
            fx_->toast(QStringLiteral("%1 · tap SYSTEMS for help").arg(QString::fromStdString(a.title)),
                       a.severity == Severity::Critical ? kRed : kAmber, QStringLiteral("Alert"));
        }
    }
    for (auto it = announced_.begin(); it != announced_.end();) {
        it = present.count(*it) ? std::next(it) : announced_.erase(it);  // cleared: may announce again
    }

    // red banner: the worst critical problem (tap it for the explanation)
    QString alert;
    for (const auto& a : advisories_) {
        if (a.severity == Severity::Critical) {
            alert = QString::fromStdString(a.title);
            break;
        }
    }
    if (alert.isEmpty() && alive && s_.q_guided_mode == 1) {
        alert = QStringLiteral("Fly To/Hold would hover on the rotors (Q_GUIDED_MODE=1)");
    }
    fx_->setAlert(alert);
}

void MainWindow::updateEndurance(qint64 now_ms) {
    using namespace endurance;
    const bool flying = s_.armed && isAirborne(s_);
    if (!s_.armed) {
        // landed (or a fresh pack): learn the currents again next flight
        if (endurance_tracker_.cruiseSeconds() > 0.0 || endurance_tracker_.hoverAmps() >= 0.0 ||
            endurance_latch_.level() != Level::None || !endurance_cards_.empty()) {
            endurance_tracker_.reset();
            endurance_latch_.reset();
            endurance_shown_.reset();
            endurance_learned_ = false;
            endurance_cards_.clear();
            bar_->setEndurance(false, 0.0, 0, Clock::now());
        }
        return;
    }
    if (now_ms - endurance_sample_ms_ < kEnduranceSampleMs) {
        return;
    }
    endurance_sample_ms_ = now_ms;

    // learn the currents (the takeoff hover, then cruise on the wing) on the
    // autopilot's clock, so a short takeoff isn't missed between samples
    Phase phase = Phase::Other;
    if (flying) {
        const LiftRotors lift = liftRotorState(s_, Clock::now());
        if (lift == LiftRotors::Active && effectiveVtolState(s_) == VtolState::Hover) {
            phase = Phase::Hover;
        } else if (effectiveVtolState(s_) == VtolState::Cruise) {
            phase = Phase::Cruise;  // Q_ASSIST included: it costs charge too
        }
    }
    const double vehicle_t = s_.time_boot_ms > 0 ? s_.time_boot_ms / 1000.0 : now_ms / 1000.0;
    endurance_tracker_.addSample(vehicle_t, s_.battery_a, phase);

    if (now_ms - endurance_ms_ < kEnduranceMs || !flying || !s_.has_position) {
        return;  // the estimate: a few hundred flops every 2 s; the bar counts down in between
    }
    endurance_ms_ = now_ms;
    const double t = now_ms / 1000.0;

    Inputs in;
    in.capacity_mah = s_.batt_capacity_mah;
    in.consumed_mah = s_.battery_consumed_mah;
    in.battery_pct = s_.battery_pct;
    in.reserve = settings_.battery_reserve_pct / 100.0;
    in.current_a = s_.battery_a;
    in.cruise_a = endurance_tracker_.cruiseAmps();
    in.hover_a = endurance_tracker_.hoverAmps();
    in.in_cruise = phase == Phase::Cruise;
    in.airspeed = s_.airspeed_cruise > 0 ? s_.airspeed_cruise : settings_.cruise_speed;
    in.wind_from_deg = s_.wind_from_deg;
    in.wind_speed = s_.has_wind ? s_.wind_speed : 0.0;
    in.here = {s_.lat, s_.lon, s_.alt_rel};

    // where it lands: L, or home (what Land and the failsafes fly)
    const MissionPlan plan = withLanding(s_.mission_known ? extractPlan(s_.mission) : MissionPlan{});
    for (const auto& wp : plan.wps) {
        in.route.push_back({wp.lat, wp.lon, wp.alt_rel});
    }
    if (orbit_) {
        in.route.push_back({orbit_->lat, orbit_->lon, orbit_->alt});  // a Guide target counts as a place it may be
    }
    if (plan.land) {
        in.land_lat = plan.land->lat;
        in.land_lon = plan.land->lon;
        const auto appr = s_.mission_known ? extractApproach(s_.mission) : std::nullopt;
        const LandingApproach a = appr ? *appr : approachFor(plan, s_.lat, s_.lon);
        in.approach_alt = a.alt;
        in.approach_leg_m = a.leg_m;
        in.approach_radius = a.radius;
    } else {
        endurance_cards_.clear();  // nowhere known to land yet (no home): nothing to cost
        bar_->setEndurance(false, 0.0, 0, Clock::now());
        return;
    }

    Estimate e = estimate(in);
    if (e.valid) {
        // the pilot sees a smoothed time (a climb or a gust doesn't make it
        // jump); Critical still comes from the raw estimate. The first
        // estimates use a cautious guess of the cruise current; once it is
        // learned, start the smoothing over from the real figure instead of
        // creeping up to it for minutes.
        if (e.cruise_learned && !endurance_learned_) {
            endurance_shown_.reset();
        }
        endurance_learned_ = e.cruise_learned;
        e.time_left_s = endurance_shown_.update(e.time_left_s, t);
    }
    const Level before = endurance_latch_.level();
    const Level level = endurance_latch_.update(levelFor(e, e.time_left_s), t);
    endurance_cards_ = advisories(e, level);
    bar_->setEndurance(e.valid, e.time_left_s, static_cast<int>(level), Clock::now());
    // warnings and criticals are toasted with the other alerts; "soon" once here
    if (level == Level::Caution && before == Level::None) {
        fx_->toast(QStringLiteral("Battery: %1 of flying left, then head back and land")
                       .arg(QString::fromStdString(clockText(e.time_left_s))),
                   kAmber, QStringLiteral("Battery"));
    }
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
        const bool prearm = text.startsWith(QLatin1String("PreArm")) || text.startsWith(QLatin1String("Arm:"));
        if (prearm) {
            c = kAmber;
            tag = QStringLiteral("PreArm");
        }
        // known problems become a card in SYSTEMS; the toast says what it
        // means instead of the raw text (repeats are not toasted again)
        if (const auto adv = book_.onStatusText(ev.text, ev.severity, Clock::now())) {
            fx_->toast(QStringLiteral("%1 · tap SYSTEMS for help").arg(QString::fromStdString(adv->title)),
                       adv->severity == Severity::Critical ? kRed : (prearm ? kAmber : c), tag);
        } else if (!explainStatusText(ev.text, ev.severity)) {
            fx_->toast(text, c, tag);
        }
        break;
    }
    case T::CommandStep:
        break;
    case T::CommandResult: {
        if (ev.tag.rfind("pf:", 0) == 0) {
            pf_->onEvent(ev);  // the preflight window shows these itself
            break;
        }
        if (ev.tag == "orbit-trim") {
            if (!ev.ok) {
                fx_->toast(QStringLiteral("Orbit radius trim: %1").arg(text), kAmber, QStringLiteral("Orbit"));
            }
            break;
        }
        if (ev.tag == "launch" && !ev.ok) {
            takeoff_handoff_ = false;
        }
        if (ev.tag == "climb-out" && !ev.ok) {
            climb_out_ = false;
            hold();  // circle here instead
        }
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
        if (approach_refresh_) {
            approach_refresh_ = false;
            if (const auto appr = extractApproach(ev.items)) {
                fx_->toast(QStringLiteral("Wind turned: landing approach now %1°").arg(appr->heading_deg, 0, 'f', 0),
                           kCyan, QStringLiteral("Land"));
            }
            break;
        }
        if (land_pending_) {
            // the approach (with the current wind) is on board: jump into it
            land_pending_ = false;
            const int seq = landSeq(ev.items);
            if (seq >= 0) {
                client_->runSequence("land", {link::makeSetMissionCurrent(static_cast<uint16_t>(seq)),
                                              link::makeSetMode(plane::AUTO, "AUTO")});
                const auto appr = extractApproach(ev.items);
                fx_->toast(appr ? QStringLiteral("Approach heading %1°, leg %2 m")
                                      .arg(appr->heading_deg, 0, 'f', 0)
                                      .arg(appr->leg_m, 0, 'f', 0)
                                : QStringLiteral("Landing"),
                           kGreen, QStringLiteral("Land"));
            }
            break;
        }
        if (rejoin_after_upload_ || end_hold_upload_) {
            const bool silent = end_hold_upload_;
            rejoin_after_upload_ = end_hold_upload_ = false;
            int seq = -1;
            if (s_.custom_mode == plane::AUTO && isAirborne(s_)) {
                seq = silent ? holdSeq(ev.items) : rejoinSeq(ev.items);
            }
            if (silent && !end_hold_dirty_) {
                end_hold_plan_.reset();  // on board now; the panel reads it back from the mission
            }
            if (seq >= 0) {
                // setting the current item makes ArduPlane load it now
                client_->runSequence(silent ? "end-hold" : "auto",
                                     {link::makeSetMissionCurrent(static_cast<uint16_t>(seq))});
                if (!silent) {
                    announceRejoin(ev.items, seq, QStringLiteral("New plan"));
                }
            }
            if (silent) {
                break;  // a live hold edit: no toast or dock flash
            }
        }
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
        rejoin_after_upload_ = false;
        if (end_hold_upload_) {
            end_hold_upload_ = false;  // quiet: the edit is still pending and is sent again
            end_hold_dirty_ = true;
            uploading_ = false;
            break;
        }
        if (approach_refresh_) {
            approach_refresh_ = false;  // quiet: tried again on the next wind check
            uploading_ = false;
            approach_wind_.reset();
            break;
        }
        if (land_pending_) {
            land_pending_ = false;
            dock_->flash(kLand, false);
            fx_->toast(QStringLiteral("Landing approach upload failed (%1); still holding").arg(text), kRed,
                       QStringLiteral("Land"));
        }
        uploading_ = false;
        dock_->setProgress(kPlan, -1.0);
        dock_->flash(kPlan, false);
        fx_->toast(QStringLiteral("Mission %1 failed: %2").arg(ev.upload ? QStringLiteral("upload") : QStringLiteral("download"), text),
                   kRed, QStringLiteral("Plan"));
        break;
    case T::VehicleConnected:
        ever_connected_ = true;
        fx_->banner(QStringLiteral("Link established"), kGreen);
        fx_->toast(QStringLiteral("Vehicle sysid %1 connected").arg(ev.done), kGreen, QStringLiteral("Link"));
        break;
    case T::Error:
        fx_->toast(text, kRed, QStringLiteral("Error"));
        break;
    case T::ParamSet:
        pf_->onEvent(ev);
        fx_->toast(ev.ok ? QStringLiteral("%1 = %2 (confirmed)").arg(QString::fromStdString(ev.tag), text)
                         : QStringLiteral("%1 not set: %2").arg(QString::fromStdString(ev.tag), text),
                   ev.ok ? kGreen : kRed, QStringLiteral("Param"));
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
    if (tag == "land") {
        return kLand;
    }
    if (tag == "guide") {
        return kGuide;
    }
    return -1;
}

// ---- dock -----------------------------------------------------------------------

void MainWindow::refreshDock() {
    const bool alive = linkAlive(s_, Clock::now());
    const bool airborne = isAirborne(s_);
    const QString no_link = QStringLiteral("No vehicle link");

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
            } else if (map_->plan().wps.empty()) {
                spec.enabled = false;
                spec.reason = QStringLiteral("Tap the map to add waypoints");
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
                                 : QStringLiteral("Takeoff %1").arg(units::altText(settings_.takeoff_alt).remove(QLatin1Char(' ')));
            spec.icon = mission ? DockIcon::Launch : DockIcon::Takeoff;
            spec.hold_to_confirm = true;
            spec.hint = mission ? QStringLiteral("Hold: vertical takeoff, straight transition, then the plan")
                                : QStringLiteral("Hold: vertical takeoff, straight transition, then Hold");
            if (safetyState(s_) == SafetyState::Live && !s_.armed) {
                spec.hint += QStringLiteral("\nSafety OFF: motors live (it turns back ON after landing)");
            }
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
            } else if (!pf_->launchAllowed(s_, Clock::now())) {
                spec.enabled = false;
                spec.reason = pf_->launchBlockReason(s_, Clock::now());
            } else if (safetyState(s_) == SafetyState::Safe) {
                // first hold: motors live; the button then becomes Takeoff / Launch
                spec.label = QStringLiteral("Safety Off");
                spec.icon = DockIcon::Safety;
                spec.accent = kAmber;
                spec.hint = QStringLiteral("Safety is ON: motors cut.\nHold 1 s: motors live. Then hold again to take off");
                if (clock_.elapsed() < safety_pending_until_ms_) {
                    spec.label = QStringLiteral("Safety…");
                    spec.enabled = false;
                    spec.reason = QStringLiteral("Turning the safety off");
                }
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
            // during a landing the vehicle is in AUTO too, but on the land
            // items: Auto then aborts it and resumes the loop
            const bool landing = isLandingInMission(s_);
            spec.active = s_.custom_mode == plane::AUTO && !landing;
            spec.label = spec.active ? QStringLiteral("On Plan") : QStringLiteral("Resume Plan");
            spec.icon = DockIcon::Auto;
            spec.hint = landing ? QStringLiteral("Abort the landing and go back to the planned loop (AUTO)")
                                : QStringLiteral("Go back to the planned loop on the wing (AUTO)");
            if (!alive) {
                spec.enabled = false;
                spec.reason = no_link;
            } else if (!hasFlyableMission(s_)) {
                spec.enabled = false;
                spec.reason = QStringLiteral("No mission on the vehicle");
            } else if (spec.active) {
                spec.enabled = false;
                spec.reason = QStringLiteral("Already flying the plan");
            }
        }
        dock_->setSlot(kLaunch, spec);
    }

    // 4 guide (airborne) / preflight (on the ground)
    if (!airborne && !s_.armed) {
        DockSlotSpec spec;
        spec.label = QStringLiteral("Preflight");
        spec.icon = DockIcon::Preflight;
        const bool ok = pf_->launchAllowed(s_, Clock::now());
        spec.accent = ok ? kGreen : (pf_->session().active() && pf_->session().failCount() ? kRed : kAmber);
        spec.active = preflight_->isOpen();
        spec.hint = ok ? QStringLiteral("Preflight passed · tap to review")
                       : QStringLiteral("Every check, test and calibration before launch");
        if (!alive) {
            spec.enabled = false;
            spec.reason = no_link;
        }
        dock_->setSlot(kGuide, spec);
        const auto& ses = pf_->session();
        dock_->setProgress(kGuide, ses.active() && !ok ? static_cast<double>(ses.doneCount()) / std::max(1, ses.totalCount()) : -1.0);
        if (guiding_) {
            setGuide(false);
        }
    } else {
        dock_->setProgress(kGuide, -1.0);
        DockSlotSpec spec;
        spec.label = QStringLiteral("Fly To");
        spec.icon = DockIcon::Guide;
        spec.active = guiding_;
        spec.hint = QStringLiteral("Tap the map or camera to send the aircraft there (on the wing)");
        if (s_.q_guided_mode == 1) {
            spec.hint += QStringLiteral("\n⚠ Q_GUIDED_MODE=1: the aircraft will hover on the lift rotors at the target");
        }
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
        spec.label = QStringLiteral("Hold");
        spec.icon = DockIcon::Hold;
        spec.active = s_.custom_mode == plane::GUIDED && orbit_ && orbit_->hold;
        spec.hint = QStringLiteral("Circle here on the wing (also aborts a landing); press again to hide or show the "
                                   "altitude / radius panel");
        if (!alive) {
            spec.enabled = false;
            spec.reason = no_link;
        } else if (!airborne) {
            spec.enabled = false;
            spec.reason = QStringLiteral("Not flying");
        }
        dock_->setSlot(kHold, spec);
    }

    // 5 land
    {
        DockSlotSpec spec;
        const int seq = s_.mission_known ? landSeq(s_.mission) : -1;
        spec.label = QStringLiteral("Land");
        spec.icon = DockIcon::Land;
        spec.accent = kGreen;
        spec.hold_to_confirm = true;
        spec.active = land_pending_ || (seq >= 0 && s_.custom_mode == plane::AUTO && s_.mission_current >= seq);
        const bool has_l = s_.mission_known && extractPlan(s_.mission).land.has_value();
        spec.hint = has_l ? QStringLiteral("Hold: circle down beside L on the wing, leave into the wind, land vertically")
                          : QStringLiteral("Hold: no landing point, so it lands at home: circle down on the wing, "
                                           "leave into the wind, land vertically");
        if (!alive) {
            spec.enabled = false;
            spec.reason = no_link;
        } else if (!s_.mission_known || extractPlan(s_.mission).wps.empty() || !s_.has_home) {
            spec.enabled = false;
            spec.reason = QStringLiteral("No mission on the vehicle (Plan → upload)");
        } else if (!airborne) {
            spec.enabled = false;
            spec.reason = QStringLiteral("Not flying");
        } else if (spec.active) {
            spec.enabled = false;
            spec.reason = QStringLiteral("Already landing");
        } else if (uploading_ || s_.mission_transfer_active) {
            spec.enabled = false;
            spec.reason = QStringLiteral("Mission transfer in progress");
        }
        dock_->setSlot(kLand, spec);
    }
}

void MainWindow::onTriggered(int slot) {
    switch (slot) {
    case kPlan:
        if (planning_) {
            syncPlan();
        } else {
            enterPlan();
        }
        break;
    case kLaunch:
        if (!isAirborne(s_) && !s_.armed && safetyState(s_) == SafetyState::Safe) {
            // the first of the two holds: motors live
            client_->runSequence("safety", {link::makeSetSafety(false)});
            safety_pending_until_ms_ = clock_.elapsed() + 2500;
            fx_->toast(QStringLiteral("Safety OFF: motors live · hold again to take off"), kRed,
                       QStringLiteral("Safety"));
            refreshDock();
            break;
        }
        launchOrAuto();
        break;
    case kGuide:
        if (!isAirborne(s_) && !s_.armed) {
            openPreflight();
        } else {
            if (guiding_ && orbit_ && !orbit_->hold) {
                hold_panel_hidden_ = true;  // Fly To off by hand: its panel goes too (the circle goes on)
            }
            setGuide(!guiding_);
            refreshDock();
        }
        break;
    case kHold:
        if (orbit_ && orbit_->hold && s_.custom_mode == plane::GUIDED && isAirborne(s_)) {
            // already holding: a second press hides (or shows) the panel; the circle goes on
            hold_panel_hidden_ = !hold_panel_hidden_;
            refreshOrbitPanel();
        } else {
            hold_panel_hidden_ = false;
            hold();
        }
        break;
    case kLand:
        land();
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
    MissionPlan plan = s_.mission_known ? extractPlan(s_.mission) : MissionPlan{};
    fillHoldTimes(plan);  // an own-radius hold comes back as turns: the time as set
    map_->setPlan(plan);
    map_->setMode(MapView::Mode::Plan);
    if (host_->mainView() != map_) {
        host_->swap();
    }
    strip_->show();
    strip_->setLandingTool(false);
    refreshPlanStrip();
    fx_->banner(QStringLiteral("Plan mode"), kAmber);
}

void MainWindow::exitPlan() {
    planning_ = false;
    map_->setMode(guiding_ ? MapView::Mode::Guide : MapView::Mode::Normal);
    strip_->hide();
    fx_->setTopInset(barBottom());
    refreshPanel();
    refreshDock();
}

void MainWindow::refreshPlanStrip() {
    const auto& plan = map_->plan();
    const double route = routeLength(plan);
    strip_->setInfo(static_cast<int>(plan.wps.size()), route, route / std::max(1.0, settings_.cruise_speed),
                    plan.land.has_value(), map_->canUndo(), plan.pattern);
    strip_->move((width() - strip_->width()) / 2, barBottom() + 6);
    // keep toasts / alerts clear of the strip's buttons
    fx_->setTopInset(strip_->isVisible() ? strip_->geometry().bottom() + 4 : barBottom());
    refreshPanel();
    refreshDock();
}

void MainWindow::refreshPanel() {
    const int sel = map_->selected();
    const auto& plan = map_->plan();
    if (!planning_ || sel == MapView::kNone) {
        panel_->hide();
        return;
    }
    if (sel == MapView::kLandId && plan.land) {
        panel_->showLanding(plan.land->alt_rel);
    } else if (sel >= 0 && sel < static_cast<int>(plan.wps.size())) {
        panel_->showWaypoint(sel, plan.wps[sel].alt_rel);
    } else {
        panel_->setHold(WaypointPanel::HoldView{});
        panel_->hide();
        return;
    }
    {
        WaypointPanel::HoldView hv;
        hv.default_radius = defaultRadius();
        if (sel >= 0 && sel == holdIndex(plan)) {
            hv.mode = WaypointPanel::HoldMode::Final;
            hv.radius = plan.hold_radius;
            hv.ccw = plan.hold_ccw;
        } else if (sel >= 0) {
            const auto& wp = plan.wps[sel];
            hv.mode = WaypointPanel::HoldMode::Timed;
            hv.hold_s = wp.hold_s;
            hv.radius = wp.loiter_radius;
            hv.ccw = wp.loiter_ccw;
        }
        panel_->setHold(hv);
    }
    const QSize ds = dock_->sizeHint();
    panel_->move((width() - panel_->width()) / 2, height() - ds.height() - 18 - panel_->height());
    panel_->show();
    panel_->raise();
    fx_->raise();
}

MissionPlan MainWindow::withLanding(MissionPlan plan) const {
    // Every uploaded mission carries a landing sequence: failsafe RTLs jump to
    // it (RTL_AUTOLAND = 2). Without an L the aircraft lands back at home.
    if (!plan.land && s_.has_home) {
        plan.land = UserWaypoint{s_.home_lat, s_.home_lon, settings_.land_approach_alt};
    }
    // own-radius holds go up as turns: re-derive them at today's loiter speed
    // (a rebuild from the vehicle's mission gets its times back first)
    fillHoldTimes(plan);
    for (auto& wp : plan.wps) {
        if (wp.hold_s > 0.0f) {
            syncHoldTurns(wp);
        }
    }
    return plan;
}

LandingApproach MainWindow::approachFor(const MissionPlan& plan, double from_lat, double from_lon) const {
    ApproachInputs in;
    in.from_lat = from_lat;
    in.from_lon = from_lon;
    in.wind_from_deg = s_.wind_from_deg;
    in.wind_speed = s_.has_wind ? s_.wind_speed : 0.0;
    in.airspeed = std::max(s_.airspeed_cruise > 0 ? s_.airspeed_cruise : settings_.cruise_speed,
                           isAirborne(s_) ? s_.airspeed : 0.0);
    in.trans_decel = s_.trans_decel;
    in.radius = defaultRadius();
    return computeLandingApproach(*plan.land, in);
}

void MainWindow::syncPlan() {
    if (map_->plan().wps.empty() || !s_.has_home) {
        return;
    }
    const bool had_land = map_->plan().land.has_value();
    const MissionPlan plan = withLanding(map_->plan());
    // provisional approach (from the loop's side, current wind); it is
    // refreshed in flight when the wind turns, and Land recomputes it
    double from_lat = 0.0, from_lon = 0.0;
    for (const auto& wp : plan.wps) {
        from_lat += wp.lat / plan.wps.size();
        from_lon += wp.lon / plan.wps.size();
    }
    const LandingApproach appr = approachFor(plan, from_lat, from_lon);
    auto items = buildLoopMission(s_.home_lat, s_.home_lon, s_.home_alt_amsl, plan, settings_.takeoff_alt, &appr);
    rememberHolds(plan);
    uploading_ = true;
    // flying the plan: switch to the new one as soon as it is on board
    // (ArduPlane would otherwise finish the leg it has in memory)
    rejoin_after_upload_ = flyingThePlan();
    approach_wind_ = s_.has_wind ? std::optional<double>(s_.wind_from_deg) : std::nullopt;
    dock_->setProgress(kPlan, 0.0);
    client_->uploadMission(std::move(items));
    const QString route = plan.wps.size() < 2                      ? QStringLiteral("circle at %1 waypoint")
                          : plan.pattern == Pattern::BackForth ? QStringLiteral("back-and-forth of %1 waypoints")
                          : plan.pattern == Pattern::OneWay    ? QStringLiteral("one-way route of %1 waypoints, then hold")
                                                               : QStringLiteral("loop of %1 waypoints");
    fx_->toast(QStringLiteral("Uploading ") + route.arg(plan.wps.size()) +
                   (had_land ? QStringLiteral(" + landing point") : QStringLiteral(" · lands at home (no L set)")),
               kAmber, QStringLiteral("Plan"));
}

void MainWindow::refreshApproachForWind(qint64 now_ms) {
    // A failsafe RTL flies the approach that is on board, so keep it into the
    // current wind. Same item count as before: ArduPlane keeps flying the leg
    // it is on while the items are replaced.
    if (!s_.has_wind || s_.wind_speed < kMinWindForApproach || !isAirborne(s_) || !s_.mission_known ||
        uploading_ || land_pending_ || s_.mission_transfer_active || planning_ || isLandingInMission(s_) ||
        landSeq(s_.mission) < 0 || now_ms - approach_sent_ms_ < kApproachRefreshMs) {
        return;
    }
    if (!approach_wind_) {
        // the approach on board (from another session): it points into the wind it was made for
        if (const auto appr = extractApproach(s_.mission)) {
            approach_wind_ = appr->heading_deg;
        }
    }
    if (approach_wind_ && std::abs(geo::wrap180(s_.wind_from_deg - *approach_wind_)) < kApproachRefreshDeg) {
        return;
    }
    const MissionPlan plan = withLanding(extractPlan(s_.mission));
    if (plan.wps.empty() || !plan.land) {
        return;
    }
    const LandingApproach appr = approachFor(plan, s_.lat, s_.lon);
    auto items = buildLoopMission(s_.home_lat, s_.home_lon, s_.home_alt_amsl, plan, settings_.takeoff_alt, &appr);
    if (items.size() != s_.mission.size()) {
        return;  // not a mission this GCS laid out; leave it alone
    }
    approach_sent_ms_ = now_ms;
    approach_wind_ = s_.wind_from_deg;
    uploading_ = true;
    approach_refresh_ = true;
    client_->uploadMission(std::move(items));
}

void MainWindow::setGuide(bool on) {
    if (guiding_ == on) {
        return;
    }
    guiding_ = on;
    if (on && planning_) {
        exitPlan();
    }
    // no circle of ours to adjust yet (AUTO, climb-out): the panel edits the
    // altitude / radius / direction the first target will be flown at
    guide_draft_.reset();
    if (on && !(orbit_ && s_.custom_mode == plane::GUIDED)) {
        Orbit d;
        d.alt = static_cast<float>(std::max(30.0, std::round(s_.alt_rel)));
        d.radius = static_cast<float>(defaultRadius());
        d.hold = false;
        guide_draft_ = d;
    }
    if (!planning_) {
        map_->setMode(on ? MapView::Mode::Guide : MapView::Mode::Normal);
    }
    camera_->setGuideMode(on);
    refreshOrbitPanel();
}

void MainWindow::sendGuided(double lat, double lon) {
    takeoff_handoff_ = false;  // a guided target from the hover flies off on the wing
    climb_out_ = false;
    Orbit o;
    if (guide_draft_) {
        o = *guide_draft_;  // what the panel was set to before the first target
    } else if (orbit_) {
        o = *orbit_;  // keep the altitude / radius / direction being flown
    } else {
        o.alt = static_cast<float>(std::max(30.0, std::round(s_.alt_rel)));
        o.radius = static_cast<float>(defaultRadius());
    }
    o.lat = lat;
    o.lon = lon;
    o.hold = false;
    orbit_ = o;
    trim_.setRequested(o.radius);
    sendOrbit();
    map_->ping(lat, lon, kCyan);
    dock_->setPending(kGuide, true);
    // one target per press of Fly To: the button and its panel go off (press Fly To
    // again to adjust this circle or pick a new target); the circle goes on
    hold_panel_hidden_ = true;
    setGuide(false);
    refreshDock();
}

double MainWindow::defaultRadius() const {
    return s_.loiter_radius > 1.0 ? s_.loiter_radius : 80.0;
}

void MainWindow::sendOrbit() {
    if (!orbit_) {
        return;
    }
    // trim corrections are silent (no toast or dock flash unless they fail)
    client_->runSequence(orbit_trim_only_ ? "orbit-trim" : (orbit_->hold ? "hold" : "guide"),
                         {link::makeReposition(orbit_->lat, orbit_->lon, orbit_->alt,
                                               static_cast<float>(trim_.commandRadius()), orbit_->ccw)});
    orbit_dirty_ = false;
    orbit_trim_only_ = false;
    orbit_sent_ms_ = clock_.elapsed();
    map_->setOrbit(orbitDraw());
    refreshOrbitPanel();
}

MapView::OrbitDraw MainWindow::orbitDraw() const {
    MapView::OrbitDraw d{orbit_->lat, orbit_->lon, orbit_->radius, orbit_->ccw, orbit_->hold};
    d.flown = trim_.liveRadius() > 0.0 ? trim_.liveRadius() : trim_.flownRadius();
    d.trimmed = trim_.liveRadius() > 0.0 ? trim_.liveMatched() : trim_.converged();
    return d;
}

void MainWindow::updateOrbitTrim() {
    if (!orbit_ || s_.custom_mode != plane::GUIDED || !isAirborne(s_) || !s_.has_position ||
        s_.pos_rx == trim_pos_rx_) {
        return;
    }
    trim_pos_rx_ = s_.pos_rx;  // one sample per telemetry position, not per frame
    const geo::NE d = geo::delta(orbit_->lat, orbit_->lon, s_.lat, s_.lon);
    if (trim_.addSample(d.north, d.east)) {
        orbit_trim_only_ = !orbit_dirty_;  // a user edit already pending stays loud
        orbit_dirty_ = true;  // re-sent (same centre = live change) from frame()
    }
    map_->setOrbit(orbitDraw());  // the live flown radius changes every sample
    refreshOrbitPanel();
}

void MainWindow::adjustOrbit(float d_alt, float d_radius, bool flip) {
    if (guiding_ && guide_draft_) {
        // nothing flown yet: only the values the first target will use
        guide_draft_->alt = std::clamp(guide_draft_->alt + d_alt, kMinOrbitAlt, kMaxOrbitAlt);
        guide_draft_->radius = std::clamp(guide_draft_->radius + d_radius, kMinOrbitRadius, kMaxOrbitRadius);
        if (flip) {
            guide_draft_->ccw = !guide_draft_->ccw;
        }
        refreshOrbitPanel();
        return;
    }
    if (!orbit_) {
        if (onEndHold()) {
            adjustEndHold(d_alt, d_radius, flip);
        }
        return;
    }
    {
        // never below the minimum height above the ground at the centre
        const double floor = s_.has_home ? altitude::minRelHome(groundsAt(orbit_->lat, orbit_->lon)) : kMinOrbitAlt;
        orbit_->alt = static_cast<float>(std::clamp<double>(orbit_->alt + d_alt, std::max<double>(kMinOrbitAlt, floor),
                                                            kMaxOrbitAlt));
    }
    orbit_->radius = std::clamp(orbit_->radius + d_radius, kMinOrbitRadius, kMaxOrbitRadius);
    if (flip) {
        orbit_->ccw = !orbit_->ccw;
    }
    trim_.setRequested(orbit_->radius);  // re-measure; the learned ratio carries over
    orbit_trim_only_ = false;
    orbit_dirty_ = true;  // sent from frame(), coalescing auto-repeat presses
    map_->setOrbit(orbitDraw());
    refreshOrbitPanel();
}

void MainWindow::refreshOrbitPanel() {
    // Hold: shown until hidden (Hold pressed again); Fly To: while its button is on
    const bool show = orbit_ && (guiding_ || !hold_panel_hidden_) && !planning_ &&
                      s_.custom_mode == plane::GUIDED && isAirborne(s_);
    const bool draft = guiding_ && guide_draft_ && !planning_ && isAirborne(s_);
    orbit_panel_->setCoordButton(draft || (show && !orbit_->hold));
    if (draft) {
        orbit_panel_->setValues(QStringLiteral("FLY TO"), guide_draft_->alt, guide_draft_->radius, guide_draft_->ccw,
                                s_.alt_rel);
    } else if (show) {
        orbit_panel_->setValues(orbit_->hold ? QStringLiteral("HOLD") : QStringLiteral("FLY TO"), orbit_->alt,
                                orbit_->radius, orbit_->ccw, s_.alt_rel, orbitDraw().flown, orbitDraw().trimmed);
    } else if (!planning_ && onEndHold()) {
        // circling the end of a one-way route: the same controls, applied to the plan's hold
        const MissionPlan plan = end_hold_plan_ ? *end_hold_plan_ : extractPlan(s_.mission);
        const int h = holdIndex(plan);
        orbit_panel_->setValues(QStringLiteral("HOLD · END"), plan.wps[h].alt_rel,
                                plan.hold_radius > 0.0f ? plan.hold_radius : static_cast<float>(defaultRadius()),
                                plan.hold_ccw, s_.alt_rel);
    } else {
        orbit_panel_->hide();
        return;
    }
    const QSize ds = dock_->sizeHint();
    orbit_panel_->move(orbitPanelX(), height() - ds.height() - 18 - orbit_panel_->height());
    if (!orbit_panel_->isVisible()) {
        orbit_panel_->show();
        orbit_panel_->raise();
        fx_->raise();
    }
}

bool MainWindow::onEndHold() const {
    if (s_.custom_mode != plane::AUTO || !isAirborne(s_) || !s_.mission_known || s_.mission_current < 1 ||
        s_.mission_current >= static_cast<int>(s_.mission.size()) ||
        s_.mission[s_.mission_current].command != mav::kCmdNavLoiterUnlim) {
        return false;
    }
    const int h = holdIndex(extractPlan(s_.mission));
    return h >= 0 && seqToWaypointIndex(s_.mission, s_.mission_current) == h;
}

void MainWindow::adjustEndHold(float d_alt, float d_radius, bool flip) {
    if (!end_hold_plan_) {
        end_hold_plan_ = extractPlan(s_.mission);
    }
    MissionPlan& plan = *end_hold_plan_;
    const int h = holdIndex(plan);
    if (h < 0) {
        end_hold_plan_.reset();
        return;
    }
    auto& alt = plan.wps[h].alt_rel;
    const double floor = altitude::minRelHome(groundsAt(plan.wps[h].lat, plan.wps[h].lon));
    alt = static_cast<float>(std::clamp<double>(alt + d_alt, std::max<double>(kMinOrbitAlt, floor), kMaxOrbitAlt));
    if (d_radius != 0.0f) {
        const float now = plan.hold_radius > 0.0f ? plan.hold_radius : static_cast<float>(defaultRadius());
        plan.hold_radius = std::clamp(now + d_radius, kMinOrbitRadius, kMaxOrbitRadius);
    }
    if (flip) {
        plan.hold_ccw = !plan.hold_ccw;
    }
    end_hold_dirty_ = true;  // sent from frame(), coalescing auto-repeat presses
    refreshOrbitPanel();
}

void MainWindow::sendEndHold(qint64 now_ms) {
    end_hold_dirty_ = false;
    if (!end_hold_plan_ || !onEndHold() || !s_.has_home) {
        end_hold_plan_.reset();  // it left the hold meanwhile: drop the edit
        return;
    }
    const MissionPlan plan = withLanding(*end_hold_plan_);
    // keep the approach on board (same layout, so the same item count)
    const auto on_board = extractApproach(s_.mission);
    const LandingApproach appr = on_board ? *on_board : approachFor(plan, s_.lat, s_.lon);
    auto items = buildLoopMission(s_.home_lat, s_.home_lon, s_.home_alt_amsl, plan, settings_.takeoff_alt, &appr);
    if (items.size() != s_.mission.size()) {
        end_hold_plan_.reset();  // not a mission this GCS laid out; leave it alone
        return;
    }
    end_hold_sent_ms_ = now_ms;
    end_hold_upload_ = true;
    uploading_ = true;
    client_->uploadMission(std::move(items));
}

void MainWindow::onCameraClick(double px, double py, double w, double h) {
    if (!guiding_ || !s_.has_position) {
        return;
    }
    // cast through the camera's actual line of sight (gimbal, or the fixed mount)
    const auto now = Clock::now();
    const geo::Attitude att = predictAttitude(s_, now);
    const CameraLos los = cameraLos(s_, att, settings_.camera_tilt, now, settings_.gimbal_body_frame);
    geo::CameraModel cam;
    cam.hfov_deg = settings_.camera_hfov;
    cam.mount_pitch_deg = 0.0;  // los.earth already includes the mount / gimbal
    cam.width = w;
    cam.height = h;
    const auto t = geo::cameraRayToGround(cam, los.earth, px, py, std::max(1.0, s_.alt_rel));
    double lat = 0.0, lon = 0.0;
    geo::offset(s_.lat, s_.lon, t.north, t.east, lat, lon);
    if (!t.hit_ground) {
        fx_->toast(QStringLiteral("Above the horizon — target %1 m out on bearing %2°")
                       .arg(t.range, 0, 'f', 0)
                       .arg(t.bearing_deg, 0, 'f', 0),
                   kAmber, QStringLiteral("Fly To"));
    }
    sendGuided(lat, lon);
}

int MainWindow::holdSeq(const std::vector<MissionItem>& items) {
    const int h = holdIndex(extractPlan(items));
    for (size_t i = 1; h >= 0 && i < items.size(); ++i) {
        if (items[i].command == mav::kCmdNavLoiterUnlim && seqToWaypointIndex(items, static_cast<int>(i)) == h) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

bool MainWindow::flyingThePlan() const {
    return s_.custom_mode == plane::AUTO && isAirborne(s_) && s_.mission_known && !isLandingInMission(s_);
}

int MainWindow::rejoinSeq(const std::vector<MissionItem>& items) const {
    // the nearest route point, carrying on in the direction it is flying
    const PredictedPosition here = predictPosition(s_, Clock::now());
    const double gs = std::hypot(s_.vn, s_.ve);
    const double track = gs > 3.0 ? geo::wrap360(std::atan2(s_.ve, s_.vn) * geo::kRadToDeg) : -1.0;
    return nearestLoopSeq(items, here.lat, here.lon, defaultRadius(), track);
}

void MainWindow::announceRejoin(const std::vector<MissionItem>& items, int seq, const QString& what) {
    const PredictedPosition here = predictPosition(s_, Clock::now());
    const auto& it = items[seq];
    map_->ping(it.lat(), it.lon(), kGreen);
    fx_->toast(QStringLiteral("%1 → WP%2, %3 m")
                   .arg(what)
                   .arg(seqToWaypointIndex(items, seq) + 1)
                   .arg(std::lround(geo::distance(here.lat, here.lon, it.lat(), it.lon()))),
               kGreen, QStringLiteral("Plan"));
}

void MainWindow::launchOrAuto() {
    setGuide(false);
    climb_out_ = false;  // Resume Plan during the climb-out: straight to the plan
    if (isAirborne(s_)) {
        // Rejoin the loop at the nearest waypoint (not where it left off). This
        // also keeps AUTO off the VTOL takeoff (a Hold during the climb) and
        // the landing items (an aborted landing), which it would otherwise
        // resume on the rotors.
        const int seq = s_.mission_known ? rejoinSeq(s_.mission) : -1;
        if (seq >= 0) {
            client_->runSequence("auto", {link::makeSetMissionCurrent(static_cast<uint16_t>(seq)),
                                          link::makeSetMode(plane::AUTO, "AUTO")});
            announceRejoin(s_.mission, seq, QStringLiteral("Resume Plan"));
        } else {
            client_->setMode(plane::AUTO, "auto");
        }
        dock_->setPending(kLaunch, true);
        return;
    }
    // Always a GUIDED vertical takeoff: at the takeoff altitude the GCS sends a
    // straight climb-out leg (into the wind) for the transition, and only once
    // the aircraft is wing-borne does it turn to the plan (or Hold). An AUTO
    // takeoff would start turning toward WP1 during the transition.
    std::vector<link::CommandRequest> steps;
    if (hasFlyableMission(s_)) {
        // the mission is left on its takeoff item, so a later AUTO doesn't
        // resume a finished landing ("In landing sequence")
        steps.push_back(link::makeSetMissionCurrent(1, true));
    }
    steps.push_back(link::makeSetMode(plane::GUIDED, "GUIDED"));
    steps.push_back(link::makeArm(true));
    steps.push_back(link::makeVtolTakeoff(settings_.takeoff_alt));
    takeoff_handoff_ = true;
    handoff_seen_guided_ = false;
    climb_out_ = false;
    client_->runSequence("launch", std::move(steps));
    dock_->setPending(kLaunch, true);
}

void MainWindow::hold() {
    // instant: no confirmation. Circle here on the wing in GUIDED: the lift
    // rotors are for takeoff and landing only (mid-landing this is a go-around)
    takeoff_handoff_ = false;
    climb_out_ = false;
    land_pending_ = false;
    hold_panel_hidden_ = false;  // a new hold always shows its panel
    setGuide(false);
    if (!s_.has_position) {
        return;
    }
    const PredictedPosition p = predictPosition(s_, Clock::now());
    Orbit o;
    o.lat = p.lat;
    o.lon = p.lon;
    // keep flying at least the altitude it's at; a landing abort climbs back
    // to the approach altitude rather than descending further
    o.alt = static_cast<float>(std::max(30.0, std::round(p.alt_rel)));
    o.radius = orbit_ ? orbit_->radius : static_cast<float>(defaultRadius());
    o.ccw = orbit_ ? orbit_->ccw : false;
    o.hold = true;
    orbit_ = o;
    trim_.setRequested(o.radius);
    sendOrbit();
    dock_->setPending(kHold, true);
}

void MainWindow::land() {
    if (!s_.mission_known || !s_.has_home || extractPlan(s_.mission).wps.empty()) {
        return;
    }
    takeoff_handoff_ = false;
    setGuide(false);
    startLandingApproach();
}

void MainWindow::startLandingApproach() {
    // circle here while the approach is recomputed with the current wind and
    // uploaded; the jump into it happens on MissionUploaded. No L: home.
    const MissionPlan plan = withLanding(extractPlan(s_.mission));
    if (!plan.land || plan.wps.empty()) {
        return;
    }
    hold();
    const LandingApproach appr = approachFor(plan, s_.lat, s_.lon);
    auto items = buildLoopMission(s_.home_lat, s_.home_lon, s_.home_alt_amsl, plan, settings_.takeoff_alt, &appr);
    land_pending_ = true;
    uploading_ = true;
    approach_wind_ = s_.has_wind ? std::optional<double>(s_.wind_from_deg) : std::nullopt;
    client_->uploadMission(std::move(items));
    dock_->setPending(kLand, true);
    const QString where = extractPlan(s_.mission).land ? QStringLiteral("Landing") : QStringLiteral("Landing at home");
    fx_->banner(s_.has_wind && s_.wind_speed >= kMinWindForApproach
                    ? QStringLiteral("%1 · into wind %2°").arg(where).arg(appr.heading_deg, 0, 'f', 0)
                    : QStringLiteral("%1 · approach %2°").arg(where).arg(appr.heading_deg, 0, 'f', 0),
                kGreen);
}

void MainWindow::checkTakeoffHandoff() {
    if (!takeoff_handoff_) {
        return;
    }
    if (s_.custom_mode == plane::GUIDED) {
        handoff_seen_guided_ = true;
    } else if (handoff_seen_guided_) {
        takeoff_handoff_ = false;  // left GUIDED some other way (operator, failsafe)
        return;
    }
    if (handoff_seen_guided_ && s_.armed && s_.has_position && s_.alt_rel >= settings_.takeoff_alt - 3.0) {
        takeoff_handoff_ = false;
        startClimbOut();
    }
}

void MainWindow::startClimbOut() {
    // a straight leg for the transition: into the wind when there is some
    // (shortest, and the hover usually weathervaned into it already), else
    // straight ahead
    const bool wind = s_.has_wind && s_.wind_speed >= kMinWindForApproach;
    const double dir = wind ? s_.wind_from_deg : s_.yaw_deg;
    const double cruise = s_.airspeed_cruise > 0 ? s_.airspeed_cruise : settings_.cruise_speed;
    const double dist = std::max(kClimbOutMin, cruise * kClimbOutSeconds);
    geo::destination(s_.lat, s_.lon, dir, dist, climb_out_lat_, climb_out_lon_);
    client_->runSequence("climb-out", {link::makeReposition(climb_out_lat_, climb_out_lon_, settings_.takeoff_alt,
                                                            static_cast<float>(defaultRadius()), false)});
    climb_out_ = true;
    climb_out_ms_ = clock_.elapsed();
    map_->ping(climb_out_lat_, climb_out_lon_, kCyan);
    fx_->banner(QStringLiteral("Takeoff complete · transition %1 %2°")
                    .arg(wind ? QStringLiteral("into the wind") : QStringLiteral("straight ahead"))
                    .arg(geo::wrap360(dir), 0, 'f', 0),
                kGreen);
}

void MainWindow::checkClimbOut() {
    if (!climb_out_) {
        return;
    }
    if (s_.custom_mode != plane::GUIDED || !s_.armed) {
        climb_out_ = false;  // left GUIDED some other way (operator, failsafe)
        return;
    }
    const bool wingborne = effectiveVtolState(s_) == VtolState::Cruise && s_.has_vfr_hud &&
                           s_.airspeed >= kCruiseAirspeed && liftRotorState(s_, Clock::now()) != LiftRotors::Active;
    const bool timed_out = clock_.elapsed() - climb_out_ms_ > kClimbOutTimeoutMs;
    if (!wingborne && !timed_out) {
        return;
    }
    climb_out_ = false;
    if (timed_out && !wingborne) {
        fx_->toast(QStringLiteral("Transition still not reported after %1 s: continuing")
                       .arg(kClimbOutTimeoutMs / 1000),
                   kAmber, QStringLiteral("Takeoff"));
    }
    // the plan from WP1, or circle here
    const int wp1 = s_.mission_known ? firstRouteSeq(s_.mission) : -1;
    if (wp1 >= 0) {
        client_->runSequence("auto", {link::makeSetMissionCurrent(static_cast<uint16_t>(wp1)),
                                      link::makeSetMode(plane::AUTO, "AUTO")});
        announceRejoin(s_.mission, wp1, QStringLiteral("Transition done · plan"));
    } else {
        hold();
        fx_->banner(QStringLiteral("Transition done · Hold"), kGreen);
    }
}

int MainWindow::firstRouteSeq(const std::vector<MissionItem>& items) {
    for (size_t i = 1; i < items.size(); ++i) {
        if (seqToWaypointIndex(items, static_cast<int>(i)) == 0) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

// ---- input filter ---------------------------------------------------------------

bool MainWindow::eventFilter(QObject* obj, QEvent* e) {
    if ((e->type() == QEvent::MouseButtonPress || e->type() == QEvent::TouchBegin) && isActiveWindow() &&
        !settings_screen_->isOpen() && !(pad_ && pad_->isOpen())) {
        // the alert banner lives on the mouse-transparent fx layer: open the help from here
        QPointF global;
        if (e->type() == QEvent::MouseButtonPress) {
            global = static_cast<QMouseEvent*>(e)->globalPosition();
        } else if (!static_cast<QTouchEvent*>(e)->points().isEmpty()) {
            global = static_cast<QTouchEvent*>(e)->points().front().globalPosition();
        }
        if (!fx_->alertRect().isEmpty() && fx_->alertRect().contains(mapFromGlobal(global))) {
            showAlerts(true);
            return true;
        }
        // the airspeed panel has no close button either: a tap outside closes it
        // (taps on the top bar are left to it: its AS · GS chip toggles the panel)
        if (speed_panel_ && speed_panel_->isVisible()) {
            const QPoint local = mapFromGlobal(global).toPoint();
            if (!speed_panel_->geometry().contains(local) && !bar_->geometry().contains(local)) {
                speed_panel_->hide();
                return !dock_->geometry().contains(local);  // the dock still does what it says
            }
        }
        // the HUD panel: a tap outside closes it, except on the HUD chip (it toggles)
        if (hud_panel_->isVisible()) {
            const QPoint local = mapFromGlobal(global).toPoint();
            const QRectF chip = camera_->hudChipRect().translated(camera_->mapTo(this, QPoint(0, 0)));
            if (!hud_panel_->geometry().contains(local) && !chip.contains(local) && !bar_->geometry().contains(local)) {
                showHudPanel(false);
                return !dock_->geometry().contains(local);
            }
        }
        // the SYSTEMS panel has no close button: a tap outside it closes it (the
        // top bar is left alone, its SYSTEMS chip toggles the panel itself)
        if (alerts_->isVisible() && !preflight_->isOpen()) {
            const QPoint local = mapFromGlobal(global).toPoint();
            if (!alerts_->geometry().contains(local) && !bar_->geometry().contains(local)) {
                showAlerts(false);
                // a tap on the dock still does what it says; on the map/video it only closes
                return !dock_->geometry().contains(local);
            }
        }
    }
    if (e->type() == QEvent::KeyPress || e->type() == QEvent::KeyRelease || e->type() == QEvent::ShortcutOverride) {
        // The keyboard only types into the manual-entry pad; no key triggers a
        // button or action anywhere else (operator decision).
        return !(pad_ && pad_->isOpen());
    }
    return QWidget::eventFilter(obj, e);
}

void MainWindow::autoSafetyOnLanded() {
    // Back on the ground and disarmed (ArduPlane disarms a few seconds after
    // touchdown): safety ON, so the motors can't start by accident. Only after
    // the disarm: while armed it would cut every output, the pusher too.
    const bool disarmed_now = was_armed_ && !s_.armed && s_.has_heartbeat;
    was_armed_ = s_.armed;
    if (disarmed_now && !isAirborne(s_) && safetyState(s_) == SafetyState::Live) {
        client_->runSequence("safety", {link::makeSetSafety(true)});
        fx_->toast(QStringLiteral("On the ground: safety ON, motors cut"), kAmber, QStringLiteral("Safety"));
    }
}

std::optional<double> MainWindow::commandedAltRel() const {
    // what the GCS asked for (height above home), not the autopilot's slewing
    // target, so the bar's "→" stays put during a climb
    if (!s_.armed) {
        return std::nullopt;
    }
    if (takeoff_handoff_ || climb_out_) {
        return settings_.takeoff_alt;
    }
    if (s_.custom_mode == plane::GUIDED && orbit_) {
        return orbit_->alt;
    }
    if (s_.custom_mode == plane::AUTO && s_.mission_known && s_.mission_current >= 1 &&
        s_.mission_current < static_cast<int>(s_.mission.size())) {
        const MissionItem& it = s_.mission[s_.mission_current];
        switch (it.command) {
        case mav::kCmdNavWaypoint:
        case mav::kCmdNavLoiterUnlim:
        case mav::kCmdNavLoiterTurns:
        case mav::kCmdNavLoiterTime:
        case mav::kCmdNavLoiterToAlt:
            return it.z;
        default:
            break;
        }
    }
    return std::nullopt;
}

}  // namespace gcs::ui
