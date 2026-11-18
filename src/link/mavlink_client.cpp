#include "mavlink_client.hpp"

#include "core/geo.hpp"
#include "core/plane_modes.hpp"

#include "serial_transport.hpp"
#include "udp_transport.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iterator>
#include <utility>

namespace gcs::link {

namespace {

constexpr size_t kTrailMax = 10000;      // ~40 km: the map's "ALL" trail
constexpr size_t kTrailTrim = 500;       // erase in chunks, not per point
constexpr double kTrailSpacing = 4.0;  // metres

VtolState toVtol(uint8_t v) {
    switch (v) {
    case MAV_VTOL_STATE_MC: return VtolState::Hover;
    case MAV_VTOL_STATE_FW: return VtolState::Cruise;
    case MAV_VTOL_STATE_TRANSITION_TO_FW: return VtolState::TransitionToCruise;
    case MAV_VTOL_STATE_TRANSITION_TO_MC: return VtolState::TransitionToHover;
    default: return VtolState::Unknown;
    }
}

LandedState toLanded(uint8_t v) {
    switch (v) {
    case MAV_LANDED_STATE_ON_GROUND: return LandedState::OnGround;
    case MAV_LANDED_STATE_IN_AIR: return LandedState::InAir;
    case MAV_LANDED_STATE_TAKEOFF: return LandedState::Takeoff;
    case MAV_LANDED_STATE_LANDING: return LandedState::Landing;
    default: return LandedState::Unknown;
    }
}

bool isVehicleHeartbeat(const mavlink_heartbeat_t& hb) {
    if (hb.autopilot == MAV_AUTOPILOT_INVALID) {
        return false;
    }
    switch (hb.type) {
    case MAV_TYPE_GCS:
    case MAV_TYPE_GIMBAL:
    case MAV_TYPE_ADSB:
    case MAV_TYPE_CAMERA:
    case MAV_TYPE_ONBOARD_CONTROLLER:
        return false;
    default:
        return true;
    }
}

std::string modeString(uint32_t mode) {
    const char* name = plane::modeName(mode);
    return name != nullptr ? std::string(name) : "MODE " + std::to_string(mode);
}

CommandRequest messageInterval(uint32_t msg_id, double hz) {
    CommandRequest r;
    r.command = MAV_CMD_SET_MESSAGE_INTERVAL;
    r.params[0] = static_cast<float>(msg_id);
    r.params[1] = static_cast<float>(1.0e6 / hz);
    r.label = "STREAM";
    return r;
}

}  // namespace

MavlinkClient::MavlinkClient(Config cfg) : cfg_(std::move(cfg)) {
    ids_.own_sys = cfg_.gcs_sysid;
    ids_.own_comp = cfg_.gcs_compid;
    transport_ = makeTransport(cfg_);
    state_.endpoint = transport_->describe();
    shared_ = state_;

    mission_ = std::make_unique<MissionTransfer>(
        [this](const mavlink_message_t& m) { sendMessage(m); },
        [this](const MissionTransfer::Result& r) {
            LinkEvent ev;
            ev.upload = r.upload;
            ev.done = r.done;
            ev.total = r.total;
            switch (r.kind) {
            case MissionTransfer::Result::Kind::Progress:
                ev.type = LinkEvent::Type::MissionProgress;
                state_.mission_transfer_active = true;
                break;
            case MissionTransfer::Result::Kind::UploadDone:
            case MissionTransfer::Result::Kind::DownloadDone:
                ev.type = r.kind == MissionTransfer::Result::Kind::UploadDone
                              ? LinkEvent::Type::MissionUploaded
                              : LinkEvent::Type::MissionDownloaded;
                ev.ok = true;
                ev.items = r.items;
                state_.mission = r.items;
                state_.mission_known = true;
                state_.mission_transfer_active = false;
                break;
            case MissionTransfer::Result::Kind::Failed:
                ev.type = LinkEvent::Type::MissionFailed;
                ev.text = r.error;
                state_.mission_transfer_active = false;
                break;
            }
            state_dirty_ = true;
            publish(std::move(ev));
        });

    commands_ = std::make_unique<CommandSender>(
        [this](const mavlink_message_t& m) { sendMessage(m); },
        [this](const CommandSender::Result& r) {
            LinkEvent ev;
            ev.type = r.final ? LinkEvent::Type::CommandResult : LinkEvent::Type::CommandStep;
            ev.tag = r.tag;
            ev.ok = r.ok;
            ev.done = r.step + 1;
            ev.total = r.steps;
            ev.text = r.ok ? r.label : (r.label.empty() ? r.text : r.label + " " + r.text);
            publish(std::move(ev));
        });
}

MavlinkClient::~MavlinkClient() {
    stop();
}

void MavlinkClient::start() {
    if (thread_.joinable()) {
        return;
    }
    stop_.store(false);
    thread_ = std::thread(&MavlinkClient::run, this);
}

void MavlinkClient::stop() {
    stop_.store(true);
    if (thread_.joinable()) {
        thread_.join();
    }
}

VehicleState MavlinkClient::snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return shared_;
}

std::vector<LinkEvent> MavlinkClient::pollEvents() {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<LinkEvent> out(std::make_move_iterator(events_.begin()),
                               std::make_move_iterator(events_.end()));
    events_.clear();
    return out;
}

void MavlinkClient::publish(LinkEvent ev) {
    std::lock_guard<std::mutex> lock(mutex_);
    events_.push_back(std::move(ev));
    while (events_.size() > 256) {
        events_.pop_front();
    }
}

void MavlinkClient::post(Task task) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        tasks_.push_back(std::move(task));
    }
    // picked up within one 20 ms wait of the worker loop
}

std::unique_ptr<Transport> MavlinkClient::makeTransport(const Config& cfg) {
    if (cfg.type == LinkType::Serial) {
        return std::make_unique<SerialTransport>(cfg.serial_port, cfg.serial_baud);
    }
    return std::make_unique<UdpTransport>(cfg.bind_host, cfg.port, cfg.remote_host, cfg.remote_port);
}

void MavlinkClient::reconfigure(Config cfg) {
    post([this, cfg = std::move(cfg)] { applyConfig(cfg); });
}

void MavlinkClient::applyConfig(const Config& cfg) {
    transport_->close();
    const uint8_t sys = cfg_.gcs_sysid;
    const uint8_t comp = cfg_.gcs_compid;
    cfg_ = cfg;
    cfg_.gcs_sysid = sys;
    cfg_.gcs_compid = comp;
    transport_ = makeTransport(cfg_);
    state_.endpoint = transport_->describe();
    state_.socket_open = false;
    state_.peer_known = false;
    state_.error.clear();
    state_dirty_ = true;
    next_open_ = {};  // open it on this loop
    // find the vehicle again on the new link (the same one, usually)
    vehicle_locked_ = false;
    streams_requested_ = false;
    mavlink_reset_channel_status(MAVLINK_COMM_0);
}

// ---- request API ---------------------------------------------------------------

void MavlinkClient::runSequence(const std::string& tag, std::vector<CommandRequest> steps) {
    post([this, tag, steps = std::move(steps)]() mutable {
        if (!vehicle_locked_) {
            LinkEvent ev;
            ev.type = LinkEvent::Type::CommandResult;
            ev.tag = tag;
            ev.ok = false;
            ev.text = "no vehicle link";
            publish(std::move(ev));
            return;
        }
        commands_->runSequence(tag, std::move(steps), Clock::now());
    });
}

void MavlinkClient::setMode(uint32_t custom_mode, const std::string& tag) {
    runSequence(tag, {makeSetMode(custom_mode, modeString(custom_mode))});
}

void MavlinkClient::uploadMission(std::vector<MissionItem> items) {
    post([this, items = std::move(items)]() mutable {
        LinkEvent fail;
        fail.type = LinkEvent::Type::MissionFailed;
        fail.upload = true;
        if (!vehicle_locked_) {
            fail.text = "no vehicle link";
            publish(std::move(fail));
            return;
        }
        if (mission_->busy()) {
            fail.text = "a mission transfer is already running";
            publish(std::move(fail));
            return;
        }
        mission_->startUpload(std::move(items), Clock::now());
    });
}

void MavlinkClient::downloadMission() {
    post([this]() {
        LinkEvent fail;
        fail.type = LinkEvent::Type::MissionFailed;
        fail.upload = false;
        if (!vehicle_locked_) {
            fail.text = "no vehicle link";
            publish(std::move(fail));
            return;
        }
        if (mission_->busy()) {
            fail.text = "a mission transfer is already running";
            publish(std::move(fail));
            return;
        }
        mission_->startDownload(Clock::now());
    });
}

void MavlinkClient::fetchParams(std::vector<std::string> names) {
    post([this, names = std::move(names)] {
        for (const auto& n : names) {
            const bool queued = std::any_of(param_wants_.begin(), param_wants_.end(),
                                            [&](const ParamWant& w) { return w.name == n; });
            if (!queued) {
                param_wants_.push_back({n, 0, {}});
            }
        }
    });
}

void MavlinkClient::setParam(const std::string& name, float value) {
    post([this, name, value] {
        param_writes_.erase(std::remove_if(param_writes_.begin(), param_writes_.end(),
                                           [&](const ParamWrite& w) { return w.name == name; }),
                            param_writes_.end());
        param_writes_.push_back({name, value, 0, {}});
    });
}

void MavlinkClient::manualControl(int x, int y, int z, int r, int hold_ms) {
    post([this, x, y, z, r, hold_ms] {
        manual_x_ = std::clamp(x, -1000, 1000);
        manual_y_ = std::clamp(y, -1000, 1000);
        manual_z_ = std::clamp(z, 0, 1000);
        manual_r_ = std::clamp(r, -1000, 1000);
        manual_until_ = Clock::now() + std::chrono::milliseconds(std::clamp(hold_ms, 100, 5000));
        if (!manual_active_) {
            manual_active_ = true;
            manual_last_tx_ = {};
        }
    });
}

void MavlinkClient::stopManualControl() {
    post([this] {
        if (manual_active_) {
            // one neutral frame (throttle zero) so the vehicle doesn't keep the last input
            manual_x_ = manual_y_ = manual_z_ = manual_r_ = 0;
            sendManual(Clock::now());
        }
        manual_active_ = false;
    });
}

void MavlinkClient::sendManual(Clock::time_point now) {
    mavlink_message_t msg{};
    mavlink_msg_manual_control_pack(ids_.own_sys, ids_.own_comp, &msg, ids_.target_sys,
                                    static_cast<int16_t>(manual_x_), static_cast<int16_t>(manual_y_),
                                    static_cast<int16_t>(manual_z_), static_cast<int16_t>(manual_r_),
                                    0, 0, 0,                 // buttons, buttons2, extensions
                                    0, 0, 0, 0, 0, 0, 0, 0);  // s, t, aux1..6
    sendMessage(msg);
    manual_last_tx_ = now;
}

void MavlinkClient::sendParamRequests(Clock::time_point now) {
    using std::chrono::milliseconds;
    auto idOf = [](const std::string& name, char (&id)[16]) {
        std::memset(id, 0, sizeof(id));
        std::memcpy(id, name.data(), std::min(name.size(), sizeof(id)));
    };
    int sent = 0;  // a few per pass: don't flood a slow radio
    for (auto it = param_wants_.begin(); it != param_wants_.end() && sent < 8;) {
        if (state_.params.count(it->name)) {
            it = param_wants_.erase(it);
            continue;
        }
        if (it->tries >= 4) {
            it = param_wants_.erase(it);  // the vehicle doesn't have it
            continue;
        }
        if (it->last == Clock::time_point{} || now - it->last >= milliseconds(1500)) {
            char id[16];
            idOf(it->name, id);
            mavlink_message_t msg{};
            mavlink_msg_param_request_read_pack(ids_.own_sys, ids_.own_comp, &msg, ids_.target_sys,
                                                ids_.target_comp, id, -1);
            sendMessage(msg);
            it->last = now;
            ++it->tries;
            ++sent;
        }
        ++it;
    }
    for (auto it = param_writes_.begin(); it != param_writes_.end();) {
        if (it->last == Clock::time_point{} || now - it->last >= milliseconds(1500)) {
            if (it->tries >= 3) {
                LinkEvent ev;
                ev.type = LinkEvent::Type::ParamSet;
                ev.tag = it->name;
                ev.ok = false;
                ev.text = "no confirmation from the vehicle";
                publish(std::move(ev));
                it = param_writes_.erase(it);
                continue;
            }
            char id[16];
            idOf(it->name, id);
            const auto t = param_types_.find(it->name);
            mavlink_message_t msg{};
            mavlink_msg_param_set_pack(ids_.own_sys, ids_.own_comp, &msg, ids_.target_sys, ids_.target_comp, id,
                                       it->value,
                                       t != param_types_.end() ? t->second : static_cast<uint8_t>(MAV_PARAM_TYPE_REAL32));
            sendMessage(msg);
            it->last = now;
            ++it->tries;
        }
        ++it;
    }
}

void MavlinkClient::tryOpen(Clock::time_point now) {
    std::string error;
    if (transport_->open(error)) {
        state_.socket_open = true;
        state_.error.clear();
        state_.peer_known = transport_->canSend();
    } else {
        state_.socket_open = false;
        state_.error = error;
        next_open_ = now + std::chrono::seconds(1);  // the port may come back (USB re-plugged, autopilot rebooted)
    }
    state_dirty_ = true;
}

void MavlinkClient::linkFailed(const std::string& error, Clock::time_point now) {
    transport_->close();
    state_.socket_open = false;
    state_.peer_known = false;
    state_.error = error;
    state_dirty_ = true;
    next_open_ = now + std::chrono::seconds(1);
}

void MavlinkClient::run() {
    uint8_t buffer[4096];
    while (!stop_.load()) {
        if (!transport_->isOpen() && Clock::now() >= next_open_) {
            tryOpen(Clock::now());
        }

        int ready = 0;
        if (transport_->isOpen()) {
            std::string error;
            ready = transport_->waitReadable(20, error);
            if (ready < 0) {
                linkFailed(error, Clock::now());
            }
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }

        std::vector<Task> tasks;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            tasks.swap(tasks_);
        }
        for (auto& task : tasks) {
            task();  // may swap the transport (reconfigure)
        }

        if (ready > 0 && transport_->isOpen()) {
            for (;;) {
                std::string error;
                const long n = transport_->read(buffer, sizeof(buffer), error);
                if (n < 0) {
                    linkFailed(error, Clock::now());
                    break;
                }
                if (n == 0) {
                    break;
                }
                handleBytes(buffer, static_cast<size_t>(n));
            }
        }

        periodic(Clock::now());

        if (state_dirty_) {
            state_dirty_ = false;
            std::lock_guard<std::mutex> lock(mutex_);
            shared_ = state_;
        }
    }

    transport_->close();
    std::lock_guard<std::mutex> lock(mutex_);
    state_.socket_open = false;
    shared_ = state_;
}

void MavlinkClient::handleBytes(const uint8_t* data, size_t len) {
    state_.datagrams += 1;
    state_.bytes += len;
    state_dirty_ = true;
    state_.peer_known = transport_->canSend();

    mavlink_message_t msg{};
    mavlink_status_t status{};
    for (size_t i = 0; i < len; ++i) {
        if (mavlink_parse_char(MAVLINK_COMM_0, data[i], &msg, &status)) {
            handleMessage(msg);
        }
    }
}

void MavlinkClient::handleMessage(const mavlink_message_t& msg) {
    const auto now = Clock::now();

    if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT && !vehicle_locked_) {
        mavlink_heartbeat_t hb{};
        mavlink_msg_heartbeat_decode(&msg, &hb);
        if (isVehicleHeartbeat(hb)) {
            if (had_vehicle_ && msg.sysid != ids_.target_sys) {
                // a different aircraft on the new link: forget the old one's plan and params
                state_.params.clear();
                param_types_.clear();
                state_.mission.clear();
                state_.mission_known = false;
                initial_download_done_ = false;
                param_rounds_ = 0;
            }
            had_vehicle_ = true;
            vehicle_locked_ = true;
            ids_.target_sys = msg.sysid;
            ids_.target_comp = msg.compid;
            mission_->setIds(ids_);
            commands_->setIds(ids_);
            LinkEvent ev;
            ev.type = LinkEvent::Type::VehicleConnected;
            ev.done = msg.sysid;
            publish(std::move(ev));
        }
    }
    if (msg.msgid == MAVLINK_MSG_ID_RADIO_STATUS && vehicle_locked_) {
        // injected by the telemetry radios themselves (their own sysid)
        mavlink_radio_status_t r{};
        mavlink_msg_radio_status_decode(&msg, &r);
        state_.has_radio = true;
        state_.radio_rssi = r.rssi;
        state_.radio_remrssi = r.remrssi;
        state_.radio_noise = r.noise;
        state_.radio_remnoise = r.remnoise;
        state_.radio_txbuf = r.txbuf;
        state_.radio_rxerrors = r.rxerrors;
        state_dirty_ = true;
        return;
    }
    if (!vehicle_locked_ || msg.sysid != ids_.target_sys) {
        return;
    }

    state_.messages += 1;
    state_.last_rx = now;

    if (msg.msgid == MAVLINK_MSG_ID_STATUSTEXT) {
        mavlink_statustext_t st{};
        mavlink_msg_statustext_decode(&msg, &st);
        LinkEvent ev;
        ev.type = LinkEvent::Type::StatusText;
        ev.severity = st.severity;
        ev.text.assign(st.text, strnlen(st.text, sizeof(st.text)));
        publish(std::move(ev));
        return;
    }
    if (msg.msgid == MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS) {
        // from the autopilot (its mount driver) or a MAVLink gimbal component
        decodeGimbal(msg);
        return;
    }
    if (msg.compid != ids_.target_comp) {
        return;
    }

    if (msg.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
        // the vehicle asking us something: accelerometer calibration poses
        mavlink_command_long_t c{};
        mavlink_msg_command_long_decode(&msg, &c);
        if (c.command == MAV_CMD_ACCELCAL_VEHICLE_POS) {
            state_.accelcal_pos = static_cast<int>(c.param1);
            state_.accelcal_rx = now;
            state_dirty_ = true;
        }
        return;
    }
    if (mission_->handle(msg, now)) {
        return;
    }
    if (commands_->handle(msg, now)) {
        return;
    }
    decodeTelemetry(msg);
}

void MavlinkClient::decodeTelemetry(const mavlink_message_t& msg) {
    const auto now = Clock::now();
    state_dirty_ = true;
    VehicleState& s = state_;

    switch (msg.msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT: {
        mavlink_heartbeat_t hb{};
        mavlink_msg_heartbeat_decode(&msg, &hb);
        s.has_heartbeat = true;
        s.last_heartbeat = now;
        s.sysid = msg.sysid;
        s.compid = msg.compid;
        s.mav_type = hb.type;
        s.autopilot = hb.autopilot;
        s.base_mode = hb.base_mode;
        s.system_status = hb.system_status;
        s.custom_mode = hb.custom_mode;
        s.armed = (hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
        s.mode_name = modeString(hb.custom_mode);
        break;
    }
    case MAVLINK_MSG_ID_ATTITUDE: {
        mavlink_attitude_t a{};
        mavlink_msg_attitude_decode(&msg, &a);
        s.has_attitude = true;
        s.roll_deg = a.roll * geo::kRadToDeg;
        s.pitch_deg = a.pitch * geo::kRadToDeg;
        s.yaw_deg = geo::wrap360(a.yaw * geo::kRadToDeg);
        bodyRatesToEuler(a.roll, a.pitch, a.rollspeed, a.pitchspeed, a.yawspeed, s.roll_rate, s.pitch_rate,
                         s.yaw_rate);
        s.att_rx = now;
        ++attitude_count_;
        break;
    }
    case MAVLINK_MSG_ID_VFR_HUD: {
        mavlink_vfr_hud_t h{};
        mavlink_msg_vfr_hud_decode(&msg, &h);
        s.has_vfr_hud = true;
        s.airspeed = h.airspeed;
        s.groundspeed = h.groundspeed;
        s.heading_deg = h.heading;
        s.throttle_pct = h.throttle;
        s.climb = h.climb;
        break;
    }
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
        mavlink_global_position_int_t p{};
        mavlink_msg_global_position_int_decode(&msg, &p);
        if (p.lat == 0 && p.lon == 0) {
            break;  // no fix yet
        }
        s.has_position = true;
        s.lat = p.lat / 1.0e7;
        s.lon = p.lon / 1.0e7;
        s.alt_amsl = p.alt / 1000.0;
        s.alt_rel = p.relative_alt / 1000.0;
        s.vn = p.vx / 100.0;
        s.ve = p.vy / 100.0;
        s.vd = p.vz / 100.0;
        s.pos_rx = now;
        ++position_count_;
        if (s.trail.empty() ||
            geo::distance(s.trail.back().lat, s.trail.back().lon, s.lat, s.lon) >= kTrailSpacing) {
            s.trail.push_back({s.lat, s.lon, static_cast<float>(s.alt_rel), now});
            if (s.trail.size() > kTrailMax + kTrailTrim) {
                s.trail.erase(s.trail.begin(), s.trail.begin() + (s.trail.size() - kTrailMax));
            }
        }
        break;
    }
    case MAVLINK_MSG_ID_HOME_POSITION: {
        mavlink_home_position_t h{};
        mavlink_msg_home_position_decode(&msg, &h);
        s.has_home = true;
        s.home_lat = h.latitude / 1.0e7;
        s.home_lon = h.longitude / 1.0e7;
        s.home_alt_amsl = h.altitude / 1000.0;
        break;
    }
    case MAVLINK_MSG_ID_SYS_STATUS: {
        mavlink_sys_status_t st{};
        mavlink_msg_sys_status_decode(&msg, &st);
        s.has_sys_status = true;
        s.sensors_present = st.onboard_control_sensors_present;
        s.sensors_enabled = st.onboard_control_sensors_enabled;
        s.sensors_health = st.onboard_control_sensors_health;
        if (st.voltage_battery != UINT16_MAX && st.voltage_battery > 0) {
            s.has_battery = true;
            s.battery_v = st.voltage_battery / 1000.0;
            s.battery_a = st.current_battery >= 0 ? st.current_battery / 100.0 : -1.0;
            s.battery_pct = st.battery_remaining;
        }
        break;
    }
    case MAVLINK_MSG_ID_BATTERY_STATUS: {
        mavlink_battery_status_t b{};
        mavlink_msg_battery_status_decode(&msg, &b);
        if (b.id != 0) {
            break;
        }
        if (b.battery_remaining >= 0) {
            s.battery_pct = b.battery_remaining;
        }
        if (b.current_battery >= 0) {
            s.battery_a = b.current_battery / 100.0;
        }
        s.battery_consumed_mah = b.current_consumed;
        s.cell_v.clear();
        for (int i = 0; i < 10; ++i) {
            // UINT16_MAX = no cell; ArduPilot puts the whole pack in [0] when it
            // has no per-cell data, so only keep plausible single-cell values
            if (b.voltages[i] != UINT16_MAX && b.voltages[i] > 2000 && b.voltages[i] < 5000) {
                s.cell_v.push_back(b.voltages[i] / 1000.0);
            }
        }
        break;
    }
    case MAVLINK_MSG_ID_DISTANCE_SENSOR: {
        mavlink_distance_sensor_t d{};
        mavlink_msg_distance_sensor_decode(&msg, &d);
        if (d.orientation != 25 && s.has_rangefinder && s.rangefinder_orient == 25) {
            break;  // keep the downward one when there are several
        }
        s.has_rangefinder = true;
        s.rangefinder_m = d.current_distance / 100.0;
        s.rangefinder_min = d.min_distance / 100.0;
        s.rangefinder_max = d.max_distance / 100.0;
        s.rangefinder_quality = d.signal_quality;
        s.rangefinder_orient = d.orientation;
        s.rangefinder_rx = now;
        break;
    }
    case MAVLINK_MSG_ID_GPS_RAW_INT: {
        mavlink_gps_raw_int_t g{};
        mavlink_msg_gps_raw_int_decode(&msg, &g);
        s.has_gps = true;
        s.gps_fix = g.fix_type;
        s.gps_sats = g.satellites_visible == UINT8_MAX ? 0 : g.satellites_visible;
        s.gps_hdop = g.eph == UINT16_MAX ? 99.0 : g.eph / 100.0;
        break;
    }
    case MAVLINK_MSG_ID_EKF_STATUS_REPORT: {
        mavlink_ekf_status_report_t e{};
        mavlink_msg_ekf_status_report_decode(&msg, &e);
        s.has_ekf = true;
        s.ekf_flags = e.flags;
        s.ekf_vel_var = e.velocity_variance;
        s.ekf_pos_var = e.pos_horiz_variance;
        s.ekf_hgt_var = e.pos_vert_variance;
        s.ekf_mag_var = e.compass_variance;
        break;
    }
    case MAVLINK_MSG_ID_EXTENDED_SYS_STATE: {
        mavlink_extended_sys_state_t e{};
        mavlink_msg_extended_sys_state_decode(&msg, &e);
        s.has_ext_state = true;
        s.reported_vtol = toVtol(e.vtol_state);
        s.landed = toLanded(e.landed_state);
        updateCruiseLatch(s);
        updateAssistTimer(s, now);
        break;
    }
    case MAVLINK_MSG_ID_WIND: {
        mavlink_wind_t w{};
        mavlink_msg_wind_decode(&msg, &w);
        // ArduPilot: direction the wind is coming FROM, degrees
        s.has_wind = true;
        s.wind_from_deg = geo::wrap360(w.direction);
        s.wind_speed = w.speed;
        break;
    }
    case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT: {
        mavlink_nav_controller_output_t n{};
        mavlink_msg_nav_controller_output_decode(&msg, &n);
        s.has_nav = true;
        s.nav_alt_error = n.alt_error;
        break;
    }
    case MAVLINK_MSG_ID_PARAM_VALUE:
        handleParamValue(msg);
        break;
    case MAVLINK_MSG_ID_RC_CHANNELS: {
        mavlink_rc_channels_t r{};
        mavlink_msg_rc_channels_decode(&msg, &r);
        s.has_rc = true;
        s.rc_count = r.chancount;
        const uint16_t v[18] = {r.chan1_raw,  r.chan2_raw,  r.chan3_raw,  r.chan4_raw,  r.chan5_raw,  r.chan6_raw,
                                r.chan7_raw,  r.chan8_raw,  r.chan9_raw,  r.chan10_raw, r.chan11_raw, r.chan12_raw,
                                r.chan13_raw, r.chan14_raw, r.chan15_raw, r.chan16_raw, r.chan17_raw, r.chan18_raw};
        for (int i = 0; i < 18; ++i) {
            s.rc[i] = v[i] == UINT16_MAX ? 0 : v[i];
        }
        s.rc_rssi = r.rssi;
        s.rc_rx = now;
        break;
    }
    case MAVLINK_MSG_ID_VIBRATION: {
        mavlink_vibration_t v{};
        mavlink_msg_vibration_decode(&msg, &v);
        s.has_vibe = true;
        s.vibe_x = v.vibration_x;
        s.vibe_y = v.vibration_y;
        s.vibe_z = v.vibration_z;
        s.clip0 = v.clipping_0;
        s.clip1 = v.clipping_1;
        s.clip2 = v.clipping_2;
        break;
    }
    case MAVLINK_MSG_ID_POWER_STATUS: {
        mavlink_power_status_t pw{};
        mavlink_msg_power_status_decode(&msg, &pw);
        s.has_power = true;
        s.board_vcc = pw.Vcc / 1000.0;
        s.servo_vcc = pw.Vservo / 1000.0;
        break;
    }
    case MAVLINK_MSG_ID_SYSTEM_TIME: {
        mavlink_system_time_t t{};
        mavlink_msg_system_time_decode(&msg, &t);
        if (s.time_boot_ms != 0 && t.time_boot_ms + 5000 < s.time_boot_ms) {
            ++s.boot_count;  // the clock went back: it rebooted
        }
        s.time_boot_ms = t.time_boot_ms;
        break;
    }
    case MAVLINK_MSG_ID_AUTOPILOT_VERSION: {
        mavlink_autopilot_version_t a{};
        mavlink_msg_autopilot_version_decode(&msg, &a);
        s.has_version = true;
        s.fw_version = a.flight_sw_version;
        s.board_uid = a.uid;
        break;
    }
    case MAVLINK_MSG_ID_GPS2_RAW: {
        mavlink_gps2_raw_t g{};
        mavlink_msg_gps2_raw_decode(&msg, &g);
        s.has_gps2 = true;
        s.gps2_fix = g.fix_type;
        s.gps2_sats = g.satellites_visible == UINT8_MAX ? 0 : g.satellites_visible;
        s.gps2_hdop = g.eph == UINT16_MAX ? 99.0 : g.eph / 100.0;
        break;
    }
    case MAVLINK_MSG_ID_MAG_CAL_PROGRESS: {
        mavlink_mag_cal_progress_t m{};
        mavlink_msg_mag_cal_progress_decode(&msg, &m);
        if (m.compass_id < s.magcal.size()) {
            auto& c = s.magcal[m.compass_id];
            c.active = true;
            c.status = m.cal_status;
            c.pct = m.completion_pct;
            std::copy(std::begin(m.completion_mask), std::end(m.completion_mask), c.mask.begin());
            c.rx = now;
        }
        break;
    }
    case MAVLINK_MSG_ID_MAG_CAL_REPORT: {
        mavlink_mag_cal_report_t m{};
        mavlink_msg_mag_cal_report_decode(&msg, &m);
        if (m.compass_id < s.magcal.size()) {
            auto& c = s.magcal[m.compass_id];
            c.reported = true;
            c.status = m.cal_status;
            c.fitness = m.fitness;
            c.autosaved = m.autosaved != 0;
            c.active = false;
            c.pct = m.cal_status == MAG_CAL_SUCCESS ? 100 : c.pct;
            c.rx = now;
        }
        break;
    }
    case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW: {
        mavlink_servo_output_raw_t o{};
        mavlink_msg_servo_output_raw_decode(&msg, &o);
        if (o.port != 0) {
            break;  // a second bank would be outputs 17..32; not used
        }
        const uint16_t raw[16] = {o.servo1_raw,  o.servo2_raw,  o.servo3_raw,  o.servo4_raw,
                                  o.servo5_raw,  o.servo6_raw,  o.servo7_raw,  o.servo8_raw,
                                  o.servo9_raw,  o.servo10_raw, o.servo11_raw, o.servo12_raw,
                                  o.servo13_raw, o.servo14_raw, o.servo15_raw, o.servo16_raw};
        std::copy(std::begin(raw), std::end(raw), s.servo_raw.begin());
        s.has_servo_raw = true;
        updateAssistTimer(s, now);
        break;
    }
    case MAVLINK_MSG_ID_MISSION_CURRENT: {
        mavlink_mission_current_t m{};
        mavlink_msg_mission_current_decode(&msg, &m);
        s.mission_current = m.seq;
        break;
    }
    case MAVLINK_MSG_ID_MISSION_ITEM_REACHED: {
        mavlink_mission_item_reached_t m{};
        mavlink_msg_mission_item_reached_decode(&msg, &m);
        s.last_reached = m.seq;
        break;
    }
    default:
        break;
    }
}

void MavlinkClient::requestStreams(Clock::time_point now) {
    // Legacy group request first: works on every ArduPilot version.
    mavlink_message_t msg{};
    mavlink_msg_request_data_stream_pack(ids_.own_sys, ids_.own_comp, &msg, ids_.target_sys,
                                         ids_.target_comp, MAV_DATA_STREAM_ALL, 4, 1);
    sendMessage(msg);

    commands_->sendBackground(messageInterval(MAVLINK_MSG_ID_ATTITUDE, 25.0), now);
    commands_->sendBackground(messageInterval(MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 10.0), now);
    commands_->sendBackground(messageInterval(MAVLINK_MSG_ID_VFR_HUD, 10.0), now);
    commands_->sendBackground(messageInterval(MAVLINK_MSG_ID_SYS_STATUS, 2.0), now);
    commands_->sendBackground(messageInterval(MAVLINK_MSG_ID_GPS_RAW_INT, 2.0), now);
    commands_->sendBackground(messageInterval(MAVLINK_MSG_ID_EXTENDED_SYS_STATE, 2.0), now);
    commands_->sendBackground(messageInterval(MAVLINK_MSG_ID_BATTERY_STATUS, 1.0), now);
    commands_->sendBackground(messageInterval(MAVLINK_MSG_ID_EKF_STATUS_REPORT, 1.0), now);
    commands_->sendBackground(messageInterval(MAVLINK_MSG_ID_MISSION_CURRENT, 1.0), now);
    commands_->sendBackground(messageInterval(MAVLINK_MSG_ID_HOME_POSITION, 0.5), now);
    commands_->sendBackground(messageInterval(MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, 4.0), now);
    commands_->sendBackground(messageInterval(MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT, 4.0), now);
    commands_->sendBackground(messageInterval(MAVLINK_MSG_ID_WIND, 1.0), now);
    commands_->sendBackground(messageInterval(MAVLINK_MSG_ID_DISTANCE_SENSOR, 5.0), now);
    commands_->sendBackground(messageInterval(MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS, 10.0), now);
    // preflight data
    commands_->sendBackground(messageInterval(MAVLINK_MSG_ID_RC_CHANNELS, 5.0), now);
    commands_->sendBackground(messageInterval(MAVLINK_MSG_ID_VIBRATION, 2.0), now);
    commands_->sendBackground(messageInterval(MAVLINK_MSG_ID_POWER_STATUS, 1.0), now);
    commands_->sendBackground(messageInterval(MAVLINK_MSG_ID_SYSTEM_TIME, 1.0), now);
    commands_->sendBackground(messageInterval(MAVLINK_MSG_ID_GPS2_RAW, 1.0), now);
    commands_->sendBackground(messageInterval(MAVLINK_MSG_ID_MAG_CAL_PROGRESS, 5.0), now);
    commands_->sendBackground(messageInterval(MAVLINK_MSG_ID_MAG_CAL_REPORT, 1.0), now);
    if (!state_.has_version) {
        commands_->sendBackground(makeRequestMessage(MAVLINK_MSG_ID_AUTOPILOT_VERSION), now);
    }
    last_stream_request_ = now;
    streams_requested_ = true;
}

// The few parameters the UI needs: Q_GUIDED_MODE (Guide must stay on the
// wing) and SERVOn_FUNCTION (which outputs are lift motors). Missing ones are
// re-requested every 3 s, for a bounded number of rounds.
void MavlinkClient::requestParams(Clock::time_point now) {
    last_param_request_ = now;
    ++param_rounds_;
    auto request = [&](const std::string& name) {
        char id[16] = {};
        std::memcpy(id, name.data(), std::min(name.size(), sizeof(id)));
        mavlink_message_t msg{};
        mavlink_msg_param_request_read_pack(ids_.own_sys, ids_.own_comp, &msg, ids_.target_sys,
                                            ids_.target_comp, id, -1);
        sendMessage(msg);
    };
    if (state_.q_guided_mode < 0) {
        request("Q_GUIDED_MODE");
    }
    if (state_.airspeed_cruise < 0) {
        request("AIRSPEED_CRUISE");
    }
    if (state_.airspeed_min < 0) {
        request("AIRSPEED_MIN");
    }
    if (state_.airspeed_max < 0) {
        request("AIRSPEED_MAX");
    }
    if (state_.q_assist_speed < 0) {
        request("Q_ASSIST_SPEED");
    }
    if (state_.trans_decel < 0) {
        request("Q_TRANS_DECEL");
    }
    if (state_.loiter_radius < 0) {
        request("WP_LOITER_RAD");
    }
    if (state_.batt_capacity_mah < 0) {
        request("BATT_CAPACITY");
    }
    for (int i = 0; i < 16; ++i) {
        if (state_.servo_function[i] == 0) {
            request("SERVO" + std::to_string(i + 1) + "_FUNCTION");
        }
    }
}

void MavlinkClient::handleParamValue(const mavlink_message_t& msg) {
    mavlink_param_value_t p{};
    mavlink_msg_param_value_decode(&msg, &p);
    const std::string name(p.param_id, strnlen(p.param_id, sizeof(p.param_id)));
    const int value = static_cast<int>(std::lround(p.param_value));
    param_types_[name] = p.param_type;
    {
        // generic cache; a change nobody here asked for is counted (preflight validity)
        const auto old = state_.params.find(name);
        bool ours = false;
        for (auto it = param_writes_.begin(); it != param_writes_.end(); ++it) {
            if (it->name == name) {
                ours = true;
                const bool match = std::abs(p.param_value - it->value) <= std::max(1e-4f, std::abs(it->value) * 1e-5f);
                LinkEvent ev;
                ev.type = LinkEvent::Type::ParamSet;
                ev.tag = name;
                ev.ok = match;
                ev.text = std::to_string(p.param_value);
                if (!match) {
                    ev.text = "the vehicle kept " + std::to_string(p.param_value);
                }
                publish(std::move(ev));
                param_writes_.erase(it);
                break;
            }
        }
        if (!ours && old != state_.params.end() && std::abs(old->second - p.param_value) > 1e-6f) {
            ++state_.params_changed;
        }
        state_.params[name] = p.param_value;
        state_dirty_ = true;
    }
    if (name == "Q_GUIDED_MODE") {
        state_.q_guided_mode = value;
        return;
    }
    if (name == "AIRSPEED_CRUISE") {
        state_.airspeed_cruise = p.param_value;
        return;
    }
    if (name == "AIRSPEED_MIN") {
        state_.airspeed_min = p.param_value;
        return;
    }
    if (name == "AIRSPEED_MAX") {
        state_.airspeed_max = p.param_value;
        return;
    }
    if (name == "Q_ASSIST_SPEED") {
        state_.q_assist_speed = p.param_value;
        return;
    }
    if (name == "Q_TRANS_DECEL") {
        state_.trans_decel = p.param_value;
        return;
    }
    if (name == "WP_LOITER_RAD") {
        state_.loiter_radius = std::abs(p.param_value);
        return;
    }
    if (name == "BATT_CAPACITY") {
        state_.batt_capacity_mah = p.param_value;  // also when someone changes it later
        return;
    }
    // SERVOn_FUNCTION, n = 1..16
    if (name.size() > 14 && name.compare(0, 5, "SERVO") == 0 &&
        name.compare(name.size() - 9, 9, "_FUNCTION") == 0) {
        const int n = std::atoi(name.substr(5, name.size() - 14).c_str());
        if (n >= 1 && n <= 16) {
            // 0 doubles as "not read yet", so store "disabled" as -1
            const int stored = value == 0 ? -1 : value;
            if (state_.servo_function[n - 1] == 0) {
                ++state_.servo_functions_known;
            }
            state_.servo_function[n - 1] = stored;
        }
    }
}

void MavlinkClient::decodeGimbal(const mavlink_message_t& msg) {
    mavlink_gimbal_device_attitude_status_t g{};
    mavlink_msg_gimbal_device_attitude_status_decode(&msg, &g);
    const double w = g.q[0], x = g.q[1], y = g.q[2], z = g.q[3];
    if (!std::isfinite(w) || std::abs(w * w + x * x + y * y + z * z - 1.0) > 0.1) {
        return;  // NaN quaternion = attitude unknown
    }
    VehicleState& s = state_;
    s.gimbal_roll = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)) * geo::kRadToDeg;
    s.gimbal_pitch = std::asin(std::clamp(2.0 * (w * y - z * x), -1.0, 1.0)) * geo::kRadToDeg;
    s.gimbal_yaw = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)) * geo::kRadToDeg;
    // Spec: yaw is earth-frame unless YAW_IN_VEHICLE_FRAME (ArduPilot sets it).
    // Older senders without either flag used the vehicle frame unless YAW_LOCK.
    if (g.flags & GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME) {
        s.gimbal_yaw_earth = false;
    } else if (g.flags & GIMBAL_DEVICE_FLAGS_YAW_IN_EARTH_FRAME) {
        s.gimbal_yaw_earth = true;
    } else {
        s.gimbal_yaw_earth = (g.flags & GIMBAL_DEVICE_FLAGS_YAW_LOCK) != 0;
    }
    s.has_gimbal = true;
    s.gimbal_rx = Clock::now();
    state_dirty_ = true;
}

void MavlinkClient::periodic(Clock::time_point now) {
    using std::chrono::milliseconds;
    using std::chrono::seconds;

    if (transport_->canSend() && now - last_gcs_heartbeat_ >= seconds(1)) {
        last_gcs_heartbeat_ = now;
        mavlink_message_t msg{};
        mavlink_msg_heartbeat_pack(ids_.own_sys, ids_.own_comp, &msg, MAV_TYPE_GCS,
                                   MAV_AUTOPILOT_INVALID, 0, 0, MAV_STATE_ACTIVE);
        sendMessage(msg);
    }

    if (!vehicle_locked_) {
        return;
    }

    if (!streams_requested_ ||
        (now - last_stream_request_ >= seconds(10) &&
         (!state_.has_attitude || !state_.has_ext_state || !state_.has_vfr_hud))) {
        requestStreams(now);
    }

    const bool params_missing = state_.q_guided_mode < 0 || state_.servo_functions_known < 16 ||
                                state_.airspeed_cruise < 0 || state_.trans_decel < 0 ||
                                state_.loiter_radius < 0 || state_.batt_capacity_mah < 0 ||
                                state_.airspeed_min < 0 || state_.airspeed_max < 0 || state_.q_assist_speed < 0;

    if (rate_t0_ == Clock::time_point{}) {
        rate_t0_ = now;
    } else if (now - rate_t0_ >= seconds(1)) {
        const double span = std::chrono::duration<double>(now - rate_t0_).count();
        state_.attitude_hz = attitude_count_ / span;
        state_.position_hz = position_count_ / span;
        attitude_count_ = position_count_ = 0;
        rate_t0_ = now;
        state_dirty_ = true;
    }
    if (params_missing && state_.has_heartbeat && param_rounds_ < 10 &&
        now - last_param_request_ >= seconds(3) && now - last_stream_request_ >= milliseconds(500)) {
        requestParams(now);
    }
    // a board with fewer outputs never answers for the missing SERVOn; go with
    // what arrived once the retries are used up
    const bool map_final = state_.servo_functions_known >= 16 || (param_rounds_ >= 10 && now - last_param_request_ >= seconds(3));
    if (map_final != state_.servo_map_final) {
        state_.servo_map_final = map_final;
        state_dirty_ = true;
    }

    if (!state_.has_home && now - last_home_request_ >= seconds(5)) {
        last_home_request_ = now;
        CommandRequest req;
        req.command = MAV_CMD_REQUEST_MESSAGE;
        req.params[0] = static_cast<float>(MAVLINK_MSG_ID_HOME_POSITION);
        req.label = "HOME";
        commands_->sendBackground(req, now);
    }

    if (!initial_download_done_ && state_.has_heartbeat &&
        now - last_stream_request_ >= milliseconds(1500) && !mission_->busy()) {
        initial_download_done_ = true;
        mission_->startDownload(now);
    }

    sendParamRequests(now);
    if (manual_active_) {
        if (now >= manual_until_) {
            manual_active_ = false;  // not refreshed: let go (the vehicle reverts to RC)
        } else if (now - manual_last_tx_ >= milliseconds(100)) {
            sendManual(now);
        }
    }

    mission_->tick(now);
    commands_->tick(now);
}

void MavlinkClient::sendMessage(const mavlink_message_t& msg) {
    if (!transport_->isOpen() || !transport_->canSend()) {
        return;
    }
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    transport_->write(buf, len);
}

}  // namespace gcs::link
