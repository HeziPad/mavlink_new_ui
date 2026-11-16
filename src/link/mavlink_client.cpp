#include "mavlink_client.hpp"

#include "core/geo.hpp"
#include "core/plane_modes.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/eventfd.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cmath>
#include <cstring>
#include <utility>

namespace gcs::link {

namespace {

constexpr size_t kTrailMax = 2000;
constexpr double kTrailSpacing = 2.0;  // metres

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
    state_.endpoint = cfg_.bind_host + ":" + std::to_string(cfg_.port);
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
            emit(std::move(ev));
        });

    commands_ = std::make_unique<CommandSender>(
        [this](const mavlink_message_t& m) { sendMessage(m); },
        [this](const CommandSender::Result& r) {
            if (r.tag == "guide") {
                if (!r.final) {
                    return;
                }
                if (r.ok && guided_.armed_send) {
                    guided_.armed_send = false;
                    sendGuidedItem();
                } else {
                    guided_.armed_send = false;
                    LinkEvent ev;
                    ev.type = LinkEvent::Type::GuidedResult;
                    ev.ok = false;
                    ev.text = r.cancelled ? r.text : "GUIDED mode " + r.text;
                    emit(std::move(ev));
                }
                return;
            }
            LinkEvent ev;
            ev.type = r.final ? LinkEvent::Type::CommandResult : LinkEvent::Type::CommandStep;
            ev.tag = r.tag;
            ev.ok = r.ok;
            ev.done = r.step + 1;
            ev.total = r.steps;
            ev.text = r.ok ? r.label : (r.label.empty() ? r.text : r.label + " " + r.text);
            emit(std::move(ev));
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
    wake();
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

void MavlinkClient::emit(LinkEvent ev) {
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
    wake();
}

void MavlinkClient::wake() {
    if (wake_fd_ >= 0) {
        const uint64_t one = 1;
        [[maybe_unused]] const ssize_t n = ::write(wake_fd_, &one, sizeof(one));
    }
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
            emit(std::move(ev));
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
            emit(std::move(fail));
            return;
        }
        if (mission_->busy()) {
            fail.text = "a mission transfer is already running";
            emit(std::move(fail));
            return;
        }
        guided_.pending = false;
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
            emit(std::move(fail));
            return;
        }
        if (mission_->busy()) {
            fail.text = "a mission transfer is already running";
            emit(std::move(fail));
            return;
        }
        mission_->startDownload(Clock::now());
    });
}

void MavlinkClient::guidedGoto(double lat, double lon, float alt_rel) {
    post([this, lat, lon, alt_rel]() {
        auto reject = [this](const std::string& why) {
            LinkEvent ev;
            ev.type = LinkEvent::Type::GuidedResult;
            ev.ok = false;
            ev.text = why;
            emit(std::move(ev));
        };
        if (!vehicle_locked_) {
            reject("no vehicle link");
            return;
        }
        if (mission_->busy()) {
            reject("mission transfer in progress");
            return;
        }
        guided_.lat = lat;
        guided_.lon = lon;
        guided_.alt = alt_rel;
        guided_.attempts = 0;
        guided_.pending = false;
        if (state_.custom_mode != plane::GUIDED) {
            guided_.armed_send = true;
            commands_->runSequence("guide", {makeSetMode(plane::GUIDED, "GUIDED")}, Clock::now());
        } else {
            guided_.armed_send = false;
            sendGuidedItem();
        }
    });
}

void MavlinkClient::sendGuidedItem() {
    mavlink_message_t msg{};
    mavlink_msg_mission_item_int_pack(ids_.own_sys, ids_.own_comp, &msg, ids_.target_sys,
                                      ids_.target_comp, 0, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                                      MAV_CMD_NAV_WAYPOINT, 2 /* current=2: guided */, 0, 0.0f,
                                      0.0f, 0.0f, 0.0f, toE7(guided_.lat), toE7(guided_.lon),
                                      guided_.alt, MAV_MISSION_TYPE_MISSION);
    guided_.pending = true;
    guided_.sent = Clock::now();
    guided_.attempts += 1;
    sendMessage(msg);
}

// ---- worker ---------------------------------------------------------------------

bool MavlinkClient::openSocket() {
    auto fail = [this](const std::string& what) {
        state_.error = what + ": " + std::strerror(errno);
        state_dirty_ = true;
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
        return false;
    };

    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0) {
        return fail("socket");
    }
    const int reuse = 1;
    ::setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    const int flags = ::fcntl(fd_, F_GETFL, 0);
    if (flags < 0 || ::fcntl(fd_, F_SETFL, flags | O_NONBLOCK) < 0) {
        return fail("fcntl");
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(cfg_.port);
    if (::inet_pton(AF_INET, cfg_.bind_host.c_str(), &addr.sin_addr) != 1) {
        errno = EINVAL;
        return fail("invalid bind address " + cfg_.bind_host);
    }
    if (::bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        return fail("bind " + state_.endpoint);
    }
    state_.socket_open = true;
    state_.error.clear();
    state_dirty_ = true;
    return true;
}

void MavlinkClient::run() {
    wake_fd_ = ::eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
    if (!openSocket()) {
        std::lock_guard<std::mutex> lock(mutex_);
        shared_ = state_;
        return;
    }

    uint8_t buffer[4096];
    while (!stop_.load()) {
        pollfd pfds[2]{};
        pfds[0].fd = fd_;
        pfds[0].events = POLLIN;
        pfds[1].fd = wake_fd_;
        pfds[1].events = POLLIN;
        const int ready = ::poll(pfds, wake_fd_ >= 0 ? 2 : 1, 20);
        if (ready < 0 && errno != EINTR) {
            state_.error = std::string("poll: ") + std::strerror(errno);
            state_dirty_ = true;
            break;
        }

        if (ready > 0 && (pfds[1].revents & POLLIN) != 0) {
            uint64_t drain = 0;
            [[maybe_unused]] const ssize_t n = ::read(wake_fd_, &drain, sizeof(drain));
        }

        std::vector<Task> tasks;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            tasks.swap(tasks_);
        }
        for (auto& task : tasks) {
            task();
        }

        if (ready > 0 && (pfds[0].revents & POLLIN) != 0) {
            for (;;) {
                sockaddr_in from{};
                socklen_t from_len = sizeof(from);
                const ssize_t n = ::recvfrom(fd_, buffer, sizeof(buffer), 0,
                                             reinterpret_cast<sockaddr*>(&from), &from_len);
                if (n < 0) {
                    if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR &&
                        errno != ECONNREFUSED) {
                        state_.error = std::string("recvfrom: ") + std::strerror(errno);
                        state_dirty_ = true;
                    }
                    break;
                }
                if (n == 0) {
                    break;
                }
                handleDatagram(buffer, static_cast<size_t>(n), from);
            }
        }

        periodic(Clock::now());

        if (state_dirty_) {
            state_dirty_ = false;
            std::lock_guard<std::mutex> lock(mutex_);
            shared_ = state_;
        }
    }

    ::close(fd_);
    fd_ = -1;
    if (wake_fd_ >= 0) {
        ::close(wake_fd_);
        wake_fd_ = -1;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    state_.socket_open = false;
    shared_ = state_;
}

void MavlinkClient::handleDatagram(const uint8_t* data, size_t len, const sockaddr_in& from) {
    state_.datagrams += 1;
    state_.bytes += len;
    state_dirty_ = true;
    peer_ = from;
    peer_known_ = true;
    state_.peer_known = true;

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
            vehicle_locked_ = true;
            ids_.target_sys = msg.sysid;
            ids_.target_comp = msg.compid;
            mission_->setIds(ids_);
            commands_->setIds(ids_);
            LinkEvent ev;
            ev.type = LinkEvent::Type::VehicleConnected;
            ev.done = msg.sysid;
            emit(std::move(ev));
        }
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
        emit(std::move(ev));
        return;
    }
    if (msg.compid != ids_.target_comp) {
        return;
    }

    if (mission_->handle(msg, now)) {
        return;
    }
    if (commands_->handle(msg, now)) {
        return;
    }
    if (msg.msgid == MAVLINK_MSG_ID_MISSION_ACK && guided_.pending) {
        mavlink_mission_ack_t ack{};
        mavlink_msg_mission_ack_decode(&msg, &ack);
        guided_.pending = false;
        LinkEvent ev;
        ev.type = LinkEvent::Type::GuidedResult;
        ev.ok = ack.type == MAV_MISSION_ACCEPTED;
        ev.text = ev.ok ? "target accepted"
                        : std::string("target ") + MissionTransfer::ackName(ack.type);
        emit(std::move(ev));
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
        if (s.trail.empty() ||
            geo::distance(s.trail.back().lat, s.trail.back().lon, s.lat, s.lon) >= kTrailSpacing) {
            s.trail.push_back({s.lat, s.lon, static_cast<float>(s.alt_rel)});
            if (s.trail.size() > kTrailMax) {
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
        break;
    }
    case MAVLINK_MSG_ID_EXTENDED_SYS_STATE: {
        mavlink_extended_sys_state_t e{};
        mavlink_msg_extended_sys_state_decode(&msg, &e);
        s.has_ext_state = true;
        s.reported_vtol = toVtol(e.vtol_state);
        s.landed = toLanded(e.landed_state);
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

    commands_->sendBackground(messageInterval(MAVLINK_MSG_ID_ATTITUDE, 20.0), now);
    commands_->sendBackground(messageInterval(MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 10.0), now);
    commands_->sendBackground(messageInterval(MAVLINK_MSG_ID_VFR_HUD, 10.0), now);
    commands_->sendBackground(messageInterval(MAVLINK_MSG_ID_SYS_STATUS, 2.0), now);
    commands_->sendBackground(messageInterval(MAVLINK_MSG_ID_GPS_RAW_INT, 2.0), now);
    commands_->sendBackground(messageInterval(MAVLINK_MSG_ID_EXTENDED_SYS_STATE, 2.0), now);
    commands_->sendBackground(messageInterval(MAVLINK_MSG_ID_BATTERY_STATUS, 1.0), now);
    commands_->sendBackground(messageInterval(MAVLINK_MSG_ID_EKF_STATUS_REPORT, 1.0), now);
    commands_->sendBackground(messageInterval(MAVLINK_MSG_ID_MISSION_CURRENT, 1.0), now);
    commands_->sendBackground(messageInterval(MAVLINK_MSG_ID_HOME_POSITION, 0.5), now);
    last_stream_request_ = now;
    streams_requested_ = true;
}

void MavlinkClient::periodic(Clock::time_point now) {
    using std::chrono::milliseconds;
    using std::chrono::seconds;

    if (peer_known_ && now - last_gcs_heartbeat_ >= seconds(1)) {
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

    if (guided_.pending && now - guided_.sent >= milliseconds(1500)) {
        if (guided_.attempts < 3) {
            sendGuidedItem();
        } else {
            guided_.pending = false;
            LinkEvent ev;
            ev.type = LinkEvent::Type::GuidedResult;
            ev.ok = false;
            ev.text = "no response to guided target";
            emit(std::move(ev));
        }
    }

    mission_->tick(now);
    commands_->tick(now);
}

void MavlinkClient::sendMessage(const mavlink_message_t& msg) {
    if (!peer_known_ || fd_ < 0) {
        return;
    }
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    ::sendto(fd_, buf, len, 0, reinterpret_cast<const sockaddr*>(&peer_), sizeof(peer_));
}

}  // namespace gcs::link
