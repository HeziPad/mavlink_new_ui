#include "mavlink_receiver.hpp"

#include <ardupilotmega/mavlink.h>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

namespace {

constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;

double wrap360(double deg) {
    double wrapped = std::fmod(deg, 360.0);
    if (wrapped < 0.0) {
        wrapped += 360.0;
    }
    return wrapped;
}

const char* vehicleTypeName(uint8_t type) {
    switch (type) {
    case MAV_TYPE_GENERIC: return "Generic";
    case MAV_TYPE_FIXED_WING: return "Fixed wing";
    case MAV_TYPE_QUADROTOR: return "Quadrotor";
    case MAV_TYPE_COAXIAL: return "Coaxial";
    case MAV_TYPE_HELICOPTER: return "Helicopter";
    case MAV_TYPE_HEXAROTOR: return "Hexarotor";
    case MAV_TYPE_OCTOROTOR: return "Octorotor";
    case MAV_TYPE_TRICOPTER: return "Tricopter";
    case MAV_TYPE_GROUND_ROVER: return "Ground rover";
    case MAV_TYPE_SUBMARINE: return "Submarine";
    default: return nullptr;
    }
}

const char* autopilotName(uint8_t autopilot) {
    switch (autopilot) {
    case MAV_AUTOPILOT_GENERIC: return "Generic";
    case MAV_AUTOPILOT_ARDUPILOTMEGA: return "ArduPilot";
    case MAV_AUTOPILOT_PX4: return "PX4";
    default: return nullptr;
    }
}

const char* copterModeName(uint32_t mode) {
    switch (mode) {
    case COPTER_MODE_STABILIZE: return "STABILIZE";
    case COPTER_MODE_ACRO: return "ACRO";
    case COPTER_MODE_ALT_HOLD: return "ALT_HOLD";
    case COPTER_MODE_AUTO: return "AUTO";
    case COPTER_MODE_GUIDED: return "GUIDED";
    case COPTER_MODE_LOITER: return "LOITER";
    case COPTER_MODE_RTL: return "RTL";
    case COPTER_MODE_CIRCLE: return "CIRCLE";
    case COPTER_MODE_LAND: return "LAND";
    case COPTER_MODE_DRIFT: return "DRIFT";
    case COPTER_MODE_SPORT: return "SPORT";
    case COPTER_MODE_FLIP: return "FLIP";
    case COPTER_MODE_AUTOTUNE: return "AUTOTUNE";
    case COPTER_MODE_POSHOLD: return "POSHOLD";
    case COPTER_MODE_BRAKE: return "BRAKE";
    case COPTER_MODE_THROW: return "THROW";
    case COPTER_MODE_AVOID_ADSB: return "AVOID_ADSB";
    case COPTER_MODE_GUIDED_NOGPS: return "GUIDED_NOGPS";
    case COPTER_MODE_SMART_RTL: return "SMART_RTL";
    case COPTER_MODE_FLOWHOLD: return "FLOWHOLD";
    case COPTER_MODE_FOLLOW: return "FOLLOW";
    case COPTER_MODE_ZIGZAG: return "ZIGZAG";
    case COPTER_MODE_SYSTEMID: return "SYSTEMID";
    case COPTER_MODE_AUTOROTATE: return "AUTOROTATE";
    case COPTER_MODE_AUTO_RTL: return "AUTO_RTL";
    default: return nullptr;
    }
}

const char* systemStatusName(uint8_t status) {
    switch (status) {
    case MAV_STATE_UNINIT: return "UNINIT";
    case MAV_STATE_BOOT: return "BOOT";
    case MAV_STATE_CALIBRATING: return "CALIBRATING";
    case MAV_STATE_STANDBY: return "STANDBY";
    case MAV_STATE_ACTIVE: return "ACTIVE";
    case MAV_STATE_CRITICAL: return "CRITICAL";
    case MAV_STATE_EMERGENCY: return "EMERGENCY";
    case MAV_STATE_POWEROFF: return "POWEROFF";
    case MAV_STATE_FLIGHT_TERMINATION: return "FLIGHT_TERMINATION";
    default: return nullptr;
    }
}

std::string namedOrNumber(const char* name, unsigned value) {
    if (name != nullptr) {
        return name;
    }
    return std::to_string(value);
}

void applyMessage(TelemetrySnapshot& snap, const mavlink_message_t& msg) {
    const auto now = std::chrono::steady_clock::now();
    snap.last_rx = now;
    snap.messages += 1;
    snap.sysid = msg.sysid;
    snap.compid = msg.compid;

    switch (msg.msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT: {
        mavlink_heartbeat_t hb{};
        mavlink_msg_heartbeat_decode(&msg, &hb);
        snap.has_heartbeat = true;
        snap.heartbeats += 1;
        snap.last_heartbeat = now;
        snap.vehicle_type = namedOrNumber(vehicleTypeName(hb.type), hb.type);
        snap.autopilot = namedOrNumber(autopilotName(hb.autopilot), hb.autopilot);
        snap.custom_mode = hb.custom_mode;
        snap.base_mode = hb.base_mode;
        snap.armed = (hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
        snap.mavlink_version = hb.mavlink_version;
        snap.system_status = namedOrNumber(systemStatusName(hb.system_status), hb.system_status);
        if (hb.autopilot == MAV_AUTOPILOT_ARDUPILOTMEGA &&
            (hb.type == MAV_TYPE_QUADROTOR || hb.type == MAV_TYPE_HEXAROTOR ||
             hb.type == MAV_TYPE_OCTOROTOR || hb.type == MAV_TYPE_HELICOPTER ||
             hb.type == MAV_TYPE_TRICOPTER || hb.type == MAV_TYPE_COAXIAL)) {
            snap.flight_mode = namedOrNumber(copterModeName(hb.custom_mode), hb.custom_mode);
        } else {
            snap.flight_mode = std::to_string(hb.custom_mode);
        }
        break;
    }
    case MAVLINK_MSG_ID_ATTITUDE: {
        mavlink_attitude_t att{};
        mavlink_msg_attitude_decode(&msg, &att);
        snap.has_attitude = true;
        snap.roll_deg = att.roll * kRadToDeg;
        snap.pitch_deg = att.pitch * kRadToDeg;
        snap.yaw_deg = wrap360(att.yaw * kRadToDeg);
        snap.roll_rate_dps = att.rollspeed * kRadToDeg;
        snap.pitch_rate_dps = att.pitchspeed * kRadToDeg;
        snap.yaw_rate_dps = att.yawspeed * kRadToDeg;
        break;
    }
    case MAVLINK_MSG_ID_VFR_HUD: {
        mavlink_vfr_hud_t hud{};
        mavlink_msg_vfr_hud_decode(&msg, &hud);
        snap.has_vfr_hud = true;
        snap.airspeed_mps = hud.airspeed;
        snap.groundspeed_mps = hud.groundspeed;
        snap.heading_deg = hud.heading;
        snap.throttle_pct = hud.throttle;
        snap.hud_alt_m = hud.alt;
        snap.climb_mps = hud.climb;
        break;
    }
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
        mavlink_global_position_int_t pos{};
        mavlink_msg_global_position_int_decode(&msg, &pos);
        snap.has_global_position = true;
        snap.latitude_deg = static_cast<double>(pos.lat) / 1.0e7;
        snap.longitude_deg = static_cast<double>(pos.lon) / 1.0e7;
        snap.alt_amsl_m = static_cast<double>(pos.alt) / 1000.0;
        snap.alt_rel_m = static_cast<double>(pos.relative_alt) / 1000.0;
        snap.vn_mps = static_cast<double>(pos.vx) / 100.0;
        snap.ve_mps = static_cast<double>(pos.vy) / 100.0;
        snap.vd_mps = static_cast<double>(pos.vz) / 100.0;
        snap.heading_cdeg = pos.hdg;
        snap.heading_valid = pos.hdg != UINT16_MAX;
        break;
    }
    default:
        break;
    }
}

}  // namespace

MavlinkReceiver::MavlinkReceiver(std::string host, uint16_t port)
    : host_(std::move(host)), port_(port) {
    snap_.endpoint = host_ + ":" + std::to_string(port_);
}

MavlinkReceiver::~MavlinkReceiver() {
    stop();
}

void MavlinkReceiver::start() {
    if (thread_.joinable()) {
        return;
    }
    stop_.store(false);
    thread_ = std::thread(&MavlinkReceiver::run, this);
}

void MavlinkReceiver::stop() {
    stop_.store(true);
    if (thread_.joinable()) {
        thread_.join();
    }
}

TelemetrySnapshot MavlinkReceiver::snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return snap_;
}

void MavlinkReceiver::run() {
    const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        std::lock_guard<std::mutex> lock(mutex_);
        snap_.error = std::string("socket: ") + std::strerror(errno);
        return;
    }

    const int reuse = 1;
    ::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    const int flags = ::fcntl(fd, F_GETFL, 0);
    if (flags < 0 || ::fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) {
        std::lock_guard<std::mutex> lock(mutex_);
        snap_.error = std::string("fcntl: ") + std::strerror(errno);
        ::close(fd);
        return;
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port_);
    if (::inet_pton(AF_INET, host_.c_str(), &addr.sin_addr) != 1) {
        std::lock_guard<std::mutex> lock(mutex_);
        snap_.error = "invalid bind address " + host_;
        ::close(fd);
        return;
    }

    if (::bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        std::lock_guard<std::mutex> lock(mutex_);
        snap_.error = std::string("bind ") + snap_.endpoint + ": " + std::strerror(errno);
        ::close(fd);
        return;
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        snap_.socket_open = true;
        snap_.error.clear();
    }

    mavlink_message_t message{};
    mavlink_status_t status{};
    uint8_t buffer[2048];
    auto last_log = std::chrono::steady_clock::now();

    while (!stop_.load()) {
        pollfd pfd{};
        pfd.fd = fd;
        pfd.events = POLLIN;
        const int ready = ::poll(&pfd, 1, 50);
        if (ready < 0) {
            if (errno == EINTR) {
                continue;
            }
            std::lock_guard<std::mutex> lock(mutex_);
            snap_.error = std::string("poll: ") + std::strerror(errno);
            break;
        }
        if (ready == 0 || (pfd.revents & POLLIN) == 0) {
            continue;
        }

        for (;;) {
            const ssize_t n = ::recvfrom(fd, buffer, sizeof(buffer), 0, nullptr, nullptr);
            if (n < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
                    break;
                }
                std::lock_guard<std::mutex> lock(mutex_);
                snap_.error = std::string("recvfrom: ") + std::strerror(errno);
                stop_.store(true);
                break;
            }
            if (n == 0) {
                break;
            }

            TelemetrySnapshot local;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                snap_.datagrams += 1;
                snap_.bytes += static_cast<uint64_t>(n);
                local = snap_;
            }

            for (ssize_t i = 0; i < n; ++i) {
                if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &message, &status)) {
                    applyMessage(local, message);
                }
            }

            {
                std::lock_guard<std::mutex> lock(mutex_);
                const uint64_t datagrams = snap_.datagrams;
                const uint64_t bytes = snap_.bytes;
                const bool socket_open = snap_.socket_open;
                const std::string endpoint = snap_.endpoint;
                const std::string error = snap_.error;
                snap_ = std::move(local);
                snap_.datagrams = datagrams;
                snap_.bytes = bytes;
                snap_.socket_open = socket_open;
                snap_.endpoint = endpoint;
                if (snap_.error.empty()) {
                    snap_.error = error;
                }
            }
        }
    }

    ::close(fd);
    std::lock_guard<std::mutex> lock(mutex_);
    snap_.socket_open = false;
}
