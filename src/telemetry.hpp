#pragma once

#include <chrono>
#include <cstdint>
#include <string>

struct TelemetrySnapshot {
    bool has_heartbeat = false;
    bool has_attitude = false;
    bool has_vfr_hud = false;
    bool has_global_position = false;

    uint8_t sysid = 0;
    uint8_t compid = 0;
    std::string vehicle_type;
    std::string autopilot;
    std::string flight_mode;
    std::string system_status;
    bool armed = false;
    uint32_t custom_mode = 0;
    uint8_t base_mode = 0;
    uint8_t mavlink_version = 0;

    double roll_deg = 0.0;
    double pitch_deg = 0.0;
    double yaw_deg = 0.0;
    double roll_rate_dps = 0.0;
    double pitch_rate_dps = 0.0;
    double yaw_rate_dps = 0.0;

    double airspeed_mps = 0.0;
    double groundspeed_mps = 0.0;
    int heading_deg = 0;
    int throttle_pct = 0;
    double hud_alt_m = 0.0;
    double climb_mps = 0.0;

    double latitude_deg = 0.0;
    double longitude_deg = 0.0;
    double alt_amsl_m = 0.0;
    double alt_rel_m = 0.0;
    double vn_mps = 0.0;
    double ve_mps = 0.0;
    double vd_mps = 0.0;
    bool heading_valid = false;
    int heading_cdeg = 0;

    uint64_t datagrams = 0;
    uint64_t messages = 0;
    uint64_t heartbeats = 0;
    uint64_t bytes = 0;

    std::chrono::steady_clock::time_point last_rx{};
    std::chrono::steady_clock::time_point last_heartbeat{};
    bool socket_open = false;
    std::string endpoint = "127.0.0.1:14551";
    std::string error;
};
