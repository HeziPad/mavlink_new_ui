#pragma once

#include "command_sender.hpp"
#include "core/vehicle_state.hpp"
#include "link_events.hpp"
#include "mission_transfer.hpp"

#include <atomic>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <netinet/in.h>

namespace gcs::link {

// Thread-safe MAVLink v2 UDP endpoint.
//
// A single worker thread owns the socket, parser, mission state machine and
// command sender. The UI thread interacts only through:
//   snapshot()   - copy of the latest VehicleState
//   pollEvents() - drain of results / status texts
//   request API  - closures queued to and executed on the worker
class MavlinkClient {
public:
    struct Config {
        std::string bind_host = "127.0.0.1";
        uint16_t port = 14551;
        uint8_t gcs_sysid = 255;
        uint8_t gcs_compid = 190;
    };

    explicit MavlinkClient(Config cfg);
    ~MavlinkClient();

    MavlinkClient(const MavlinkClient&) = delete;
    MavlinkClient& operator=(const MavlinkClient&) = delete;

    void start();
    void stop();

    VehicleState snapshot() const;
    std::vector<LinkEvent> pollEvents();

    // ---- requests (any thread) -------------------------------------------
    void runSequence(const std::string& tag, std::vector<CommandRequest> steps);
    void setMode(uint32_t custom_mode, const std::string& tag);
    void uploadMission(std::vector<MissionItem> items);
    void downloadMission();

    // ---- parameters -----------------------------------------------------------
    // Read these (cached in VehicleState::params; missing ones are retried).
    void fetchParams(std::vector<std::string> names);
    // PARAM_SET, then confirmed by the echoed PARAM_VALUE (retried up to 3x).
    // Reported as LinkEvent::ParamSet.
    void setParam(const std::string& name, float value);

    // ---- stick input (MANUAL_CONTROL) -------------------------------------------
    // Streamed at 10 Hz until `hold_ms` after the last call, then stopped (the
    // vehicle falls back to RC after RC_OVERRIDE_TIME). Ranges: x pitch, y
    // roll, r yaw -1000..1000; z throttle 0..1000.
    void manualControl(int x, int y, int z, int r, int hold_ms = 400);
    void stopManualControl();


private:
    using Task = std::function<void()>;

    void post(Task task);
    void run();
    bool openSocket();
    void handleDatagram(const uint8_t* data, size_t len, const sockaddr_in& from);
    void handleMessage(const mavlink_message_t& msg);
    void decodeTelemetry(const mavlink_message_t& msg);
    void periodic(Clock::time_point now);
    void sendMessage(const mavlink_message_t& msg);
    void requestStreams(Clock::time_point now);
    void requestParams(Clock::time_point now);
    void handleParamValue(const mavlink_message_t& msg);
    void decodeGimbal(const mavlink_message_t& msg);
    void sendParamRequests(Clock::time_point now);
    void sendManual(Clock::time_point now);
    void publish(LinkEvent ev);
    void wake();

    Config cfg_;
    std::atomic<bool> stop_{false};
    std::thread thread_;
    int fd_ = -1;
    int wake_fd_ = -1;

    // shared with UI (mutex_)
    mutable std::mutex mutex_;
    VehicleState shared_;
    std::deque<LinkEvent> events_;
    std::vector<Task> tasks_;

    // worker-owned -------------------------------------------------------
    VehicleState state_;
    bool state_dirty_ = false;
    sockaddr_in peer_{};
    bool peer_known_ = false;
    LinkIds ids_;
    bool vehicle_locked_ = false;
    std::unique_ptr<MissionTransfer> mission_;
    std::unique_ptr<CommandSender> commands_;
    Clock::time_point last_gcs_heartbeat_{};
    Clock::time_point last_stream_request_{};
    Clock::time_point last_home_request_{};
    bool streams_requested_ = false;
    bool initial_download_done_ = false;
    Clock::time_point last_param_request_{};
    int param_rounds_ = 0;

    // parameter service
    struct ParamWant {
        std::string name;
        int tries = 0;
        Clock::time_point last{};
    };
    std::vector<ParamWant> param_wants_;  // reads outstanding
    struct ParamWrite {
        std::string name;
        float value = 0.0f;
        int tries = 0;
        Clock::time_point last{};
    };
    std::vector<ParamWrite> param_writes_;
    std::map<std::string, uint8_t> param_types_;

    // MANUAL_CONTROL stream
    bool manual_active_ = false;
    int manual_x_ = 0, manual_y_ = 0, manual_z_ = 0, manual_r_ = 0;
    Clock::time_point manual_until_{};
    Clock::time_point manual_last_tx_{};

    // message rate measurement (1 s window)
    Clock::time_point rate_t0_{};
    int attitude_count_ = 0;
    int position_count_ = 0;
};

}  // namespace gcs::link
