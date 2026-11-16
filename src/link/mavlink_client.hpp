#pragma once

#include "command_sender.hpp"
#include "core/vehicle_state.hpp"
#include "link_events.hpp"
#include "mission_transfer.hpp"

#include <atomic>
#include <deque>
#include <functional>
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
    // Switches to GUIDED first when needed, then sends MISSION_ITEM_INT current=2.
    void guidedGoto(double lat, double lon, float alt_rel);

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
    void sendGuidedItem();
    void requestStreams(Clock::time_point now);
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

    struct GuidedTarget {
        bool pending = false;     // waiting for MISSION_ACK
        bool armed_send = false;  // waiting for mode change before sending
        double lat = 0.0;
        double lon = 0.0;
        float alt = 0.0f;
        Clock::time_point sent{};
        int attempts = 0;
    } guided_;
};

}  // namespace gcs::link
