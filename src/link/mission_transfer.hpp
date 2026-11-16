#pragma once

#include "core/mission.hpp"
#include "mavlink_headers.hpp"

#include <chrono>
#include <functional>
#include <string>
#include <vector>

namespace gcs::link {

// MAVLink mission micro-service, as a socket-free state machine.
//
// Upload:   MISSION_COUNT -> (MISSION_REQUEST_INT | MISSION_REQUEST)* -> MISSION_ITEM_INT* -> MISSION_ACK
// Download: MISSION_REQUEST_LIST -> MISSION_COUNT -> MISSION_REQUEST_INT* / MISSION_ITEM_INT* -> MISSION_ACK
//
// Feed every inbound message to handle(); call tick() periodically to drive
// timeouts and retransmissions.
class MissionTransfer {
public:
    using Clock = std::chrono::steady_clock;
    using SendFn = std::function<void(const mavlink_message_t&)>;

    enum class State { Idle, Uploading, Downloading };

    struct Result {
        enum class Kind { Progress, UploadDone, DownloadDone, Failed };
        Kind kind = Kind::Progress;
        bool upload = true;
        int done = 0;
        int total = 0;
        std::string error;
        std::vector<MissionItem> items;  // DownloadDone / UploadDone
    };
    using ResultFn = std::function<void(const Result&)>;

    static constexpr auto kTimeout = std::chrono::milliseconds(1500);
    static constexpr int kMaxRetries = 5;

    MissionTransfer(SendFn send, ResultFn result);

    void setIds(const LinkIds& ids) { ids_ = ids; }
    State state() const { return state_; }
    bool busy() const { return state_ != State::Idle; }

    bool startUpload(std::vector<MissionItem> items, Clock::time_point now);
    bool startDownload(Clock::time_point now);
    void cancel(const std::string& reason);

    // Returns true when the message belonged to an active transfer.
    bool handle(const mavlink_message_t& msg, Clock::time_point now);
    void tick(Clock::time_point now);

    static const char* ackName(uint8_t type);

private:
    enum class Pending { None, Count, Item, RequestList, RequestItem };

    void sendCount();
    void sendItem(uint16_t seq);
    void sendRequestList();
    void sendRequestItem(uint16_t seq);
    void sendAck(uint8_t type);
    void fail(const std::string& why);
    void finish(Result::Kind kind);
    bool addressedToUs(uint8_t target_sys) const;

    SendFn send_;
    ResultFn result_;
    LinkIds ids_;
    State state_ = State::Idle;

    std::vector<MissionItem> items_;
    int total_ = 0;
    int highest_sent_ = -1;     // upload: highest seq served
    uint16_t expect_seq_ = 0;   // download: next seq wanted

    Pending pending_ = Pending::None;
    uint16_t pending_seq_ = 0;
    Clock::time_point last_tx_{};
    int retries_ = 0;
};

}  // namespace gcs::link
