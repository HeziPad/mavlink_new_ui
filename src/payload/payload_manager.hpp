#pragma once

#include "link/mavlink_headers.hpp"
#include "payload/icd.hpp"
#include "payload/icd_codec.hpp"

#include <chrono>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace gcs::payload {

using Clock = std::chrono::steady_clock;

// Latest of one report type, with its measured rate.
struct ReportSlot {
    DecodedReport report;
    Clock::time_point rx{};
    int count = 0;            // since the last rate window
    double hz = 0.0;
    Clock::time_point window{};
};

// What the UI sees of one payload (copied out of the link thread).
struct DeviceSnapshot {
    std::shared_ptr<const Icd> icd;
    bool alive = false;                 // a report within the last 3 s
    int sysid = -1, compid = -1;        // where its reports come from
    std::map<int, ReportSlot> reports;  // by report type
    Clock::time_point last_rx{};
};

struct PayloadSnapshot {
    std::vector<DeviceSnapshot> devices;
    const DeviceSnapshot* find(const std::string& icd_id) const;
};

// Result of one command, for the UI log / toasts.
struct CommandOutcome {
    std::string icd_id;
    std::string name;
    bool ok = false;
    std::string text;   // "ACCEPTED", "DENIED", "no answer"...
};

// Runs every loaded payload ICD on the MAVLink link (worker thread only):
// decodes the payloads' reports, sends their commands with the ICD's ACK
// policy (retries, fire-and-forget, answered-by-report), repeats keep-alive /
// streamed commands. Its command queue is separate from the vehicle's
// CommandSender, so payload commands never pre-empt flight actions.
class PayloadManager {
public:
    using SendFn = std::function<void(const mavlink_message_t&)>;

    explicit PayloadManager(SendFn send);

    void setOwnIds(uint8_t sysid, uint8_t compid);
    // icds/*.json in `dir`; problems are returned (one line per file)
    std::vector<std::string> loadDirectory(const std::string& dir);
    bool addIcd(const std::string& id, const std::string& json_text, std::string& error);
    const std::vector<std::shared_ptr<const Icd>>& icds() const { return icds_; }

    // true: the message belonged to a payload and was consumed
    bool handleMessage(const mavlink_message_t& msg, Clock::time_point now);
    void periodic(Clock::time_point now);

    // queue a command (args in the ICD's params order); false: unknown
    bool send(const std::string& icd_id, const std::string& command, const std::vector<double>& args,
              Clock::time_point now);
    // repeat a command at hz until stopped (Set Gimbal joystick, illuminator
    // safety); restarting with new args replaces it. hz <= 0 stops it.
    bool stream(const std::string& icd_id, const std::string& command, const std::vector<double>& args, double hz,
                Clock::time_point now);
    void stopStream(const std::string& icd_id, const std::string& command);

    PayloadSnapshot snapshot() const;
    std::vector<CommandOutcome> takeOutcomes();
    bool dirty() const { return dirty_; }
    void clearDirty() { dirty_ = false; }

private:
    struct Device {
        std::shared_ptr<const Icd> icd;
        DeviceSnapshot snap;
    };
    struct Pending {
        size_t device = 0;
        const CommandDef* def = nullptr;
        EncodedCommand enc;
        int attempts = 0;
        Clock::time_point last_tx{};
        bool sent = false;
    };
    struct Stream {
        size_t device = 0;
        const CommandDef* def = nullptr;
        EncodedCommand enc;
        double hz = 0.0;
        Clock::time_point next{};
    };

    void transmit(const EncodedCommand& e, int attempt);
    void finish(Pending& p, bool ok, const std::string& text);
    int deviceIndex(const std::string& icd_id) const;

    SendFn send_;
    uint8_t own_sys_ = 255, own_comp_ = 190;
    std::vector<std::shared_ptr<const Icd>> icds_;
    std::vector<Device> devices_;
    std::deque<Pending> queue_;          // ACKed commands, one in flight at a time
    std::vector<Stream> streams_;
    std::vector<CommandOutcome> outcomes_;
    bool dirty_ = false;
};

}  // namespace gcs::payload
