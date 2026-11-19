#include "payload/payload_manager.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace gcs::payload {

namespace fs = std::filesystem;

const DeviceSnapshot* PayloadSnapshot::find(const std::string& icd_id) const {
    for (const auto& d : devices) {
        if (d.icd && d.icd->id == icd_id) {
            return &d;
        }
    }
    return nullptr;
}

PayloadManager::PayloadManager(SendFn send) : send_(std::move(send)) {}

void PayloadManager::setOwnIds(uint8_t sysid, uint8_t compid) {
    own_sys_ = sysid;
    own_comp_ = compid;
}

std::vector<std::string> PayloadManager::loadDirectory(const std::string& dir) {
    std::vector<std::string> problems;
    std::error_code ec;
    if (!fs::is_directory(dir, ec)) {
        return problems;
    }
    std::vector<fs::path> files;
    for (const auto& e : fs::directory_iterator(dir, ec)) {
        if (e.path().extension() == ".json") {
            files.push_back(e.path());
        }
    }
    std::sort(files.begin(), files.end());
    for (const auto& p : files) {
        std::ifstream in(p, std::ios::binary);
        std::stringstream ss;
        ss << in.rdbuf();
        std::string error;
        if (!addIcd(p.stem().string(), ss.str(), error)) {
            problems.push_back(p.filename().string() + ": " + error);
        }
    }
    return problems;
}

bool PayloadManager::addIcd(const std::string& id, const std::string& json_text, std::string& error) {
    auto icd = std::make_shared<Icd>();
    if (!loadIcd(json_text, *icd, error)) {
        return false;
    }
    if (icd->id.empty()) {
        icd->id = id;
    }
    if (deviceIndex(icd->id) >= 0) {
        error = "duplicate ICD id " + icd->id;
        return false;
    }
    icds_.push_back(icd);
    Device d;
    d.icd = icd;
    d.snap.icd = icd;
    devices_.push_back(std::move(d));
    dirty_ = true;
    return true;
}

int PayloadManager::deviceIndex(const std::string& icd_id) const {
    for (size_t i = 0; i < devices_.size(); ++i) {
        if (devices_[i].icd->id == icd_id) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

bool PayloadManager::handleMessage(const mavlink_message_t& msg, Clock::time_point now) {
    for (size_t di = 0; di < devices_.size(); ++di) {
        Device& d = devices_[di];
        const Icd& icd = *d.icd;
        if (!icd.matches(msg.sysid, msg.compid)) {
            continue;
        }
        if (static_cast<int>(msg.msgid) == icd.report_msg_id && msg.msgid == MAVLINK_MSG_ID_V2_EXTENSION) {
            // the ICD's own layout starts at message_type (the first wire field);
            // msg.len is the received length (MAVLink 2 may have truncated zeros)
            const auto* payload = reinterpret_cast<const uint8_t*>(_MAV_PAYLOAD(&msg));
            DecodedReport r;
            if (!decodeReport(icd, payload, msg.len, r)) {
                return true;
            }
            ReportSlot& slot = d.snap.reports[r.type];
            if (slot.window == Clock::time_point{}) {
                slot.window = now;
            }
            slot.count += 1;
            const double win = std::chrono::duration<double>(now - slot.window).count();
            if (win >= 1.0) {
                slot.hz = slot.count / win;
                slot.count = 0;
                slot.window = now;
            }
            slot.rx = now;
            d.snap.last_rx = now;
            d.snap.alive = true;
            d.snap.sysid = msg.sysid;
            d.snap.compid = msg.compid;
            // a command answered by this report (Config Get -> Parameter)
            if (!queue_.empty() && queue_.front().device == di && queue_.front().sent &&
                queue_.front().def->ack == AckPolicy::Report && queue_.front().def->ack_report == r.type) {
                finish(queue_.front(), r.num("status", 1) != 0, r.def->name);
                queue_.pop_front();
            }
            slot.report = std::move(r);
            dirty_ = true;
            return true;
        }
        if (msg.msgid == MAVLINK_MSG_ID_COMMAND_ACK && !queue_.empty() && queue_.front().device == di &&
            queue_.front().sent) {
            mavlink_command_ack_t ack{};
            mavlink_msg_command_ack_decode(&msg, &ack);
            Pending& p = queue_.front();
            if (ack.command != p.enc.command || p.def->ack != AckPolicy::Ack) {
                continue;
            }
            if (ack.result == MAV_RESULT_IN_PROGRESS) {
                p.last_tx = now;  // still working on it: wait more
                return true;
            }
            static const char* names[] = {"ACCEPTED", "TEMPORARILY_REJECTED", "DENIED", "UNSUPPORTED", "FAILED",
                                          "IN_PROGRESS", "CANCELLED"};
            const std::string text = ack.result < 7 ? names[ack.result] : "result " + std::to_string(ack.result);
            finish(p, ack.result == MAV_RESULT_ACCEPTED, text);
            queue_.pop_front();
            return true;
        }
    }
    return false;
}

void PayloadManager::finish(Pending& p, bool ok, const std::string& text) {
    CommandOutcome o;
    o.icd_id = devices_[p.device].icd->id;
    o.name = p.def->name;
    o.ok = ok;
    o.text = text;
    outcomes_.push_back(std::move(o));
    dirty_ = true;
}

void PayloadManager::transmit(const EncodedCommand& e, int attempt) {
    mavlink_message_t msg{};
    mavlink_msg_command_long_pack(own_sys_, own_comp_, &msg, static_cast<uint8_t>(e.target_system),
                                  static_cast<uint8_t>(e.target_component), static_cast<uint16_t>(e.command),
                                  static_cast<uint8_t>(std::min(attempt, 255)), e.params[0], e.params[1],
                                  e.params[2], e.params[3], e.params[4], e.params[5], e.params[6]);
    send_(msg);
}

bool PayloadManager::send(const std::string& icd_id, const std::string& command, const std::vector<double>& args,
                          Clock::time_point now) {
    const int di = deviceIndex(icd_id);
    if (di < 0) {
        return false;
    }
    const Icd& icd = *devices_[static_cast<size_t>(di)].icd;
    const CommandDef* def = icd.command_def(command);
    if (!def) {
        return false;
    }
    const EncodedCommand enc = encodeCommand(icd, *def, args);
    if (def->ack == AckPolicy::None) {
        transmit(enc, 0);  // fire and forget (Set Gimbal)
        return true;
    }
    Pending p;
    p.device = static_cast<size_t>(di);
    p.def = def;
    p.enc = enc;
    queue_.push_back(p);
    periodic(now);
    return true;
}

bool PayloadManager::stream(const std::string& icd_id, const std::string& command, const std::vector<double>& args,
                            double hz, Clock::time_point now) {
    if (hz <= 0) {
        stopStream(icd_id, command);
        return true;
    }
    const int di = deviceIndex(icd_id);
    if (di < 0) {
        return false;
    }
    const Icd& icd = *devices_[static_cast<size_t>(di)].icd;
    const CommandDef* def = icd.command_def(command);
    if (!def) {
        return false;
    }
    const EncodedCommand enc = encodeCommand(icd, *def, args);
    for (auto& s : streams_) {
        if (s.device == static_cast<size_t>(di) && s.def == def) {
            s.enc = enc;
            s.hz = hz;
            return true;
        }
    }
    Stream s;
    s.device = static_cast<size_t>(di);
    s.def = def;
    s.enc = enc;
    s.hz = hz;
    s.next = now;
    streams_.push_back(s);
    periodic(now);
    return true;
}

void PayloadManager::stopStream(const std::string& icd_id, const std::string& command) {
    const int di = deviceIndex(icd_id);
    streams_.erase(std::remove_if(streams_.begin(), streams_.end(),
                                  [&](const Stream& s) {
                                      return static_cast<int>(s.device) == di && s.def->name == command;
                                  }),
                   streams_.end());
}

void PayloadManager::periodic(Clock::time_point now) {
    // streamed / keep-alive commands: never ACK-tracked (a 20 Hz stream would
    // otherwise drown the queue); a payload answering them is ignored
    for (auto& s : streams_) {
        if (now >= s.next) {
            transmit(s.enc, 0);
            const auto period = std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1.0 / s.hz));
            s.next = (now - s.next < period) ? s.next + period : now + period;
        }
    }
    // reports going quiet
    for (auto& d : devices_) {
        if (d.snap.alive && now - d.snap.last_rx > std::chrono::seconds(3)) {
            d.snap.alive = false;
            dirty_ = true;
        }
    }
    // the ACKed queue
    while (!queue_.empty()) {
        Pending& p = queue_.front();
        const Icd& icd = *devices_[p.device].icd;
        const int timeout_ms = p.def->ack_timeout_ms > 0 ? p.def->ack_timeout_ms : icd.ack_timeout_ms;
        if (!p.sent || now - p.last_tx >= std::chrono::milliseconds(timeout_ms)) {
            if (p.attempts > icd.ack_retries) {
                finish(p, false, "no answer");
                queue_.pop_front();
                continue;
            }
            transmit(p.enc, p.attempts);
            p.attempts += 1;
            p.last_tx = now;
            p.sent = true;
        }
        break;
    }
}

PayloadSnapshot PayloadManager::snapshot() const {
    PayloadSnapshot s;
    for (const auto& d : devices_) {
        s.devices.push_back(d.snap);
    }
    return s;
}

std::vector<CommandOutcome> PayloadManager::takeOutcomes() {
    std::vector<CommandOutcome> out;
    out.swap(outcomes_);
    return out;
}

}  // namespace gcs::payload
