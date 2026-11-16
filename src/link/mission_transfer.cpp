#include "mission_transfer.hpp"

#include <utility>

namespace gcs::link {

MissionTransfer::MissionTransfer(SendFn send, ResultFn result)
    : send_(std::move(send)), result_(std::move(result)) {}

const char* MissionTransfer::ackName(uint8_t type) {
    switch (type) {
    case MAV_MISSION_ACCEPTED: return "accepted";
    case MAV_MISSION_ERROR: return "error";
    case MAV_MISSION_UNSUPPORTED_FRAME: return "unsupported frame";
    case MAV_MISSION_UNSUPPORTED: return "unsupported command";
    case MAV_MISSION_NO_SPACE: return "no space on vehicle";
    case MAV_MISSION_INVALID: return "invalid item";
    case MAV_MISSION_INVALID_PARAM1:
    case MAV_MISSION_INVALID_PARAM2:
    case MAV_MISSION_INVALID_PARAM3:
    case MAV_MISSION_INVALID_PARAM4:
    case MAV_MISSION_INVALID_PARAM5_X:
    case MAV_MISSION_INVALID_PARAM6_Y:
    case MAV_MISSION_INVALID_PARAM7: return "invalid parameter";
    case MAV_MISSION_INVALID_SEQUENCE: return "invalid sequence";
    case MAV_MISSION_DENIED: return "denied";
    case MAV_MISSION_OPERATION_CANCELLED: return "cancelled";
    default: return "rejected";
    }
}

bool MissionTransfer::addressedToUs(uint8_t target_sys) const {
    return target_sys == 0 || target_sys == ids_.own_sys;
}

// ---- start / cancel ----------------------------------------------------------

bool MissionTransfer::startUpload(std::vector<MissionItem> items, Clock::time_point now) {
    if (busy() || items.empty()) {
        return false;
    }
    items_ = std::move(items);
    for (size_t i = 0; i < items_.size(); ++i) {
        items_[i].seq = static_cast<uint16_t>(i);
        items_[i].mission_type = MAV_MISSION_TYPE_MISSION;
    }
    total_ = static_cast<int>(items_.size());
    highest_sent_ = -1;
    state_ = State::Uploading;
    retries_ = 0;
    last_tx_ = now;
    sendCount();

    Result r;
    r.kind = Result::Kind::Progress;
    r.upload = true;
    r.done = 0;
    r.total = total_;
    result_(r);
    return true;
}

bool MissionTransfer::startDownload(Clock::time_point now) {
    if (busy()) {
        return false;
    }
    items_.clear();
    total_ = -1;
    expect_seq_ = 0;
    state_ = State::Downloading;
    retries_ = 0;
    last_tx_ = now;
    sendRequestList();
    return true;
}

void MissionTransfer::cancel(const std::string& reason) {
    if (!busy()) {
        return;
    }
    sendAck(MAV_MISSION_OPERATION_CANCELLED);
    fail(reason);
}

// ---- inbound -------------------------------------------------------------------

bool MissionTransfer::handle(const mavlink_message_t& msg, Clock::time_point now) {
    if (state_ == State::Idle) {
        return false;
    }

    if (state_ == State::Uploading) {
        uint16_t seq = 0;
        uint8_t target = 0;
        uint8_t type = 0;
        bool is_request = false;
        if (msg.msgid == MAVLINK_MSG_ID_MISSION_REQUEST_INT) {
            mavlink_mission_request_int_t req{};
            mavlink_msg_mission_request_int_decode(&msg, &req);
            seq = req.seq;
            target = req.target_system;
            type = req.mission_type;
            is_request = true;
        } else if (msg.msgid == MAVLINK_MSG_ID_MISSION_REQUEST) {
            mavlink_mission_request_t req{};
            mavlink_msg_mission_request_decode(&msg, &req);
            seq = req.seq;
            target = req.target_system;
            type = req.mission_type;
            is_request = true;
        }
        if (is_request) {
            if (!addressedToUs(target) || type != MAV_MISSION_TYPE_MISSION) {
                return false;
            }
            if (seq >= items_.size()) {
                sendAck(MAV_MISSION_INVALID_SEQUENCE);
                fail("vehicle requested item " + std::to_string(seq) + " of " +
                     std::to_string(total_));
                return true;
            }
            last_tx_ = now;
            retries_ = 0;
            sendItem(seq);
            if (static_cast<int>(seq) > highest_sent_) {
                highest_sent_ = seq;
                Result r;
                r.kind = Result::Kind::Progress;
                r.upload = true;
                r.done = highest_sent_ + 1;
                r.total = total_;
                result_(r);
            }
            return true;
        }
        if (msg.msgid == MAVLINK_MSG_ID_MISSION_ACK) {
            mavlink_mission_ack_t ack{};
            mavlink_msg_mission_ack_decode(&msg, &ack);
            if (!addressedToUs(ack.target_system) || ack.mission_type != MAV_MISSION_TYPE_MISSION) {
                return false;
            }
            if (ack.type == MAV_MISSION_ACCEPTED) {
                finish(Result::Kind::UploadDone);
            } else {
                fail(std::string("vehicle rejected mission: ") + ackName(ack.type));
            }
            return true;
        }
        return false;
    }

    // ---- downloading ----
    if (msg.msgid == MAVLINK_MSG_ID_MISSION_COUNT) {
        mavlink_mission_count_t cnt{};
        mavlink_msg_mission_count_decode(&msg, &cnt);
        if (!addressedToUs(cnt.target_system) || cnt.mission_type != MAV_MISSION_TYPE_MISSION) {
            return false;
        }
        if (total_ >= 0) {
            return true;  // duplicate COUNT; already underway
        }
        total_ = cnt.count;
        retries_ = 0;
        last_tx_ = now;
        if (total_ == 0) {
            sendAck(MAV_MISSION_ACCEPTED);
            finish(Result::Kind::DownloadDone);
            return true;
        }
        expect_seq_ = 0;
        sendRequestItem(0);
        Result r;
        r.kind = Result::Kind::Progress;
        r.upload = false;
        r.done = 0;
        r.total = total_;
        result_(r);
        return true;
    }
    if (msg.msgid == MAVLINK_MSG_ID_MISSION_ITEM_INT) {
        mavlink_mission_item_int_t it{};
        mavlink_msg_mission_item_int_decode(&msg, &it);
        if (!addressedToUs(it.target_system) || it.mission_type != MAV_MISSION_TYPE_MISSION ||
            total_ < 0) {
            return false;
        }
        if (it.seq != expect_seq_) {
            return true;  // stale duplicate; the timer will re-request if needed
        }
        MissionItem item;
        item.seq = it.seq;
        item.frame = it.frame;
        item.command = it.command;
        item.current = it.current;
        item.autocontinue = it.autocontinue;
        item.param1 = it.param1;
        item.param2 = it.param2;
        item.param3 = it.param3;
        item.param4 = it.param4;
        item.x = it.x;
        item.y = it.y;
        item.z = it.z;
        items_.push_back(item);
        ++expect_seq_;
        retries_ = 0;
        last_tx_ = now;

        Result r;
        r.kind = Result::Kind::Progress;
        r.upload = false;
        r.done = expect_seq_;
        r.total = total_;
        result_(r);

        if (expect_seq_ >= total_) {
            sendAck(MAV_MISSION_ACCEPTED);
            finish(Result::Kind::DownloadDone);
        } else {
            sendRequestItem(expect_seq_);
        }
        return true;
    }
    if (msg.msgid == MAVLINK_MSG_ID_MISSION_ACK) {
        mavlink_mission_ack_t ack{};
        mavlink_msg_mission_ack_decode(&msg, &ack);
        if (!addressedToUs(ack.target_system) || ack.mission_type != MAV_MISSION_TYPE_MISSION) {
            return false;
        }
        if (ack.type != MAV_MISSION_ACCEPTED) {
            fail(std::string("download refused: ") + ackName(ack.type));
            return true;
        }
        return false;
    }
    return false;
}

void MissionTransfer::tick(Clock::time_point now) {
    if (state_ == State::Idle || pending_ == Pending::None) {
        return;
    }
    if (now - last_tx_ < kTimeout) {
        return;
    }
    if (++retries_ > kMaxRetries) {
        const bool up = state_ == State::Uploading;
        sendAck(MAV_MISSION_OPERATION_CANCELLED);
        fail(up ? "upload timed out (no response from vehicle)"
                : "download timed out (no response from vehicle)");
        return;
    }
    last_tx_ = now;
    switch (pending_) {
    case Pending::Count: sendCount(); break;
    case Pending::Item: sendItem(pending_seq_); break;
    case Pending::RequestList: sendRequestList(); break;
    case Pending::RequestItem: sendRequestItem(pending_seq_); break;
    case Pending::None: break;
    }
}

// ---- outbound --------------------------------------------------------------------

void MissionTransfer::sendCount() {
    mavlink_message_t msg{};
    mavlink_msg_mission_count_pack(ids_.own_sys, ids_.own_comp, &msg, ids_.target_sys,
                                   ids_.target_comp, static_cast<uint16_t>(total_),
                                   MAV_MISSION_TYPE_MISSION, 0);
    pending_ = Pending::Count;
    send_(msg);
}

void MissionTransfer::sendItem(uint16_t seq) {
    const MissionItem& it = items_[seq];
    mavlink_message_t msg{};
    mavlink_msg_mission_item_int_pack(ids_.own_sys, ids_.own_comp, &msg, ids_.target_sys,
                                      ids_.target_comp, it.seq, it.frame, it.command, it.current,
                                      it.autocontinue, it.param1, it.param2, it.param3, it.param4,
                                      it.x, it.y, it.z, MAV_MISSION_TYPE_MISSION);
    pending_ = Pending::Item;
    pending_seq_ = seq;
    send_(msg);
}

void MissionTransfer::sendRequestList() {
    mavlink_message_t msg{};
    mavlink_msg_mission_request_list_pack(ids_.own_sys, ids_.own_comp, &msg, ids_.target_sys,
                                          ids_.target_comp, MAV_MISSION_TYPE_MISSION);
    pending_ = Pending::RequestList;
    send_(msg);
}

void MissionTransfer::sendRequestItem(uint16_t seq) {
    mavlink_message_t msg{};
    mavlink_msg_mission_request_int_pack(ids_.own_sys, ids_.own_comp, &msg, ids_.target_sys,
                                         ids_.target_comp, seq, MAV_MISSION_TYPE_MISSION);
    pending_ = Pending::RequestItem;
    pending_seq_ = seq;
    send_(msg);
}

void MissionTransfer::sendAck(uint8_t type) {
    mavlink_message_t msg{};
    mavlink_msg_mission_ack_pack(ids_.own_sys, ids_.own_comp, &msg, ids_.target_sys,
                                 ids_.target_comp, type, MAV_MISSION_TYPE_MISSION, 0);
    send_(msg);
}

void MissionTransfer::fail(const std::string& why) {
    Result r;
    r.kind = Result::Kind::Failed;
    r.upload = state_ == State::Uploading;
    r.total = total_;
    r.error = why;
    state_ = State::Idle;
    pending_ = Pending::None;
    items_.clear();
    result_(r);
}

void MissionTransfer::finish(Result::Kind kind) {
    Result r;
    r.kind = kind;
    r.upload = kind == Result::Kind::UploadDone;
    r.done = total_;
    r.total = total_;
    r.items = std::move(items_);
    items_.clear();
    state_ = State::Idle;
    pending_ = Pending::None;
    result_(r);
}

}  // namespace gcs::link
