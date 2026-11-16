#include "command_sender.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace gcs::link {

CommandRequest makeSetMode(uint32_t custom_mode, const std::string& label) {
    CommandRequest r;
    r.command = MAV_CMD_DO_SET_MODE;
    r.params[0] = static_cast<float>(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED);
    r.params[1] = static_cast<float>(custom_mode);
    r.label = label;
    return r;
}

CommandRequest makeArm(bool arm) {
    CommandRequest r;
    r.command = MAV_CMD_COMPONENT_ARM_DISARM;
    r.params[0] = arm ? 1.0f : 0.0f;
    r.label = arm ? "ARM" : "DISARM";
    return r;
}

CommandRequest makeVtolTakeoff(float alt_rel) {
    // ArduPlane runs a GUIDED quadplane takeoff from MAV_CMD_NAV_TAKEOFF
    // (quadplane.do_user_takeoff); NAV_VTOL_TAKEOFF (84) is only a mission item
    // and comes back UNSUPPORTED as a command. Needs GUIDED + armed.
    CommandRequest r;
    r.command = MAV_CMD_NAV_TAKEOFF;
    r.params[6] = alt_rel;
    r.label = "TAKEOFF";
    return r;
}

CommandRequest makeSetMissionCurrent(uint16_t seq) {
    CommandRequest r;
    r.command = MAV_CMD_DO_SET_MISSION_CURRENT;
    r.params[0] = static_cast<float>(seq);
    r.label = "MISSION " + std::to_string(seq);
    return r;
}

CommandRequest makeReposition(double lat, double lon, float alt_rel, float radius_m, bool ccw) {
    CommandRequest r;
    r.command = MAV_CMD_DO_REPOSITION;
    r.as_int = true;
    r.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
    r.params[0] = -1.0f;                                                    // ground speed: default
    r.params[1] = static_cast<float>(MAV_DO_REPOSITION_FLAGS_CHANGE_MODE);  // -> GUIDED
    r.params[2] = radius_m;                                                 // loiter radius (planes)
    r.params[3] = ccw ? 1.0f : 0.0f;                                        // ArduPlane: 0 = clockwise
    r.x = static_cast<int32_t>(std::llround(lat * 1.0e7));
    r.y = static_cast<int32_t>(std::llround(lon * 1.0e7));
    r.z = alt_rel;
    r.label = "ORBIT";
    return r;
}

CommandSender::CommandSender(SendFn send, ResultFn result)
    : send_(std::move(send)), result_(std::move(result)) {}

const char* CommandSender::resultName(uint8_t mav_result) {
    switch (mav_result) {
    case MAV_RESULT_ACCEPTED: return "accepted";
    case MAV_RESULT_TEMPORARILY_REJECTED: return "temporarily rejected";
    case MAV_RESULT_DENIED: return "denied";
    case MAV_RESULT_UNSUPPORTED: return "unsupported";
    case MAV_RESULT_FAILED: return "failed";
    case MAV_RESULT_IN_PROGRESS: return "in progress";
    case MAV_RESULT_CANCELLED: return "cancelled";
    default: return "rejected";
    }
}

void CommandSender::transmit(Pending& p, Clock::time_point now) {
    mavlink_message_t msg{};
    const auto& q = p.req.params;
    if (p.req.as_int) {
        mavlink_msg_command_int_pack(ids_.own_sys, ids_.own_comp, &msg, ids_.target_sys, ids_.target_comp,
                                     p.req.frame, p.req.command, 0, 0, q[0], q[1], q[2], q[3], p.req.x, p.req.y,
                                     p.req.z);
    } else {
        mavlink_msg_command_long_pack(ids_.own_sys, ids_.own_comp, &msg, ids_.target_sys,
                                      ids_.target_comp, p.req.command,
                                      static_cast<uint8_t>(std::min(p.attempts, 255)), q[0], q[1], q[2],
                                      q[3], q[4], q[5], q[6]);
    }
    p.attempts += 1;
    p.last_tx = now;
    send_(msg);
}

void CommandSender::runSequence(const std::string& tag, std::vector<CommandRequest> steps,
                                Clock::time_point now) {
    if (active_) {
        cancel("superseded by " + tag);
    }
    if (steps.empty()) {
        return;
    }
    tag_ = tag;
    steps_ = std::move(steps);
    index_ = 0;
    active_ = true;
    startStep(now);
}

void CommandSender::startStep(Clock::time_point now) {
    current_ = Pending{};
    current_.req = steps_[index_];
    transmit(current_, now);
}

void CommandSender::sendBackground(const CommandRequest& req, Clock::time_point now) {
    // replace an identical in-flight request rather than stacking duplicates
    background_.erase(std::remove_if(background_.begin(), background_.end(),
                                     [&](const Pending& p) {
                                         return p.req.command == req.command &&
                                                p.req.params == req.params;
                                     }),
                      background_.end());
    Pending p;
    p.req = req;
    transmit(p, now);
    background_.push_back(p);
}

void CommandSender::cancel(const std::string& reason) {
    if (!active_) {
        return;
    }
    Result r;
    r.tag = tag_;
    r.label = current_.req.label;
    r.step = static_cast<int>(index_);
    r.steps = static_cast<int>(steps_.size());
    r.command = current_.req.command;
    r.final = true;
    r.ok = false;
    r.cancelled = true;
    r.text = reason;
    active_ = false;
    tag_.clear();
    steps_.clear();
    result_(r);
}

void CommandSender::stepFinished(uint8_t mav_result, const std::string& text, Clock::time_point now) {
    const bool accepted = mav_result == MAV_RESULT_ACCEPTED;
    const bool tolerated = !accepted && current_.req.optional;
    const bool last = index_ + 1 >= steps_.size();

    Result r;
    r.tag = tag_;
    r.label = current_.req.label;
    r.step = static_cast<int>(index_);
    r.steps = static_cast<int>(steps_.size());
    r.command = current_.req.command;
    r.mav_result = mav_result;
    r.text = text;

    if ((accepted || tolerated) && !last) {
        r.ok = true;
        r.final = false;
        result_(r);
        ++index_;
        startStep(now);
        return;
    }

    r.final = true;
    r.ok = accepted || tolerated;
    active_ = false;
    tag_.clear();
    steps_.clear();
    result_(r);
}

bool CommandSender::handle(const mavlink_message_t& msg, Clock::time_point now) {
    if (msg.msgid != MAVLINK_MSG_ID_COMMAND_ACK) {
        return false;
    }
    mavlink_command_ack_t ack{};
    mavlink_msg_command_ack_decode(&msg, &ack);
    if (ack.target_system != 0 && ack.target_system != ids_.own_sys) {
        return false;
    }

    if (active_ && ack.command == current_.req.command) {
        if (ack.result == MAV_RESULT_IN_PROGRESS) {
            current_.in_progress = true;
            current_.last_tx = now;
            return true;
        }
        stepFinished(ack.result, ack.result == MAV_RESULT_ACCEPTED ? "" : resultName(ack.result), now);
        return true;
    }

    for (auto it = background_.begin(); it != background_.end(); ++it) {
        if (it->req.command == ack.command) {
            if (ack.result == MAV_RESULT_IN_PROGRESS) {
                it->in_progress = true;
                it->last_tx = now;
            } else {
                background_.erase(it);
            }
            return true;
        }
    }
    return false;
}

void CommandSender::tick(Clock::time_point now) {
    if (active_) {
        const auto limit = current_.in_progress ? kInProgressTimeout : kRetryInterval;
        if (now - current_.last_tx >= limit) {
            if (current_.in_progress || current_.attempts >= kMaxAttempts) {
                stepFinished(MAV_RESULT_FAILED, "no response from vehicle", now);
            } else {
                transmit(current_, now);
            }
        }
    }

    for (auto it = background_.begin(); it != background_.end();) {
        const auto limit = it->in_progress ? kInProgressTimeout : kRetryInterval;
        if (now - it->last_tx >= limit) {
            if (it->in_progress || it->attempts >= kMaxAttempts) {
                it = background_.erase(it);
                continue;
            }
            transmit(*it, now);
        }
        ++it;
    }
}

}  // namespace gcs::link
