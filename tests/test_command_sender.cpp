#include "check.hpp"
#include "core/plane_modes.hpp"
#include "link/command_sender.hpp"

#include <chrono>
#include <vector>

using namespace gcs;
using namespace gcs::link;
using Clock = std::chrono::steady_clock;
using std::chrono::milliseconds;

namespace {

struct Harness {
    std::vector<mavlink_message_t> sent;
    std::vector<CommandSender::Result> results;
    CommandSender cs{[this](const mavlink_message_t& m) { sent.push_back(m); },
                     [this](const CommandSender::Result& r) { results.push_back(r); }};
    Clock::time_point now = Clock::now();

    Harness() { cs.setIds(LinkIds{}); }

    mavlink_command_long_t lastCmd() const {
        mavlink_command_long_t c{};
        mavlink_msg_command_long_decode(&sent.back(), &c);
        return c;
    }
    void ack(uint16_t cmd, uint8_t result) {
        mavlink_message_t m{};
        mavlink_msg_command_ack_pack(1, 1, &m, cmd, result, 0, 0, 255, 190);
        cs.handle(m, now);
    }
};

std::vector<CommandRequest> takeoffSequence() {
    return {makeSetMode(plane::GUIDED, "GUIDED"), makeArm(true), makeVtolTakeoff(40.0f)};
}

}  // namespace

static void testSequenceHappy() {
    Harness h;
    h.cs.runSequence("launch", takeoffSequence(), h.now);
    auto c = h.lastCmd();
    CHECK(c.command == MAV_CMD_DO_SET_MODE);
    CHECK_NEAR(c.param1, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 1e-6);
    CHECK_NEAR(c.param2, 15.0, 1e-6);
    CHECK(c.target_system == 1);

    h.ack(MAV_CMD_DO_SET_MODE, MAV_RESULT_ACCEPTED);
    c = h.lastCmd();
    CHECK(c.command == MAV_CMD_COMPONENT_ARM_DISARM);
    CHECK_NEAR(c.param1, 1.0, 1e-6);
    CHECK(!h.results.back().final);

    h.ack(MAV_CMD_COMPONENT_ARM_DISARM, MAV_RESULT_ACCEPTED);
    c = h.lastCmd();
    CHECK(c.command == MAV_CMD_NAV_TAKEOFF);
    CHECK_NEAR(c.param7, 40.0, 1e-6);

    h.ack(MAV_CMD_NAV_TAKEOFF, MAV_RESULT_ACCEPTED);
    CHECK(h.results.back().final);
    CHECK(h.results.back().ok);
    CHECK(h.results.back().tag == "launch");
    CHECK(!h.cs.busy());
}

static void testSequenceAbortsOnDenied() {
    Harness h;
    h.cs.runSequence("launch", takeoffSequence(), h.now);
    h.ack(MAV_CMD_DO_SET_MODE, MAV_RESULT_ACCEPTED);
    const size_t sent = h.sent.size();
    h.ack(MAV_CMD_COMPONENT_ARM_DISARM, MAV_RESULT_FAILED);
    CHECK(h.results.back().final);
    CHECK(!h.results.back().ok);
    CHECK(h.results.back().label == "ARM");
    CHECK(h.sent.size() == sent);  // takeoff never sent
    CHECK(!h.cs.busy());
}

static void testOptionalStepTolerated() {
    Harness h;
    CommandRequest opt;
    opt.command = MAV_CMD_DO_SET_MISSION_CURRENT;
    opt.optional = true;
    h.cs.runSequence("launch", {opt, makeSetMode(plane::AUTO, "AUTO")}, h.now);
    h.ack(MAV_CMD_DO_SET_MISSION_CURRENT, MAV_RESULT_UNSUPPORTED);
    CHECK(h.lastCmd().command == MAV_CMD_DO_SET_MODE);
    CHECK(h.cs.busy());
}

static void testRetryThenTimeout() {
    Harness h;
    h.cs.runSequence("hold", {makeSetMode(plane::QLOITER, "QLOITER")}, h.now);
    CHECK(h.sent.size() == 1);
    CHECK(h.lastCmd().confirmation == 0);
    h.now += milliseconds(1001);
    h.cs.tick(h.now);
    CHECK(h.sent.size() == 2);
    CHECK(h.lastCmd().confirmation == 1);
    h.now += milliseconds(1001);
    h.cs.tick(h.now);
    CHECK(h.sent.size() == 3);
    h.now += milliseconds(1001);
    h.cs.tick(h.now);
    CHECK(h.sent.size() == 3);
    CHECK(h.results.back().final);
    CHECK(!h.results.back().ok);
}

static void testInProgressExtendsTimeout() {
    Harness h;
    h.cs.runSequence("arm", {makeArm(true)}, h.now);
    h.ack(MAV_CMD_COMPONENT_ARM_DISARM, MAV_RESULT_IN_PROGRESS);
    h.now += milliseconds(3000);
    h.cs.tick(h.now);
    CHECK(h.sent.size() == 1);  // no resend while in progress
    CHECK(h.cs.busy());
    h.ack(MAV_CMD_COMPONENT_ARM_DISARM, MAV_RESULT_ACCEPTED);
    CHECK(h.results.back().ok);
}

static void testPreemption() {
    Harness h;
    h.cs.runSequence("launch", takeoffSequence(), h.now);
    h.cs.runSequence("qrtl", {makeSetMode(plane::QRTL, "QRTL")}, h.now);
    CHECK(h.results.size() == 1);
    CHECK(h.results[0].tag == "launch");
    CHECK(h.results[0].cancelled);
    CHECK(h.cs.activeTag() == "qrtl");
    CHECK_NEAR(h.lastCmd().param2, 21.0, 1e-6);
}

static void testBackgroundSilent() {
    Harness h;
    CommandRequest r;
    r.command = MAV_CMD_SET_MESSAGE_INTERVAL;
    r.params[0] = 30;
    h.cs.sendBackground(r, h.now);
    h.ack(MAV_CMD_SET_MESSAGE_INTERVAL, MAV_RESULT_ACCEPTED);
    CHECK(h.results.empty());
    h.now += milliseconds(5000);
    h.cs.tick(h.now);
    CHECK(h.sent.size() == 1);  // acked, so never resent
}

int main() {
    testSequenceHappy();
    testSequenceAbortsOnDenied();
    testOptionalStepTolerated();
    testRetryThenTimeout();
    testInProgressExtendsTimeout();
    testPreemption();
    testBackgroundSilent();
    return TEST_MAIN_RESULT();
}
