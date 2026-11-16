#include "check.hpp"
#include "core/mission.hpp"
#include "link/mission_transfer.hpp"

#include <chrono>
#include <vector>

using namespace gcs;
using namespace gcs::link;
using Clock = std::chrono::steady_clock;
using std::chrono::milliseconds;

namespace {

constexpr uint8_t kVehSys = 1;
constexpr uint8_t kVehComp = 1;

struct Harness {
    std::vector<mavlink_message_t> sent;
    std::vector<MissionTransfer::Result> results;
    MissionTransfer mt{[this](const mavlink_message_t& m) { sent.push_back(m); },
                       [this](const MissionTransfer::Result& r) { results.push_back(r); }};
    Clock::time_point now = Clock::now();

    Harness() { mt.setIds(LinkIds{}); }

    void request(uint16_t seq, bool legacy = false) {
        mavlink_message_t m{};
        if (legacy) {
            mavlink_msg_mission_request_pack(kVehSys, kVehComp, &m, 255, 190, seq,
                                             MAV_MISSION_TYPE_MISSION);
        } else {
            mavlink_msg_mission_request_int_pack(kVehSys, kVehComp, &m, 255, 190, seq,
                                                 MAV_MISSION_TYPE_MISSION);
        }
        CHECK(mt.handle(m, now));
    }
    void ack(uint8_t type) {
        mavlink_message_t m{};
        mavlink_msg_mission_ack_pack(kVehSys, kVehComp, &m, 255, 190, type, MAV_MISSION_TYPE_MISSION, 0);
        CHECK(mt.handle(m, now));
    }
    void count(uint16_t n) {
        mavlink_message_t m{};
        mavlink_msg_mission_count_pack(kVehSys, kVehComp, &m, 255, 190, n, MAV_MISSION_TYPE_MISSION, 0);
        CHECK(mt.handle(m, now));
    }
    void item(const MissionItem& it) {
        mavlink_message_t m{};
        mavlink_msg_mission_item_int_pack(kVehSys, kVehComp, &m, 255, 190, it.seq, it.frame, it.command,
                                          0, 1, 0, 0, 0, 0, it.x, it.y, it.z, MAV_MISSION_TYPE_MISSION);
        CHECK(mt.handle(m, now));
    }
    const mavlink_message_t& last() const { return sent.back(); }
    const MissionTransfer::Result& lastResult() const { return results.back(); }
};

std::vector<MissionItem> sampleMission() {
    MissionPlan plan;
    plan.wps = {{-35.36, 149.16, 80.0f}, {-35.35, 149.17, 80.0f}};
    plan.land = UserWaypoint{-35.362, 149.166, 30.0f};
    return buildLoopMission(-35.363, 149.165, 584.0, plan, 40.0f);
}

uint16_t itemSeq(const mavlink_message_t& m) {
    mavlink_mission_item_int_t it{};
    mavlink_msg_mission_item_int_decode(&m, &it);
    return it.seq;
}

}  // namespace

static void testHappyUpload() {
    Harness h;
    const auto items = sampleMission();
    CHECK(h.mt.startUpload(items, h.now));
    CHECK(h.mt.busy());
    CHECK(h.last().msgid == MAVLINK_MSG_ID_MISSION_COUNT);
    mavlink_mission_count_t c{};
    mavlink_msg_mission_count_decode(&h.last(), &c);
    CHECK(c.count == items.size());
    CHECK(c.target_system == 1);

    for (uint16_t i = 0; i < items.size(); ++i) {
        h.request(i);
        CHECK(h.last().msgid == MAVLINK_MSG_ID_MISSION_ITEM_INT);
        mavlink_mission_item_int_t it{};
        mavlink_msg_mission_item_int_decode(&h.last(), &it);
        CHECK(it.seq == i);
        CHECK(it.command == items[i].command);
        CHECK(it.x == items[i].x);
    }
    CHECK(h.lastResult().kind == MissionTransfer::Result::Kind::Progress);
    CHECK(h.lastResult().done == static_cast<int>(items.size()));

    // a second start while busy is refused
    CHECK(!h.mt.startUpload(items, h.now));

    h.ack(MAV_MISSION_ACCEPTED);
    CHECK(!h.mt.busy());
    CHECK(h.lastResult().kind == MissionTransfer::Result::Kind::UploadDone);
    CHECK(h.lastResult().items.size() == items.size());
}

static void testDuplicateAndLegacyRequests() {
    Harness h;
    const auto items = sampleMission();
    h.mt.startUpload(items, h.now);
    h.request(0);
    h.request(0);  // duplicate is re-served
    CHECK(itemSeq(h.last()) == 0);
    h.request(1, /*legacy=*/true);  // old-style MISSION_REQUEST still answered with ITEM_INT
    CHECK(h.last().msgid == MAVLINK_MSG_ID_MISSION_ITEM_INT);
    CHECK(itemSeq(h.last()) == 1);
}

static void testTimeoutResend() {
    Harness h;
    const auto items = sampleMission();
    h.mt.startUpload(items, h.now);
    const size_t before = h.sent.size();
    h.now += milliseconds(500);
    h.mt.tick(h.now);
    CHECK(h.sent.size() == before);  // not yet
    h.now += milliseconds(1100);
    h.mt.tick(h.now);
    CHECK(h.sent.size() == before + 1);
    CHECK(h.last().msgid == MAVLINK_MSG_ID_MISSION_COUNT);

    h.request(0);
    h.now += milliseconds(1600);  // request for item 1 got lost
    h.mt.tick(h.now);
    CHECK(h.last().msgid == MAVLINK_MSG_ID_MISSION_ITEM_INT);
    CHECK(itemSeq(h.last()) == 0);  // re-sends the last item

    for (int i = 0; i < MissionTransfer::kMaxRetries + 1; ++i) {
        h.now += milliseconds(1600);
        h.mt.tick(h.now);
    }
    CHECK(!h.mt.busy());
    CHECK(h.lastResult().kind == MissionTransfer::Result::Kind::Failed);
}

static void testNack() {
    Harness h;
    h.mt.startUpload(sampleMission(), h.now);
    h.request(0);
    h.ack(MAV_MISSION_NO_SPACE);
    CHECK(!h.mt.busy());
    CHECK(h.lastResult().kind == MissionTransfer::Result::Kind::Failed);
    CHECK(h.lastResult().error.find("no space") != std::string::npos);
}

static void testOutOfRangeRequest() {
    Harness h;
    const auto items = sampleMission();
    h.mt.startUpload(items, h.now);
    h.request(static_cast<uint16_t>(items.size() + 3));
    CHECK(!h.mt.busy());
    CHECK(h.lastResult().kind == MissionTransfer::Result::Kind::Failed);
}

static void testDownload() {
    Harness h;
    const auto items = sampleMission();
    CHECK(h.mt.startDownload(h.now));
    CHECK(h.last().msgid == MAVLINK_MSG_ID_MISSION_REQUEST_LIST);
    h.count(static_cast<uint16_t>(items.size()));
    for (const auto& it : items) {
        CHECK(h.last().msgid == MAVLINK_MSG_ID_MISSION_REQUEST_INT);
        mavlink_mission_request_int_t r{};
        mavlink_msg_mission_request_int_decode(&h.last(), &r);
        CHECK(r.seq == it.seq);
        h.item(it);
    }
    CHECK(h.last().msgid == MAVLINK_MSG_ID_MISSION_ACK);
    CHECK(!h.mt.busy());
    CHECK(h.lastResult().kind == MissionTransfer::Result::Kind::DownloadDone);
    CHECK(h.lastResult().items.size() == items.size());
    CHECK(h.lastResult().items.back().command == mav::kCmdNavVtolLand);
}

static void testEmptyDownload() {
    Harness h;
    h.mt.startDownload(h.now);
    h.count(0);
    CHECK(!h.mt.busy());
    CHECK(h.lastResult().kind == MissionTransfer::Result::Kind::DownloadDone);
    CHECK(h.lastResult().items.empty());
}

static void testIdleIgnoresAck() {
    Harness h;
    mavlink_message_t m{};
    mavlink_msg_mission_ack_pack(kVehSys, kVehComp, &m, 255, 190, MAV_MISSION_ACCEPTED,
                                 MAV_MISSION_TYPE_MISSION, 0);
    CHECK(!h.mt.handle(m, h.now));  // guided-target acks must fall through
}

int main() {
    testHappyUpload();
    testDuplicateAndLegacyRequests();
    testTimeoutResend();
    testNack();
    testOutOfRangeRequest();
    testDownload();
    testEmptyDownload();
    testIdleIgnoresAck();
    return TEST_MAIN_RESULT();
}
