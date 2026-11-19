// Payload ICD engine: JSON reader, ICD loading, report decode / command encode
// against byte fixtures from the Python TRIP simulator, the NextVision adapter,
// and the PayloadManager's ACK / retry / stream policies.

#include "check.hpp"
#include "payload/adapter.hpp"
#include "payload/icd.hpp"
#include "payload/icd_codec.hpp"
#include "payload/json.hpp"
#include "payload/payload_manager.hpp"

#include <array>
#include <cstring>
#include <fstream>
#include <sstream>
#include <vector>

#include "payload_fixtures.inc"

using namespace gcs;
using namespace gcs::payload;

namespace {

std::string readFile(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    std::stringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

const std::string kIcdPath = std::string(GCS_SOURCE_DIR) + "/icds/nextvision_trip.json";

Icd loadNv() {
    Icd icd;
    std::string error;
    const bool ok = loadIcd(readFile(kIcdPath), icd, error);
    if (!ok) {
        std::fprintf(stderr, "ICD load failed: %s\n", error.c_str());
    }
    CHECK(ok);
    return icd;
}

void testJson() {
    Json j;
    std::string err;
    CHECK(parseJson(R"({"a": [1, -2.5e1, "xé\n"], "b": {"c": true, "d": null}})", j, err));
    CHECK(j.get("a")->array.size() == 3);
    CHECK_NEAR(j.get("a")->array[1].number, -25.0, 1e-12);
    CHECK(j.get("a")->array[2].string == "x\xc3\xa9\n");
    CHECK(j.get("b")->flag("c"));
    CHECK(j.get("b")->get("d")->isNull());
    CHECK(!parseJson("{\"a\": 1,\n \"b\" 2}", j, err));
    CHECK(err.find("line 2") == 0);
    CHECK(!parseJson("[1, 2", j, err));
}

void testLoad() {
    const Icd icd = loadNv();
    CHECK(icd.id == "nextvision_trip");
    CHECK(icd.reports.size() == 27);
    CHECK(icd.commands.size() > 100);
    CHECK(icd.report(0)->header.size == 76);
    CHECK(icd.report("IMU")->header.size == 78);
    CHECK(icd.report(25)->records.size == 32);
    CHECK(icd.matches(1, 100) && !icd.matches(1, 1));
    CHECK(icd.enumLabel("SystemModeReport", 9) == "GlobalPosition");
    CHECK(icd.enumLabel("@config_params", 29) == "ReportComponentID");
    CHECK(icd.enumValue("ZoomCmd", "in") == 1.0);
    CHECK(icd.command_def("SetGimbal")->ack == AckPolicy::None);
    CHECK(icd.command_def("Config.Get")->ack == AckPolicy::Report);
    CHECK(icd.command_def("Config.Get")->ack_report == 15);
    CHECK(icd.command_def("Illuminator.Safety")->periodic_hz == 10.0);

    // a declared length that disagrees with the fields is refused
    std::string text = readFile(kIcdPath), err;
    const auto at = text.find("\"length\": 78");
    CHECK(at != std::string::npos);
    text.replace(at, 12, "\"length\": 77");
    Icd bad;
    CHECK(!loadIcd(text, bad, err));
    CHECK(err.find("System") != std::string::npos);
}

void testDecode() {
    const Icd icd = loadNv();
    DecodedReport r;
    CHECK(decodeReport(icd, kFix_system, sizeof kFix_system, r));
    CHECK(r.def->name == "System");
    CHECK_NEAR(r.num("roll"), 12.5, 1e-6);
    CHECK_NEAR(r.num("pitch"), -30.25, 1e-6);
    CHECK(r.num("system_mode") == 9);
    CHECK(r.num("tracker_roi_x") == 700);
    CHECK(r.num("utc") == 1700000000123.0);
    CHECK_NEAR(r.num("tracker_quality"), 0.75, 1e-6);
    const FieldDef& rec = r.def->header.fields[static_cast<size_t>(r.def->header.index("recording_status"))];
    CHECK(formatValue(icd, rec, r.fields[static_cast<size_t>(r.def->header.index("recording_status"))]) ==
          "ch0 Recording · ch1 Disabled");
    const int bi = r.def->header.index("bit_status");
    CHECK(formatValue(icd, r.def->header.fields[static_cast<size_t>(bi)], r.fields[static_cast<size_t>(bi)]) ==
          "InternalComms|CameraCommsError");
    const int mi = r.def->header.index("system_mode");
    CHECK(formatValue(icd, r.def->header.fields[static_cast<size_t>(mi)], r.fields[static_cast<size_t>(mi)]) ==
          "9 GlobalPosition");
    // re-encoding gives the same bytes
    std::vector<std::pair<std::string, double>> vals;
    for (size_t k = 0; k < r.fields.size(); ++k) {
        vals.emplace_back(r.def->header.fields[k].name, r.fields[k].num());
    }
    const auto again = encodeReport(icd, *r.def, vals);
    CHECK(again.size() == sizeof kFix_system);
    CHECK(std::memcmp(again.data(), kFix_system, sizeof kFix_system) == 0);

    CHECK(decodeReport(icd, kFix_los, sizeof kFix_los, r));
    const FieldDef& ur = r.def->header.fields[static_cast<size_t>(r.def->header.index("ur_lat"))];
    CHECK(formatValue(icd, ur, r.fields[static_cast<size_t>(r.def->header.index("ur_lat"))]) == "--");

    CHECK(decodeReport(icd, kFix_param, sizeof kFix_param, r));
    CHECK(r.num("param_id") == 29 && r.num("value") == 154.0 && r.num("status") == 1);

    CHECK(decodeReport(icd, kFix_snapshot, sizeof kFix_snapshot, r));
    CHECK(r.text("file_name") == "IMG_0001.jpg");
    CHECK(r.num("channel") == 1);

    // variable records, also after MAVLink 2 strips the trailing zeros
    CHECK(decodeReport(icd, kFix_odv2, sizeof kFix_odv2, r));
    CHECK(r.records.size() == 2);
    CHECK(r.records[0][0].num() == 21);
    CHECK_NEAR(r.records[0][2].num(), 0.25, 1e-7);
    size_t n = sizeof kFix_odv2;
    while (n > 0 && kFix_odv2[n - 1] == 0) {
        --n;
    }
    CHECK(n < sizeof kFix_odv2);
    CHECK(decodeReport(icd, kFix_odv2, n, r));
    CHECK(r.records.size() == 2);
    // extra bytes from a newer firmware are ignored
    std::vector<uint8_t> longer(kFix_gndcrs, kFix_gndcrs + sizeof kFix_gndcrs);
    longer.insert(longer.end(), {1, 2, 3, 4});
    CHECK(decodeReport(icd, longer.data(), longer.size(), r));
    CHECK_NEAR(r.num("lat"), 32.0005, 1e-5);
    // unknown type / too short
    const uint8_t unknown[] = {99, 0, 1, 2};
    CHECK(!decodeReport(icd, unknown, sizeof unknown, r));
    CHECK(!decodeReport(icd, unknown, 1, r));
}

void testCommands() {
    const Icd icd = loadNv();
    for (const auto& f : kCmdFixtures) {
        const CommandDef* def = icd.command_def(f.name);
        CHECK(def != nullptr);
        if (!def) {
            continue;
        }
        const EncodedCommand e = encodeCommand(icd, *def, f.args);
        CHECK(e.command == f.command);
        for (size_t k = 0; k < 7; ++k) {
            CHECK_NEAR(e.params[k], f.params[k], 1e-6);
        }
        CHECK(e.target_system == 1 && e.target_component == 0);
        CHECK(matchCommand(icd, e.command, e.params) == def);
    }
}

void testAdapter() {
    const Icd icd = loadNv();
    auto ad = makeAdapter("nextvision_trip");
    CHECK(ad != nullptr);
    CHECK(makeAdapter("something_else") == nullptr);
    DeviceSnapshot dev;
    dev.icd = std::make_shared<Icd>(icd);
    PayloadView v;
    CHECK(!ad->view(dev, v));
    for (auto [buf, len] : {std::pair<const uint8_t*, size_t>{kFix_system, sizeof kFix_system},
                            {kFix_los, sizeof kFix_los}, {kFix_gndcrs, sizeof kFix_gndcrs}}) {
        DecodedReport r;
        CHECK(decodeReport(icd, buf, len, r));
        dev.reports[r.type].report = r;
    }
    CHECK(ad->view(dev, v));
    CHECK(v.camera);
    CHECK_NEAR(v.earth.yaw_deg, 90.0, 1e-3);    // the fixture looks east,
    CHECK_NEAR(v.earth.pitch_deg, -45.0, 1e-3); // 45 deg down,
    CHECK_NEAR(v.earth.roll_deg, 0.0, 1e-3);    // image level
    CHECK_NEAR(v.hfov_deg, 20.0, 1e-6);
    CHECK(v.mode == "GlobalPosition");
    CHECK(v.sensor == "IR");
    CHECK(v.recording);
    CHECK(v.corners[0].has_value() && !v.corners[1].has_value());
    CHECK(v.ground.has_value());
    CHECK_NEAR(v.slant_range, 141.4, 1e-3);
    const auto t = ad->trackAt(0.5, 0.5, 0.1, 0.2);
    CHECK(t && t->name == "SmartTracker.TrackBBox");
    CHECK_NEAR(t->args[1], 0.45, 1e-9);
    CHECK_NEAR(t->args[2], 0.4, 1e-9);
    const auto j = ad->joystick(0.3, -1, 1);
    CHECK(j && j->name == "SetGimbal" && j->args[2] == 1);
    CHECK(!ad->quickActions(dev).empty());
    for (const auto& qa : ad->quickActions(dev)) {
        CHECK(icd.command_def(qa.second.name) != nullptr);
    }
}

// ---- PayloadManager ------------------------------------------------------------------

mavlink_message_t reportMsg(const uint8_t* p, size_t len, uint8_t sysid = 1, uint8_t compid = 100) {
    // pack as V2_EXTENSION (message_type, target_network, target_system, target_component, payload)
    uint8_t rest[249] = {};
    std::memcpy(rest, p + 5, len > 5 ? len - 5 : 0);
    mavlink_message_t m{};
    mavlink_msg_v2_extension_pack(sysid, compid, &m, p[2], p[3], p[4], static_cast<uint16_t>(p[0] | (p[1] << 8)), rest);
    // round trip through the wire so msg.len is what a receiver sees (truncated)
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    const uint16_t n = mavlink_msg_to_send_buffer(buf, &m);
    mavlink_message_t out{};
    mavlink_status_t st{};
    for (uint16_t k = 0; k < n; ++k) {
        mavlink_parse_char(MAVLINK_COMM_2, buf[k], &out, &st);
    }
    return out;
}

mavlink_message_t ackMsg(uint16_t cmd, uint8_t result, uint8_t compid = 100) {
    mavlink_message_t m{};
    mavlink_msg_command_ack_pack(1, compid, &m, cmd, result, 0, 0, 255, 190);
    return m;
}

void testManager() {
    std::vector<mavlink_command_long_t> sent;
    PayloadManager pm([&](const mavlink_message_t& m) {
        if (m.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
            mavlink_command_long_t c{};
            mavlink_msg_command_long_decode(&m, &c);
            sent.push_back(c);
        }
    });
    std::string err;
    CHECK(pm.addIcd("nextvision_trip", readFile(kIcdPath), err));
    CHECK(!pm.addIcd("nextvision_trip", readFile(kIcdPath), err));  // duplicate id
    auto t = Clock::now();

    // reports: decoded, only from the ICD's ids
    CHECK(pm.handleMessage(reportMsg(kFix_system, sizeof kFix_system), t));
    CHECK(!pm.handleMessage(reportMsg(kFix_system, sizeof kFix_system, 1, 1), t));  // autopilot compid
    CHECK(pm.handleMessage(reportMsg(kFix_odv2, sizeof kFix_odv2), t));
    PayloadSnapshot s = pm.snapshot();
    const DeviceSnapshot* d = s.find("nextvision_trip");
    CHECK(d && d->alive && d->compid == 100);
    CHECK(d && d->reports.at(0).report.num("system_mode") == 9);
    CHECK(d && d->reports.at(25).report.records.size() == 2);

    // an ACKed command: sent once, retried after the timeout, accepted
    CHECK(pm.send("nextvision_trip", "SetSystemMode", {3}, t));
    CHECK(!pm.send("nextvision_trip", "NoSuchCommand", {}, t));
    CHECK(sent.size() == 1 && sent[0].command == 203 && sent[0].param1 == 0 && sent[0].param2 == 3);
    CHECK(sent[0].target_system == 1 && sent[0].target_component == 0);
    pm.periodic(t + std::chrono::milliseconds(1000));
    CHECK(sent.size() == 1);
    pm.periodic(t + std::chrono::milliseconds(2100));
    CHECK(sent.size() == 2 && sent[1].confirmation == 1);
    CHECK(!pm.handleMessage(ackMsg(203, MAV_RESULT_ACCEPTED, 1), t));  // the autopilot's ACK isn't ours
    CHECK(pm.handleMessage(ackMsg(203, MAV_RESULT_ACCEPTED), t));
    auto out = pm.takeOutcomes();
    CHECK(out.size() == 1 && out[0].ok && out[0].name == "SetSystemMode" && out[0].text == "ACCEPTED");

    // queued one at a time; DENIED reported
    t += std::chrono::seconds(10);
    sent.clear();
    pm.send("nextvision_trip", "TakeSnapshot", {0}, t);
    pm.send("nextvision_trip", "SetSensor", {1}, t);
    CHECK(sent.size() == 1);
    pm.handleMessage(ackMsg(203, MAV_RESULT_DENIED), t);
    pm.periodic(t);
    CHECK(sent.size() == 2 && sent[1].param1 == 3);
    pm.handleMessage(ackMsg(203, MAV_RESULT_ACCEPTED), t);
    out = pm.takeOutcomes();
    CHECK(out.size() == 2 && !out[0].ok && out[0].text == "DENIED" && out[1].ok);

    // no answer after the retries
    sent.clear();
    pm.send("nextvision_trip", "DoNUC", {}, t);
    for (int k = 1; k <= 5; ++k) {
        pm.periodic(t + std::chrono::milliseconds(2100 * k));
    }
    CHECK(sent.size() == 4);  // first try + 3 retries
    out = pm.takeOutcomes();
    CHECK(out.size() == 1 && !out[0].ok && out[0].text == "no answer");

    // fire-and-forget and answered-by-report
    t += std::chrono::seconds(60);
    sent.clear();
    pm.send("nextvision_trip", "SetGimbal", {0.5, 0, 3, 0}, t);
    CHECK(sent.size() == 1 && sent[0].param1 == 6);
    pm.send("nextvision_trip", "Config.Get", {29}, t);
    CHECK(sent.size() == 2);
    CHECK(pm.takeOutcomes().empty());
    CHECK(!pm.handleMessage(ackMsg(203, MAV_RESULT_ACCEPTED, 1), t));
    pm.handleMessage(reportMsg(kFix_param, sizeof kFix_param), t);
    out = pm.takeOutcomes();
    CHECK(out.size() == 1 && out[0].ok && out[0].name == "Config.Get");

    // a stream: 20 Hz for one second, then stopped
    sent.clear();
    pm.stream("nextvision_trip", "SetGimbal", {0.2, 0.1, 0, 0}, 20.0, t);
    for (int ms = 0; ms <= 1000; ms += 10) {
        pm.periodic(t + std::chrono::milliseconds(ms));
    }
    CHECK(sent.size() >= 20 && sent.size() <= 22);
    pm.stopStream("nextvision_trip", "SetGimbal");
    const size_t n = sent.size();
    pm.periodic(t + std::chrono::milliseconds(1500));
    CHECK(sent.size() == n);

    // reports going quiet
    pm.periodic(t + std::chrono::seconds(200));
    CHECK(!pm.snapshot().find("nextvision_trip")->alive);
}

void testLoadDirectory() {
    std::vector<mavlink_message_t> sent;
    PayloadManager pm([&](const mavlink_message_t& m) { sent.push_back(m); });
    const auto problems = pm.loadDirectory(std::string(GCS_SOURCE_DIR) + "/icds");
    CHECK(problems.empty());
    CHECK(pm.icds().size() >= 1);
    CHECK(pm.loadDirectory("/nonexistent/dir").empty());
}

}  // namespace

int main() {
    testJson();
    testLoad();
    testDecode();
    testCommands();
    testAdapter();
    testManager();
    testLoadDirectory();
    return TEST_MAIN_RESULT();
}
