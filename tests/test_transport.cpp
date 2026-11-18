// Transports (UDP, serial) and MavlinkClient over them: a loopback UDP pair,
// a pseudo-terminal standing in for a Pixhawk on USB, live switching between
// links, and the port vanishing and coming back.

#include "check.hpp"
#include "link/mavlink_client.hpp"
#include "link/mavlink_headers.hpp"
#include "link/serial_transport.hpp"
#include "link/udp_transport.hpp"

#include <chrono>
#include <cstring>
#include <functional>
#include <string>
#include <thread>
#include <vector>

#if !defined(_WIN32)
#include <fcntl.h>
#include <poll.h>
#include <stdlib.h>
#include <unistd.h>
#endif

using namespace gcs;
using namespace gcs::link;
using Clock = std::chrono::steady_clock;

namespace {

bool waitFor(const std::function<bool()>& cond, int ms = 2000) {
    const auto until = Clock::now() + std::chrono::milliseconds(ms);
    while (Clock::now() < until) {
        if (cond()) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return cond();
}

std::vector<uint8_t> vehicleHeartbeat(uint8_t sysid) {
    mavlink_message_t m{};
    mavlink_msg_heartbeat_pack(sysid, 1, &m, MAV_TYPE_VTOL_TILTROTOR, MAV_AUTOPILOT_ARDUPILOTMEGA, 0, 0,
                               MAV_STATE_STANDBY);
    std::vector<uint8_t> buf(MAVLINK_MAX_PACKET_LEN);
    buf.resize(mavlink_msg_to_send_buffer(buf.data(), &m));
    return buf;
}

void testUdpLoopback() {
    std::string err;
    UdpTransport gcs("127.0.0.1", 0);
    CHECK(gcs.open(err));
    CHECK(!gcs.canSend());  // nobody has talked to it yet
    const uint16_t port = gcs.boundPort();
    CHECK(port != 0);

    // the "vehicle" side sends to the GCS from the start
    UdpTransport veh("127.0.0.1", 0, "127.0.0.1", port);
    CHECK(veh.open(err));
    CHECK(veh.canSend());
    const uint8_t hello[] = {1, 2, 3, 4};
    veh.write(hello, sizeof(hello));

    CHECK(gcs.waitReadable(500, err) == 1);
    uint8_t buf[64];
    CHECK(gcs.read(buf, sizeof(buf), err) == 4);
    CHECK(std::memcmp(buf, hello, 4) == 0);
    CHECK(gcs.canSend());  // learned the peer
    CHECK(gcs.read(buf, sizeof(buf), err) == 0);  // nothing more

    const uint8_t reply[] = {9, 8};
    gcs.write(reply, sizeof(reply));
    CHECK(veh.waitReadable(500, err) == 1);
    CHECK(veh.read(buf, sizeof(buf), err) == 2);
    CHECK(buf[0] == 9 && buf[1] == 8);

    // a port in use is reported in plain words
    UdpTransport clash("127.0.0.1", port);
    CHECK(!clash.open(err));
    CHECK(err.find("in use") != std::string::npos);

    UdpTransport bad("not-an-ip", 14550);
    CHECK(!bad.open(err));
    CHECK(err.find("not an IP") != std::string::npos);
}

#if !defined(_WIN32)
struct Pty {
    int master = -1;
    std::string slave;
    Pty() {
        master = ::posix_openpt(O_RDWR | O_NOCTTY);
        if (master >= 0 && ::grantpt(master) == 0 && ::unlockpt(master) == 0) {
            slave = ::ptsname(master);
            ::fcntl(master, F_SETFL, ::fcntl(master, F_GETFL) | O_NONBLOCK);
        }
    }
    ~Pty() { closeMaster(); }
    void closeMaster() {
        if (master >= 0) {
            ::close(master);
            master = -1;
        }
    }
    void write(const std::vector<uint8_t>& b) const { [[maybe_unused]] auto n = ::write(master, b.data(), b.size()); }
    std::vector<uint8_t> readAll() const {
        std::vector<uint8_t> out;
        uint8_t buf[512];
        for (;;) {
            const auto n = ::read(master, buf, sizeof(buf));
            if (n <= 0) {
                break;
            }
            out.insert(out.end(), buf, buf + n);
        }
        return out;
    }
};

// MAVLink messages in a byte stream from the GCS
std::vector<mavlink_message_t> parse(const std::vector<uint8_t>& bytes) {
    std::vector<mavlink_message_t> out;
    mavlink_message_t msg{};
    mavlink_status_t st{};
    for (uint8_t c : bytes) {
        if (mavlink_parse_char(MAVLINK_COMM_1, c, &msg, &st)) {
            out.push_back(msg);
        }
    }
    return out;
}

void testSerialTransport() {
    Pty pty;
    CHECK(!pty.slave.empty());
    std::string err;
    SerialTransport s(pty.slave, 57600);
    const bool opened = s.open(err);
    if (!opened) {
        std::fprintf(stderr, "open %s: %s\n", pty.slave.c_str(), err.c_str());
    }
    CHECK(opened);
    CHECK(s.canSend());
    pty.write({0xFD, 1, 2});
    CHECK(s.waitReadable(500, err) == 1);
    uint8_t buf[16];
    CHECK(s.read(buf, sizeof(buf), err) == 3);
    const uint8_t out[] = {7, 7, 7};
    s.write(out, sizeof(out));
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    CHECK(pty.readAll().size() == 3);

    SerialTransport missing("/dev/does-not-exist-gcs", 57600);
    CHECK(!missing.open(err));
    CHECK(err.find("not found") != std::string::npos);
#if !defined(__APPLE__)
    SerialTransport odd(pty.slave, 12345);
    CHECK(!odd.open(err));  // not a standard rate (macOS takes any on a real port)
#endif
}

void testClientOverSerialAndSwitching() {
    Pty pty;
    MavlinkClient::Config cfg;
    cfg.type = MavlinkClient::LinkType::Udp;
    cfg.bind_host = "127.0.0.1";
    cfg.port = 0;  // any free port: nobody talks to it
    MavlinkClient client(cfg);
    client.start();
    CHECK(waitFor([&] { return client.snapshot().socket_open; }));
    CHECK(client.snapshot().endpoint.rfind("UDP ", 0) == 0);

    // switch to "the Pixhawk on USB"
    cfg.type = MavlinkClient::LinkType::Serial;
    cfg.serial_port = pty.slave;
    cfg.serial_baud = 115200;
    client.reconfigure(cfg);
    CHECK(waitFor([&] { return client.snapshot().endpoint == pty.slave + " · 115200"; }));
    CHECK(waitFor([&] { return client.snapshot().socket_open; }));

    // the vehicle talks; the GCS locks on and heartbeats back over the port
    bool gcs_heartbeat = false;
    CHECK(waitFor(
        [&] {
            pty.write(vehicleHeartbeat(7));
            for (const auto& m : parse(pty.readAll())) {
                gcs_heartbeat = gcs_heartbeat || (m.msgid == MAVLINK_MSG_ID_HEARTBEAT && m.sysid == 255);
            }
            return client.snapshot().has_heartbeat && gcs_heartbeat;
        },
        4000));
    bool connected = false;
    for (const auto& ev : client.pollEvents()) {
        connected = connected || (ev.type == LinkEvent::Type::VehicleConnected && ev.done == 7);
    }
    CHECK(connected);

    // unplugged: an error, and the client keeps trying
    pty.closeMaster();
    CHECK(waitFor([&] { return !client.snapshot().socket_open && !client.snapshot().error.empty(); }));

    // back to UDP: the error clears, the socket opens
    cfg.type = MavlinkClient::LinkType::Udp;
    client.reconfigure(cfg);
    CHECK(waitFor([&] { return client.snapshot().socket_open && client.snapshot().error.empty(); }));
    client.stop();
}

void testSerialReconnect() {
    // a port that isn't there yet (Pixhawk still booting): retried until it appears
    MavlinkClient::Config cfg;
    cfg.type = MavlinkClient::LinkType::Serial;
    cfg.serial_port = "/dev/does-not-exist-gcs";
    MavlinkClient client(cfg);
    client.start();
    CHECK(waitFor([&] { return client.snapshot().error.find("not found") != std::string::npos; }));
    CHECK(!client.snapshot().socket_open);
    client.stop();
}
#endif

void testPortList() {
    // no assumptions about what's plugged in: just that it runs and labels what it finds
    for (const auto& p : listSerialPorts()) {
        CHECK(!p.path.empty());
        CHECK(!p.label.empty());
    }
    CHECK(serialBauds().front() == 57600);
}

}  // namespace

int main() {
    testUdpLoopback();
#if !defined(_WIN32)
    testSerialTransport();
    testClientOverSerialAndSwitching();
    testSerialReconnect();
#endif
    testPortList();
    return TEST_MAIN_RESULT();
}
