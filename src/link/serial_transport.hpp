#pragma once

#include "transport.hpp"

#include <vector>

namespace gcs::link {

// MAVLink over a serial port: a Pixhawk on USB, or a telemetry radio (SiK).
// 8N1, no flow control. When the port vanishes (USB unplugged, or the
// autopilot rebooting re-enumerates it), read/waitReadable report a failure
// and the client re-opens it until it is back.
class SerialTransport : public Transport {
public:
    SerialTransport(std::string port, int baud);
    ~SerialTransport() override;

    SerialTransport(const SerialTransport&) = delete;
    SerialTransport& operator=(const SerialTransport&) = delete;

    bool open(std::string& error) override;
    void close() override;
    bool isOpen() const override;
    int waitReadable(int timeout_ms, std::string& error) override;
    long read(uint8_t* buf, size_t max, std::string& error) override;
    bool canSend() const override { return isOpen(); }
    void write(const uint8_t* data, size_t len) override;
    std::string describe() const override;

private:
    std::string port_;
    int baud_;
    intptr_t fd_ = -1;  // a file descriptor (a HANDLE on Windows)
};

// The baud rates offered (and accepted by every platform)
const std::vector<int>& serialBauds();

struct SerialPortInfo {
    std::string path;   // what to open: "/dev/ttyACM0", "/dev/cu.usbmodem1101", "COM4"
    std::string label;  // what to show: "ArduPilot Pixhawk6C (ttyACM0)"
    bool likely_autopilot = false;  // the name says Pixhawk / ArduPilot / PX4 / Cube
};

// Serial ports present now, autopilots first.
std::vector<SerialPortInfo> listSerialPorts();

}  // namespace gcs::link
