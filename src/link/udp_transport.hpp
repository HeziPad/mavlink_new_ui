#pragma once

#include "transport.hpp"

#include <array>

namespace gcs::link {

// MAVLink over UDP. Listens on bind_host:port and replies to whoever sent the
// last datagram. With a remote host:port set, it also sends there from the
// start, for radios and companions that wait for the GCS to talk first.
class UdpTransport : public Transport {
public:
    UdpTransport(std::string bind_host, uint16_t port, std::string remote_host = {}, uint16_t remote_port = 0);
    ~UdpTransport() override;

    UdpTransport(const UdpTransport&) = delete;
    UdpTransport& operator=(const UdpTransport&) = delete;

    bool open(std::string& error) override;
    void close() override;
    bool isOpen() const override;
    int waitReadable(int timeout_ms, std::string& error) override;
    long read(uint8_t* buf, size_t max, std::string& error) override;
    bool canSend() const override { return peer_known_; }
    void write(const uint8_t* data, size_t len) override;
    std::string describe() const override;

    uint16_t boundPort() const { return bound_port_; }  // the actual port (when 0 was asked)

private:
    std::string bind_host_;
    uint16_t port_;
    std::string remote_host_;
    uint16_t remote_port_;
    uint16_t bound_port_ = 0;
    intptr_t fd_ = -1;  // a socket handle (SOCKET on Windows)
    // the peer as a raw sockaddr_in, so no socket header leaks out of the .cpp
    alignas(8) std::array<uint8_t, 16> peer_{};
    bool peer_known_ = false;
};

}  // namespace gcs::link
