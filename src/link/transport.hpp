#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace gcs::link {

// A byte pipe to the vehicle: a UDP socket or a serial port. Owned and used by
// the MavlinkClient worker thread only. Implementations keep every OS call
// (sockets, termios, Win32) inside their .cpp, so the rest of the link layer
// stays portable.
class Transport {
public:
    virtual ~Transport() = default;

    // Opens the port or socket; false with `error` set when it can't (yet).
    virtual bool open(std::string& error) = 0;
    virtual void close() = 0;
    virtual bool isOpen() const = 0;

    // Waits up to `timeout_ms` for incoming bytes. 1 = readable, 0 = timeout,
    // -1 = the transport failed (unplugged, closed); `error` says why.
    virtual int waitReadable(int timeout_ms, std::string& error) = 0;

    // Reads what is there without blocking: bytes read (0 = nothing more),
    // -1 = the transport failed. UDP reads one datagram per call.
    virtual long read(uint8_t* buf, size_t max, std::string& error) = 0;

    // Whether write() has somewhere to go (UDP: a peer is known).
    virtual bool canSend() const = 0;
    virtual void write(const uint8_t* data, size_t len) = 0;

    // "UDP 0.0.0.0:14550", "/dev/ttyACM0 · 115200": for the status line.
    virtual std::string describe() const = 0;
};

}  // namespace gcs::link
