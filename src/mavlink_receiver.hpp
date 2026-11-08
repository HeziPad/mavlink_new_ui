#pragma once

#include "telemetry.hpp"

#include <atomic>
#include <mutex>
#include <thread>

class MavlinkReceiver {
public:
    MavlinkReceiver(std::string host, uint16_t port);
    ~MavlinkReceiver();

    MavlinkReceiver(const MavlinkReceiver&) = delete;
    MavlinkReceiver& operator=(const MavlinkReceiver&) = delete;

    void start();
    void stop();
    TelemetrySnapshot snapshot() const;

private:
    void run();

    std::string host_;
    uint16_t port_;
    std::atomic<bool> stop_{false};
    std::thread thread_;
    mutable std::mutex mutex_;
    TelemetrySnapshot snap_;
};
