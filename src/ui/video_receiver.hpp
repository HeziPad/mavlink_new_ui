#pragma once

#include <QImage>
#include <QString>

#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>

namespace gcs::ui {

// RTP/H.264 over UDP -> RGB frames, decoded on its own thread by GStreamer:
//   udpsrc port=N ! rtpjitterbuffer ! rtph264depay ! avdec_h264 ! videoconvert
//   ! video/x-raw,format=RGB ! appsink drop=true max-buffers=1
// The UI pulls the newest frame; older ones are simply overwritten.
// Without GStreamer at build time this is an inert stub.
class VideoReceiver {
public:
    explicit VideoReceiver(int port);
    ~VideoReceiver();

    VideoReceiver(const VideoReceiver&) = delete;
    VideoReceiver& operator=(const VideoReceiver&) = delete;

    void start();
    void stop();
    // Listens on another port (drops the current picture).
    void restart(int port);

    static bool compiledIn();
    int port() const { return port_; }
    QString status() const;

    // Copies the newest frame if it is newer than `seq`; returns true then.
    bool takeFrame(QImage& out, uint64_t& seq) const;
    // Seconds since the last decoded frame (infinity if none yet).
    double frameAge() const;

private:
    void run();

    int port_;
    std::atomic<bool> stop_{false};
    std::thread thread_;

    mutable std::mutex mutex_;
    QImage frame_;
    uint64_t seq_ = 0;
    std::chrono::steady_clock::time_point last_frame_{};
    QString status_;
};

}  // namespace gcs::ui
