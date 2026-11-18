#include "video_receiver.hpp"

#include <limits>

#ifdef GCS_HAVE_GSTREAMER
#include <gst/app/gstappsink.h>
#include <gst/gst.h>
#include <gst/video/video.h>
#endif

namespace gcs::ui {

VideoReceiver::VideoReceiver(int port) : port_(port) {
    status_ = compiledIn() ? QStringLiteral("idle") : QStringLiteral("built without GStreamer");
}

VideoReceiver::~VideoReceiver() {
    stop();
}

bool VideoReceiver::compiledIn() {
#ifdef GCS_HAVE_GSTREAMER
    return true;
#else
    return false;
#endif
}

QString VideoReceiver::status() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return status_;
}

void VideoReceiver::start() {
    if (!compiledIn() || thread_.joinable()) {
        return;
    }
    stop_ = false;
    thread_ = std::thread([this] { run(); });
}

void VideoReceiver::stop() {
    stop_ = true;
    if (thread_.joinable()) {
        thread_.join();
    }
}

void VideoReceiver::restart(int port) {
    stop();
    port_ = port;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        frame_ = QImage();
        status_.clear();
    }
    start();
}

bool VideoReceiver::takeFrame(QImage& out, uint64_t& seq) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (seq_ == seq || frame_.isNull()) {
        return false;
    }
    out = frame_;  // implicitly shared, cheap
    seq = seq_;
    return true;
}

double VideoReceiver::frameAge() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (seq_ == 0) {
        return std::numeric_limits<double>::infinity();
    }
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - last_frame_).count();
}

#ifdef GCS_HAVE_GSTREAMER

void VideoReceiver::run() {
    static std::once_flag once;
    std::call_once(once, [] { gst_init(nullptr, nullptr); });

    const std::string desc =
        "udpsrc port=" + std::to_string(port_) +
        " caps=\"application/x-rtp,media=video,clock-rate=90000,encoding-name=H264,payload=96\""
        " ! rtpjitterbuffer latency=50 ! rtph264depay ! avdec_h264"
        " ! videoconvert ! video/x-raw,format=RGB ! appsink name=sink drop=true max-buffers=1 sync=false";

    while (!stop_) {
        GError* err = nullptr;
        GstElement* pipeline = gst_parse_launch(desc.c_str(), &err);
        if (!pipeline || err) {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                status_ = QStringLiteral("pipeline error: %1").arg(err ? QString::fromUtf8(err->message) : QString());
            }
            if (err) {
                g_error_free(err);
            }
            if (pipeline) {
                gst_object_unref(pipeline);
            }
            return;  // missing plugin: retrying will not help
        }
        GstElement* sink = gst_bin_get_by_name(GST_BIN(pipeline), "sink");
        GstBus* bus = gst_element_get_bus(pipeline);
        gst_element_set_state(pipeline, GST_STATE_PLAYING);
        {
            std::lock_guard<std::mutex> lock(mutex_);
            status_ = QStringLiteral("listening udp:%1").arg(port_);
        }

        bool failed = false;
        while (!stop_ && !failed) {
            GstSample* sample = gst_app_sink_try_pull_sample(GST_APP_SINK(sink), 100 * GST_MSECOND);
            if (sample) {
                GstCaps* caps = gst_sample_get_caps(sample);
                GstBuffer* buf = gst_sample_get_buffer(sample);
                GstVideoInfo info;
                GstMapInfo map;
                if (caps && buf && gst_video_info_from_caps(&info, caps) &&
                    gst_buffer_map(buf, &map, GST_MAP_READ)) {
                    const int w = GST_VIDEO_INFO_WIDTH(&info);
                    const int h = GST_VIDEO_INFO_HEIGHT(&info);
                    const int stride = GST_VIDEO_INFO_PLANE_STRIDE(&info, 0);
                    QImage img(map.data, w, h, stride, QImage::Format_RGB888);
                    QImage copy = img.copy();  // detach from the GstBuffer
                    gst_buffer_unmap(buf, &map);
                    std::lock_guard<std::mutex> lock(mutex_);
                    frame_ = std::move(copy);
                    ++seq_;
                    last_frame_ = std::chrono::steady_clock::now();
                    status_ = QStringLiteral("%1x%2 udp:%3").arg(w).arg(h).arg(port_);
                }
                gst_sample_unref(sample);
            }
            while (GstMessage* msg = gst_bus_pop_filtered(bus, static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_EOS))) {
                if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_ERROR) {
                    GError* e = nullptr;
                    gst_message_parse_error(msg, &e, nullptr);
                    std::lock_guard<std::mutex> lock(mutex_);
                    status_ = QStringLiteral("stream error: %1").arg(e ? QString::fromUtf8(e->message) : QString());
                    if (e) {
                        g_error_free(e);
                    }
                }
                failed = true;
                gst_message_unref(msg);
            }
        }

        gst_element_set_state(pipeline, GST_STATE_NULL);
        gst_object_unref(bus);
        gst_object_unref(sink);
        gst_object_unref(pipeline);
        // after a stream error, restart the pipeline (e.g. sender restarted)
        for (int i = 0; i < 10 && !stop_; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
}

#else

void VideoReceiver::run() {}

#endif

}  // namespace gcs::ui
