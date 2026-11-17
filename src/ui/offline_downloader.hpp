#pragma once

#include <QNetworkAccessManager>
#include <QObject>
#include <QString>

#include <array>
#include <deque>
#include <vector>

namespace gcs::ui {

// Downloads the map imagery and elevation around a point into the offline
// folder (offline_store.hpp), so the map, the hover ground height and the
// approach all work without internet. Polite: two requests at a time, files
// already on disk are skipped (a cancelled download resumes).
class OfflineDownloader : public QObject {
    Q_OBJECT

public:
    struct Area {
        double lat = 0.0, lon = 0.0;
        double radius_km = 25.0;
        std::array<bool, 3> layers{true, true, true};  // offline::Layer
    };
    struct Estimate {
        std::array<size_t, 3> tiles{};  // per layer
        size_t total = 0;
        qint64 bytes = 0;               // rough
        int map_max_zoom = 0, elev_zoom = 0;
    };
    // map imagery to a zoom that keeps ~3000 tiles per layer; elevation at one zoom
    static int mapMaxZoom(double radius_km);
    static int elevationZoom(double radius_km);
    static Estimate estimate(const Area& a);
    // bytes in the offline folder's tile layers
    static qint64 diskUsage();
    // removes osm/, esri_imagery/ and elevation/ from the offline folder (nothing else)
    static bool clear();

    explicit OfflineDownloader(QObject* parent = nullptr);

    void start(const Area& a);
    void cancel();
    bool running() const { return running_; }
    size_t done() const { return done_; }
    size_t total() const { return total_; }
    size_t failed() const { return failed_; }
    qint64 bytes() const { return bytes_; }

signals:
    void progress();
    void finished(bool cancelled);

private:
    struct Job {
        int layer, z, x, y;
    };
    void pump();
    void finish(bool cancelled);

    QNetworkAccessManager nam_;
    std::deque<Job> queue_;
    int in_flight_ = 0;
    bool running_ = false;
    size_t done_ = 0, total_ = 0, failed_ = 0;
    qint64 bytes_ = 0;
    static constexpr int kParallel = 2;
};

}  // namespace gcs::ui
