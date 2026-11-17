#pragma once

#include "pmtiles_extract.hpp"

#include <QNetworkAccessManager>
#include <QObject>
#include <QString>

#include <deque>
#include <vector>

namespace gcs::ui {

// Fills the offline folder (offline_store.hpp):
//  - the country pack: the street map (a vector extract, map.pmtiles),
//    satellite z8-14 and elevation z12 for Israel + ~50 km;
//  - full-detail areas the user draws on the map: satellite z15-18 and
//    elevation z13, listed in areas.json.
// Every tile lives at one path by z/x/y, and files already present (or
// marked as having no imagery) are skipped, so overlapping areas share tiles
// and an interrupted download resumes. Two requests at a time.
class OfflineDownloader : public QObject {
    Q_OBJECT

public:
    struct Box {
        double min_lon = 0, min_lat = 0, max_lon = 0, max_lat = 0;
        bool valid() const { return max_lon > min_lon && max_lat > min_lat; }
        double widthKm() const;
        double heightKm() const;
    };
    struct Count {
        size_t fresh = 0;     // to download
        size_t have = 0;      // already on disk (shared with other areas / the country)
        qint64 bytes = 0;     // rough size of the fresh ones
    };

    static Box country();  // Israel + ~50 km
    static Count countryCount();
    static Count areaCount(const Box& b);
    static std::vector<Box> areas();  // full-detail areas downloaded so far
    static qint64 diskUsage();        // everything in the offline folder
    static qint64 fullDetailUsage();  // satellite z15+ and elevation z13
    static bool clearFullDetail();
    static bool clearAll();
    static bool hasStreetMap();

    explicit OfflineDownloader(QObject* parent = nullptr);

    void startCountry();
    void startArea(const Box& b);
    void cancel();
    bool running() const { return running_; }
    // progress line: what it is doing, and how far
    QString stage() const;
    double fraction() const;
    size_t failed() const { return failed_; }

signals:
    void progress();
    void streetMapInstalled();
    void finished(bool cancelled, const QString& summary);

private:
    struct Job {
        int layer, z, x, y;
    };
    static std::vector<Job> jobs(const Box& b, int layer, int z0, int z1, bool fresh_only);
    void startTiles(const QString& stage);
    void pump();
    void finish(bool cancelled, const QString& summary);

    QNetworkAccessManager nam_;
    PmtilesExtract extract_;
    std::deque<Job> queue_;
    int in_flight_ = 0;
    bool running_ = false;
    bool cancelled_ = false;
    bool area_ = false;
    Box area_box_;
    QString stage_;
    size_t done_ = 0, total_ = 0, failed_ = 0;
    qint64 bytes_ = 0;
    static constexpr int kParallel = 2;
};

}  // namespace gcs::ui
