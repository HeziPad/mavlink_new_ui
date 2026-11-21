#pragma once

#include "pmtiles_extract.hpp"

#include <QNetworkAccessManager>
#include <QObject>
#include <QString>

#include <deque>
#include <vector>

namespace gcs::ui {

// Fills the offline folder (offline_store.hpp):
//  - the world street map: z0-6 of the whole planet (world.pmtiles, ~45 MB),
//    so there are coasts, borders, main roads and cities everywhere;
//  - the country pack: the street map (a vector extract, map.pmtiles),
//    satellite z8-14 and elevation z12 for Israel + ~50 km;
//  - areas the user draws on the map: the street map z0-15
//    (streets/<box>.pmtiles), ground height, and satellite as deep as the
//    operator chose for that area (Detail), listed in areas.json.
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
        bool streets = false; // an area: its street map is to download too
        bool exact = true;    // false: too many tiles to check the disk, fresh = all of them
    };
    // how deep a drawn area's satellite goes (streets and ground height always come)
    enum class Detail { Streets, Sat10m, Sat2m, Sat1m, Sat05m };
    static constexpr int kDetails = 5;
    struct Zooms {
        int sat_min = 1, sat_max = 0;  // sat_max < sat_min: no satellite
        int elev = 12;
        bool hasSat() const { return sat_max >= sat_min; }
    };
    static Zooms zooms(Detail d);
    static QString detailLabel(Detail d);  // the chip: "2 m"
    static QString detailText(Detail d);   // "satellite to ~2 m/px"
    struct Area {
        Box box;
        Zooms z;
    };
    // what each level would fetch for a box, from tile arithmetic only (instant)
    struct Level {
        Detail detail;
        size_t tiles = 0;
        qint64 bytes = 0;
        bool allowed = true;  // under the cap
    };
    static std::vector<Level> areaLevels(const Box& b);
    static Detail preferredDetail(const std::vector<Level>& levels);  // the sharpest under ~2 GB

    static Box country();  // Israel + ~50 km
    static Count countryCount();
    static Count areaCount(const Box& b, Detail d);
    static std::vector<Area> areas();  // areas downloaded so far
    static qint64 diskUsage();        // everything in the offline folder
    static qint64 fullDetailUsage();  // what the areas added (satellite, ground height, streets)
    static bool clearFullDetail();
    static bool clearAll();
    static bool hasStreetMap();  // the country's
    static bool hasWorldMap();
    static QString installedBuild();  // street map build on disk ("" unknown / none)
    static Count areasMissing();      // tiles the full-detail areas still lack

    explicit OfflineDownloader(QObject* parent = nullptr);

    // the world street map if missing, the country street map if a newer build
    // exists (or none is installed), the street map of every area that lacks
    // one, then every missing country and full-detail-area tile
    void startCountry();
    void startArea(const Box& b, Detail d);
    void cancel();
    bool running() const { return running_; }
    // progress line: what it is doing, and how far
    QString stage() const;
    double fraction() const;
    size_t failed() const { return failed_; }

signals:
    void progress();
    void streetMapInstalled(const QString& what);  // "World street map", ...
    void finished(bool cancelled, const QString& summary);

private:
    struct Job {
        int layer, z, x, y;
    };
    struct StreetJob {
        Box box;
        int max_zoom;
        QString path;
        QString build_file;  // "" = not kept up to date
        QString installed;   // the build on disk ("" = fetch anyway)
        QString what;        // for messages
        bool required;       // failing stops the run (else it's reported and the run goes on)
    };
    static QString areaStreetsPath(const Box& b);
    static bool needsAreaStreets(const Box& b);
    static StreetJob areaStreetJob(const Box& b);
    void nextStreet();
    static std::vector<Job> jobs(const Box& b, int layer, int z0, int z1, bool fresh_only);
    static std::vector<Job> areaJobs(const Area& a, bool fresh_only);  // ground height first, then coarse to fine
    static std::vector<QString> areaCoarseFiles();  // the areas' z8-14 / z12 tiles outside the country
    static std::vector<Job> missing();  // country + every area, each tile once
    void startTiles(const QString& stage);
    void pump();
    void finish(bool cancelled, const QString& summary);

    QNetworkAccessManager nam_;
    PmtilesExtract extract_;
    std::deque<StreetJob> streets_;
    StreetJob street_{};  // the one running
    QStringList street_failed_;
    std::deque<Job> queue_;
    int in_flight_ = 0;
    bool running_ = false;
    bool cancelled_ = false;
    bool area_ = false;
    Box area_box_;
    Zooms area_z_;
    QString stage_;
    QString street_msg_;  // what happened to the street map in this run
    size_t done_ = 0, total_ = 0, failed_ = 0;
    qint64 bytes_ = 0;
    static constexpr int kParallel = 2;
};

}  // namespace gcs::ui
