#pragma once

#include "core/pmtiles.hpp"

#include <QFile>
#include <QNetworkAccessManager>
#include <QObject>
#include <QString>

#include <functional>
#include <map>
#include <memory>
#include <vector>

namespace gcs::ui {

// Cuts a region out of the latest Protomaps daily build (the whole planet as
// one PMTiles file on build.protomaps.com) with HTTP range requests, and
// writes it as a local PMTiles file. Only the index pieces covering the region
// are read, and the tile bytes come in a few large ranges. The result is
// written to `<path>.part` and renamed when complete.
class PmtilesExtract : public QObject {
    Q_OBJECT

public:
    struct Box {
        double min_lon, min_lat, max_lon, max_lat;
    };
    explicit PmtilesExtract(QObject* parent = nullptr);

    // `installed`: the build already on disk ("" none). When it's still the
    // latest, nothing is downloaded and finished() reports up_to_date.
    void start(const Box& box, int max_zoom, const QString& path, const QString& installed = {});
    bool upToDate() const { return up_to_date_; }
    QString build() const { return build_; }  // the build checked / installed, e.g. "20261117"
    void cancel();
    bool running() const { return running_; }
    QString stage() const { return stage_; }  // what it is doing, for the progress line
    qint64 done() const { return done_; }
    qint64 total() const { return total_; }

signals:
    void progress();
    void finished(bool ok, const QString& message);

private:
    void get(const QString& url, qint64 from, qint64 to, std::function<void(QByteArray)> ok);
    void fail(const QString& why);
    void readBuilds();
    void readHeader();
    void walkIndex();
    void nextLeaf();
    void planData();
    void nextRange();
    void writeFile();

    QNetworkAccessManager nam_;
    bool running_ = false;
    bool cancelled_ = false;
    QString stage_;
    qint64 done_ = 0, total_ = 0;

    Box box_{};
    int max_zoom_ = 15;
    QString path_;
    QString url_;
    QString installed_;
    QString build_;
    bool up_to_date_ = false;
    pmtiles::Header src_;
    QByteArray metadata_;
    std::vector<pmtiles::Entry> root_;
    std::vector<uint64_t> want_;                                      // tile ids, sorted
    std::map<uint64_t, std::vector<pmtiles::Entry>> leaves_;          // by leaf offset
    std::vector<uint64_t> leaf_queue_;
    std::map<uint64_t, std::pair<uint64_t, uint32_t>> found_;         // tile id -> source (offset, length)
    std::vector<std::pair<uint64_t, uint64_t>> ranges_;               // source byte ranges to fetch
    std::vector<std::pair<uint64_t, uint32_t>> blobs_;                // distinct source blobs, by offset
    size_t blob_i_ = 0;
    size_t range_i_ = 0;
    std::map<uint64_t, uint64_t> new_offset_;                         // source offset -> offset in our data
    std::unique_ptr<QFile> data_;                                     // tile bytes, streamed
    uint64_t data_len_ = 0;
};

}  // namespace gcs::ui
