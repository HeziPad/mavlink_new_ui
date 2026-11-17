#include "pmtiles_extract.hpp"

#include "core/gzip.hpp"
#include "core/tiles.hpp"

#include <QDir>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QNetworkReply>
#include <QNetworkRequest>

#include <algorithm>
#include <set>

namespace gcs::ui {

namespace {
const char* kBuilds = "https://build-metadata.protomaps.dev/builds.json";
const char* kBuildUrl = "https://build.protomaps.com/%1";
constexpr uint64_t kMergeGap = 256 * 1024;        // join ranges closer than this
constexpr uint64_t kMaxRange = 16 * 1024 * 1024;  // but keep requests at most this big

std::string_view view(const QByteArray& b) {
    return std::string_view(b.constData(), static_cast<size_t>(b.size()));
}
}  // namespace

PmtilesExtract::PmtilesExtract(QObject* parent) : QObject(parent) {}

void PmtilesExtract::start(const Box& box, int max_zoom, const QString& path, const QString& installed) {
    if (running_) return;
    running_ = true;
    cancelled_ = false;
    installed_ = installed;
    up_to_date_ = false;
    build_.clear();
    box_ = box;
    max_zoom_ = max_zoom;
    path_ = path;
    done_ = total_ = 0;
    root_.clear();
    leaves_.clear();
    found_.clear();
    ranges_.clear();
    new_offset_.clear();
    want_.clear();
    data_len_ = 0;
    readBuilds();
}

void PmtilesExtract::cancel() {
    if (!running_) return;
    cancelled_ = true;
}

void PmtilesExtract::fail(const QString& why) {
    running_ = false;
    if (data_) {
        data_->remove();
        data_.reset();
    }
    emit finished(false, why);
}

void PmtilesExtract::get(const QString& url, qint64 from, qint64 to, std::function<void(QByteArray)> ok) {
    if (cancelled_) {
        fail(QStringLiteral("cancelled"));
        return;
    }
    QNetworkRequest req{QUrl(url)};
    req.setRawHeader("User-Agent", "mavlink_gcs_ui/0.1 (Qt; offline map)");
    req.setTransferTimeout(120000);
    if (from >= 0) {
        req.setRawHeader("Range", QStringLiteral("bytes=%1-%2").arg(from).arg(to).toLatin1());
    }
    QNetworkReply* r = nam_.get(req);
    connect(r, &QNetworkReply::finished, this, [this, r, ok] {
        r->deleteLater();
        if (r->error() != QNetworkReply::NoError) {
            fail(r->errorString());
            return;
        }
        ok(r->readAll());
    });
}

void PmtilesExtract::readBuilds() {
    stage_ = QStringLiteral("finding the latest street map");
    emit progress();
    get(QString::fromLatin1(kBuilds), -1, -1, [this](QByteArray b) {
        const QJsonArray builds = QJsonDocument::fromJson(b).array();
        if (builds.isEmpty()) {
            fail(QStringLiteral("no Protomaps build listed"));
            return;
        }
        const QString key = builds.last().toObject().value(QStringLiteral("key")).toString();
        build_ = QString(key).remove(QStringLiteral(".pmtiles"));
        if (!installed_.isEmpty() && build_ == installed_ && QFileInfo::exists(path_)) {
            // what's on disk is the latest: nothing to fetch
            up_to_date_ = true;
            running_ = false;
            emit finished(true, QStringLiteral("street map up to date (%1)").arg(build_));
            return;
        }
        url_ = QString::fromLatin1(kBuildUrl).arg(key);
        readHeader();
    });
}

void PmtilesExtract::readHeader() {
    get(url_, 0, static_cast<qint64>(pmtiles::kHeaderSize) - 1, [this](QByteArray b) {
        const auto h = pmtiles::parseHeader(view(b));
        if (!h || h->tile_type != pmtiles::kMvt) {
            fail(QStringLiteral("not a vector PMTiles file"));
            return;
        }
        src_ = *h;
        max_zoom_ = std::min<int>(max_zoom_, h->max_zoom);
        get(url_, static_cast<qint64>(h->root_offset), static_cast<qint64>(h->root_offset + h->root_length) - 1,
            [this](QByteArray rb) {
                const auto dir = gz::inflate(view(rb));
                const auto root = dir ? pmtiles::decodeDirectory(*dir) : std::nullopt;
                if (!root) {
                    fail(QStringLiteral("unreadable index"));
                    return;
                }
                root_ = *root;
                get(url_, static_cast<qint64>(src_.metadata_offset),
                    static_cast<qint64>(src_.metadata_offset + src_.metadata_length) - 1, [this](QByteArray mb) {
                        metadata_ = mb;
                        walkIndex();
                    });
            });
    });
}

void PmtilesExtract::walkIndex() {
    // every tile id of the box, all zooms
    for (int z = 0; z <= max_zoom_; ++z) {
        const tiles::TilePos a = tiles::locate(box_.max_lat, box_.min_lon, z);
        const tiles::TilePos b = tiles::locate(box_.min_lat, box_.max_lon, z);
        for (int x = a.x; x <= b.x; ++x) {
            for (int y = a.y; y <= b.y; ++y) {
                want_.push_back(pmtiles::zxyToId(static_cast<uint8_t>(z), static_cast<uint32_t>(x), static_cast<uint32_t>(y)));
            }
        }
    }
    std::sort(want_.begin(), want_.end());
    // the leaves those ids live in
    std::set<uint64_t> leafs;
    for (uint64_t id : want_) {
        const auto e = pmtiles::findEntry(root_, id);
        if (e && e->run_length == 0) {
            leafs.insert(e->offset);
        }
    }
    leaf_queue_.assign(leafs.begin(), leafs.end());
    stage_ = QStringLiteral("reading the street map index");
    total_ = static_cast<qint64>(leaf_queue_.size());
    done_ = 0;
    nextLeaf();
}

void PmtilesExtract::nextLeaf() {
    if (leaf_queue_.empty()) {
        planData();
        return;
    }
    const uint64_t off = leaf_queue_.back();
    leaf_queue_.pop_back();
    // the root entry for this leaf (to know its length)
    uint32_t len = 0;
    for (const auto& e : root_) {
        if (e.run_length == 0 && e.offset == off) {
            len = e.length;
            break;
        }
    }
    get(url_, static_cast<qint64>(src_.leaf_offset + off), static_cast<qint64>(src_.leaf_offset + off + len) - 1,
        [this, off](QByteArray b) {
            const auto dir = gz::inflate(view(b));
            const auto entries = dir ? pmtiles::decodeDirectory(*dir) : std::nullopt;
            if (!entries) {
                fail(QStringLiteral("unreadable index piece"));
                return;
            }
            leaves_[off] = *entries;
            ++done_;
            emit progress();
            nextLeaf();
        });
}

void PmtilesExtract::planData() {
    // (the planet's leaves are one level deep; deeper ones would need another pass)
    for (uint64_t id : want_) {
        auto e = pmtiles::findEntry(root_, id);
        if (e && e->run_length == 0) {
            const auto it = leaves_.find(e->offset);
            e = it != leaves_.end() ? pmtiles::findEntry(it->second, id) : std::nullopt;
        }
        if (e && e->run_length > 0) {
            found_[id] = {e->offset, e->length};
        }
    }
    std::set<std::pair<uint64_t, uint32_t>> blobs;
    for (const auto& [id, b] : found_) blobs.insert(b);
    blobs_.assign(blobs.begin(), blobs.end());
    blob_i_ = 0;
    for (const auto& [off, len] : blobs_) {
        if (!ranges_.empty() && off <= ranges_.back().second + kMergeGap && off + len - ranges_.back().first <= kMaxRange) {
            ranges_.back().second = std::max<uint64_t>(ranges_.back().second, off + len);
        } else {
            ranges_.emplace_back(off, off + len);
        }
    }
    total_ = 0;
    for (const auto& r : ranges_) total_ += static_cast<qint64>(r.second - r.first);
    done_ = 0;
    stage_ = QStringLiteral("downloading the street map");
    QDir().mkpath(QFileInfo(path_).absolutePath());
    data_ = std::make_unique<QFile>(path_ + QStringLiteral(".data.part"));
    if (!data_->open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        fail(QStringLiteral("can't write %1").arg(data_->fileName()));
        return;
    }
    // the blobs, in source order, written once each
    range_i_ = 0;
    nextRange();
}

void PmtilesExtract::nextRange() {
    if (range_i_ >= ranges_.size()) {
        writeFile();
        return;
    }
    const auto [a, b] = ranges_[range_i_];
    get(url_, static_cast<qint64>(src_.data_offset + a), static_cast<qint64>(src_.data_offset + b) - 1,
        [this, a = a, b = b](QByteArray chunk) {
            if (static_cast<uint64_t>(chunk.size()) != b - a) {
                fail(QStringLiteral("short read from the server"));
                return;
            }
            // the distinct blobs inside this range, in source order
            for (; blob_i_ < blobs_.size() && blobs_[blob_i_].first + blobs_[blob_i_].second <= b; ++blob_i_) {
                const auto [off, len] = blobs_[blob_i_];
                if (off < a || new_offset_.count(off)) continue;
                new_offset_[off] = data_len_;
                data_->write(chunk.constData() + (off - a), len);
                data_len_ += len;
            }
            done_ += static_cast<qint64>(b - a);
            emit progress();
            ++range_i_;
            nextRange();
        });
}

void PmtilesExtract::writeFile() {
    stage_ = QStringLiteral("writing the street map");
    emit progress();
    data_->close();
    // entries in tile id order; consecutive ids with the same bytes become runs
    std::vector<pmtiles::Entry> entries;
    for (const auto& [id, bl] : found_) {
        const uint64_t off = new_offset_.at(bl.first);
        if (!entries.empty() && entries.back().tile_id + entries.back().run_length == id && entries.back().offset == off) {
            ++entries.back().run_length;
        } else {
            entries.push_back({id, off, bl.second, 1});
        }
    }
    // the root must fit in the first 16 KiB with the header: split into leaves if not
    std::string root = gz::deflateGzip(pmtiles::encodeDirectory(entries));
    std::string leaves;
    for (size_t leaf_size = 4096; pmtiles::kHeaderSize + root.size() > 16384; leaf_size *= 2) {
        std::vector<pmtiles::Entry> root_entries;
        leaves.clear();
        for (size_t i = 0; i < entries.size(); i += leaf_size) {
            const std::vector<pmtiles::Entry> part(entries.begin() + static_cast<long>(i),
                                                   entries.begin() + static_cast<long>(std::min(entries.size(), i + leaf_size)));
            const std::string chunk = gz::deflateGzip(pmtiles::encodeDirectory(part));
            root_entries.push_back({part.front().tile_id, leaves.size(), static_cast<uint32_t>(chunk.size()), 0});
            leaves += chunk;
        }
        root = gz::deflateGzip(pmtiles::encodeDirectory(root_entries));
    }
    pmtiles::Header h;
    h.root_offset = pmtiles::kHeaderSize;
    h.root_length = root.size();
    h.metadata_offset = h.root_offset + h.root_length;
    h.metadata_length = static_cast<uint64_t>(metadata_.size());
    h.leaf_offset = h.metadata_offset + h.metadata_length;
    h.leaf_length = leaves.size();
    h.data_offset = h.leaf_offset + h.leaf_length;
    h.data_length = data_len_;
    h.addressed_tiles = found_.size();
    h.tile_entries = entries.size();
    h.tile_contents = new_offset_.size();
    h.clustered = true;  // the source is clustered and blobs were written in its order
    h.internal_compression = pmtiles::kGzip;
    h.tile_compression = src_.tile_compression;
    h.tile_type = pmtiles::kMvt;
    h.min_zoom = 0;
    h.max_zoom = static_cast<uint8_t>(max_zoom_);
    h.min_lon_e7 = static_cast<int32_t>(box_.min_lon * 1e7);
    h.min_lat_e7 = static_cast<int32_t>(box_.min_lat * 1e7);
    h.max_lon_e7 = static_cast<int32_t>(box_.max_lon * 1e7);
    h.max_lat_e7 = static_cast<int32_t>(box_.max_lat * 1e7);
    h.center_zoom = 10;
    h.center_lon_e7 = static_cast<int32_t>((box_.min_lon + box_.max_lon) / 2 * 1e7);
    h.center_lat_e7 = static_cast<int32_t>((box_.min_lat + box_.max_lat) / 2 * 1e7);

    QFile out(path_ + QStringLiteral(".part"));
    if (!out.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        fail(QStringLiteral("can't write %1").arg(out.fileName()));
        return;
    }
    const std::string head = pmtiles::serializeHeader(h);
    out.write(head.data(), static_cast<qint64>(head.size()));
    out.write(root.data(), static_cast<qint64>(root.size()));
    out.write(metadata_);
    out.write(leaves.data(), static_cast<qint64>(leaves.size()));
    QFile in(data_->fileName());
    if (!in.open(QIODevice::ReadOnly)) {
        fail(QStringLiteral("lost the downloaded data"));
        return;
    }
    while (!in.atEnd()) {
        out.write(in.read(4 * 1024 * 1024));
    }
    in.close();
    in.remove();
    out.close();
    data_.reset();
    QFile::remove(path_);
    if (!QFile::rename(out.fileName(), path_)) {
        fail(QStringLiteral("can't move the map into place"));
        return;
    }
    running_ = false;
    emit finished(true, QStringLiteral("street map %1: %2 MB").arg(build_).arg(data_len_ / 1e6, 0, 'f', 0));
}

}  // namespace gcs::ui
