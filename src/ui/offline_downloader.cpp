#include "offline_downloader.hpp"

#include "core/geo.hpp"
#include "core/tiles.hpp"
#include "offline_store.hpp"

#include <QDate>
#include <QDir>
#include <QDirIterator>
#include <QFile>
#include <QFileInfo>
#include <QImage>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QNetworkReply>
#include <QNetworkRequest>

#include <cmath>

namespace gcs::ui {

namespace {
// typical tile sizes, for estimates only
constexpr qint64 kSatBytes = 22000;
constexpr qint64 kElevBytes = 60000;
constexpr int kCountrySat0 = 8, kCountrySat1 = 14, kCountryElev = 12;
constexpr int kAreaSat0 = 15, kAreaSat1 = 18, kAreaElev = 13;

QString areasFile() {
    return offline::folder() + QStringLiteral("/areas.json");
}

bool present(int layer, int z, int x, int y) {
    return !offline::find(layer, z, x, y).isEmpty() ||
           QFileInfo::exists(offline::noDataPathFor(offline::folder(), layer, z, x, y));
}

qint64 dirSize(const QString& path) {
    qint64 sum = 0;
    QDirIterator it(path, QDir::Files, QDirIterator::Subdirectories);
    while (it.hasNext()) {
        it.next();
        sum += it.fileInfo().size();
    }
    return sum;
}
}  // namespace

double OfflineDownloader::Box::widthKm() const {
    return geo::distance((min_lat + max_lat) / 2, min_lon, (min_lat + max_lat) / 2, max_lon) / 1000.0;
}

double OfflineDownloader::Box::heightKm() const {
    return geo::distance(min_lat, min_lon, max_lat, min_lon) / 1000.0;
}

OfflineDownloader::Box OfflineDownloader::country() {
    return {33.7, 29.0, 36.5, 33.8};
}

std::vector<OfflineDownloader::Job> OfflineDownloader::jobs(const Box& b, int layer, int z0, int z1, bool fresh_only) {
    std::vector<Job> out;
    for (int z = z0; z <= z1; ++z) {
        const tiles::TilePos a = tiles::locate(b.max_lat, b.min_lon, z);
        const tiles::TilePos c = tiles::locate(b.min_lat, b.max_lon, z);
        for (int x = a.x; x <= c.x; ++x) {
            for (int y = a.y; y <= c.y; ++y) {
                if (!fresh_only || !present(layer, z, x, y)) {
                    out.push_back({layer, z, x, y});
                }
            }
        }
    }
    return out;
}

OfflineDownloader::Count OfflineDownloader::countryCount() {
    Count c;
    const Box b = country();
    const auto all_sat = jobs(b, offline::Satellite, kCountrySat0, kCountrySat1, false);
    const auto all_elev = jobs(b, offline::Elevation, kCountryElev, kCountryElev, false);
    for (const auto& j : all_sat) {
        present(j.layer, j.z, j.x, j.y) ? ++c.have : (++c.fresh, c.bytes += kSatBytes);
    }
    for (const auto& j : all_elev) {
        present(j.layer, j.z, j.x, j.y) ? ++c.have : (++c.fresh, c.bytes += kElevBytes);
    }
    if (!hasStreetMap()) {
        c.bytes += 240LL * 1000 * 1000;  // measured: Israel + 50 km, z0-15
    }
    return c;
}

OfflineDownloader::Count OfflineDownloader::areaCount(const Box& b) {
    Count c;
    for (const auto& j : jobs(b, offline::Satellite, kAreaSat0, kAreaSat1, false)) {
        present(j.layer, j.z, j.x, j.y) ? ++c.have : (++c.fresh, c.bytes += kSatBytes);
    }
    for (const auto& j : jobs(b, offline::Elevation, kAreaElev, kAreaElev, false)) {
        present(j.layer, j.z, j.x, j.y) ? ++c.have : (++c.fresh, c.bytes += kElevBytes);
    }
    return c;
}

std::vector<OfflineDownloader::Box> OfflineDownloader::areas() {
    std::vector<Box> out;
    QFile f(areasFile());
    if (!f.open(QIODevice::ReadOnly)) return out;
    for (const auto& v : QJsonDocument::fromJson(f.readAll()).array()) {
        const QJsonObject o = v.toObject();
        out.push_back({o.value(QStringLiteral("min_lon")).toDouble(), o.value(QStringLiteral("min_lat")).toDouble(),
                       o.value(QStringLiteral("max_lon")).toDouble(), o.value(QStringLiteral("max_lat")).toDouble()});
    }
    return out;
}

qint64 OfflineDownloader::diskUsage() {
    return dirSize(offline::folder());
}

qint64 OfflineDownloader::fullDetailUsage() {
    qint64 sum = 0;
    for (int z = kAreaSat0; z <= 19; ++z) {
        sum += dirSize(QStringLiteral("%1/%2/%3").arg(offline::folder(), offline::subdir(offline::Satellite)).arg(z));
    }
    sum += dirSize(QStringLiteral("%1/%2/%3").arg(offline::folder(), offline::subdir(offline::Elevation)).arg(kAreaElev));
    return sum;
}

bool OfflineDownloader::clearFullDetail() {
    bool ok = true;
    for (int z = kAreaSat0; z <= 19; ++z) {
        QDir d(QStringLiteral("%1/%2/%3").arg(offline::folder(), offline::subdir(offline::Satellite)).arg(z));
        if (d.exists()) ok = d.removeRecursively() && ok;
    }
    QDir e(QStringLiteral("%1/%2/%3").arg(offline::folder(), offline::subdir(offline::Elevation)).arg(kAreaElev));
    if (e.exists()) ok = e.removeRecursively() && ok;
    QFile::remove(areasFile());
    return ok;
}

bool OfflineDownloader::clearAll() {
    bool ok = true;
    for (int layer = 0; layer < offline::kLayers; ++layer) {
        QDir d(offline::folder() + QLatin1Char('/') + offline::subdir(layer));
        if (d.exists()) ok = d.removeRecursively() && ok;
    }
    const QDir dir(offline::folder());
    for (const QString& f : dir.entryList({QStringLiteral("*.pmtiles"), QStringLiteral("*.part")}, QDir::Files)) {
        ok = QFile::remove(dir.filePath(f)) && ok;
    }
    QFile::remove(areasFile());
    return ok;
}

bool OfflineDownloader::hasStreetMap() {
    return !QDir(offline::folder()).entryList({QStringLiteral("*.pmtiles")}, QDir::Files).isEmpty();
}

OfflineDownloader::OfflineDownloader(QObject* parent) : QObject(parent) {
    connect(&extract_, &PmtilesExtract::progress, this, &OfflineDownloader::progress);
    connect(&extract_, &PmtilesExtract::finished, this, [this](bool ok, const QString& msg) {
        if (cancelled_) {
            finish(true, QStringLiteral("cancelled"));
            return;
        }
        if (!ok) {
            finish(false, QStringLiteral("street map failed: %1 (download again to retry)").arg(msg));
            return;
        }
        emit streetMapInstalled();
        // then the country's satellite and elevation
        const Box b = country();
        auto sat = jobs(b, offline::Satellite, kCountrySat0, kCountrySat1, true);
        auto elev = jobs(b, offline::Elevation, kCountryElev, kCountryElev, true);
        queue_.assign(elev.begin(), elev.end());  // elevation first: small, and hover needs it
        queue_.insert(queue_.end(), sat.begin(), sat.end());
        startTiles(QStringLiteral("satellite + ground height"));
    });
}

void OfflineDownloader::startCountry() {
    if (running_) return;
    running_ = true;
    cancelled_ = false;
    area_ = false;
    failed_ = 0;
    done_ = total_ = 0;
    bytes_ = 0;
    const Box b = country();
    // always fetch the street map fresh (it's the latest OSM data)
    extract_.start({b.min_lon, b.min_lat, b.max_lon, b.max_lat}, 15, offline::folder() + QStringLiteral("/map.pmtiles"));
    emit progress();
}

void OfflineDownloader::startArea(const Box& b) {
    if (running_ || !b.valid()) return;
    running_ = true;
    cancelled_ = false;
    area_ = true;
    area_box_ = b;
    failed_ = 0;
    bytes_ = 0;
    auto elev = jobs(b, offline::Elevation, kAreaElev, kAreaElev, true);
    auto sat = jobs(b, offline::Satellite, kAreaSat0, kAreaSat1, true);
    queue_.assign(elev.begin(), elev.end());
    queue_.insert(queue_.end(), sat.begin(), sat.end());  // coarse zooms first: usable early
    startTiles(QStringLiteral("full-detail area"));
}

void OfflineDownloader::cancel() {
    if (!running_) return;
    cancelled_ = true;
    if (extract_.running()) {
        extract_.cancel();
        return;
    }
    queue_.clear();
    if (in_flight_ == 0) {
        finish(true, QStringLiteral("cancelled · downloading again resumes"));
    }
}

QString OfflineDownloader::stage() const {
    if (extract_.running()) return extract_.stage();
    return stage_;
}

double OfflineDownloader::fraction() const {
    if (extract_.running()) {
        return extract_.total() > 0 ? static_cast<double>(extract_.done()) / extract_.total() : 0.0;
    }
    return total_ > 0 ? static_cast<double>(done_) / total_ : 0.0;
}

void OfflineDownloader::startTiles(const QString& stage) {
    stage_ = stage;
    done_ = 0;
    total_ = queue_.size();
    emit progress();
    pump();
}

void OfflineDownloader::finish(bool cancelled, const QString& summary) {
    running_ = false;
    queue_.clear();
    emit finished(cancelled, summary);
}

void OfflineDownloader::pump() {
    while (!cancelled_ && in_flight_ < kParallel && !queue_.empty()) {
        const Job j = queue_.front();
        queue_.pop_front();
        if (present(j.layer, j.z, j.x, j.y)) {
            ++done_;
            continue;
        }
        QNetworkRequest req(offline::url(j.layer, j.z, j.x, j.y));
        req.setRawHeader("User-Agent", "mavlink_gcs_ui/0.1 (Qt; offline area)");
        req.setTransferTimeout(20000);
        ++in_flight_;
        QNetworkReply* reply = nam_.get(req);
        connect(reply, &QNetworkReply::finished, this, [this, reply, j] {
            reply->deleteLater();
            --in_flight_;
            const QByteArray data = reply->readAll();
            QImage probe;
            if (reply->error() == QNetworkReply::NoError && probe.loadFromData(data)) {
                QString path = offline::pathFor(j.layer, j.z, j.x, j.y);
                QByteArray body = data;
                if (j.layer == offline::Satellite && offline::isNoDataTile(data)) {
                    path = offline::noDataPathFor(offline::folder(), j.layer, j.z, j.x, j.y);
                    body.clear();  // "no imagery this deep": a marker, never fetched again
                }
                QDir().mkpath(QFileInfo(path).absolutePath());
                QFile f(path + QStringLiteral(".part"));
                if (f.open(QIODevice::WriteOnly) && f.write(body) == body.size()) {
                    f.close();
                    QFile::remove(path);
                    f.rename(path);
                    bytes_ += body.size();
                } else {
                    ++failed_;
                }
            } else {
                ++failed_;
            }
            ++done_;
            emit progress();
            pump();
        });
    }
    if (in_flight_ == 0 && (queue_.empty() || cancelled_)) {
        if (cancelled_) {
            finish(true, QStringLiteral("cancelled · downloading again resumes"));
            return;
        }
        if (area_ && failed_ == 0) {
            // remember the area (outline on the map, listed in Settings)
            QJsonArray arr;
            QFile f(areasFile());
            if (f.open(QIODevice::ReadOnly)) {
                arr = QJsonDocument::fromJson(f.readAll()).array();
                f.close();
            }
            QJsonObject o;
            o[QStringLiteral("min_lon")] = area_box_.min_lon;
            o[QStringLiteral("min_lat")] = area_box_.min_lat;
            o[QStringLiteral("max_lon")] = area_box_.max_lon;
            o[QStringLiteral("max_lat")] = area_box_.max_lat;
            o[QStringLiteral("date")] = QDate::currentDate().toString(Qt::ISODate);
            arr.append(o);
            QDir().mkpath(offline::folder());
            if (f.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
                f.write(QJsonDocument(arr).toJson());
            }
        }
        finish(false, failed_ ? QStringLiteral("done · %1 tiles failed (download again to retry)").arg(failed_)
                              : QStringLiteral("done · %1 MB new").arg(bytes_ / 1e6, 0, 'f', 0));
    }
}

}  // namespace gcs::ui
