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
#include <QSet>

#include <algorithm>
#include <cmath>

namespace gcs::ui {

namespace {
// typical tile sizes, for estimates only
constexpr qint64 kSatBytes = 22000;
constexpr qint64 kElevBytes = 60000;
constexpr int kCountrySat0 = 8, kCountrySat1 = 14, kCountryElev = 12;
constexpr int kAreaSat0 = 15, kAreaElev = 13;  // the fine zooms: what "clear full detail" removes by folder
constexpr size_t kMaxAreaTiles = 1000000;      // ~22 GB: a level above this can't be picked
constexpr qint64 kPreferBytes = 2000LL * 1000 * 1000;
constexpr size_t kMaxDiskCheck = 200000;       // more tiles than this: estimate instead of a disk walk
constexpr int kWorldMaxZoom = 6;
constexpr int kStreetMaxZoom = 15;
constexpr qint64 kWorldBytes = 45LL * 1000 * 1000;  // measured: planet z0-6, build 20261121
// an area's street map z0-15, rough. Measured: 1 MB / 100 km² and 4 MB / 900 km² of
// Sicily, 7 MB / 45 km² of central Rome, 56 MB / 33,000 km² for Cyprus (much of it sea)
qint64 streetBytes(const OfflineDownloader::Box& b) {
    return static_cast<qint64>(b.widthKm() * b.heightKm() * 3000) + 2000000;
}

QString areasFile() {
    return offline::folder() + QStringLiteral("/areas.json");
}

// which street map build is installed (written next to map.pmtiles)
QString buildFile() {
    return offline::folder() + QStringLiteral("/map.build");
}

QString countryFile() {
    return offline::folder() + QStringLiteral("/map.pmtiles");
}

QString worldFile() {
    return offline::folder() + QStringLiteral("/world.pmtiles");
}

QString worldBuildFile() {
    return offline::folder() + QStringLiteral("/world.build");
}

// one street map per full-detail area
QString streetsDir() {
    return offline::folder() + QStringLiteral("/streets");
}

bool inside(const OfflineDownloader::Box& in, const OfflineDownloader::Box& out) {
    return in.min_lon >= out.min_lon && in.max_lon <= out.max_lon && in.min_lat >= out.min_lat &&
           in.max_lat <= out.max_lat;
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

QString OfflineDownloader::areaStreetsPath(const Box& b) {
    return streetsDir() + QStringLiteral("/%1_%2_%3_%4.pmtiles")
                              .arg(b.min_lon, 0, 'f', 4)
                              .arg(b.min_lat, 0, 'f', 4)
                              .arg(b.max_lon, 0, 'f', 4)
                              .arg(b.max_lat, 0, 'f', 4);
}

// an area's own street map: not when the country map already has it all
bool OfflineDownloader::needsAreaStreets(const Box& b) {
    if (hasStreetMap() && inside(b, country())) return false;
    return !QFileInfo::exists(areaStreetsPath(b));
}

OfflineDownloader::StreetJob OfflineDownloader::areaStreetJob(const Box& b) {
    return {b, kStreetMaxZoom, areaStreetsPath(b), {}, {}, QStringLiteral("Area street map"), false};
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
    if (!hasWorldMap()) {
        c.bytes += kWorldBytes;
    }
    return c;
}

OfflineDownloader::Zooms OfflineDownloader::zooms(Detail d) {
    switch (d) {
        case Detail::Streets: return {1, 0, 12};
        case Detail::Sat10m: return {8, 14, 12};
        case Detail::Sat2m: return {8, 16, 13};
        case Detail::Sat1m: return {8, 17, 13};
        case Detail::Sat05m: return {8, 18, 13};
    }
    return {};
}

QString OfflineDownloader::detailLabel(Detail d) {
    switch (d) {
        case Detail::Streets: return QStringLiteral("STREETS");
        case Detail::Sat10m: return QStringLiteral("10 m");
        case Detail::Sat2m: return QStringLiteral("2 m");
        case Detail::Sat1m: return QStringLiteral("1 m");
        case Detail::Sat05m: return QStringLiteral("0.5 m");
    }
    return {};
}

QString OfflineDownloader::detailText(Detail d) {
    switch (d) {
        case Detail::Streets: return QStringLiteral("no satellite");
        case Detail::Sat10m: return QStringLiteral("satellite to ~10 m/px");
        case Detail::Sat2m: return QStringLiteral("satellite to ~2 m/px");
        case Detail::Sat1m: return QStringLiteral("satellite to ~1 m/px");
        case Detail::Sat05m: return QStringLiteral("satellite to ~0.5 m/px");
    }
    return {};
}

std::vector<OfflineDownloader::Level> OfflineDownloader::areaLevels(const Box& b) {
    std::vector<Level> out;
    const qint64 streets = needsAreaStreets(b)
                               ? streetBytes(b)
                               : 0;
    for (int i = 0; i < kDetails; ++i) {
        const Detail d = static_cast<Detail>(i);
        const Zooms z = zooms(d);
        Level l{d};
        const size_t elev = tiles::box(b.min_lon, b.min_lat, b.max_lon, b.max_lat, z.elev).count();
        size_t sat = 0;
        for (int zz = z.sat_min; zz <= z.sat_max; ++zz) {
            sat += tiles::box(b.min_lon, b.min_lat, b.max_lon, b.max_lat, zz).count();
        }
        l.tiles = sat + elev;
        l.bytes = static_cast<qint64>(sat) * kSatBytes + static_cast<qint64>(elev) * kElevBytes + streets;
        l.allowed = l.tiles <= kMaxAreaTiles;
        out.push_back(l);
    }
    return out;
}

OfflineDownloader::Detail OfflineDownloader::preferredDetail(const std::vector<Level>& levels) {
    Detail best = Detail::Streets;
    for (const Level& l : levels) {
        if (l.allowed && l.bytes <= kPreferBytes) best = l.detail;
    }
    return best;
}

std::vector<OfflineDownloader::Job> OfflineDownloader::areaJobs(const Area& a, bool fresh_only) {
    auto out = jobs(a.box, offline::Elevation, a.z.elev, a.z.elev, fresh_only);  // small, and hover needs it
    if (a.z.hasSat()) {
        auto sat = jobs(a.box, offline::Satellite, a.z.sat_min, a.z.sat_max, fresh_only);  // coarse first: usable early
        out.insert(out.end(), sat.begin(), sat.end());
    }
    return out;
}

OfflineDownloader::Count OfflineDownloader::areaCount(const Box& b, Detail d) {
    Count c;
    const auto levels = areaLevels(b);
    const Level& l = levels[static_cast<size_t>(d)];
    c.streets = needsAreaStreets(b);
    if (l.tiles > kMaxDiskCheck) {
        // too many to look for on disk while the finger is up: all of them, roughly
        c.exact = false;
        c.fresh = l.tiles;
        c.bytes = l.bytes;
        return c;
    }
    for (const auto& j : areaJobs({b, zooms(d)}, false)) {
        const bool sat = j.layer == offline::Satellite;
        present(j.layer, j.z, j.x, j.y) ? ++c.have : (++c.fresh, c.bytes += sat ? kSatBytes : kElevBytes);
    }
    if (c.streets) {
        c.bytes += streetBytes(b);
    }
    return c;
}

std::vector<OfflineDownloader::Area> OfflineDownloader::areas() {
    std::vector<Area> out;
    QFile f(areasFile());
    if (!f.open(QIODevice::ReadOnly)) return out;
    for (const auto& v : QJsonDocument::fromJson(f.readAll()).array()) {
        const QJsonObject o = v.toObject();
        Area a;
        a.box = {o.value(QStringLiteral("min_lon")).toDouble(), o.value(QStringLiteral("min_lat")).toDouble(),
                 o.value(QStringLiteral("max_lon")).toDouble(), o.value(QStringLiteral("max_lat")).toDouble()};
        if (o.contains(QStringLiteral("sat_max"))) {
            a.z.sat_max = o.value(QStringLiteral("sat_max")).toInt();
            a.z.sat_min = a.z.sat_max > 0 ? o.value(QStringLiteral("sat_min")).toInt(8) : 1;
            a.z.elev = o.value(QStringLiteral("elev")).toInt(12);
        } else {
            a.z = {kAreaSat0, 18, kAreaElev};  // before areas had a depth: what was fetched then
        }
        out.push_back(a);
    }
    return out;
}

std::vector<QString> OfflineDownloader::areaCoarseFiles() {
    std::vector<QString> out;
    QSet<QString> seen;
    const Box c = country();
    for (const Area& a : areas()) {
        auto add = [&](int layer, int z) {
            const tiles::Range in = tiles::box(c.min_lon, c.min_lat, c.max_lon, c.max_lat, z);
            for (const Job& j : jobs(a.box, layer, z, z, false)) {
                if (j.x >= in.x0 && j.x <= in.x1 && j.y >= in.y0 && j.y <= in.y1) continue;  // the country's own
                for (const QString& p : {offline::find(layer, z, j.x, j.y),
                                         offline::noDataPathFor(offline::folder(), layer, z, j.x, j.y)}) {
                    if (!p.isEmpty() && !seen.contains(p) && QFileInfo::exists(p)) {
                        seen.insert(p);
                        out.push_back(p);
                    }
                }
            }
        };
        if (a.z.hasSat()) {
            for (int z = a.z.sat_min; z <= std::min(a.z.sat_max, kAreaSat0 - 1); ++z) add(offline::Satellite, z);
        }
        if (a.z.elev < kAreaElev) add(offline::Elevation, a.z.elev);
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
    sum += dirSize(streetsDir());
    for (const QString& p : areaCoarseFiles()) sum += QFileInfo(p).size();
    return sum;
}

bool OfflineDownloader::clearFullDetail() {
    bool ok = true;
    for (const QString& p : areaCoarseFiles()) ok = QFile::remove(p) && ok;  // before areas.json goes
    for (int z = kAreaSat0; z <= 19; ++z) {
        QDir d(QStringLiteral("%1/%2/%3").arg(offline::folder(), offline::subdir(offline::Satellite)).arg(z));
        if (d.exists()) ok = d.removeRecursively() && ok;
    }
    QDir streets(streetsDir());
    if (streets.exists()) ok = streets.removeRecursively() && ok;
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
    for (const QString& f :
         dir.entryList({QStringLiteral("*.pmtiles"), QStringLiteral("*.part"), QStringLiteral("*.build")}, QDir::Files)) {
        ok = QFile::remove(dir.filePath(f)) && ok;
    }
    QDir streets(streetsDir());
    if (streets.exists()) ok = streets.removeRecursively() && ok;
    QFile::remove(areasFile());
    return ok;
}

QString OfflineDownloader::installedBuild() {
    if (!hasStreetMap()) return {};
    QFile f(buildFile());
    return f.open(QIODevice::ReadOnly) ? QString::fromUtf8(f.readAll()).trimmed() : QString();
}

std::vector<OfflineDownloader::Job> OfflineDownloader::missing() {
    std::vector<Job> out;
    QSet<QString> seen;  // overlapping areas: each tile once
    auto add = [&](const std::vector<Job>& js) {
        for (const Job& j : js) {
            const QString k = QStringLiteral("%1/%2/%3/%4").arg(j.layer).arg(j.z).arg(j.x).arg(j.y);
            if (!seen.contains(k)) {
                seen.insert(k);
                out.push_back(j);
            }
        }
    };
    const Box c = country();
    add(jobs(c, offline::Elevation, kCountryElev, kCountryElev, true));  // small, and hover needs it: first
    add(jobs(c, offline::Satellite, kCountrySat0, kCountrySat1, true));
    for (const Area& a : areas()) add(areaJobs(a, true));
    return out;
}

OfflineDownloader::Count OfflineDownloader::areasMissing() {
    Count c;
    QSet<QString> seen;
    for (const Area& a : areas()) {
        for (const Job& j : areaJobs(a, true)) {
            const QString k = QStringLiteral("%1/%2/%3/%4").arg(j.layer).arg(j.z).arg(j.x).arg(j.y);
            if (seen.contains(k)) continue;
            seen.insert(k);
            ++c.fresh;
            c.bytes += j.layer == offline::Satellite ? kSatBytes : kElevBytes;
        }
    }
    return c;
}

bool OfflineDownloader::hasStreetMap() {
    return QFileInfo::exists(countryFile());
}

bool OfflineDownloader::hasWorldMap() {
    return QFileInfo::exists(worldFile());
}

OfflineDownloader::OfflineDownloader(QObject* parent) : QObject(parent) {
    connect(&extract_, &PmtilesExtract::progress, this, &OfflineDownloader::progress);
    connect(&extract_, &PmtilesExtract::finished, this, [this](bool ok, const QString& msg) {
        if (cancelled_) {
            finish(true, QStringLiteral("cancelled"));
            return;
        }
        if (!ok) {
            if (street_.required) {
                finish(false, QStringLiteral("street map failed: %1 (try again)").arg(msg));
                return;
            }
            street_failed_ << street_.what.toLower();  // reported at the end; the next run retries it
        } else {
            if (street_.required) street_msg_ = msg;
            if (!extract_.upToDate()) {
                if (!street_.build_file.isEmpty()) {
                    QFile f(street_.build_file);
                    if (f.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
                        f.write(extract_.build().toUtf8());
                    }
                }
                emit streetMapInstalled(street_.what);
            }
        }
        nextStreet();
    });
}

void OfflineDownloader::nextStreet() {
    if (!streets_.empty()) {
        street_ = streets_.front();
        streets_.pop_front();
        const Box& b = street_.box;
        extract_.start({b.min_lon, b.min_lat, b.max_lon, b.max_lat}, street_.max_zoom, street_.path, street_.installed);
        emit progress();
        return;
    }
    // then the raster tiles
    queue_.clear();
    if (area_) {
        for (const Job& j : areaJobs({area_box_, area_z_}, true)) queue_.push_back(j);
        startTiles(QStringLiteral("area"));
    } else {
        // whatever is missing: the country's satellite and ground height, and
        // the tiles of every full-detail area (each tile once)
        for (const Job& j : missing()) queue_.push_back(j);
        startTiles(QStringLiteral("missing satellite + ground height"));
    }
}

void OfflineDownloader::startCountry() {
    if (running_) return;
    running_ = true;
    cancelled_ = false;
    area_ = false;
    failed_ = 0;
    done_ = total_ = 0;
    bytes_ = 0;
    street_msg_.clear();
    street_failed_.clear();
    streets_.clear();
    // the world map once (it's coarse: a newer build changes nothing you'd see)
    if (!hasWorldMap()) {
        streets_.push_back({{-180.0, -85.0511, 180.0, 85.0511}, kWorldMaxZoom, worldFile(), worldBuildFile(), {},
                            QStringLiteral("World street map"), false});
    }
    // the country map only when there's a newer build than the installed one
    streets_.push_back({country(), kStreetMaxZoom, countryFile(), buildFile(), installedBuild(),
                        QStringLiteral("Street map"), true});
    for (const Area& a : areas()) {
        if (needsAreaStreets(a.box)) streets_.push_back(areaStreetJob(a.box));
    }
    nextStreet();
}

void OfflineDownloader::startArea(const Box& b, Detail d) {
    if (running_ || !b.valid()) return;
    running_ = true;
    cancelled_ = false;
    area_ = true;
    area_box_ = b;
    area_z_ = zooms(d);
    failed_ = 0;
    done_ = total_ = 0;
    bytes_ = 0;
    street_msg_.clear();
    street_failed_.clear();
    streets_.clear();
    if (needsAreaStreets(b)) streets_.push_back(areaStreetJob(b));
    nextStreet();
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
    if (extract_.running()) {
        // which street map: the country's says nothing more, as before
        return street_.required ? extract_.stage() : extract_.stage() + QStringLiteral(" · ") + street_.what.toLower();
    }
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
        if (area_) {
            // recorded even if some tiles failed: CHECK & COMPLETE fetches them later
            // remember the area (outline on the map, listed in Settings)
            QJsonArray arr;
            QFile f(areasFile());
            if (f.open(QIODevice::ReadOnly)) {
                arr = QJsonDocument::fromJson(f.readAll()).array();
                f.close();
            }
            // drawing the same area again: one entry, as deep as the deepest download
            Zooms z = area_z_;
            for (int i = 0; i < arr.size(); ++i) {
                const QJsonObject e = arr[i].toObject();
                if (e.value(QStringLiteral("min_lon")).toDouble() == area_box_.min_lon &&
                    e.value(QStringLiteral("min_lat")).toDouble() == area_box_.min_lat &&
                    e.value(QStringLiteral("max_lon")).toDouble() == area_box_.max_lon &&
                    e.value(QStringLiteral("max_lat")).toDouble() == area_box_.max_lat) {
                    for (const Area& old : areas()) {
                        if (old.box.min_lon == area_box_.min_lon && old.box.min_lat == area_box_.min_lat &&
                            old.box.max_lon == area_box_.max_lon && old.box.max_lat == area_box_.max_lat &&
                            old.z.hasSat()) {
                            z.sat_min = z.hasSat() ? std::min(z.sat_min, old.z.sat_min) : old.z.sat_min;
                            z.sat_max = std::max(z.sat_max, old.z.sat_max);
                            z.elev = std::max(z.elev, old.z.elev);
                        }
                    }
                    arr.removeAt(i--);
                }
            }
            QJsonObject o;
            o[QStringLiteral("min_lon")] = area_box_.min_lon;
            o[QStringLiteral("min_lat")] = area_box_.min_lat;
            o[QStringLiteral("max_lon")] = area_box_.max_lon;
            o[QStringLiteral("max_lat")] = area_box_.max_lat;
            o[QStringLiteral("sat_min")] = z.hasSat() ? z.sat_min : 0;
            o[QStringLiteral("sat_max")] = z.hasSat() ? z.sat_max : 0;
            o[QStringLiteral("elev")] = z.elev;
            o[QStringLiteral("date")] = QDate::currentDate().toString(Qt::ISODate);
            arr.append(o);
            QDir().mkpath(offline::folder());
            if (f.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
                f.write(QJsonDocument(arr).toJson());
            }
        }
        QString summary = failed_ ? QStringLiteral("%1 tiles failed: press CHECK & COMPLETE to retry them").arg(failed_)
                          : total_ == 0 ? QStringLiteral("all tiles were already there")
                                        : QStringLiteral("%1 MB of tiles fetched").arg(bytes_ / 1e6, 0, 'f', 0);
        if (!street_msg_.isEmpty()) summary = street_msg_ + QStringLiteral(" · ") + summary;
        if (!street_failed_.isEmpty()) {
            summary = street_failed_.join(QStringLiteral(", ")) + QStringLiteral(" failed (") +
                      (area_ ? QStringLiteral("CHECK & COMPLETE retries it") : QStringLiteral("try again")) +
                      QStringLiteral(") · ") + summary;
        }
        finish(false, summary);
    }
}

}  // namespace gcs::ui
