#include "offline_downloader.hpp"

#include "core/tiles.hpp"
#include "offline_store.hpp"

#include <QDir>
#include <QDirIterator>
#include <QFile>
#include <QFileInfo>
#include <QImage>
#include <QNetworkReply>
#include <QNetworkRequest>

#include <cmath>

namespace gcs::ui {

namespace {
// typical tile sizes, for the estimate only
constexpr qint64 kBytes[3] = {16000, 28000, 110000};
constexpr int kMapMinZoom = 8;
}  // namespace

int OfflineDownloader::mapMaxZoom(double radius_km) {
    return radius_km <= 12.0 ? 16 : radius_km <= 30.0 ? 15 : 14;
}

int OfflineDownloader::elevationZoom(double radius_km) {
    return radius_km <= 30.0 ? 13 : 12;
}

OfflineDownloader::Estimate OfflineDownloader::estimate(const Area& a) {
    Estimate e;
    e.map_max_zoom = mapMaxZoom(a.radius_km);
    e.elev_zoom = elevationZoom(a.radius_km);
    const double r = a.radius_km * 1000.0;
    for (int layer = 0; layer < 3; ++layer) {
        if (!a.layers[layer]) {
            continue;
        }
        if (layer == offline::Elevation) {
            e.tiles[layer] = tiles::around(a.lat, a.lon, r, e.elev_zoom).count();
        } else {
            for (int z = kMapMinZoom; z <= e.map_max_zoom; ++z) {
                e.tiles[layer] += tiles::around(a.lat, a.lon, r, z).count();
            }
        }
        e.total += e.tiles[layer];
        e.bytes += static_cast<qint64>(e.tiles[layer]) * kBytes[layer];
    }
    return e;
}

qint64 OfflineDownloader::diskUsage() {
    qint64 sum = 0;
    for (int layer = 0; layer < offline::kLayers; ++layer) {
        QDirIterator it(offline::folder() + QLatin1Char('/') + offline::subdir(layer), QDir::Files,
                        QDirIterator::Subdirectories);
        while (it.hasNext()) {
            it.next();
            sum += it.fileInfo().size();
        }
    }
    return sum;
}

bool OfflineDownloader::clear() {
    bool ok = true;
    for (int layer = 0; layer < offline::kLayers; ++layer) {
        QDir d(offline::folder() + QLatin1Char('/') + offline::subdir(layer));
        if (d.exists()) {
            ok = d.removeRecursively() && ok;
        }
    }
    return ok;
}

OfflineDownloader::OfflineDownloader(QObject* parent) : QObject(parent) {}

void OfflineDownloader::start(const Area& a) {
    if (running_) {
        return;
    }
    queue_.clear();
    done_ = failed_ = 0;
    bytes_ = 0;
    const Estimate e = estimate(a);
    const double r = a.radius_km * 1000.0;
    // coarse zooms first: a cancelled download still leaves a usable overview
    auto add = [&](int layer, int z) {
        const tiles::Range rg = tiles::around(a.lat, a.lon, r, z);
        for (int x = rg.x0; x <= rg.x1; ++x) {
            for (int y = rg.y0; y <= rg.y1; ++y) {
                queue_.push_back({layer, z, x, y});
            }
        }
    };
    if (a.layers[offline::Elevation]) {
        add(offline::Elevation, e.elev_zoom);
    }
    for (int z = kMapMinZoom; z <= e.map_max_zoom; ++z) {
        for (int layer : {offline::Satellite, offline::Streets}) {
            if (a.layers[layer]) {
                add(layer, z);
            }
        }
    }
    total_ = queue_.size();
    running_ = true;
    emit progress();
    pump();
}

void OfflineDownloader::cancel() {
    if (!running_) {
        return;
    }
    queue_.clear();
    if (in_flight_ == 0) {
        finish(true);
    } else {
        running_ = false;  // replies still arriving are written, then it reports
        emit finished(true);
    }
}

void OfflineDownloader::finish(bool cancelled) {
    running_ = false;
    emit finished(cancelled);
}

void OfflineDownloader::pump() {
    while (running_ && in_flight_ < kParallel && !queue_.empty()) {
        const Job j = queue_.front();
        queue_.pop_front();
        if (!offline::find(j.layer, j.z, j.x, j.y).isEmpty()) {
            ++done_;  // already there: resume
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
                const QString path = offline::pathFor(j.layer, j.z, j.x, j.y);
                QDir().mkpath(QFileInfo(path).absolutePath());
                QFile f(path);
                if (f.open(QIODevice::WriteOnly) && f.write(data) == data.size()) {
                    bytes_ += data.size();
                } else {
                    ++failed_;
                }
            } else {
                ++failed_;
            }
            ++done_;
            if (running_) {
                emit progress();
                if (queue_.empty() && in_flight_ == 0) {
                    finish(false);
                } else {
                    pump();
                }
            }
        });
    }
    if (running_ && queue_.empty() && in_flight_ == 0) {
        finish(false);
    } else if (running_) {
        emit progress();
    }
}

}  // namespace gcs::ui
