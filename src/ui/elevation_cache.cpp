#include "elevation_cache.hpp"

#include "core/tiles.hpp"
#include "offline_store.hpp"

#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QNetworkReply>
#include <QNetworkRequest>
#include <QStandardPaths>

#include <algorithm>
#include <cmath>

namespace gcs::ui {

namespace {
constexpr int kMaxPending = 8;
constexpr qint64 kRetryMs = 30000;

QString key(int z, int x, int y) {
    return QStringLiteral("%1/%2/%3").arg(z).arg(x).arg(y);
}

double sample(const QImage& img, double px, double py) {
    // bilinear between the four nearest pixel centres
    const double fx = std::clamp(px - 0.5, 0.0, img.width() - 1.0);
    const double fy = std::clamp(py - 0.5, 0.0, img.height() - 1.0);
    const int x0 = static_cast<int>(fx), y0 = static_cast<int>(fy);
    const int x1 = std::min(x0 + 1, img.width() - 1), y1 = std::min(y0 + 1, img.height() - 1);
    const double tx = fx - x0, ty = fy - y0;
    auto h = [&](int x, int y) {
        const QRgb c = img.pixel(x, y);
        return tiles::terrarium(static_cast<uint8_t>(qRed(c)), static_cast<uint8_t>(qGreen(c)),
                                static_cast<uint8_t>(qBlue(c)));
    };
    const double top = h(x0, y0) * (1 - tx) + h(x1, y0) * tx;
    const double bot = h(x0, y1) * (1 - tx) + h(x1, y1) * tx;
    return top * (1 - ty) + bot * ty;
}
}  // namespace

ElevationCache::ElevationCache(QObject* parent) : QObject(parent), memory_(64) {
    dir_ = QStandardPaths::writableLocation(QStandardPaths::CacheLocation) + QStringLiteral("/tiles/elevation");
}

QString ElevationCache::cachePath(int z, int x, int y) const {
    return QStringLiteral("%1/%2/%3/%4.png").arg(dir_).arg(z).arg(x).arg(y);
}

const QImage* ElevationCache::tile(int z, int x, int y, bool fetch) {
    const QString k = key(z, x, y);
    if (const QImage* img = memory_.object(k)) {
        return img;
    }
    const qint64 now = QDateTime::currentMSecsSinceEpoch();
    if (auto it = absent_.find(k); it != absent_.end()) {
        // looked on disk a moment ago and it wasn't there (a fetch is pending or failed)
        if (now < it.value()) {
            if (fetch) {
                request(z, x, y);
            }
            return nullptr;
        }
        absent_.erase(it);
    }
    for (const QString& path : {offline::find(offline::Elevation, z, x, y), cachePath(z, x, y)}) {
        if (path.isEmpty() || !QFileInfo::exists(path)) {
            continue;
        }
        QImage img(path);
        if (!img.isNull()) {
            // exact colours: never let a format conversion touch the encoding
            memory_.insert(k, new QImage(img.convertToFormat(QImage::Format_RGB32)));
            return memory_.object(k);
        }
    }
    if (absent_.size() > 4096) {
        absent_.clear();
    }
    absent_.insert(k, now + 3000);
    if (fetch) {
        request(z, x, y);
    }
    return nullptr;
}

std::optional<double> ElevationCache::elevation(double lat, double lon) {
    for (int z = kZoom; z >= kMinZoom; --z) {
        const tiles::TilePos t = tiles::locate(lat, lon, z);
        if (const QImage* img = tile(z, t.x, t.y, z == kZoom)) {
            const double scale = img->width() / tiles::kTilePx;  // 512 px tiles work too
            return sample(*img, t.px * scale, t.py * scale);
        }
    }
    return std::nullopt;
}

void ElevationCache::request(int z, int x, int y) {
    const QString k = key(z, x, y);
    if (pending_.contains(k) || pending_.size() >= kMaxPending) {
        return;
    }
    const qint64 now = QDateTime::currentMSecsSinceEpoch();
    if (auto it = failed_.find(k); it != failed_.end()) {
        if (now < it.value()) {
            return;
        }
        failed_.erase(it);
    }
    QNetworkRequest req(offline::url(offline::Elevation, z, x, y));
    req.setRawHeader("User-Agent", "mavlink_gcs_ui/0.1 (Qt)");
    req.setTransferTimeout(15000);
    pending_.insert(k);
    QNetworkReply* reply = nam_.get(req);
    connect(reply, &QNetworkReply::finished, this, [this, reply, k, z, x, y] {
        reply->deleteLater();
        pending_.remove(k);
        const QByteArray data = reply->readAll();
        QImage img;
        if (reply->error() != QNetworkReply::NoError || !img.loadFromData(data)) {
            failed_.insert(k, QDateTime::currentMSecsSinceEpoch() + kRetryMs);
            return;
        }
        const QString path = cachePath(z, x, y);
        QDir().mkpath(QFileInfo(path).absolutePath());
        QFile f(path);
        if (f.open(QIODevice::WriteOnly)) {
            f.write(data);
        }
        memory_.insert(k, new QImage(img.convertToFormat(QImage::Format_RGB32)));
        emit ready();
    });
}

}  // namespace gcs::ui
