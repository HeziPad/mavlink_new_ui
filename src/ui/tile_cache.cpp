#include "tile_cache.hpp"

#include "offline_store.hpp"

#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QImage>
#include <QNetworkReply>
#include <QNetworkRequest>
#include <QStandardPaths>

#include <algorithm>

namespace gcs::ui {

namespace {
constexpr int kMaxPending = 48;
constexpr qint64 kRetryMs = 30000;
}  // namespace

TileCache::TileCache(QObject* parent) : QObject(parent), memory_(600) {
    dir_ = QStandardPaths::writableLocation(QStandardPaths::CacheLocation) + QStringLiteral("/tiles");
}

int TileCache::maxZoom(int layer) {
    return layer == Satellite ? 19 : 19;
}

QString TileCache::layerName(int layer) {
    return layer == Satellite ? QStringLiteral("SAT") : QStringLiteral("MAP");
}

QString TileCache::key(int layer, int z, int x, int y) {
    return QStringLiteral("%1/%2/%3/%4").arg(layer).arg(z).arg(x).arg(y);
}

QString TileCache::diskPath(int layer, int z, int x, int y) const {
    const QString sub = layer == Satellite ? QStringLiteral("esri_imagery") : QStringLiteral("osm");
    return QStringLiteral("%1/%2/%3/%4/%5.img").arg(dir_, sub).arg(z).arg(x).arg(y);
}

const QPixmap* TileCache::peek(int layer, int z, int x, int y) {
    return memory_.object(key(layer, z, x, y));
}

const QPixmap* TileCache::tile(int layer, int z, int x, int y) {
    const QString k = key(layer, z, x, y);
    if (const QPixmap* pm = memory_.object(k)) {
        return pm;
    }
    // the offline folder first (downloaded areas, hand-installed tiles), then
    // the browsing cache
    for (const QString& path : {offline::find(layer, z, x, y), diskPath(layer, z, x, y)}) {
        if (path.isEmpty() || !QFileInfo::exists(path)) {
            continue;
        }
        QImage img(path);
        if (!img.isNull()) {
            memory_.insert(k, new QPixmap(process(img, layer)));
            return memory_.object(k);
        }
    }
    request(layer, z, x, y);
    return nullptr;
}

void TileCache::request(int layer, int z, int x, int y) {
    const QString k = key(layer, z, x, y);
    if (pending_.contains(k) || pending_.size() >= kMaxPending) {
        return;
    }
    const qint64 now = QDateTime::currentMSecsSinceEpoch();
    auto it = failed_.find(k);
    if (it != failed_.end()) {
        if (now < it.value()) {
            return;
        }
        failed_.erase(it);
    }

    const QUrl url = offline::url(layer == Satellite ? offline::Satellite : offline::Streets, z, x, y);
    QNetworkRequest req(url);
    // generic client id; no user details leave the machine
    req.setRawHeader("User-Agent", "mavlink_gcs_ui/0.1 (Qt)");
    req.setTransferTimeout(15000);
    pending_.insert(k);

    QNetworkReply* reply = nam_.get(req);
    connect(reply, &QNetworkReply::finished, this, [this, reply, k, layer, z, x, y] {
        reply->deleteLater();
        pending_.remove(k);
        const QByteArray data = reply->readAll();
        QImage img;
        if (reply->error() != QNetworkReply::NoError || !img.loadFromData(data)) {
            failed_.insert(k, QDateTime::currentMSecsSinceEpoch() + kRetryMs);
            return;
        }
        const QString path = diskPath(layer, z, x, y);
        QDir().mkpath(QFileInfo(path).absolutePath());
        QFile f(path);
        if (f.open(QIODevice::WriteOnly)) {
            f.write(data);
        }
        memory_.insert(k, new QPixmap(process(img, layer)));
        emit tileReady();
    });
}

QPixmap TileCache::process(const QImage& src, int layer) const {
    QImage img = src.convertToFormat(QImage::Format_RGB32);
    const int w = img.width();
    const int h = img.height();
    for (int yy = 0; yy < h; ++yy) {
        auto* line = reinterpret_cast<QRgb*>(img.scanLine(yy));
        for (int xx = 0; xx < w; ++xx) {
            const QRgb c = line[xx];
            const int r = qRed(c), g = qGreen(c), b = qBlue(c);
            const int lum = (r * 54 + g * 183 + b * 19) >> 8;
            if (layer == Satellite) {
                // keep the imagery readable: partial desaturation, cool tint, darker
                const int mr = (r + lum) / 2, mg = (g + lum) / 2, mb = (b + lum) / 2;
                line[xx] = qRgb(mr * 62 / 100, mg * 68 / 100, std::min(255, mb * 76 / 100 + 6));
            } else {
                // inverted luminance -> dark blue map. OSM's white minor roads
                // would invert to black, so lift near-white pixels instead.
                if (lum >= 250) {
                    line[xx] = qRgb(36, 64, 80);
                    continue;
                }
                const int v = 255 - lum;
                line[xx] = qRgb(7 + v * 22 / 100, 11 + v * 36 / 100, 16 + v * 46 / 100);
            }
        }
    }
    return QPixmap::fromImage(img);
}

}  // namespace gcs::ui
