#include "offline_store.hpp"

#include <QCryptographicHash>
#include <QDir>
#include <QFileInfo>
#include <QSettings>
#include <QStandardPaths>

namespace gcs::ui::offline {

QString subdir(int layer) {
    switch (layer) {
    case Satellite:
        return QStringLiteral("esri_imagery");
    case Elevation:
        return QStringLiteral("elevation");
    default:
        return QStringLiteral("osm");
    }
}

QString defaultExtension(int layer) {
    return layer == Satellite ? QStringLiteral("jpg") : QStringLiteral("png");
}

QString defaultFolder() {
    return QStandardPaths::writableLocation(QStandardPaths::AppDataLocation) + QStringLiteral("/offline");
}

namespace {
// read once: lookups happen for every tile that isn't in memory
QString& cachedFolder() {
    static QString dir = [] {
        const QString d = QSettings().value(QStringLiteral("map/offline_dir")).toString();
        return d.isEmpty() ? defaultFolder() : d;
    }();
    return dir;
}
}  // namespace

QString folder() {
    return cachedFolder();
}

void setFolder(const QString& dir) {
    QSettings q;
    if (dir.isEmpty() || QDir(dir) == QDir(defaultFolder())) {
        q.remove(QStringLiteral("map/offline_dir"));
        cachedFolder() = defaultFolder();
    } else {
        q.setValue(QStringLiteral("map/offline_dir"), dir);
        cachedFolder() = dir;
    }
}

QString find(int layer, int z, int x, int y) {
    static const char* exts[] = {"png", "jpg", "jpeg", "webp", "img"};
    const QString base = QStringLiteral("%1/%2/%3/%4/%5.").arg(folder(), subdir(layer)).arg(z).arg(x).arg(y);
    for (const char* e : exts) {
        const QString path = base + QLatin1String(e);
        if (QFileInfo::exists(path)) {
            return path;
        }
    }
    return {};
}

QString pathFor(int layer, int z, int x, int y) {
    return QStringLiteral("%1/%2/%3/%4/%5.%6")
        .arg(folder(), subdir(layer))
        .arg(z)
        .arg(x)
        .arg(y)
        .arg(defaultExtension(layer));
}

QString defaultUrlTemplate(int layer) {
    switch (layer) {
    case Satellite:
        return QStringLiteral("https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}");
    case Elevation:
        return QStringLiteral("https://s3.amazonaws.com/elevation-tiles-prod/terrarium/{z}/{x}/{y}.png");
    default:
        return QStringLiteral("https://tile.openstreetmap.org/{z}/{x}/{y}.png");
    }
}

QString urlTemplate(int layer) {
    static const QString t[kLayers] = {
        QSettings().value(QStringLiteral("map/streets_url")).toString(),
        QSettings().value(QStringLiteral("map/satellite_url")).toString(),
        QSettings().value(QStringLiteral("map/elevation_url")).toString(),
    };
    return t[layer].isEmpty() ? defaultUrlTemplate(layer) : t[layer];
}

QUrl url(int layer, int z, int x, int y) {
    QString t = urlTemplate(layer);
    t.replace(QStringLiteral("{z}"), QString::number(z));
    t.replace(QStringLiteral("{x}"), QString::number(x));
    t.replace(QStringLiteral("{y}"), QString::number(y));
    return QUrl(t);
}

bool isNoDataTile(const QByteArray& bytes) {
    return bytes.size() < 4096 &&
           QCryptographicHash::hash(bytes, QCryptographicHash::Md5).toHex() == QByteArray(kEsriNoDataMd5);
}

QString noDataPathFor(const QString& base_dir, int layer, int z, int x, int y) {
    return QStringLiteral("%1/%2/%3/%4/%5.none").arg(base_dir, subdir(layer)).arg(z).arg(x).arg(y);
}

bool hasNoDataMarker(int layer, int z, int x, int y, const QString& cache_dir) {
    if (QFileInfo::exists(noDataPathFor(folder(), layer, z, x, y))) {
        return true;
    }
    return !cache_dir.isEmpty() && QFileInfo::exists(noDataPathFor(cache_dir, layer, z, x, y));
}

namespace {
bool& onlineFlag() {
    static bool on = QSettings().value(QStringLiteral("map/online"), false).toBool();
    return on;
}
}  // namespace

bool online() {
    return onlineFlag();
}

void setOnline(bool on) {
    onlineFlag() = on;
    QSettings().setValue(QStringLiteral("map/online"), on);
}

}  // namespace gcs::ui::offline
