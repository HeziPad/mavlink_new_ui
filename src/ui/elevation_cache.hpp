#pragma once

#include <QCache>
#include <QHash>
#include <QImage>
#include <QNetworkAccessManager>
#include <QObject>
#include <QSet>
#include <QString>

#include <optional>

namespace gcs::ui {

// Ground height (metres above sea level) from AWS Terrain Tiles ("terrarium"
// PNG, SRTM-based, ~30 m). Lookups read the offline folder, then the browsing
// cache, then download; a miss returns nullopt and emits ready() once the
// tile arrives. Accuracy is about +-10 m and it is the surface (roofs, tree
// tops): good for a readout, not for landing on.
class ElevationCache : public QObject {
    Q_OBJECT

public:
    static constexpr int kZoom = 13;     // ~19 m/px at the equator, finer than the data
    static constexpr int kMinZoom = 9;   // coarser tiles used while kZoom isn't there

    explicit ElevationCache(QObject* parent = nullptr);

    std::optional<double> elevation(double lat, double lon);

signals:
    void ready();

private:
    const QImage* tile(int z, int x, int y, bool fetch);
    QString cachePath(int z, int x, int y) const;
    void request(int z, int x, int y);

    QNetworkAccessManager nam_;
    QCache<QString, QImage> memory_;
    QSet<QString> pending_;
    QHash<QString, qint64> failed_;  // retry-after, ms since epoch
    QString dir_;
};

}  // namespace gcs::ui
