#pragma once

#include <QCache>
#include <QHash>
#include <QNetworkAccessManager>
#include <QObject>
#include <QPixmap>
#include <QSet>
#include <QString>

namespace gcs::ui {

class VectorMap;

// Slippy-map tiles for the map view.
//  - Streets: drawn from the offline vector map (VectorMap) when there is one.
//  - Satellite: the offline folder, then the browsing cache, then (only when
//    online tiles are allowed) the network. Recoloured once on load so it sits
//    under the cyan HUD without per-frame compositing.
// Where a source has nothing at a zoom (Esri's "no data" tile, no vector
// tile) the tile is remembered as empty, and the map draws a coarser one
// enlarged instead.
class TileCache : public QObject {
    Q_OBJECT

public:
    enum Layer { Streets = 0, Satellite = 1 };
    static constexpr int kMinZoom = 2;

    explicit TileCache(QObject* parent = nullptr);

    static int maxZoom(int layer);
    static QString layerName(int layer);

    void setVectorMap(VectorMap* vm);
    // the vector map's look changed (labels language, new file): drop rendered tiles
    void vectorChanged();
    // offline data changed (download, delete, folder): forget what's in memory
    void clearMemory();
    bool hasStreetSource() const;  // a vector map is open, or online raster streets are allowed

    // Returns the tile if it is in memory or on disk, otherwise queues a
    // render / download and returns nullptr. The pointer is valid until the next call.
    const QPixmap* tile(int layer, int z, int x, int y);
    // Memory only, never touches disk or network (used for fallbacks).
    const QPixmap* peek(int layer, int z, int x, int y);
    // known to have nothing at this zoom (draw a parent enlarged)
    bool empty(int layer, int z, int x, int y);

signals:
    void tileReady();

private:
    static QString key(int layer, int z, int x, int y);
    QString diskPath(int layer, int z, int x, int y) const;
    QPixmap process(const QImage& img, int layer) const;
    void request(int layer, int z, int x, int y);
    void markEmpty(int layer, int z, int x, int y, bool persist);

    QNetworkAccessManager nam_;
    QCache<QString, QPixmap> memory_;
    QSet<QString> pending_;
    QSet<QString> empty_;            // nothing there (no-data marker / outside the vector map)
    QHash<QString, qint64> failed_;  // key -> retry-after (ms since epoch)
    QString dir_;
    VectorMap* vector_ = nullptr;
};

}  // namespace gcs::ui
