#pragma once

#include <QCache>
#include <QHash>
#include <QNetworkAccessManager>
#include <QObject>
#include <QPixmap>
#include <QSet>
#include <QString>

namespace gcs::ui {

// Slippy-map tiles: memory LRU over a disk cache over HTTP. Tiles are
// recoloured once on load (dark "tactical" streets, dimmed satellite) so the
// map never needs per-frame compositing to sit under the cyan HUD.
class TileCache : public QObject {
    Q_OBJECT

public:
    enum Layer { Streets = 0, Satellite = 1 };
    static constexpr int kMinZoom = 2;

    explicit TileCache(QObject* parent = nullptr);

    static int maxZoom(int layer);
    static QString layerName(int layer);

    // Returns the tile if it is in memory or on disk, otherwise queues a
    // download and returns nullptr. The pointer is valid until the next call.
    const QPixmap* tile(int layer, int z, int x, int y);
    // Memory only, never touches disk or network (used for fallbacks).
    const QPixmap* peek(int layer, int z, int x, int y);

signals:
    void tileReady();

private:
    static QString key(int layer, int z, int x, int y);
    QString diskPath(int layer, int z, int x, int y) const;
    QPixmap process(const QImage& img, int layer) const;
    void request(int layer, int z, int x, int y);

    QNetworkAccessManager nam_;
    QCache<QString, QPixmap> memory_;
    QSet<QString> pending_;
    QHash<QString, qint64> failed_;  // key -> retry-after (ms since epoch)
    QString dir_;
};

}  // namespace gcs::ui
