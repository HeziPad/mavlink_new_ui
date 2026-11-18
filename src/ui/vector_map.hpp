#pragma once

#include "core/pmtiles.hpp"

#include <QFile>
#include <QHash>
#include <QImage>
#include <QObject>
#include <QSet>
#include <QString>
#include <QThreadPool>

#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace gcs::ui {

// The offline street map: OpenStreetMap vector tiles in one PMTiles file in
// the offline folder (Protomaps basemap schema), drawn by the app in the
// HUD's dark style. Tiles are rendered on a private thread pool (never Qt's
// global one, which the raster engine waits on) and delivered through
// rendered(). Zooms above the data's maximum are drawn from the deepest tile,
// so the map stays sharp all the way in.
class VectorMap : public QObject {
    Q_OBJECT

public:
    explicit VectorMap(QObject* parent = nullptr);
    ~VectorMap() override;

    // opens the first *.pmtiles in the offline folder (again after a download
    // or a folder change); false when there is none
    bool reopen();
    bool available() const { return src_ != nullptr; }
    QString path() const;
    int maxDataZoom() const;

    void setEnglish(bool english);  // labels: name:en (falling back to name) vs name
    bool english() const { return english_; }
    quint64 generation() const { return gen_; }  // bumps when what tiles look like changes

    // queue a render (no-op when it's already queued)
    void request(int z, int x, int y);

    struct Source;  // the open file (shared with render jobs)

    // a render job is done (called on this object's thread)
    Q_INVOKABLE void deliver(int z, int x, int y, quint64 generation, const QImage& image);

signals:
    void rendered(int z, int x, int y, quint64 generation, const QImage& image);

private:
    std::shared_ptr<Source> src_;
    QThreadPool pool_;
    QSet<quint64> pending_;
    bool english_ = false;
    quint64 gen_ = 1;
};

}  // namespace gcs::ui
