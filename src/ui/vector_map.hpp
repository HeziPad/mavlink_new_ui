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

// The offline street map: OpenStreetMap vector tiles (Protomaps basemap
// schema) in the offline folder's PMTiles files, drawn by the app in the
// HUD's dark style. Several files are layered: each tile comes from the most
// detailed one that has it (a drawn full-detail area in streets/, the country
// map.pmtiles, then the coarse world.pmtiles). Tiles are rendered on a private thread pool (never Qt's
// global one, which the raster engine waits on) and delivered through
// rendered(). Zooms above the data's maximum are drawn from the deepest tile,
// so the map stays sharp all the way in.
class VectorMap : public QObject {
    Q_OBJECT

public:
    explicit VectorMap(QObject* parent = nullptr);
    ~VectorMap() override;

    // opens every *.pmtiles in the offline folder and its streets/ (again after
    // a download or a folder change); false when there is none
    bool reopen();
    void close();  // lets the files go (before deleting them)
    bool available() const { return !srcs_.empty(); }
    QString path() const;  // the most detailed file
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
    std::vector<std::shared_ptr<Source>> srcs_;  // most detailed first
    QThreadPool pool_;
    QSet<quint64> pending_;
    bool english_ = false;
    quint64 gen_ = 1;
};

}  // namespace gcs::ui
