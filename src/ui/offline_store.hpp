#pragma once

#include <QString>
#include <QUrl>

namespace gcs::ui::offline {

// Where map and elevation tiles live, and where they come from.
//
// Offline folder (Settings → OFFLINE; default <AppData>/offline, i.e.
// ~/.local/share/mavlink_gcs_ui/offline). It is read before the download
// cache and the network, and has a plain slippy-tile layout so it can be
// filled by hand (tools/fetch_offline.py, or any tile exporter):
//
//   osm/{z}/{x}/{y}.png            streets
//   esri_imagery/{z}/{x}/{y}.jpg   satellite
//   elevation/{z}/{x}/{y}.png      AWS "terrarium" elevation
//
// Any of .png / .jpg / .jpeg / .webp / .img is accepted for every layer.
enum Layer { Streets = 0, Satellite = 1, Elevation = 2, kLayers };

QString subdir(int layer);              // "osm", "esri_imagery", "elevation"
QString defaultExtension(int layer);    // what the downloader writes
QString folder();                       // the offline folder (setting or default)
QString defaultFolder();
void setFolder(const QString& dir);     // "" = back to the default
// the offline file for a tile, or "" when there is none
QString find(int layer, int z, int x, int y);
QString pathFor(int layer, int z, int x, int y);  // where the downloader writes it

// Tile URL. Templates use {z} {x} {y}; QSettings map/streets_url,
// map/satellite_url and map/elevation_url override the defaults.
QString urlTemplate(int layer);
QString defaultUrlTemplate(int layer);
QUrl url(int layer, int z, int x, int y);

}  // namespace gcs::ui::offline
