#pragma once

#include "core/mission.hpp"

#include <QDateTime>
#include <QString>

#include <optional>
#include <vector>

namespace gcs::ui {

// The operator's saved plans: one JSON file per plan in
// <AppData>/plans/<id>.json (the id is a UUID, so a plan's name can be
// anything, Hebrew included, without touching file-name rules). The name lives
// inside the file. QtCore only, so the tests can use it.
//
// File: {"version": 1, "name": "...", "saved": "<ISO UTC>",
//        "pattern": "loop" | "back_forth" | "one_way",
//        "hold_radius": m, "hold_ccw": bool,
//        "wps": [{"lat", "lon", "alt", "hold_s", "radius", "ccw"}, ...],
//        "land": {"lat", "lon", "alt"} | null}
// A file that is unreadable, of another version, or has a bad value is skipped.
class PlanLibrary {
public:
    struct Entry {
        QString id;
        QString name;
        QDateTime saved;  // UTC
        int waypoints = 0;
        double route_m = 0.0;  // one cycle of the pattern
        bool has_land = false;
    };

    // `folder` empty = <AppData>/plans
    explicit PlanLibrary(QString folder = {});

    QString folder() const { return folder_; }
    std::vector<Entry> list() const;  // by name, case-insensitive
    std::optional<MissionPlan> load(const QString& id) const;
    // Writes the plan under `id` (empty = a new id). Returns the id, or empty on failure.
    QString save(const QString& id, const QString& name, const MissionPlan& plan);
    bool rename(const QString& id, const QString& name);
    bool remove(const QString& id);
    // the id of the plan with this name (trimmed, case-insensitive), or empty
    QString findByName(const QString& name) const;
    std::optional<Entry> entry(const QString& id) const;

    static constexpr int kMaxName = 40;
    static QString cleanName(const QString& name);  // trimmed, inner whitespace collapsed, at most kMaxName

    // JSON <-> plan (exposed for the tests)
    static QByteArray encode(const QString& name, const QDateTime& saved, const MissionPlan& plan);
    static bool decode(const QByteArray& json, QString* name, QDateTime* saved, MissionPlan* plan);

private:
    QString pathFor(const QString& id) const;
    QString folder_;
};

}  // namespace gcs::ui
