#include "ui/plan_library.hpp"

#include <QDir>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QSaveFile>
#include <QStandardPaths>
#include <QUuid>

#include <algorithm>
#include <cmath>

namespace gcs::ui {

namespace {

constexpr int kVersion = 1;
constexpr double kMaxAlt = 10000.0;   // m above home: anything above is a broken file
constexpr double kMaxRadius = 5000.0;
constexpr double kMaxHold = 24 * 3600.0;

QString patternName(Pattern p) {
    switch (p) {
    case Pattern::BackForth: return QStringLiteral("back_forth");
    case Pattern::OneWay: return QStringLiteral("one_way");
    case Pattern::Loop: break;
    }
    return QStringLiteral("loop");
}

bool finiteIn(double v, double lo, double hi) { return std::isfinite(v) && v >= lo && v <= hi; }

bool readPoint(const QJsonObject& o, UserWaypoint& wp, bool route) {
    const double lat = o.value(QStringLiteral("lat")).toDouble(NAN);
    const double lon = o.value(QStringLiteral("lon")).toDouble(NAN);
    const double alt = o.value(QStringLiteral("alt")).toDouble(NAN);
    if (!finiteIn(lat, -90.0, 90.0) || !finiteIn(lon, -180.0, 180.0) || !finiteIn(alt, 0.0, kMaxAlt)) {
        return false;
    }
    wp = UserWaypoint{};
    wp.lat = lat;
    wp.lon = lon;
    wp.alt_rel = static_cast<float>(alt);
    if (route) {
        const double hold = o.value(QStringLiteral("hold_s")).toDouble(0.0);
        const double radius = o.value(QStringLiteral("radius")).toDouble(0.0);
        if (!finiteIn(hold, 0.0, kMaxHold) || !finiteIn(radius, 0.0, kMaxRadius)) {
            return false;
        }
        wp.hold_s = static_cast<float>(hold);
        wp.loiter_radius = static_cast<float>(radius);
        wp.loiter_ccw = o.value(QStringLiteral("ccw")).toBool(false);
    }
    return true;
}

}  // namespace

PlanLibrary::PlanLibrary(QString folder) : folder_(std::move(folder)) {
    if (folder_.isEmpty()) {
        folder_ = QDir(QStandardPaths::writableLocation(QStandardPaths::AppDataLocation))
                      .filePath(QStringLiteral("plans"));
    }
}

QString PlanLibrary::pathFor(const QString& id) const {
    return QDir(folder_).filePath(id + QStringLiteral(".json"));
}

QString PlanLibrary::cleanName(const QString& name) {
    return name.simplified().left(kMaxName).trimmed();
}

QByteArray PlanLibrary::encode(const QString& name, const QDateTime& saved, const MissionPlan& plan) {
    QJsonObject o;
    o.insert(QStringLiteral("version"), kVersion);
    o.insert(QStringLiteral("name"), name);
    o.insert(QStringLiteral("saved"), saved.toUTC().toString(Qt::ISODate));
    o.insert(QStringLiteral("pattern"), patternName(plan.pattern));
    o.insert(QStringLiteral("hold_radius"), static_cast<double>(plan.hold_radius));
    o.insert(QStringLiteral("hold_ccw"), plan.hold_ccw);
    QJsonArray wps;
    for (const auto& wp : plan.wps) {
        QJsonObject w;
        w.insert(QStringLiteral("lat"), wp.lat);
        w.insert(QStringLiteral("lon"), wp.lon);
        w.insert(QStringLiteral("alt"), static_cast<double>(wp.alt_rel));
        w.insert(QStringLiteral("hold_s"), static_cast<double>(wp.hold_s));
        w.insert(QStringLiteral("radius"), static_cast<double>(wp.loiter_radius));
        w.insert(QStringLiteral("ccw"), wp.loiter_ccw);
        wps.append(w);
    }
    o.insert(QStringLiteral("wps"), wps);
    if (plan.land) {
        QJsonObject l;
        l.insert(QStringLiteral("lat"), plan.land->lat);
        l.insert(QStringLiteral("lon"), plan.land->lon);
        l.insert(QStringLiteral("alt"), static_cast<double>(plan.land->alt_rel));
        o.insert(QStringLiteral("land"), l);
    } else {
        o.insert(QStringLiteral("land"), QJsonValue::Null);
    }
    return QJsonDocument(o).toJson(QJsonDocument::Indented);
}

bool PlanLibrary::decode(const QByteArray& json, QString* name, QDateTime* saved, MissionPlan* plan) {
    QJsonParseError err{};
    const QJsonDocument doc = QJsonDocument::fromJson(json, &err);
    if (err.error != QJsonParseError::NoError || !doc.isObject()) {
        return false;
    }
    const QJsonObject o = doc.object();
    if (o.value(QStringLiteral("version")).toInt(0) != kVersion) {
        return false;
    }
    const QString n = cleanName(o.value(QStringLiteral("name")).toString());
    if (n.isEmpty()) {
        return false;
    }
    MissionPlan p;
    const QString pat = o.value(QStringLiteral("pattern")).toString(QStringLiteral("loop"));
    if (pat == QStringLiteral("back_forth")) {
        p.pattern = Pattern::BackForth;
    } else if (pat == QStringLiteral("one_way")) {
        p.pattern = Pattern::OneWay;
    } else if (pat == QStringLiteral("loop")) {
        p.pattern = Pattern::Loop;
    } else {
        return false;
    }
    const double hr = o.value(QStringLiteral("hold_radius")).toDouble(0.0);
    if (!finiteIn(hr, 0.0, kMaxRadius)) {
        return false;
    }
    p.hold_radius = static_cast<float>(hr);
    p.hold_ccw = o.value(QStringLiteral("hold_ccw")).toBool(false);
    for (const auto& v : o.value(QStringLiteral("wps")).toArray()) {
        UserWaypoint wp;
        if (!v.isObject() || !readPoint(v.toObject(), wp, true)) {
            return false;
        }
        p.wps.push_back(wp);
    }
    const QJsonValue land = o.value(QStringLiteral("land"));
    if (land.isObject()) {
        UserWaypoint l;
        if (!readPoint(land.toObject(), l, false)) {
            return false;
        }
        p.land = l;
    } else if (!land.isNull() && !land.isUndefined()) {
        return false;
    }
    if (name) *name = n;
    if (saved) *saved = QDateTime::fromString(o.value(QStringLiteral("saved")).toString(), Qt::ISODate).toUTC();
    if (plan) *plan = std::move(p);
    return true;
}

std::vector<PlanLibrary::Entry> PlanLibrary::list() const {
    std::vector<Entry> out;
    const QDir dir(folder_);
    for (const QString& f : dir.entryList({QStringLiteral("*.json")}, QDir::Files)) {
        const QString id = f.left(f.size() - 5);
        if (auto e = entry(id)) {
            out.push_back(std::move(*e));
        }
    }
    std::sort(out.begin(), out.end(), [](const Entry& a, const Entry& b) {
        const int c = QString::compare(a.name, b.name, Qt::CaseInsensitive);
        return c != 0 ? c < 0 : a.id < b.id;
    });
    return out;
}

std::optional<PlanLibrary::Entry> PlanLibrary::entry(const QString& id) const {
    QFile f(pathFor(id));
    if (id.isEmpty() || !f.open(QIODevice::ReadOnly)) {
        return std::nullopt;
    }
    Entry e;
    MissionPlan plan;
    if (!decode(f.readAll(), &e.name, &e.saved, &plan)) {
        return std::nullopt;
    }
    e.id = id;
    e.waypoints = static_cast<int>(plan.wps.size());
    e.route_m = routeLength(plan);
    e.has_land = plan.land.has_value();
    return e;
}

std::optional<MissionPlan> PlanLibrary::load(const QString& id) const {
    QFile f(pathFor(id));
    if (id.isEmpty() || !f.open(QIODevice::ReadOnly)) {
        return std::nullopt;
    }
    MissionPlan plan;
    if (!decode(f.readAll(), nullptr, nullptr, &plan)) {
        return std::nullopt;
    }
    return plan;
}

QString PlanLibrary::save(const QString& id, const QString& name, const MissionPlan& plan) {
    const QString n = cleanName(name);
    if (n.isEmpty() || !QDir().mkpath(folder_)) {
        return {};
    }
    const QString use = id.isEmpty() ? QUuid::createUuid().toString(QUuid::WithoutBraces) : id;
    QSaveFile f(pathFor(use));  // all or nothing: a crash mid-write keeps the old file
    if (!f.open(QIODevice::WriteOnly)) {
        return {};
    }
    f.write(encode(n, QDateTime::currentDateTimeUtc(), plan));
    return f.commit() ? use : QString{};
}

bool PlanLibrary::rename(const QString& id, const QString& name) {
    const auto plan = load(id);
    return plan && !save(id, name, *plan).isEmpty();
}

bool PlanLibrary::remove(const QString& id) {
    return !id.isEmpty() && QFile::remove(pathFor(id));
}

QString PlanLibrary::findByName(const QString& name) const {
    const QString n = cleanName(name);
    for (const auto& e : list()) {
        if (QString::compare(e.name, n, Qt::CaseInsensitive) == 0) {
            return e.id;
        }
    }
    return {};
}

}  // namespace gcs::ui
