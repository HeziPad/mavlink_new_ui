#include "check.hpp"
#include "ui/plan_library.hpp"

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QTemporaryDir>

using namespace gcs;
using gcs::ui::PlanLibrary;

static MissionPlan samplePlan() {
    MissionPlan p;
    p.pattern = Pattern::OneWay;
    p.hold_radius = 120.0f;
    p.hold_ccw = true;
    UserWaypoint a{32.1234567, 34.7654321, 80.0f};
    a.hold_s = 45.0f;
    a.loiter_radius = 150.0f;
    a.loiter_ccw = true;
    a.hold_turns = 3;  // derived: not stored
    p.wps = {a, {32.13, 34.77, 95.5f}, {32.14, 34.78, 110.0f}};
    p.land = UserWaypoint{32.12, 34.76, 30.0f};
    return p;
}

static void testRoundTrip(const QString& dir) {
    PlanLibrary lib(dir);
    CHECK(lib.list().empty());
    const MissionPlan p = samplePlan();
    const QString id = lib.save({}, QStringLiteral("  Ridge   loop "), p);
    CHECK(!id.isEmpty());
    const auto back = lib.load(id);
    CHECK(back.has_value());
    if (back) {
        CHECK(*back == p);
        CHECK(back->wps[0].hold_turns == 0);
        CHECK_NEAR(back->wps[0].lat, 32.1234567, 1e-9);
    }
    const auto list = lib.list();
    CHECK(list.size() == 1);
    if (!list.empty()) {
        CHECK(list[0].name == QStringLiteral("Ridge loop"));
        CHECK(list[0].waypoints == 3);
        CHECK(list[0].has_land);
        CHECK(list[0].route_m > 0.0);
        CHECK(list[0].saved.isValid());
    }
    CHECK(lib.findByName(QStringLiteral("ridge LOOP")) == id);
    CHECK(lib.findByName(QStringLiteral("other")).isEmpty());

    // overwrite in place
    MissionPlan q = p;
    q.land.reset();
    q.pattern = Pattern::Loop;
    CHECK(lib.save(id, QStringLiteral("Ridge loop"), q) == id);
    CHECK(lib.list().size() == 1);
    CHECK(lib.load(id) && *lib.load(id) == q);

    // a name in Hebrew, and an empty plan
    const QString heb = QString::fromUtf8("מסלול צפון");
    const QString id2 = lib.save({}, heb, MissionPlan{});
    CHECK(!id2.isEmpty());
    CHECK(lib.findByName(heb) == id2);
    CHECK(lib.load(id2) && lib.load(id2)->empty());
    CHECK(lib.list().size() == 2);

    CHECK(lib.rename(id, QStringLiteral("Alpha")));
    CHECK(lib.list().front().name == QStringLiteral("Alpha"));  // sorted by name
    CHECK(lib.load(id) && *lib.load(id) == q);

    CHECK(lib.save({}, QStringLiteral("   "), p).isEmpty());  // no name
    CHECK(lib.remove(id2));
    CHECK(!lib.remove(id2));
    CHECK(lib.list().size() == 1);
}

static void testBadFiles(const QString& dir) {
    PlanLibrary lib(dir);
    const size_t before = lib.list().size();
    auto put = [&](const char* name, const QByteArray& data) {
        QFile f(QDir(dir).filePath(QString::fromLatin1(name)));
        CHECK(f.open(QIODevice::WriteOnly));
        f.write(data);
    };
    put("junk.json", "{not json");
    put("v2.json", R"({"version":2,"name":"x","wps":[]})");
    put("badlat.json", R"({"version":1,"name":"x","wps":[{"lat":95,"lon":1,"alt":50}]})");
    put("nan.json", R"({"version":1,"name":"x","wps":[{"lat":1,"lon":1,"alt":-5}]})");
    put("noname.json", R"({"version":1,"name":"  ","wps":[]})");
    put("pattern.json", R"({"version":1,"name":"x","pattern":"zigzag","wps":[]})");
    put("notes.txt", "ignored");
    CHECK(lib.list().size() == before);
    CHECK(!lib.load(QStringLiteral("junk")));
    CHECK(!lib.load(QStringLiteral("")));

    put("ok.json", R"({"version":1,"name":"Hand made","wps":[{"lat":1,"lon":2,"alt":60}],"land":null})");
    CHECK(lib.list().size() == before + 1);
    const auto p = lib.load(QStringLiteral("ok"));
    CHECK(p && p->wps.size() == 1 && p->pattern == Pattern::Loop && !p->land);
}

int main(int argc, char** argv) {
    QCoreApplication app(argc, argv);
    QTemporaryDir tmp;
    CHECK(tmp.isValid());
    const QString dir = QDir(tmp.path()).filePath(QStringLiteral("plans"));
    testRoundTrip(dir);
    testBadFiles(dir);
    CHECK(PlanLibrary::cleanName(QString(60, QLatin1Char('a'))).size() == PlanLibrary::kMaxName);
    return TEST_MAIN_RESULT();
}
