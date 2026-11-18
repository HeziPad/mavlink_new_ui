#include "check.hpp"
#include "core/coords.hpp"

#include <cmath>
#include <string>

using namespace gcs::coords;

static void testUtm() {
    // Wikipedia's example: the CN Tower, 43°38'33.24"N 79°23'13.7"W = 17T 630084 4833438
    const double lat = 43.0 + 38.0 / 60.0 + 33.24 / 3600.0;
    const double lon = -(79.0 + 23.0 / 60.0 + 13.7 / 3600.0);
    const Utm u = toUtm(lat, lon);
    CHECK(u.zone == 17);
    CHECK(u.band == 'T');
    CHECK(u.north);
    CHECK_NEAR(u.easting, 630084.0, 1.0);
    CHECK_NEAR(u.northing, 4833438.0, 1.0);
    // on a central meridian at the equator: exactly the false easting
    const Utm c = toUtm(0.0, 3.0);
    CHECK(c.zone == 31);
    CHECK_NEAR(c.easting, 500000.0, 1e-6);
    CHECK_NEAR(c.northing, 0.0, 1e-6);
    // round trips: Tel Aviv, Jerusalem, a zone edge, the southern hemisphere
    const double pts[][2] = {{32.0853, 34.7818}, {31.7683, 35.2137}, {31.0, 35.999}, {-33.8568, 151.2153}, {60.0, -0.001}};
    for (const auto& p : pts) {
        const Utm v = toUtm(p[0], p[1]);
        const LatLon back = fromUtm(v.zone, v.north, v.easting, v.northing);
        CHECK_NEAR(back.lat, p[0], 1e-8);  // ~1 mm
        CHECK_NEAR(back.lon, p[1], 1e-8);
    }
    CHECK(toUtm(32.0853, 34.7818).zone == 36);
    CHECK(toUtm(32.0853, 34.7818).band == 'S');
    CHECK(!toUtm(-33.8568, 151.2153).north);
}

static void testFormat() {
    CHECK(format(32.0853, 34.7818, Fmt::Decimal) == "32.08530\xC2\xB0 N  34.78180\xC2\xB0 E");
    CHECK(format(-33.5, -70.25, Fmt::Decimal) == "33.50000\xC2\xB0 S  70.25000\xC2\xB0 W");
    // 32.0853 = 32°05'07.1"
    CHECK(format(32.0853, 34.7818, Fmt::Dms) == "32\xC2\xB0" "05'07.1\" N  34\xC2\xB0" "46'54.5\" E");
    // 59.96" rounds into the next minute
    CHECK(format(10.0 + 59.0 / 60.0 + 59.96 / 3600.0, 0.0, Fmt::Dms).rfind("11\xC2\xB0" "00'00.0\" N", 0) == 0);
    const std::string utm = format(32.0853, 34.7818, Fmt::Utm);
    CHECK(utm.rfind("36S ", 0) == 0);
    CHECK(utm.size() == std::string("36S 668123 3551234").size());
}

static void testParse() {
    for (Fmt f : {Fmt::Decimal, Fmt::Dms, Fmt::Utm}) {
        const auto r = parse(format(32.0853, 34.7818, f), f);
        CHECK(r.has_value());
        if (r) {
            const double tol = f == Fmt::Utm ? 2e-5 : (f == Fmt::Dms ? 3e-5 : 1e-5);  // the printed resolution
            CHECK_NEAR(r->lat, 32.0853, tol);
            CHECK_NEAR(r->lon, 34.7818, tol);
        }
    }
    auto near = [](const std::optional<LatLon>& r, double lat, double lon) {
        return r && std::abs(r->lat - lat) < 1e-6 && std::abs(r->lon - lon) < 1e-6;
    };
    CHECK(near(parse("32.0853, 34.7818", Fmt::Decimal), 32.0853, 34.7818));
    CHECK(near(parse("-33.5 -70.25", Fmt::Decimal), -33.5, -70.25));
    CHECK(near(parse("33.5 S 70.25 W", Fmt::Decimal), -33.5, -70.25));
    CHECK(near(parse("E 34.5 N 32.25", Fmt::Decimal), 32.25, 34.5));   // longitude first
    CHECK(near(parse("N 32 15 E 34 30", Fmt::Dms), 32.25, 34.5));
    CHECK(near(parse("32 15 0 N, 34 30 0 E", Fmt::Dms), 32.25, 34.5));
    CHECK(near(parse("32.25N 34.5E", Fmt::Decimal), 32.25, 34.5));
    CHECK(!parse("32.1", Fmt::Decimal));
    CHECK(!parse("95 10", Fmt::Decimal));
    CHECK(!parse("32 75 N 34 0 E", Fmt::Dms));       // 75 minutes
    CHECK(!parse("N 32 N 34", Fmt::Decimal));
    // UTM with a band, or with a separate hemisphere letter
    const auto u = parse("17T 630084 4833438", Fmt::Utm);
    CHECK(u && std::abs(u->lat - 43.64257) < 1e-4 && std::abs(u->lon + 79.38714) < 1e-4);
    const auto s = parse("56 S 334786 6252080", Fmt::Utm);
    CHECK(s && s->lat < -33.0 && s->lon > 151.0);
    CHECK(!parse("61N 500000 0", Fmt::Utm));
    CHECK(!parse("36S 5000 3551234", Fmt::Utm));
}

int main() {
    testUtm();
    testFormat();
    testParse();
    return TEST_MAIN_RESULT();
}
