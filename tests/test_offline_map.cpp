#include "check.hpp"
#include "core/gzip.hpp"
#include "core/mvt.hpp"
#include "core/map_label.hpp"
#include "core/pmtiles.hpp"
#include "core/tiles.hpp"

#include <string>

using namespace gcs;

static void testTileIds() {
    // spec values: z0 is 0; z1 runs 1..4 along the Hilbert curve
    CHECK(pmtiles::zxyToId(0, 0, 0) == 0);
    CHECK(pmtiles::zxyToId(1, 0, 0) == 1);
    CHECK(pmtiles::zxyToId(1, 0, 1) == 2);
    CHECK(pmtiles::zxyToId(1, 1, 1) == 3);
    CHECK(pmtiles::zxyToId(1, 1, 0) == 4);
    CHECK(pmtiles::zxyToId(2, 0, 0) == 5);
    // round trips, including a deep tile (Tel Aviv z15)
    for (auto [z, x, y] : {std::tuple<uint8_t, uint32_t, uint32_t>{3, 5, 2}, {12, 2443, 1661}, {15, 19549, 13297},
                           {19, 312798, 212763}}) {
        const auto back = pmtiles::idToZxy(pmtiles::zxyToId(z, x, y));
        CHECK(back.z == z && back.x == x && back.y == y);
    }
    // every tile of z4 maps to a distinct id in its zoom's range
    std::vector<bool> seen(256, false);
    for (uint32_t x = 0; x < 16; ++x) {
        for (uint32_t y = 0; y < 16; ++y) {
            const uint64_t id = pmtiles::zxyToId(4, x, y) - 85;  // 1+4+16+64 below z4
            CHECK(id < 256 && !seen[id]);
            seen[id] = true;
        }
    }
}

static void testDirectory() {
    std::vector<pmtiles::Entry> e = {{5, 0, 100, 1}, {6, 100, 50, 3}, {20, 150, 10, 0}, {40, 999, 7, 1}};
    const auto back = pmtiles::decodeDirectory(pmtiles::encodeDirectory(e));
    CHECK(back.has_value() && back->size() == e.size());
    for (size_t i = 0; back && i < e.size(); ++i) {
        CHECK((*back)[i].tile_id == e[i].tile_id && (*back)[i].offset == e[i].offset &&
              (*back)[i].length == e[i].length && (*back)[i].run_length == e[i].run_length);
    }
    CHECK(pmtiles::findEntry(e, 5)->offset == 0);
    CHECK(pmtiles::findEntry(e, 8)->offset == 100);    // inside the run 6..8
    CHECK(!pmtiles::findEntry(e, 9).has_value());      // after the run, before the leaf
    CHECK(pmtiles::findEntry(e, 25)->run_length == 0); // a leaf covers 20..39
    CHECK(!pmtiles::findEntry(e, 4).has_value());
    pmtiles::Header h;
    h.root_offset = 127;
    h.root_length = 33;
    h.max_zoom = 15;
    h.min_lat_e7 = -350000000;
    const auto hb = pmtiles::serializeHeader(h);
    CHECK(hb.size() == pmtiles::kHeaderSize);
    const auto h2 = pmtiles::parseHeader(hb);
    CHECK(h2 && h2->root_length == 33 && h2->max_zoom == 15 && h2->min_lat_e7 == -350000000);
}

static void testGzip() {
    std::string text;
    for (int i = 0; i < 5000; ++i) text += "tile " + std::to_string(i) + "\n";
    const std::string z = gz::deflateGzip(text);
    CHECK(z.size() < text.size() / 3);
    const auto back = gz::inflate(z);
    CHECK(back && *back == text);
    CHECK(gz::inflate("plain bytes") == std::string("plain bytes"));  // not compressed: unchanged
}

// --- a tiny protobuf writer to build a test vector tile -----------------------
static void varint(std::string& s, uint64_t v) {
    while (v >= 0x80) { s.push_back(static_cast<char>((v & 0x7f) | 0x80)); v >>= 7; }
    s.push_back(static_cast<char>(v));
}
static void field(std::string& s, int f, const std::string& bytes) {
    varint(s, (static_cast<uint64_t>(f) << 3) | 2);
    varint(s, bytes.size());
    s += bytes;
}
static void fieldVarint(std::string& s, int f, uint64_t v) {
    varint(s, static_cast<uint64_t>(f) << 3);
    varint(s, v);
}
static uint32_t zz(int32_t v) { return static_cast<uint32_t>((v << 1) ^ (v >> 31)); }

static void testMvt() {
    // layer "roads": one line (10,10)->(20,10)->(20,30) with kind=highway, name=Ayalon
    std::string geom;
    varint(geom, (1 << 3) | 1); varint(geom, zz(10)); varint(geom, zz(10));
    varint(geom, (2 << 3) | 2); varint(geom, zz(10)); varint(geom, zz(0)); varint(geom, zz(0)); varint(geom, zz(20));
    std::string tags;
    varint(tags, 0); varint(tags, 0); varint(tags, 1); varint(tags, 1);
    std::string feat;
    field(feat, 2, tags);
    fieldVarint(feat, 3, 2);
    field(feat, 4, geom);
    // a square polygon ring (closed)
    std::string pgeom;
    varint(pgeom, (1 << 3) | 1); varint(pgeom, zz(0)); varint(pgeom, zz(0));
    varint(pgeom, (3 << 3) | 2);
    for (auto [dx, dy] : {std::pair{100, 0}, {0, 100}, {-100, 0}}) { varint(pgeom, zz(dx)); varint(pgeom, zz(dy)); }
    varint(pgeom, 7 | (1 << 3));
    std::string pfeat;
    fieldVarint(pfeat, 3, 3);
    field(pfeat, 4, pgeom);
    std::string v1, v2;
    field(v1, 1, "highway");
    field(v2, 1, "Ayalon");
    std::string layer;
    fieldVarint(layer, 15, 2);
    field(layer, 1, "roads");
    field(layer, 2, feat);
    field(layer, 2, pfeat);
    field(layer, 3, "kind");
    field(layer, 3, "name");
    field(layer, 4, v1);
    field(layer, 4, v2);
    fieldVarint(layer, 5, 4096);
    std::string tile;
    field(tile, 3, layer);

    mvt::Tile t;
    CHECK(mvt::decode(tile, t));
    const mvt::Layer* l = t.layer("roads");
    CHECK(l != nullptr);
    if (!l) return;
    CHECK(l->extent == 4096 && l->features.size() == 2);
    const auto& f = l->features[0];
    CHECK(f.type == mvt::GeomType::Line);
    CHECK(l->str(f, "kind") == "highway" && l->str(f, "name") == "Ayalon" && l->str(f, "ref").empty());
    CHECK(f.parts.size() == 1 && f.parts[0].size() == 3);
    CHECK(f.parts[0][2].x == 20 && f.parts[0][2].y == 30);
    const auto& p = l->features[1];
    CHECK(p.type == mvt::GeomType::Polygon && p.parts.size() == 1 && p.parts[0].size() == 5);
    CHECK(p.parts[0].back().x == 0 && p.parts[0].back().y == 0);  // closed
}

// several street maps are layered: each tile comes from the most detailed file
// whose area has it
static void testCovers() {
    pmtiles::Header world;
    world.max_zoom = 6;
    world.min_lon_e7 = -1800000000;
    world.max_lon_e7 = 1800000000;
    world.min_lat_e7 = -850511000;
    world.max_lat_e7 = 850511000;
    pmtiles::Header area;  // a small box near Rome, z0-15
    area.max_zoom = 15;
    area.min_lon_e7 = 124000000;
    area.max_lon_e7 = 125000000;
    area.min_lat_e7 = 418000000;
    area.max_lat_e7 = 419000000;
    // a z16 tile over the box (12.45 E, 41.85 N), and one at Tel Aviv
    const auto rome = tiles::locate(41.85, 12.45, 16);
    const auto tlv = tiles::locate(32.08, 34.78, 16);
    CHECK(pmtiles::covers(area, 16, rome.x, rome.y));
    CHECK(!pmtiles::covers(area, 16, tlv.x, tlv.y));
    CHECK(pmtiles::covers(world, 16, rome.x, rome.y));
    CHECK(pmtiles::covers(world, 16, tlv.x, tlv.y));
    CHECK(pmtiles::covers(area, 0, 0, 0));  // the whole world contains the box
    // above the data's zoom: looked up in the ancestor
    CHECK(pmtiles::dataZoom(world, 16) == 6);
    CHECK(pmtiles::dataZoom(area, 16) == 15);
    CHECK(pmtiles::dataZoom(area, 9) == 9);
    // a tile only touching the box's edge isn't in it: x of 12.4 E exactly at z1 is
    // inside tile 1, so tile 0 (west of 0 E) misses
    CHECK(!pmtiles::covers(area, 1, 0, 0));
    pmtiles::Header deep = area;
    deep.min_zoom = 10;
    CHECK(!pmtiles::covers(deep, 9, rome.x >> 7, rome.y >> 7));
    CHECK(pmtiles::covers(deep, 10, rome.x >> 6, rome.y >> 6));
}

// what an area download fetches per zoom: counted from tile ranges, not a walk
static void testBoxCount() {
    CHECK(tiles::box(32.2, 34.5, 34.7, 35.8, 0).count() == 1);
    // a small box inside one z10 tile: one tile at every zoom up to 10
    const auto t = tiles::locate(32.08, 34.78, 10);
    CHECK(tiles::box(34.78, 32.08, 34.7801, 32.0801, 10).count() == 1);
    CHECK(tiles::box(34.78, 32.08, 34.7801, 32.0801, 10).x0 == t.x);
    // Cyprus (2.5 x 1.3 deg at 35 N): z18 tiles are ~125 m, so ~1820 x ~1150
    const auto cy = tiles::box(32.2, 34.5, 34.7, 35.8, 18);
    CHECK(cy.x1 - cy.x0 + 1 >= 1810 && cy.x1 - cy.x0 + 1 <= 1825);
    CHECK(cy.count() > 2000000 && cy.count() < 2300000);
    // a 4x larger count per zoom
    const auto cy17 = tiles::box(32.2, 34.5, 34.7, 35.8, 17);
    CHECK_NEAR(static_cast<double>(cy.count()) / cy17.count(), 4.0, 0.05);
}

// labels: English or Hebrew, never a third language
static void testLabels() {
    using map_label::Script;
    CHECK(map_label::script("Roma") == Script::Latin);
    CHECK(map_label::script("Rue d'Été") == Script::Latin);
    CHECK(map_label::script("Café 12") == Script::Latin);
    CHECK(map_label::script("Nguyễn Huệ") == Script::Latin);
    CHECK(map_label::script("תל אביב-יפו") == Script::Hebrew);
    CHECK(map_label::script("רח' הרצל 5") == Script::Hebrew);
    CHECK(map_label::script("Λευκωσία") == Script::Other);
    CHECK(map_label::script("دمشق") == Script::Other);
    CHECK(map_label::script("Москва") == Script::Other);
    CHECK(map_label::script("Roma רומא") == Script::Other);
    CHECK(map_label::script("\xff\xfe") == Script::Other);  // not UTF-8
    // Nicosia: both tags
    CHECK(map_label::pick("Λευκωσία", "Nicosia", "ניקוסיה", true) == "Nicosia");
    CHECK(map_label::pick("Λευκωσία", "Nicosia", "ניקוסיה", false) == "ניקוסיה");
    // Greek only: no label
    CHECK(map_label::pick("Λεμεσός", "", "", true).empty());
    CHECK(map_label::pick("Λεμεσός", "", "", false).empty());
    // an Italian street (local name only, Latin): shown in both
    CHECK(map_label::pick("Via del Corso", "", "", true) == "Via del Corso");
    CHECK(map_label::pick("Via del Corso", "", "", false) == "Via del Corso");
    // an Israeli street with only its Hebrew name: shown in English mode too
    CHECK(map_label::pick("הרצל", "", "", true) == "הרצל");
    // an Arabic name with a Hebrew tag
    CHECK(map_label::pick("دمشق", "", "דמשק", true) == "דמשק");
    CHECK(map_label::pick("دمشق", "Damascus", "דמשק", true) == "Damascus");
}

int main() {
    testTileIds();
    testLabels();
    testBoxCount();
    testCovers();
    testDirectory();
    testGzip();
    testMvt();
    return TEST_MAIN_RESULT();
}
