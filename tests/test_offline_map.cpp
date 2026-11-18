#include "check.hpp"
#include "core/gzip.hpp"
#include "core/mvt.hpp"
#include "core/pmtiles.hpp"

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

int main() {
    testTileIds();
    testDirectory();
    testGzip();
    testMvt();
    return TEST_MAIN_RESULT();
}
