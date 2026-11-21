#include "pmtiles.hpp"

#include "geo.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace gcs::pmtiles {

namespace {
template <class T>
T rd(const char* p) {
    T v;
    std::memcpy(&v, p, sizeof(T));  // little-endian hosts only (x86, ARM)
    return v;
}
template <class T>
void wr(std::string& s, T v) {
    char b[sizeof(T)];
    std::memcpy(b, &v, sizeof(T));
    s.append(b, sizeof(T));
}

bool readVarint(std::string_view b, size_t& pos, uint64_t& out) {
    out = 0;
    for (int shift = 0; shift < 64; shift += 7) {
        if (pos >= b.size()) {
            return false;
        }
        const uint8_t c = static_cast<uint8_t>(b[pos++]);
        out |= static_cast<uint64_t>(c & 0x7f) << shift;
        if ((c & 0x80) == 0) {
            return true;
        }
    }
    return false;
}
void writeVarint(std::string& s, uint64_t v) {
    while (v >= 0x80) {
        s.push_back(static_cast<char>((v & 0x7f) | 0x80));
        v >>= 7;
    }
    s.push_back(static_cast<char>(v));
}

void rotate(uint64_t n, uint64_t& x, uint64_t& y, uint64_t rx, uint64_t ry) {
    if (ry == 0) {
        if (rx == 1) {
            x = n - 1 - x;
            y = n - 1 - y;
        }
        std::swap(x, y);
    }
}
}  // namespace

std::optional<Header> parseHeader(std::string_view b) {
    if (b.size() < kHeaderSize || b.substr(0, 7) != "PMTiles" || static_cast<uint8_t>(b[7]) != 3) {
        return std::nullopt;
    }
    const char* p = b.data() + 8;
    Header h;
    uint64_t* f[] = {&h.root_offset,   &h.root_length, &h.metadata_offset, &h.metadata_length,
                     &h.leaf_offset,   &h.leaf_length, &h.data_offset,     &h.data_length,
                     &h.addressed_tiles, &h.tile_entries, &h.tile_contents};
    for (uint64_t* v : f) {
        *v = rd<uint64_t>(p);
        p += 8;
    }
    h.clustered = *p++ != 0;
    h.internal_compression = static_cast<uint8_t>(*p++);
    h.tile_compression = static_cast<uint8_t>(*p++);
    h.tile_type = static_cast<uint8_t>(*p++);
    h.min_zoom = static_cast<uint8_t>(*p++);
    h.max_zoom = static_cast<uint8_t>(*p++);
    h.min_lon_e7 = rd<int32_t>(p);
    h.min_lat_e7 = rd<int32_t>(p + 4);
    h.max_lon_e7 = rd<int32_t>(p + 8);
    h.max_lat_e7 = rd<int32_t>(p + 12);
    p += 16;
    h.center_zoom = static_cast<uint8_t>(*p++);
    h.center_lon_e7 = rd<int32_t>(p);
    h.center_lat_e7 = rd<int32_t>(p + 4);
    return h;
}

std::string serializeHeader(const Header& h) {
    std::string s = "PMTiles";
    s.push_back(3);
    for (uint64_t v : {h.root_offset, h.root_length, h.metadata_offset, h.metadata_length, h.leaf_offset, h.leaf_length,
                       h.data_offset, h.data_length, h.addressed_tiles, h.tile_entries, h.tile_contents}) {
        wr(s, v);
    }
    s.push_back(h.clustered ? 1 : 0);
    s.push_back(static_cast<char>(h.internal_compression));
    s.push_back(static_cast<char>(h.tile_compression));
    s.push_back(static_cast<char>(h.tile_type));
    s.push_back(static_cast<char>(h.min_zoom));
    s.push_back(static_cast<char>(h.max_zoom));
    for (int32_t v : {h.min_lon_e7, h.min_lat_e7, h.max_lon_e7, h.max_lat_e7}) {
        wr(s, v);
    }
    s.push_back(static_cast<char>(h.center_zoom));
    wr(s, h.center_lon_e7);
    wr(s, h.center_lat_e7);
    return s;
}

uint64_t zxyToId(uint8_t z, uint32_t x, uint32_t y) {
    uint64_t acc = 0;
    for (uint8_t t = 0; t < z; ++t) {
        acc += (uint64_t{1} << t) * (uint64_t{1} << t);
    }
    const uint64_t n = uint64_t{1} << z;
    uint64_t tx = x, ty = y, d = 0;
    for (uint64_t s = n / 2; s > 0; s /= 2) {
        const uint64_t rx = (tx & s) > 0 ? 1 : 0;
        const uint64_t ry = (ty & s) > 0 ? 1 : 0;
        d += s * s * ((3 * rx) ^ ry);
        rotate(n, tx, ty, rx, ry);
    }
    return acc + d;
}

Zxy idToZxy(uint64_t id) {
    uint64_t acc = 0;
    uint8_t z = 0;
    for (;; ++z) {
        const uint64_t count = (uint64_t{1} << z) * (uint64_t{1} << z);
        if (id < acc + count) {
            break;
        }
        acc += count;
    }
    const uint64_t n = uint64_t{1} << z;
    uint64_t t = id - acc, x = 0, y = 0;
    for (uint64_t s = 1; s < n; s *= 2) {
        const uint64_t rx = 1 & (t / 2);
        const uint64_t ry = 1 & (t ^ rx);
        rotate(s, x, y, rx, ry);
        x += s * rx;
        y += s * ry;
        t /= 4;
    }
    return {z, static_cast<uint32_t>(x), static_cast<uint32_t>(y)};
}

std::optional<std::vector<Entry>> decodeDirectory(std::string_view b) {
    size_t pos = 0;
    uint64_t n = 0;
    if (!readVarint(b, pos, n) || n > b.size()) {
        return std::nullopt;
    }
    std::vector<Entry> e(n);
    uint64_t last = 0, v = 0;
    for (auto& x : e) {
        if (!readVarint(b, pos, v)) return std::nullopt;
        last += v;
        x.tile_id = last;
    }
    for (auto& x : e) {
        if (!readVarint(b, pos, v)) return std::nullopt;
        x.run_length = static_cast<uint32_t>(v);
    }
    for (auto& x : e) {
        if (!readVarint(b, pos, v)) return std::nullopt;
        x.length = static_cast<uint32_t>(v);
    }
    for (size_t i = 0; i < e.size(); ++i) {
        if (!readVarint(b, pos, v)) return std::nullopt;
        e[i].offset = (v == 0 && i > 0) ? e[i - 1].offset + e[i - 1].length : v - 1;
    }
    return e;
}

std::string encodeDirectory(const std::vector<Entry>& e) {
    std::string s;
    writeVarint(s, e.size());
    uint64_t last = 0;
    for (const auto& x : e) {
        writeVarint(s, x.tile_id - last);
        last = x.tile_id;
    }
    for (const auto& x : e) writeVarint(s, x.run_length);
    for (const auto& x : e) writeVarint(s, x.length);
    for (size_t i = 0; i < e.size(); ++i) {
        if (i > 0 && e[i].offset == e[i - 1].offset + e[i - 1].length) {
            writeVarint(s, 0);
        } else {
            writeVarint(s, e[i].offset + 1);
        }
    }
    return s;
}

std::optional<Entry> findEntry(const std::vector<Entry>& dir, uint64_t id) {
    // last entry with tile_id <= id
    auto it = std::upper_bound(dir.begin(), dir.end(), id,
                               [](uint64_t v, const Entry& e) { return v < e.tile_id; });
    if (it == dir.begin()) {
        return std::nullopt;
    }
    const Entry& e = *(it - 1);
    if (e.run_length == 0) {
        return e;  // leaf: look there
    }
    if (id - e.tile_id < e.run_length) {
        return e;
    }
    return std::nullopt;
}

bool covers(const Header& h, int z, int x, int y) {
    if (z < h.min_zoom || z < 0 || z > 30) return false;
    const double n = std::ldexp(1.0, z);
    const double lon0 = x / n * 360.0 - 180.0;
    const double lon1 = (x + 1) / n * 360.0 - 180.0;
    const double lat0 = std::atan(std::sinh(geo::kPi * (1.0 - 2.0 * (y + 1) / n))) * 180.0 / geo::kPi;  // south edge
    const double lat1 = std::atan(std::sinh(geo::kPi * (1.0 - 2.0 * y / n))) * 180.0 / geo::kPi;        // north edge
    // strict: a tile only touching the area's edge isn't in it
    return lon1 > h.min_lon_e7 / 1e7 && lon0 < h.max_lon_e7 / 1e7 && lat1 > h.min_lat_e7 / 1e7 &&
           lat0 < h.max_lat_e7 / 1e7;
}

}  // namespace gcs::pmtiles
