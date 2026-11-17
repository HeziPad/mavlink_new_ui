#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

// PMTiles v3 (https://github.com/protomaps/PMTiles/blob/main/spec/v3/spec.md):
// one file holding a whole tile pyramid, addressed by a Hilbert tile id, with
// (possibly compressed) directories pointing at tile bytes. Pure functions on
// byte buffers: reading the file or the network is the caller's job.
namespace gcs::pmtiles {

constexpr size_t kHeaderSize = 127;

enum Compression : uint8_t { kUnknown = 0, kNone = 1, kGzip = 2, kBrotli = 3, kZstd = 4 };
enum TileType : uint8_t { kTypeUnknown = 0, kMvt = 1, kPng = 2, kJpeg = 3, kWebp = 4, kAvif = 5 };

struct Header {
    uint64_t root_offset = 0, root_length = 0;
    uint64_t metadata_offset = 0, metadata_length = 0;
    uint64_t leaf_offset = 0, leaf_length = 0;
    uint64_t data_offset = 0, data_length = 0;
    uint64_t addressed_tiles = 0, tile_entries = 0, tile_contents = 0;
    bool clustered = false;
    uint8_t internal_compression = kGzip;
    uint8_t tile_compression = kGzip;
    uint8_t tile_type = kMvt;
    uint8_t min_zoom = 0, max_zoom = 0;
    int32_t min_lon_e7 = 0, min_lat_e7 = 0, max_lon_e7 = 0, max_lat_e7 = 0;
    uint8_t center_zoom = 0;
    int32_t center_lon_e7 = 0, center_lat_e7 = 0;
};
std::optional<Header> parseHeader(std::string_view bytes);  // needs kHeaderSize bytes
std::string serializeHeader(const Header& h);

// Hilbert tile id: all tiles of lower zooms first, then the Hilbert index.
uint64_t zxyToId(uint8_t z, uint32_t x, uint32_t y);
struct Zxy {
    uint8_t z;
    uint32_t x, y;
};
Zxy idToZxy(uint64_t id);

struct Entry {
    uint64_t tile_id = 0;
    uint64_t offset = 0;     // in the tile data section, or the leaf section for leaves
    uint32_t length = 0;
    uint32_t run_length = 0; // 0 = this entry points at a leaf directory
};
// directory bytes (already decompressed) <-> entries
std::optional<std::vector<Entry>> decodeDirectory(std::string_view bytes);
std::string encodeDirectory(const std::vector<Entry>& entries);
// the entry covering `id`: a tile (run covers it) or the leaf to look in next
std::optional<Entry> findEntry(const std::vector<Entry>& dir, uint64_t id);

}  // namespace gcs::pmtiles
