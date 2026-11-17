#pragma once

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

// Mapbox Vector Tile decoding (spec 2.1): layers of features with tags and
// geometry. Coordinates stay in tile units (0..extent, possibly a little
// outside: the tile buffer). Qt-free so it can be unit-tested and run on a
// worker thread.
namespace gcs::mvt {

struct Value {
    enum Kind : uint8_t { None, String, Number, Bool } kind = None;
    std::string str;
    double num = 0.0;
    bool b = false;
};

struct Point {
    int32_t x, y;
};

enum class GeomType : uint8_t { Unknown = 0, Point = 1, Line = 2, Polygon = 3 };

struct Feature {
    GeomType type = GeomType::Unknown;
    std::vector<uint32_t> tags;               // key index, value index pairs
    std::vector<std::vector<Point>> parts;    // points: one part per point group; lines; polygon rings
};

struct Layer {
    std::string name;
    uint32_t extent = 4096;
    std::vector<std::string> keys;
    std::vector<Value> values;
    std::vector<Feature> features;

    // the value of `key` on feature `f`, or nullptr
    const Value* prop(const Feature& f, std::string_view key) const;
    // string property ("" when missing / not a string)
    std::string_view str(const Feature& f, std::string_view key) const;
    double num(const Feature& f, std::string_view key, double fallback = 0.0) const;
};

struct Tile {
    std::vector<Layer> layers;
    const Layer* layer(std::string_view name) const;
};

// Decodes an uncompressed tile; false on malformed data (what was read is kept).
bool decode(std::string_view bytes, Tile& out);

}  // namespace gcs::mvt
