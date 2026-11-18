#include "mvt.hpp"

#include <cstring>

namespace gcs::mvt {

namespace {

struct Reader {
    const uint8_t* p;
    const uint8_t* end;

    bool done() const { return p >= end; }
    bool varint(uint64_t& v) {
        v = 0;
        for (int shift = 0; shift < 64 && p < end; shift += 7) {
            const uint8_t c = *p++;
            v |= static_cast<uint64_t>(c & 0x7f) << shift;
            if ((c & 0x80) == 0) {
                return true;
            }
        }
        return false;
    }
    bool key(uint32_t& field, uint32_t& wire) {
        uint64_t k;
        if (!varint(k)) return false;
        field = static_cast<uint32_t>(k >> 3);
        wire = static_cast<uint32_t>(k & 7);
        return true;
    }
    bool bytes(std::string_view& out) {
        uint64_t n;
        if (!varint(n) || n > static_cast<uint64_t>(end - p)) return false;
        out = std::string_view(reinterpret_cast<const char*>(p), static_cast<size_t>(n));
        p += n;
        return true;
    }
    bool skip(uint32_t wire) {
        uint64_t v;
        std::string_view s;
        switch (wire) {
        case 0: return varint(v);
        case 1: if (end - p < 8) return false; p += 8; return true;
        case 2: return bytes(s);
        case 5: if (end - p < 4) return false; p += 4; return true;
        default: return false;
        }
    }
};

Reader sub(std::string_view s) {
    return {reinterpret_cast<const uint8_t*>(s.data()), reinterpret_cast<const uint8_t*>(s.data()) + s.size()};
}

int32_t zigzag(uint64_t v) {
    return static_cast<int32_t>((v >> 1) ^ (~(v & 1) + 1));
}

bool packed(std::string_view s, std::vector<uint32_t>& out) {
    Reader r = sub(s);
    while (!r.done()) {
        uint64_t v;
        if (!r.varint(v)) return false;
        out.push_back(static_cast<uint32_t>(v));
    }
    return true;
}

bool decodeValue(std::string_view s, Value& v) {
    Reader r = sub(s);
    while (!r.done()) {
        uint32_t f, w;
        if (!r.key(f, w)) return false;
        uint64_t u;
        std::string_view str;
        if (f == 1 && w == 2) {
            if (!r.bytes(str)) return false;
            v.kind = Value::String;
            v.str.assign(str);
        } else if (f == 2 && w == 5) {
            if (r.end - r.p < 4) return false;
            float x;
            std::memcpy(&x, r.p, 4);
            r.p += 4;
            v.kind = Value::Number;
            v.num = x;
        } else if (f == 3 && w == 1) {
            if (r.end - r.p < 8) return false;
            double x;
            std::memcpy(&x, r.p, 8);
            r.p += 8;
            v.kind = Value::Number;
            v.num = x;
        } else if ((f == 4 || f == 5) && w == 0) {
            if (!r.varint(u)) return false;
            v.kind = Value::Number;
            v.num = f == 4 ? static_cast<double>(static_cast<int64_t>(u)) : static_cast<double>(u);
        } else if (f == 6 && w == 0) {
            if (!r.varint(u)) return false;
            v.kind = Value::Number;
            v.num = zigzag(u);
        } else if (f == 7 && w == 0) {
            if (!r.varint(u)) return false;
            v.kind = Value::Bool;
            v.b = u != 0;
        } else if (!r.skip(w)) {
            return false;
        }
    }
    return true;
}

void geometry(const std::vector<uint32_t>& g, Feature& f) {
    int32_t x = 0, y = 0;  // the cursor carries over between commands and parts
    size_t i = 0;
    std::vector<Point>* cur = nullptr;
    while (i < g.size()) {
        const uint32_t cmd = g[i] & 7, count = g[i] >> 3;
        ++i;
        if (cmd == 1 || cmd == 2) {  // MoveTo / LineTo: count pairs of zigzag deltas
            for (uint32_t k = 0; k < count; ++k) {
                if (i + 2 > g.size()) {
                    return;  // truncated
                }
                x += zigzag(g[i]);
                y += zigzag(g[i + 1]);
                i += 2;
                if (cmd == 1) {
                    // points: every MoveTo point is one part; lines/rings: a new part
                    if (f.type != GeomType::Point || f.parts.empty()) {
                        f.parts.emplace_back();
                    }
                    cur = &f.parts.back();
                }
                if (cur) {
                    cur->push_back({x, y});
                }
            }
        } else if (cmd == 7) {  // ClosePath
            if (cur && !cur->empty()) {
                cur->push_back(cur->front());
            }
        } else {
            return;
        }
    }
}

bool decodeFeature(std::string_view s, Feature& f) {
    Reader r = sub(s);
    std::vector<uint32_t> geom;
    while (!r.done()) {
        uint32_t fi, w;
        if (!r.key(fi, w)) return false;
        std::string_view b;
        uint64_t u;
        if (fi == 2 && w == 2) {
            if (!r.bytes(b) || !packed(b, f.tags)) return false;
        } else if (fi == 3 && w == 0) {
            if (!r.varint(u)) return false;
            f.type = static_cast<GeomType>(u <= 3 ? u : 0);
        } else if (fi == 4 && w == 2) {
            if (!r.bytes(b) || !packed(b, geom)) return false;
        } else if (!r.skip(w)) {
            return false;
        }
    }
    geometry(geom, f);
    return true;
}

bool decodeLayer(std::string_view s, Layer& l) {
    Reader r = sub(s);
    while (!r.done()) {
        uint32_t f, w;
        if (!r.key(f, w)) return false;
        std::string_view b;
        uint64_t u;
        if (f == 1 && w == 2) {
            if (!r.bytes(b)) return false;
            l.name.assign(b);
        } else if (f == 2 && w == 2) {
            if (!r.bytes(b)) return false;
            Feature feat;
            if (!decodeFeature(b, feat)) return false;
            l.features.push_back(std::move(feat));
        } else if (f == 3 && w == 2) {
            if (!r.bytes(b)) return false;
            l.keys.emplace_back(b);
        } else if (f == 4 && w == 2) {
            if (!r.bytes(b)) return false;
            Value v;
            if (!decodeValue(b, v)) return false;
            l.values.push_back(std::move(v));
        } else if (f == 5 && w == 0) {
            if (!r.varint(u)) return false;
            l.extent = static_cast<uint32_t>(u);
        } else if (!r.skip(w)) {
            return false;
        }
    }
    return true;
}

}  // namespace

const Value* Layer::prop(const Feature& f, std::string_view key) const {
    for (size_t i = 0; i + 1 < f.tags.size(); i += 2) {
        if (f.tags[i] < keys.size() && keys[f.tags[i]] == key) {
            return f.tags[i + 1] < values.size() ? &values[f.tags[i + 1]] : nullptr;
        }
    }
    return nullptr;
}

std::string_view Layer::str(const Feature& f, std::string_view key) const {
    const Value* v = prop(f, key);
    return v && v->kind == Value::String ? std::string_view(v->str) : std::string_view();
}

double Layer::num(const Feature& f, std::string_view key, double fallback) const {
    const Value* v = prop(f, key);
    return v && v->kind == Value::Number ? v->num : fallback;
}

const Layer* Tile::layer(std::string_view name) const {
    for (const auto& l : layers) {
        if (l.name == name) return &l;
    }
    return nullptr;
}

bool decode(std::string_view bytes, Tile& out) {
    Reader r = sub(bytes);
    while (!r.done()) {
        uint32_t f, w;
        if (!r.key(f, w)) return false;
        if (f == 3 && w == 2) {
            std::string_view b;
            if (!r.bytes(b)) return false;
            Layer l;
            if (!decodeLayer(b, l)) return false;
            out.layers.push_back(std::move(l));
        } else if (!r.skip(w)) {
            return false;
        }
    }
    return true;
}

}  // namespace gcs::mvt
