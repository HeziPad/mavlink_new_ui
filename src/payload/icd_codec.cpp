#include "payload/icd_codec.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace gcs::payload {

double Value::num() const {
    switch (kind) {
        case Kind::Int: return static_cast<double>(i);
        case Kind::UInt: return static_cast<double>(u);
        case Kind::Float: return f;
        default: return 0.0;
    }
}

std::optional<double> DecodedReport::get(const std::string& name) const {
    if (!def) {
        return std::nullopt;
    }
    const int k = def->header.index(name);
    if (k < 0 || static_cast<size_t>(k) >= fields.size()) {
        return std::nullopt;
    }
    return fields[static_cast<size_t>(k)].num();
}

std::string DecodedReport::text(const std::string& name) const {
    if (!def) {
        return {};
    }
    const int k = def->header.index(name);
    return k < 0 ? std::string() : fields[static_cast<size_t>(k)].text;
}

namespace {

// little-endian reads (the ICDs are little-endian, like MAVLink)
template <typename T>
T rd(const uint8_t* p) {
    T v{};
    uint8_t b[sizeof(T)];
    for (size_t k = 0; k < sizeof(T); ++k) {
        b[k] = p[k];
    }
    std::memcpy(&v, b, sizeof(T));  // hosts are little-endian (x86, ARM)
    return v;
}

Value readField(const FieldDef& f, const uint8_t* p) {
    Value v;
    switch (f.type) {
        case FieldType::U8: v.kind = Value::Kind::UInt; v.u = p[0]; break;
        case FieldType::I8: v.i = static_cast<int8_t>(p[0]); break;
        case FieldType::U16: v.kind = Value::Kind::UInt; v.u = rd<uint16_t>(p); break;
        case FieldType::I16: v.i = rd<int16_t>(p); break;
        case FieldType::U32: v.kind = Value::Kind::UInt; v.u = rd<uint32_t>(p); break;
        case FieldType::I32: v.i = rd<int32_t>(p); break;
        case FieldType::U64: v.kind = Value::Kind::UInt; v.u = rd<uint64_t>(p); break;
        case FieldType::I64: v.i = rd<int64_t>(p); break;
        case FieldType::F32: v.kind = Value::Kind::Float; v.f = rd<float>(p); break;
        case FieldType::F64: v.kind = Value::Kind::Float; v.f = rd<double>(p); break;
        case FieldType::Chars: {
            v.kind = Value::Kind::Text;
            const auto* c = reinterpret_cast<const char*>(p);
            v.text.assign(c, strnlen(c, f.size));
            break;
        }
        case FieldType::Bytes:
            v.kind = Value::Kind::Bytes;
            v.bytes.assign(p, p + f.size);
            break;
    }
    return v;
}

void readLayout(const Layout& l, const uint8_t* data, size_t len, std::vector<Value>& out) {
    // zero-fill what MAVLink 2 truncated
    std::vector<uint8_t> buf(l.size, 0);
    std::memcpy(buf.data(), data, std::min(len, l.size));
    out.clear();
    size_t off = 0;
    for (const auto& f : l.fields) {
        out.push_back(readField(f, buf.data() + off));
        off += f.size;
    }
}

template <typename T>
void wr(std::vector<uint8_t>& out, T v) {
    uint8_t b[sizeof(T)];
    std::memcpy(b, &v, sizeof(T));
    out.insert(out.end(), b, b + sizeof(T));
}

void writeField(std::vector<uint8_t>& out, const FieldDef& f, double v) {
    switch (f.type) {
        case FieldType::U8: out.push_back(static_cast<uint8_t>(v)); break;
        case FieldType::I8: out.push_back(static_cast<uint8_t>(static_cast<int8_t>(v))); break;
        case FieldType::U16: wr(out, static_cast<uint16_t>(v)); break;
        case FieldType::I16: wr(out, static_cast<int16_t>(v)); break;
        case FieldType::U32: wr(out, static_cast<uint32_t>(v)); break;
        case FieldType::I32: wr(out, static_cast<int32_t>(v)); break;
        case FieldType::U64: wr(out, static_cast<uint64_t>(v)); break;
        case FieldType::I64: wr(out, static_cast<int64_t>(v)); break;
        case FieldType::F32: wr(out, static_cast<float>(v)); break;
        case FieldType::F64: wr(out, v); break;
        case FieldType::Chars:
        case FieldType::Bytes: out.insert(out.end(), f.size, 0); break;
    }
}

void writeLayout(std::vector<uint8_t>& out, const Layout& l, const std::vector<std::pair<std::string, double>>& vals) {
    for (const auto& f : l.fields) {
        double v = 0.0;
        for (const auto& kv : vals) {
            if (kv.first == f.name) {
                v = kv.second;
            }
        }
        writeField(out, f, v);
    }
}

std::string fmtNum(double v) {
    char b[48];
    if (std::fabs(v) >= 1e6 || (v != 0 && std::fabs(v) < 1e-4)) {
        std::snprintf(b, sizeof b, "%.6g", v);
    } else if (std::fabs(v - std::round(v)) < 1e-9) {
        std::snprintf(b, sizeof b, "%.0f", v);
    } else {
        std::snprintf(b, sizeof b, "%.7f", v);
        std::string s(b);
        while (!s.empty() && s.back() == '0') {
            s.pop_back();
        }
        if (!s.empty() && s.back() == '.') {
            s.pop_back();
        }
        return s;
    }
    return b;
}

}  // namespace

bool decodeReport(const Icd& icd, const uint8_t* payload, size_t len, DecodedReport& out) {
    const size_t nid = static_cast<size_t>(icd.report_id_bytes);
    if (len < nid) {
        return false;
    }
    const int type = nid == 1 ? payload[0] : rd<uint16_t>(payload);
    const ReportDef* def = icd.report(type);
    if (!def) {
        return false;
    }
    out = DecodedReport{};
    out.def = def;
    out.type = type;
    out.payload_len = len;
    const uint8_t* body = payload + nid;
    const size_t blen = len - nid;
    readLayout(def->header, body, blen, out.fields);
    if (def->has_records && blen > def->header.size && def->records.size > 0) {
        const size_t rest = blen - def->header.size;
        // ceil: a last record whose trailing zeros MAVLink 2 truncated still counts
        const size_t n = std::min(static_cast<size_t>(def->records_max),
                                  (rest + def->records.size - 1) / def->records.size);
        for (size_t k = 0; k < n; ++k) {
            const size_t off = def->header.size + k * def->records.size;
            out.records.emplace_back();
            readLayout(def->records, body + off, blen - off, out.records.back());
        }
    }
    return true;
}

std::vector<uint8_t> encodeReport(const Icd& icd, const ReportDef& def,
                                  const std::vector<std::pair<std::string, double>>& values,
                                  const std::vector<std::vector<std::pair<std::string, double>>>& records) {
    std::vector<uint8_t> out;
    if (icd.report_id_bytes == 1) {
        out.push_back(static_cast<uint8_t>(def.id));
    } else {
        wr(out, static_cast<uint16_t>(def.id));
    }
    writeLayout(out, def.header, values);
    for (size_t k = 0; def.has_records && k < records.size() && static_cast<int>(k) < def.records_max; ++k) {
        writeLayout(out, def.records, records[k]);
    }
    return out;
}

EncodedCommand encodeCommand(const Icd& icd, const CommandDef& def, const std::vector<double>& args) {
    EncodedCommand e;
    e.command = def.command;
    e.target_system = icd.target_system;
    e.target_component = icd.target_component;
    if (def.id) {
        e.params[0] = static_cast<float>(*def.id);
        if (def.sub) {
            e.params[1] = static_cast<float>(*def.sub);
        }
    }
    const int base = def.firstParam();
    for (size_t k = 0; k < def.params.size(); ++k) {
        const double v = k < args.size() ? args[k] : def.params[k].def;
        e.params[static_cast<size_t>(base) + k] = static_cast<float>(v);
    }
    return e;
}

const CommandDef* matchCommand(const Icd& icd, int command, const std::array<float, 7>& params) {
    const auto sel = static_cast<int>(std::lround(params[0]));
    const auto sub = static_cast<int>(std::lround(params[1]));
    const CommandDef* fallback = nullptr;
    for (const auto& c : icd.commands) {
        if (c.command != command) {
            continue;
        }
        if (!c.id) {
            return &c;
        }
        if (*c.id != sel) {
            continue;
        }
        if (!c.sub) {
            fallback = &c;
        } else if (*c.sub == sub) {
            return &c;
        }
    }
    return fallback;
}

std::string formatValue(const Icd& icd, const FieldDef& f, const Value& v) {
    if (v.kind == Value::Kind::Text) {
        return v.text;
    }
    if (v.kind == Value::Kind::Bytes) {
        std::string s;
        char b[4];
        for (uint8_t x : v.bytes) {
            std::snprintf(b, sizeof b, "%02x", x);
            s += b;
        }
        return s;
    }
    const double n = v.num();
    if (f.invalid && std::fabs(n - *f.invalid) < 1e-3) {
        return "--";
    }
    if (!f.nibbles.empty()) {
        const auto x = static_cast<int64_t>(n);
        std::string a = icd.enumLabel(f.nibbles, x & 0xF), b = icd.enumLabel(f.nibbles, (x >> 4) & 0xF);
        return "ch0 " + (a.empty() ? std::to_string(x & 0xF) : a) + " · ch1 " +
               (b.empty() ? std::to_string((x >> 4) & 0xF) : b);
    }
    if (!f.bits.empty()) {
        const auto it = icd.bitfields.find(f.bits);
        const auto x = static_cast<uint64_t>(n);
        std::string s;
        if (it != icd.bitfields.end()) {
            for (const auto& kv : it->second) {
                if (x & (1ull << kv.first)) {
                    s += (s.empty() ? "" : "|") + kv.second;
                }
            }
        }
        return s.empty() ? (x ? fmtNum(n) : std::string("none")) : s;
    }
    std::string s = fmtNum(n);
    const std::string label = icd.enumLabel(f.enum_name, static_cast<int64_t>(n));
    if (!label.empty()) {
        return s + " " + label;
    }
    if (!f.unit.empty()) {
        s += " " + f.unit;
    }
    return s;
}

}  // namespace gcs::payload
