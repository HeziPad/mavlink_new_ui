#include "payload/icd.hpp"

#include "payload/json.hpp"

#include <algorithm>
#include <cctype>
#include <cstdlib>

namespace gcs::payload {

int Layout::index(const std::string& n) const {
    for (size_t i = 0; i < fields.size(); ++i) {
        if (fields[i].name == n) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

const ReportDef* Icd::report(int rid) const {
    for (const auto& r : reports) {
        if (r.id == rid) {
            return &r;
        }
    }
    return nullptr;
}

const ReportDef* Icd::report(const std::string& n) const {
    for (const auto& r : reports) {
        if (r.name == n) {
            return &r;
        }
    }
    return nullptr;
}

const CommandDef* Icd::command_def(const std::string& n) const {
    for (const auto& c : commands) {
        if (c.name == n) {
            return &c;
        }
    }
    return nullptr;
}

const ConfigParam* Icd::config(int cid) const {
    for (const auto& c : config_params) {
        if (c.id == cid) {
            return &c;
        }
    }
    return nullptr;
}

static bool iequals(const std::string& a, const std::string& b) {
    return a.size() == b.size() && std::equal(a.begin(), a.end(), b.begin(), [](char x, char y) {
               return std::tolower(static_cast<unsigned char>(x)) == std::tolower(static_cast<unsigned char>(y));
           });
}

std::string Icd::enumLabel(const std::string& e, int64_t v) const {
    if (e.empty()) {
        return {};
    }
    if (e == "@config_params") {
        const ConfigParam* c = config(static_cast<int>(v));
        return c ? c->name : std::string();
    }
    if (e == "@reports") {
        const ReportDef* r = report(static_cast<int>(v));
        return r ? r->name : std::string();
    }
    const auto it = enums.find(e);
    if (it == enums.end()) {
        return {};
    }
    for (const auto& kv : it->second) {
        if (kv.first == v) {
            return kv.second;
        }
    }
    return {};
}

std::optional<double> Icd::enumValue(const std::string& e, const std::string& label) const {
    if (e == "@config_params") {
        for (const auto& c : config_params) {
            if (iequals(c.name, label)) {
                return c.id;
            }
        }
        return std::nullopt;
    }
    if (e == "@reports") {
        for (const auto& r : reports) {
            if (iequals(r.name, label)) {
                return r.id;
            }
        }
        return std::nullopt;
    }
    const auto it = enums.find(e);
    if (it != enums.end()) {
        for (const auto& kv : it->second) {
            if (iequals(kv.second, label)) {
                return static_cast<double>(kv.first);
            }
        }
    }
    return std::nullopt;
}

bool Icd::matches(int sysid, int compid) const {
    return (!match_sysid || *match_sysid == sysid) && (!match_compid || *match_compid == compid);
}

bool Icd::isPayloadCommand(int mav_cmd) const {
    return std::any_of(commands.begin(), commands.end(), [&](const CommandDef& c) { return c.command == mav_cmd; });
}

namespace {

bool fieldType(const std::string& t, FieldDef& f) {
    static const std::pair<const char*, std::pair<FieldType, size_t>> scalars[] = {
        {"u8", {FieldType::U8, 1}},   {"i8", {FieldType::I8, 1}},   {"u16", {FieldType::U16, 2}},
        {"i16", {FieldType::I16, 2}}, {"u32", {FieldType::U32, 4}}, {"i32", {FieldType::I32, 4}},
        {"u64", {FieldType::U64, 8}}, {"i64", {FieldType::I64, 8}}, {"f32", {FieldType::F32, 4}},
        {"f64", {FieldType::F64, 8}},
    };
    for (const auto& s : scalars) {
        if (t == s.first) {
            f.type = s.second.first;
            f.size = s.second.second;
            return true;
        }
    }
    const auto lb = t.find('[');
    if (lb == std::string::npos || t.back() != ']') {
        return false;
    }
    const std::string base = t.substr(0, lb);
    const long n = std::strtol(t.c_str() + lb + 1, nullptr, 10);
    if (n <= 0 || n > 255) {
        return false;
    }
    if (base == "char") {
        f.type = FieldType::Chars;
    } else if (base == "u8") {
        f.type = FieldType::Bytes;
    } else {
        return false;
    }
    f.size = static_cast<size_t>(n);
    return true;
}

bool layout(const Json* arr, Layout& out, std::string& error, const std::string& where) {
    if (!arr || !arr->isArray()) {
        error = where + ": 'fields' must be an array";
        return false;
    }
    for (const Json& j : arr->array) {
        FieldDef f;
        f.name = j.str("name");
        if (f.name.empty() || !fieldType(j.str("type"), f)) {
            error = where + ": bad field '" + f.name + "' type '" + j.str("type") + "'";
            return false;
        }
        f.unit = j.str("unit");
        f.enum_name = j.str("enum");
        f.nibbles = j.str("nibbles");
        f.bits = j.str("bits");
        f.desc = j.str("desc");
        if (const Json* inv = j.get("invalid"); inv && inv->isNumber()) {
            f.invalid = inv->number;
        }
        out.size += f.size;
        out.fields.push_back(std::move(f));
    }
    return true;
}

std::optional<double> optNum(const Json& j, const char* key) {
    const Json* v = j.get(key);
    return v && v->isNumber() ? std::optional<double>(v->number) : std::nullopt;
}

}  // namespace

bool loadIcd(const std::string& text, Icd& icd, std::string& error) {
    Json root;
    if (!parseJson(text, root, error)) {
        return false;
    }
    if (root.num("schema") != 1) {
        error = "unsupported ICD schema (want 1)";
        return false;
    }
    icd = Icd{};
    icd.name = root.str("name");
    icd.id = root.str("id", icd.name);
    icd.version = root.str("icd_version");
    icd.description = root.str("description");
    if (const Json* m = root.get("match")) {
        if (const Json* s = m->get("sysid"); s && s->isNumber()) {
            icd.match_sysid = static_cast<int>(s->number);
        }
        if (const Json* c = m->get("compid"); c && c->isNumber()) {
            icd.match_compid = static_cast<int>(c->number);
        }
    }
    if (const Json* a = root.get("annotations"); a && a->isArray()) {
        for (const Json& s : a->array) {
            icd.annotations.push_back(s.string);
        }
    }
    if (const Json* e = root.get("enums"); e && e->isObject()) {
        for (const auto& kv : e->object) {
            auto& vals = icd.enums[kv.first];
            for (const auto& item : kv.second.object) {
                vals.emplace_back(std::strtoll(item.first.c_str(), nullptr, 10), item.second.string);
            }
        }
    }
    if (const Json* b = root.get("bitfields"); b && b->isObject()) {
        for (const auto& kv : b->object) {
            auto& vals = icd.bitfields[kv.first];
            for (const auto& item : kv.second.object) {
                vals.emplace_back(std::atoi(item.first.c_str()), item.second.string);
            }
        }
    }
    const Json* reps = root.get("reports");
    if (!reps) {
        error = "missing 'reports'";
        return false;
    }
    if (const Json* c = reps->get("carrier")) {
        icd.report_msg_id = static_cast<int>(c->num("msg_id", 248));
        icd.report_id_bytes = static_cast<int>(c->num("id_bytes", 2));
    }
    const Json* types = reps->get("types");
    if (!types || !types->isArray()) {
        error = "missing 'reports.types'";
        return false;
    }
    for (const Json& r : types->array) {
        ReportDef d;
        d.id = static_cast<int>(r.num("id", -1));
        d.name = r.str("name");
        d.default_hz = r.num("default_hz");
        d.event = r.flag("event");
        if (!layout(r.get("fields"), d.header, error, "report " + d.name)) {
            return false;
        }
        if (const Json* recs = r.get("records")) {
            d.has_records = true;
            d.records_name = recs->str("name", "records");
            d.records_max = static_cast<int>(recs->num("max", 1));
            if (!layout(recs->get("fields"), d.records, error, "report " + d.name + " records")) {
                return false;
            }
            if (static_cast<size_t>(recs->num("size")) != d.records.size) {
                error = "report " + d.name + ": record fields give " + std::to_string(d.records.size) +
                        " bytes, declared " + std::to_string(static_cast<int>(recs->num("size")));
                return false;
            }
        }
        if (const auto len = optNum(r, "length");
            len && static_cast<size_t>(*len) != d.header.size + static_cast<size_t>(icd.report_id_bytes)) {
            error = "report " + d.name + ": fields give " +
                    std::to_string(d.header.size + static_cast<size_t>(icd.report_id_bytes)) + " bytes, declared " +
                    std::to_string(static_cast<int>(*len));
            return false;
        }
        icd.reports.push_back(std::move(d));
    }
    const Json* cmds = root.get("commands");
    if (!cmds) {
        error = "missing 'commands'";
        return false;
    }
    if (const Json* c = cmds->get("carrier")) {
        icd.command = static_cast<int>(c->num("command", 203));
        icd.target_system = static_cast<int>(c->num("target_system", 1));
        icd.target_component = static_cast<int>(c->num("target_component", 0));
    }
    if (const Json* a = cmds->get("ack")) {
        icd.ack_timeout_ms = static_cast<int>(a->num("timeout_ms", 2000));
        icd.ack_retries = static_cast<int>(a->num("retries", 3));
    }
    const Json* list = cmds->get("list");
    if (!list || !list->isArray()) {
        error = "missing 'commands.list'";
        return false;
    }
    for (const Json& c : list->array) {
        CommandDef d;
        d.name = c.str("name");
        d.command = static_cast<int>(c.num("command", icd.command));
        if (const auto v = optNum(c, "id")) {
            d.id = static_cast<int>(*v);
        }
        if (const auto v = optNum(c, "sub")) {
            d.sub = static_cast<int>(*v);
        }
        const std::string ack = c.str("ack", "ack");
        if (ack == "none") {
            d.ack = AckPolicy::None;
        } else if (ack.rfind("report:", 0) == 0) {
            d.ack = AckPolicy::Report;
            d.ack_report = std::atoi(ack.c_str() + 7);
        }
        d.ack_timeout_ms = static_cast<int>(c.num("ack_timeout_ms", 0));
        d.periodic_hz = c.num("periodic_hz");
        d.stream_hz = c.num("stream_hz");
        d.throttle_ms = static_cast<int>(c.num("throttle_ms"));
        if (const Json* ps = c.get("params"); ps && ps->isArray()) {
            for (const Json& p : ps->array) {
                ParamDef pd;
                pd.name = p.str("name");
                pd.enum_name = p.str("enum");
                pd.bits = p.str("bits");
                pd.unit = p.str("unit");
                pd.desc = p.str("desc");
                pd.min = optNum(p, "min");
                pd.max = optNum(p, "max");
                pd.def = p.num("default");
                d.params.push_back(std::move(pd));
            }
        }
        if (d.firstParam() + static_cast<int>(d.params.size()) > 7) {
            error = "command " + d.name + ": more than 7 params";
            return false;
        }
        icd.commands.push_back(std::move(d));
    }
    if (const Json* cps = root.get("config_params"); cps && cps->isArray()) {
        for (const Json& p : cps->array) {
            ConfigParam cp;
            cp.id = static_cast<int>(p.num("id"));
            cp.name = p.str("name");
            cp.kind = p.str("kind");
            cp.channel = p.flag("channel");
            cp.get_only = p.flag("get_only");
            cp.min = optNum(p, "min");
            cp.max = optNum(p, "max");
            icd.config_params.push_back(std::move(cp));
        }
    }
    return true;
}

}  // namespace gcs::payload
