#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace gcs::payload {

// A payload ICD read from icds/<id>.json (schema 1, see icds/README.md):
// how a payload's commands ride in COMMAND_LONG and how its reports ride in a
// carrier message (V2_EXTENSION), with every field's type and meaning. The
// same file drives the payload simulators, so both sides share one layout.

enum class FieldType { U8, I8, U16, I16, U32, I32, U64, I64, F32, F64, Chars, Bytes };

struct FieldDef {
    std::string name;
    FieldType type = FieldType::U8;
    size_t size = 1;          // bytes
    std::string unit;
    std::string enum_name;    // value labels: an enum, "@config_params" or "@reports"
    std::string nibbles;      // per-channel nibbles labelled by this enum
    std::string bits;         // bitfield labels
    std::optional<double> invalid;  // "no value" marker (NextVision: 400.0)
    std::string desc;
};

struct Layout {
    std::vector<FieldDef> fields;
    size_t size = 0;
    int index(const std::string& name) const;  // -1 when absent
};

struct ReportDef {
    int id = 0;
    std::string name;
    double default_hz = 0.0;
    bool event = false;       // sent on events, not periodically
    Layout header;
    bool has_records = false;
    std::string records_name;
    Layout records;
    int records_max = 0;
};

struct ParamDef {
    std::string name;
    std::string enum_name;
    std::string bits;
    std::string unit;
    std::string desc;
    std::optional<double> min, max;
    double def = 0.0;
};

enum class AckPolicy { Ack, None, Report };

struct CommandDef {
    std::string name;
    int command = 0;               // MAV_CMD
    std::optional<int> id;         // selector (param1)
    std::optional<int> sub;        // sub-selector (param2)
    std::vector<ParamDef> params;
    AckPolicy ack = AckPolicy::Ack;
    int ack_report = -1;           // AckPolicy::Report: the report that answers it
    int ack_timeout_ms = 0;        // 0: the ICD's default
    double periodic_hz = 0.0;      // must be repeated at this rate (a keep-alive)
    double stream_hz = 0.0;        // meant to be streamed (joystick)
    int throttle_ms = 0;           // the payload refuses repeats closer than this
    // param index (0-based, into param1..param7) of params[0]
    int firstParam() const { return id ? (sub ? 2 : 1) : 0; }
};

struct ConfigParam {
    int id = 0;
    std::string name;
    std::string kind;              // "", "ip", "key"
    bool channel = false;
    bool get_only = false;
    std::optional<double> min, max;
};

struct Icd {
    std::string id;                // file id, e.g. "nextvision_trip"
    std::string name;
    std::string version;
    std::string description;
    std::optional<int> match_sysid, match_compid;   // none: any
    // carrier of the reports
    int report_msg_id = 248;
    int report_id_bytes = 2;
    // carrier of the commands
    int command = 203;
    int target_system = 1;
    int target_component = 0;
    int ack_timeout_ms = 2000;
    int ack_retries = 3;
    std::map<std::string, std::vector<std::pair<int64_t, std::string>>> enums;
    std::map<std::string, std::vector<std::pair<int, std::string>>> bitfields;
    std::vector<ReportDef> reports;
    std::vector<CommandDef> commands;
    std::vector<ConfigParam> config_params;
    std::vector<std::string> annotations;

    const ReportDef* report(int id) const;
    const ReportDef* report(const std::string& name) const;
    const CommandDef* command_def(const std::string& name) const;
    const ConfigParam* config(int id) const;
    // label of value `v` in enum `e` ("@config_params" / "@reports" too); empty when none
    std::string enumLabel(const std::string& e, int64_t v) const;
    // value for a label (case-insensitive); nullopt when unknown
    std::optional<double> enumValue(const std::string& e, const std::string& label) const;
    bool matches(int sysid, int compid) const;
    bool isPayloadCommand(int mav_cmd) const;
};

// false + error on a malformed file (bad JSON, unknown type, a declared length
// that disagrees with the fields...)
bool loadIcd(const std::string& json_text, Icd& out, std::string& error);

}  // namespace gcs::payload
