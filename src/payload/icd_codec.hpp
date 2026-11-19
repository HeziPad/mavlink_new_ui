#pragma once

#include "payload/icd.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace gcs::payload {

// One decoded field value. Integers keep their exact value; num() is the
// common view for maths and display.
struct Value {
    enum class Kind { Int, UInt, Float, Text, Bytes };
    Kind kind = Kind::Int;
    int64_t i = 0;
    uint64_t u = 0;
    double f = 0.0;
    std::string text;
    std::vector<uint8_t> bytes;
    double num() const;
};

struct DecodedReport {
    const ReportDef* def = nullptr;
    int type = -1;
    std::vector<Value> fields;                  // def->header.fields order
    std::vector<std::vector<Value>> records;    // def->records.fields order
    size_t payload_len = 0;

    // header field by name (0 / nullopt when absent)
    std::optional<double> get(const std::string& name) const;
    double num(const std::string& name, double def = 0.0) const { return get(name).value_or(def); }
    std::string text(const std::string& name) const;
};

// payload = the carrier's payload from the report-type bytes on (V2_EXTENSION:
// message_type first). Short payloads (MAVLink 2 trailing-zero truncation)
// read as zeros; extra trailing bytes are ignored (forward compatibility).
// false: shorter than the type bytes, or an unknown report type.
bool decodeReport(const Icd& icd, const uint8_t* payload, size_t len, DecodedReport& out);

// The other direction, for tests and tools: named values, missing ones = 0.
std::vector<uint8_t> encodeReport(const Icd& icd, const ReportDef& def,
                                  const std::vector<std::pair<std::string, double>>& values,
                                  const std::vector<std::vector<std::pair<std::string, double>>>& records = {});

struct EncodedCommand {
    int command = 0;
    std::array<float, 7> params{};
    int target_system = 1;
    int target_component = 0;
};

// args in params order; missing ones take the param's default.
EncodedCommand encodeCommand(const Icd& icd, const CommandDef& def, const std::vector<double>& args);

// The definition a COMMAND_LONG matches (nullptr: not one of this ICD's)
const CommandDef* matchCommand(const Icd& icd, int command, const std::array<float, 7>& params);

// "12.5 deg", "9 GlobalPosition", "--" (invalid marker), "ch0 Idle · ch1 Recording"
std::string formatValue(const Icd& icd, const FieldDef& f, const Value& v);

}  // namespace gcs::payload
