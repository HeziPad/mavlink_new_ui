#pragma once

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace gcs::payload {

// A small JSON reader for the payload ICD files (pure C++, no I/O): objects
// keep their key order, numbers are doubles. Good for config-sized documents.
struct Json {
    enum class Type { Null, Bool, Number, String, Array, Object };
    Type type = Type::Null;
    bool boolean = false;
    double number = 0.0;
    std::string string;
    std::vector<Json> array;
    std::vector<std::pair<std::string, Json>> object;

    bool isNull() const { return type == Type::Null; }
    bool isNumber() const { return type == Type::Number; }
    bool isString() const { return type == Type::String; }
    bool isArray() const { return type == Type::Array; }
    bool isObject() const { return type == Type::Object; }

    // object member or nullptr
    const Json* get(const std::string& key) const;
    // typed member access with a default
    double num(const std::string& key, double def = 0.0) const;
    std::string str(const std::string& key, const std::string& def = {}) const;
    bool flag(const std::string& key, bool def = false) const;
};

// false + error ("line 3: expected ':'") on malformed input
bool parseJson(const std::string& text, Json& out, std::string& error);

}  // namespace gcs::payload
