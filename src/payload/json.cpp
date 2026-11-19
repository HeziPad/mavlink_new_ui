#include "payload/json.hpp"

#include <cstdlib>

namespace gcs::payload {

const Json* Json::get(const std::string& key) const {
    if (type != Type::Object) {
        return nullptr;
    }
    for (const auto& kv : object) {
        if (kv.first == key) {
            return &kv.second;
        }
    }
    return nullptr;
}

double Json::num(const std::string& key, double def) const {
    const Json* j = get(key);
    return j && j->isNumber() ? j->number : def;
}

std::string Json::str(const std::string& key, const std::string& def) const {
    const Json* j = get(key);
    return j && j->isString() ? j->string : def;
}

bool Json::flag(const std::string& key, bool def) const {
    const Json* j = get(key);
    return j && j->type == Type::Bool ? j->boolean : def;
}

namespace {

class Parser {
public:
    explicit Parser(const std::string& t) : s_(t) {}

    bool parse(Json& out, std::string& error) {
        ws();
        if (!value(out, 0)) {
            error = "line " + std::to_string(line()) + ": " + err_;
            return false;
        }
        ws();
        if (i_ != s_.size()) {
            error = "line " + std::to_string(line()) + ": trailing characters";
            return false;
        }
        return true;
    }

private:
    const std::string& s_;
    size_t i_ = 0;
    std::string err_;

    int line() const {
        int n = 1;
        for (size_t k = 0; k < i_ && k < s_.size(); ++k) {
            n += s_[k] == '\n';
        }
        return n;
    }
    bool fail(const char* what) {
        if (err_.empty()) {
            err_ = what;
        }
        return false;
    }
    void ws() {
        while (i_ < s_.size()) {
            const char c = s_[i_];
            if (c == ' ' || c == '\t' || c == '\n' || c == '\r') {
                ++i_;
            } else if (c == '/' && i_ + 1 < s_.size() && s_[i_ + 1] == '/') {  // // comments (lenient)
                while (i_ < s_.size() && s_[i_] != '\n') {
                    ++i_;
                }
            } else {
                break;
            }
        }
    }
    bool lit(const char* word) {
        size_t k = 0;
        for (; word[k]; ++k) {
            if (i_ + k >= s_.size() || s_[i_ + k] != word[k]) {
                return false;
            }
        }
        i_ += k;
        return true;
    }
    bool value(Json& v, int depth) {
        if (depth > 64) {
            return fail("nesting too deep");
        }
        if (i_ >= s_.size()) {
            return fail("unexpected end");
        }
        const char c = s_[i_];
        if (c == '{') {
            return obj(v, depth);
        }
        if (c == '[') {
            return arr(v, depth);
        }
        if (c == '"') {
            v.type = Json::Type::String;
            return str(v.string);
        }
        if (lit("true")) {
            v.type = Json::Type::Bool;
            v.boolean = true;
            return true;
        }
        if (lit("false")) {
            v.type = Json::Type::Bool;
            return true;
        }
        if (lit("null")) {
            v.type = Json::Type::Null;
            return true;
        }
        if (c == '-' || (c >= '0' && c <= '9')) {
            const char* begin = s_.c_str() + i_;
            char* end = nullptr;
            v.number = std::strtod(begin, &end);
            if (end == begin) {
                return fail("bad number");
            }
            i_ += static_cast<size_t>(end - begin);
            v.type = Json::Type::Number;
            return true;
        }
        return fail("unexpected character");
    }
    bool obj(Json& v, int depth) {
        v.type = Json::Type::Object;
        ++i_;
        ws();
        if (i_ < s_.size() && s_[i_] == '}') {
            ++i_;
            return true;
        }
        while (true) {
            ws();
            std::string key;
            if (i_ >= s_.size() || s_[i_] != '"' || !str(key)) {
                return fail("expected a key");
            }
            ws();
            if (i_ >= s_.size() || s_[i_] != ':') {
                return fail("expected ':'");
            }
            ++i_;
            ws();
            Json member;
            if (!value(member, depth + 1)) {
                return false;
            }
            v.object.emplace_back(std::move(key), std::move(member));
            ws();
            if (i_ < s_.size() && s_[i_] == ',') {
                ++i_;
                continue;
            }
            if (i_ < s_.size() && s_[i_] == '}') {
                ++i_;
                return true;
            }
            return fail("expected ',' or '}'");
        }
    }
    bool arr(Json& v, int depth) {
        v.type = Json::Type::Array;
        ++i_;
        ws();
        if (i_ < s_.size() && s_[i_] == ']') {
            ++i_;
            return true;
        }
        while (true) {
            ws();
            Json item;
            if (!value(item, depth + 1)) {
                return false;
            }
            v.array.push_back(std::move(item));
            ws();
            if (i_ < s_.size() && s_[i_] == ',') {
                ++i_;
                continue;
            }
            if (i_ < s_.size() && s_[i_] == ']') {
                ++i_;
                return true;
            }
            return fail("expected ',' or ']'");
        }
    }
    static void utf8(std::string& out, unsigned cp) {
        if (cp < 0x80) {
            out += static_cast<char>(cp);
        } else if (cp < 0x800) {
            out += static_cast<char>(0xC0 | (cp >> 6));
            out += static_cast<char>(0x80 | (cp & 0x3F));
        } else if (cp < 0x10000) {
            out += static_cast<char>(0xE0 | (cp >> 12));
            out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
            out += static_cast<char>(0x80 | (cp & 0x3F));
        } else {
            out += static_cast<char>(0xF0 | (cp >> 18));
            out += static_cast<char>(0x80 | ((cp >> 12) & 0x3F));
            out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
            out += static_cast<char>(0x80 | (cp & 0x3F));
        }
    }
    bool hex4(unsigned& cp) {
        if (i_ + 4 > s_.size()) {
            return false;
        }
        cp = 0;
        for (int k = 0; k < 4; ++k) {
            const char h = s_[i_++];
            cp <<= 4;
            if (h >= '0' && h <= '9') {
                cp |= static_cast<unsigned>(h - '0');
            } else if (h >= 'a' && h <= 'f') {
                cp |= static_cast<unsigned>(h - 'a' + 10);
            } else if (h >= 'A' && h <= 'F') {
                cp |= static_cast<unsigned>(h - 'A' + 10);
            } else {
                return false;
            }
        }
        return true;
    }
    bool str(std::string& out) {
        ++i_;  // opening quote
        while (i_ < s_.size()) {
            const char c = s_[i_++];
            if (c == '"') {
                return true;
            }
            if (c != '\\') {
                out += c;
                continue;
            }
            if (i_ >= s_.size()) {
                break;
            }
            const char e = s_[i_++];
            switch (e) {
                case '"': out += '"'; break;
                case '\\': out += '\\'; break;
                case '/': out += '/'; break;
                case 'b': out += '\b'; break;
                case 'f': out += '\f'; break;
                case 'n': out += '\n'; break;
                case 'r': out += '\r'; break;
                case 't': out += '\t'; break;
                case 'u': {
                    unsigned cp = 0;
                    if (!hex4(cp)) {
                        return fail("bad \\u escape");
                    }
                    if (cp >= 0xD800 && cp < 0xDC00 && i_ + 1 < s_.size() && s_[i_] == '\\' && s_[i_ + 1] == 'u') {
                        i_ += 2;
                        unsigned lo = 0;
                        if (!hex4(lo)) {
                            return fail("bad \\u escape");
                        }
                        cp = 0x10000 + ((cp - 0xD800) << 10) + (lo - 0xDC00);
                    }
                    utf8(out, cp);
                    break;
                }
                default: return fail("bad escape");
            }
        }
        return fail("unterminated string");
    }
};

}  // namespace

bool parseJson(const std::string& text, Json& out, std::string& error) {
    out = Json{};
    return Parser(text).parse(out, error);
}

}  // namespace gcs::payload
