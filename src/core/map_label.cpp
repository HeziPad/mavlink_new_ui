#include "map_label.hpp"

#include <cstdint>

namespace gcs::map_label {

namespace {
// next code point of a UTF-8 string (invalid bytes read as U+FFFD)
uint32_t next(std::string_view s, size_t& i) {
    const auto b = [&](size_t k) { return static_cast<uint8_t>(s[k]); };
    const uint8_t c = b(i);
    int extra = c < 0x80 ? 0 : (c >> 5) == 0x6 ? 1 : (c >> 4) == 0xE ? 2 : (c >> 3) == 0x1E ? 3 : -1;
    if (extra < 0 || i + static_cast<size_t>(extra) >= s.size()) {
        ++i;
        return 0xFFFD;
    }
    uint32_t cp = extra == 0 ? c : c & (0x3F >> extra);
    for (int k = 1; k <= extra; ++k) {
        const uint8_t cc = b(i + static_cast<size_t>(k));
        if ((cc >> 6) != 0x2) {
            ++i;
            return 0xFFFD;
        }
        cp = (cp << 6) | (cc & 0x3F);
    }
    i += static_cast<size_t>(extra) + 1;
    return cp;
}

bool neutral(uint32_t cp) {
    return cp < 0x80 ? !((cp >= 'A' && cp <= 'Z') || (cp >= 'a' && cp <= 'z'))
                     : (cp >= 0x00A0 && cp <= 0x00BF) || cp == 0x00D7 || cp == 0x00F7 ||  // nbsp, ·, ×, ÷ ...
                           (cp >= 0x0300 && cp <= 0x036F) ||                              // combining accents
                           (cp >= 0x2000 && cp <= 0x206F) ||                              // general punctuation
                           cp == 0x05BE || cp == 0x05F3 || cp == 0x05F4;                  // maqaf, geresh, gershayim
}
bool latin(uint32_t cp) {
    return cp < 0x80 || (cp >= 0x00C0 && cp <= 0x024F) || (cp >= 0x1E00 && cp <= 0x1EFF);
}
bool hebrew(uint32_t cp) {
    return (cp >= 0x0590 && cp <= 0x05FF) || (cp >= 0xFB1D && cp <= 0xFB4F);
}
}  // namespace

Script script(std::string_view s) {
    bool has_latin = false, has_hebrew = false;
    for (size_t i = 0; i < s.size();) {
        const uint32_t cp = next(s, i);
        if (neutral(cp)) continue;
        if (latin(cp)) {
            has_latin = true;
        } else if (hebrew(cp)) {
            has_hebrew = true;
        } else {
            return Script::Other;
        }
    }
    if (has_latin && has_hebrew) return Script::Other;
    return has_hebrew ? Script::Hebrew : Script::Latin;
}

std::string_view pick(std::string_view name, std::string_view en, std::string_view he, bool english) {
    const std::string_view e = !en.empty() ? en : (!name.empty() && script(name) == Script::Latin ? name : "");
    const std::string_view h = !he.empty() ? he : (!name.empty() && script(name) == Script::Hebrew ? name : "");
    if (english) return !e.empty() ? e : h;
    return !h.empty() ? h : e;
}

}  // namespace gcs::map_label
