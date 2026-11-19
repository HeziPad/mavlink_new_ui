#include "hud_config.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <initializer_list>

namespace gcs::hud {

namespace {

constexpr const char* kPresetNames[kPresets] = {"clean", "custom", "full"};
// v1 had a RULER REF element here (now in the HUD panel, not on the video)
constexpr size_t kV1RefChipBit = 11;

constexpr const char* kCrossNames[kCrosshairs] = {"cross", "dot", "circle", "duplex", "brackets", "fine", "measure", "none"};

void setShown(Config& c, std::initializer_list<Element> on) {
    c.show.fill(false);
    for (Element e : on) {
        c.setShown(e, true);
    }
}

// whole-number field: false if it isn't one
bool parseInt(const std::string& v, long& out) {
    if (v.empty() || v.size() > 12) {
        return false;
    }
    char* end = nullptr;
    out = std::strtol(v.c_str(), &end, 10);
    return end && *end == '\0';
}

template <size_t N>
void parseBits(const std::string& v, std::array<bool, N>& out) {
    for (size_t i = 0; i < N && i < v.size(); ++i) {
        if (v[i] == '0' || v[i] == '1') {
            out[i] = v[i] == '1';
        }
    }
}

template <size_t N>
std::string bits(const std::array<bool, N>& a) {
    std::string s;
    for (bool b : a) {
        s += b ? '1' : '0';
    }
    return s;
}

}  // namespace

double Config::lineFactor() const {
    return line <= 0 ? 0.7 : (line >= 2 ? 1.5 : 1.0);
}

bool Config::operator==(const Config& o) const {
    return preset == o.preset && custom_show == o.custom_show && custom_collapsed == o.custom_collapsed &&
           show == o.show && crosshair == o.crosshair && line == o.line && color == o.color &&
           collapsed == o.collapsed && std::abs(measure_w_deg - o.measure_w_deg) < 1e-6 &&
           std::abs(measure_h_deg - o.measure_h_deg) < 1e-6;
}

Config presetConfig(Preset p, const Config& keep) {
    Config c = keep;
    c.preset = p;
    switch (p) {
    case Preset::Clean:
        setShown(c, {});
        c.collapsed.fill(true);
        break;
    case Preset::Full:
        setShown(c, {Element::Ladder, Element::RollScale, Element::HeadingTape, Element::SpeedTape, Element::AltTape,
                     Element::PathMarker, Element::StatusLine, Element::TargetBox, Element::AircraftBox,
                     Element::LosBox, Element::WindBox, Element::SensorFx});
        c.collapsed.fill(false);
        break;
    case Preset::Custom:
        setShown(c, {Element::HeadingTape, Element::StatusLine, Element::TargetBox, Element::AircraftBox,
                     Element::LosBox, Element::WindBox});
        c.collapsed.fill(true);
        c.setCollapsed(Box::Target, false);  // the one box that matters for looking at things
        break;
    }
    return c;
}

Config defaultConfig() {
    Config c = presetConfig(Preset::Custom);
    c.custom_show = c.show;
    c.custom_collapsed = c.collapsed;
    return c;
}

void applyPreset(Config& c, Preset p) {
    if (p == Preset::Custom) {
        c.preset = Preset::Custom;
        c.show = c.custom_show;
        c.collapsed = c.custom_collapsed;
    } else {
        c = presetConfig(p, c);
    }
}

void editShown(Config& c, Element e, bool on) {
    if (c.preset != Preset::Custom) {
        c.preset = Preset::Custom;
        c.custom_show = c.show;
        c.custom_collapsed = c.collapsed;
    }
    c.setShown(e, on);
    c.custom_show[static_cast<size_t>(e)] = on;
}

void editCollapsed(Config& c, Box b, bool on) {
    c.setCollapsed(b, on);
    if (c.preset == Preset::Custom) {
        c.custom_collapsed[static_cast<size_t>(b)] = on;
    }
}

const char* presetName(Preset p) {
    switch (p) {
    case Preset::Clean: return "CLEAN";
    case Preset::Full: return "FULL";
    case Preset::Custom: break;
    }
    return "CUSTOM";
}

const char* crosshairName(Crosshair c) {
    const int i = static_cast<int>(c);
    return i >= 0 && i < kCrosshairs ? kCrossNames[i] : "cross";
}

std::string encode(const Config& c) {
    std::string s = "v2;preset=";
    s += kPresetNames[std::clamp(static_cast<int>(c.preset), 0, kPresets - 1)];
    s += ";show=" + bits(c.show);
    s += ";fold=" + bits(c.collapsed);
    s += ";cshow=" + bits(c.custom_show);
    s += ";cfold=" + bits(c.custom_collapsed);
    s += ";x=";
    s += crosshairName(c.crosshair);
    s += ";line=" + std::to_string(c.line);
    s += ";color=" + std::to_string(c.color);
    s += ";mw=" + std::to_string(std::lround(c.measure_w_deg * 1000.0));
    s += ";mh=" + std::to_string(std::lround(c.measure_h_deg * 1000.0));
    return s;
}

Config decode(const std::string& s) {
    Config c = defaultConfig();
    bool v2 = false, has_preset = false, has_custom = false;
    size_t pos = 0;
    while (pos <= s.size()) {
        size_t end = s.find(';', pos);
        if (end == std::string::npos) {
            end = s.size();
        }
        const std::string field = s.substr(pos, end - pos);
        pos = end + 1;
        if (field == "v2") {
            v2 = true;
            continue;
        }
        const size_t eq = field.find('=');
        if (eq == std::string::npos) {
            continue;
        }
        const std::string k = field.substr(0, eq);
        std::string v = field.substr(eq + 1);
        long n = 0;
        if (k == "preset") {
            for (int i = 0; i < kPresets; ++i) {
                if (v == kPresetNames[i]) {
                    c.preset = static_cast<Preset>(i);
                    has_preset = true;
                }
            }
        } else if (k == "show") {
            if (!v2 && v.size() > kV1RefChipBit + 1) {
                v.erase(kV1RefChipBit, 1);
            }
            parseBits(v, c.show);
        } else if (k == "fold") {
            parseBits(v, c.collapsed);
        } else if (k == "cshow") {
            parseBits(v, c.custom_show);
            has_custom = true;
        } else if (k == "cfold") {
            parseBits(v, c.custom_collapsed);
            has_custom = true;
        } else if (k == "x") {
            for (int i = 0; i < kCrosshairs; ++i) {
                if (v == kCrossNames[i]) {
                    c.crosshair = static_cast<Crosshair>(i);
                }
            }
        } else if (k == "line" && parseInt(v, n)) {
            c.line = static_cast<int>(std::clamp(n, 0L, static_cast<long>(kLineWidths - 1)));
        } else if (k == "color" && parseInt(v, n)) {
            c.color = static_cast<int>(std::clamp(n, 0L, static_cast<long>(kColors - 1)));
        } else if (k == "mw" && parseInt(v, n)) {
            c.measure_w_deg = std::clamp(n / 1000.0, 0.1, 170.0);
        } else if (k == "mh" && parseInt(v, n)) {
            c.measure_h_deg = std::clamp(n / 1000.0, 0.1, 170.0);
        }
    }
    if (!has_preset) {
        // v1: the preset is whichever fixed set it matches; anything else is the CUSTOM mix
        c.preset = Preset::Custom;
        for (Preset p : {Preset::Clean, Preset::Full}) {
            const Config want = presetConfig(p, c);
            if (want.show == c.show && want.collapsed == c.collapsed) {
                c.preset = p;
            }
        }
    }
    if (c.preset == Preset::Custom && !has_custom) {
        c.custom_show = c.show;
        c.custom_collapsed = c.collapsed;
    }
    return c;
}

double clampMeasureDeg(double deg, double hfov_deg) {
    return std::clamp(deg, 0.1, std::max(0.2, 0.9 * hfov_deg));
}

}  // namespace gcs::hud
