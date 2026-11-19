#include "hud_config.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <initializer_list>

namespace gcs::hud {

namespace {

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
    return show == o.show && crosshair == o.crosshair && line == o.line && color == o.color &&
           collapsed == o.collapsed && std::abs(measure_w_deg - o.measure_w_deg) < 1e-6 &&
           std::abs(measure_h_deg - o.measure_h_deg) < 1e-6;
}

Config presetConfig(Preset p, const Config& keep_style) {
    Config c = keep_style;
    switch (p) {
    case Preset::Clean:
        setShown(c, {});
        c.collapsed.fill(true);
        break;
    case Preset::Full:
        setShown(c, {Element::Ladder, Element::RollScale, Element::HeadingTape, Element::SpeedTape, Element::AltTape,
                     Element::PathMarker, Element::StatusLine, Element::TargetBox, Element::AircraftBox,
                     Element::LosBox, Element::WindBox, Element::RefChip, Element::SensorFx});
        c.collapsed.fill(false);
        break;
    case Preset::Standard:
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
    return presetConfig(Preset::Standard);
}

Preset matchingPreset(const Config& c) {
    for (Preset p : {Preset::Standard, Preset::Clean, Preset::Full}) {
        const Config want = presetConfig(p, c);
        if (want.show == c.show && want.collapsed == c.collapsed) {
            return p;
        }
    }
    return Preset::Custom;
}

const char* presetName(Preset p) {
    switch (p) {
    case Preset::Clean: return "CLEAN";
    case Preset::Standard: return "STANDARD";
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
    std::string s = "v1;show=" + bits(c.show);
    s += ";x=";
    s += crosshairName(c.crosshair);
    s += ";line=" + std::to_string(c.line);
    s += ";color=" + std::to_string(c.color);
    s += ";fold=" + bits(c.collapsed);
    s += ";mw=" + std::to_string(std::lround(c.measure_w_deg * 1000.0));
    s += ";mh=" + std::to_string(std::lround(c.measure_h_deg * 1000.0));
    return s;
}

Config decode(const std::string& s) {
    Config c = defaultConfig();
    size_t pos = 0;
    while (pos <= s.size()) {
        size_t end = s.find(';', pos);
        if (end == std::string::npos) {
            end = s.size();
        }
        const std::string field = s.substr(pos, end - pos);
        pos = end + 1;
        const size_t eq = field.find('=');
        if (eq == std::string::npos) {
            continue;
        }
        const std::string k = field.substr(0, eq), v = field.substr(eq + 1);
        long n = 0;
        if (k == "show") {
            parseBits(v, c.show);
        } else if (k == "fold") {
            parseBits(v, c.collapsed);
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
    return c;
}

double clampMeasureDeg(double deg, double hfov_deg) {
    return std::clamp(deg, 0.1, std::max(0.2, 0.9 * hfov_deg));
}

}  // namespace gcs::hud
