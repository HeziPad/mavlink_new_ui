#pragma once

// What the camera HUD draws: which elements, which crosshair, line width,
// colour, which side boxes are folded, and the measuring brackets' size.
// Pure data; CameraView draws it and HudPanel edits it. Stored in the
// settings as a compact text (encode / decode).

#include <array>
#include <string>

namespace gcs::hud {

enum class Element {
    Ladder = 0,    // horizon + pitch ladder (and the CAM marker in body mode)
    RollScale,     // the roll arc
    HeadingTape,   // LOS / heading tape at the top
    SpeedTape,     // airspeed tape, left
    AltTape,       // altitude tape, right
    PathMarker,    // flight-path marker
    StatusLine,    // flight state · GS · VS under the heading tape
    TargetBox,     // CAMERA TARGET: where the crosshair meets the ground
    AircraftBox,   // AIRCRAFT: roll, pitch, heading
    LosBox,        // gimbal / mount: pan, tilt, earth az / el
    WindBox,       // wind gauge
    SensorFx,      // vignette + scanlines ("sensor feed" look)
    Count
};
constexpr int kElements = static_cast<int>(Element::Count);

enum class Crosshair { Cross = 0, Dot, CircleDot, Duplex, Brackets, Fine, Measure, None, Count };
constexpr int kCrosshairs = static_cast<int>(Crosshair::Count);

// the side boxes that fold to a one-line summary
enum class Box { Target = 0, Aircraft, Los, Wind, Count };
constexpr int kBoxes = static_cast<int>(Box::Count);

// CLEAN and FULL are fixed; CUSTOM is the operator's own mix, remembered
// while another preset is shown.
enum class Preset { Clean = 0, Custom, Full };
constexpr int kPresets = 3;

constexpr int kColors = 6;  // green, cyan, amber, white, red, magenta (the UI owns the RGB)
constexpr int kLineWidths = 3;  // thin, normal, bold

struct Config {
    Preset preset = Preset::Custom;  // the one the panel shows lit
    std::array<bool, kElements> show{};  // what is drawn now
    Crosshair crosshair = Crosshair::Cross;
    int line = 1;   // 0 thin, 1 normal, 2 bold
    int color = 0;  // swatch index
    std::array<bool, kBoxes> collapsed{};
    // the CUSTOM slot: brought back by applyPreset(Custom)
    std::array<bool, kElements> custom_show{};
    std::array<bool, kBoxes> custom_collapsed{};
    // measuring brackets: full width / height as angles, so they stay on the
    // target when the window or the video size changes
    double measure_w_deg = 4.0;
    double measure_h_deg = 2.5;

    bool shows(Element e) const { return show[static_cast<size_t>(e)]; }
    void setShown(Element e, bool on) { show[static_cast<size_t>(e)] = on; }
    bool isCollapsed(Box b) const { return collapsed[static_cast<size_t>(b)]; }
    void setCollapsed(Box b, bool on) { collapsed[static_cast<size_t>(b)] = on; }
    double lineFactor() const;  // pen width multiplier: 0.7 / 1 / 1.5

    bool operator==(const Config& o) const;
    bool operator!=(const Config& o) const { return !(*this == o); }
};

// The built-in sets. CLEAN: just the crosshair. CUSTOM starts as: crosshair,
// heading tape, status line, target box, the other boxes folded. FULL:
// everything. Returns `keep` with show / collapsed / preset replaced; the
// crosshair, line, colour, brackets and the CUSTOM slot stay as they are.
Config presetConfig(Preset p, const Config& keep = Config{});
// preset CUSTOM, holding its starting set
Config defaultConfig();
// Switch preset: CLEAN / FULL show their built-in set, CUSTOM brings back
// the operator's mix.
void applyPreset(Config& c, Preset p);
// The operator turns an element on or off. Under CLEAN / FULL this starts a
// CUSTOM mix from what is on screen; the edit is saved in the CUSTOM slot.
void editShown(Config& c, Element e, bool on);
// A box folded or opened on the video: remembered in the CUSTOM slot while
// CUSTOM is shown (no preset switch).
void editCollapsed(Config& c, Box b, bool on);
const char* presetName(Preset p);
const char* crosshairName(Crosshair c);

// "v2;preset=custom;show=1011…;fold=0110;cshow=…;cfold=…;x=cross;line=1;
// color=0;mw=4000;mh=2500" (angles in millidegrees: no locale-dependent
// decimals). decode() starts from defaultConfig() and takes whatever fields it
// understands; a v1 string (no preset, a RULER REF bit) is migrated.
std::string encode(const Config& c);
Config decode(const std::string& s);

// Clamp a bracket angle to what the camera can show.
double clampMeasureDeg(double deg, double hfov_deg);

}  // namespace gcs::hud
