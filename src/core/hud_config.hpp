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
    RefChip,       // HUD RULERS: LOS / BODY switch
    SensorFx,      // vignette + scanlines ("sensor feed" look)
    Count
};
constexpr int kElements = static_cast<int>(Element::Count);

enum class Crosshair { Cross = 0, Dot, CircleDot, Duplex, Brackets, Fine, Measure, None, Count };
constexpr int kCrosshairs = static_cast<int>(Crosshair::Count);

// the side boxes that fold to a one-line summary
enum class Box { Target = 0, Aircraft, Los, Wind, Count };
constexpr int kBoxes = static_cast<int>(Box::Count);

enum class Preset { Clean = 0, Standard, Full, Custom };

constexpr int kColors = 6;  // green, cyan, amber, white, red, magenta (the UI owns the RGB)
constexpr int kLineWidths = 3;  // thin, normal, bold

struct Config {
    std::array<bool, kElements> show{};
    Crosshair crosshair = Crosshair::Cross;
    int line = 1;   // 0 thin, 1 normal, 2 bold
    int color = 0;  // swatch index
    std::array<bool, kBoxes> collapsed{};
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

// CLEAN: just the crosshair. STANDARD (the default): crosshair, heading
// tape, status line, target box, the other boxes folded. FULL: everything.
// A preset sets what is shown and the boxes; crosshair, line and colour are
// the operator's and stay as they are.
Config presetConfig(Preset p, const Config& keep_style = Config{});
Config defaultConfig();
// Which preset the shown elements and folded boxes match (Custom if none).
Preset matchingPreset(const Config& c);
const char* presetName(Preset p);
const char* crosshairName(Crosshair c);

// "v1;show=1011…;x=cross;line=1;color=0;fold=0110;mw=4000;mh=2500"
// (angles in millidegrees: no locale-dependent decimals). decode() starts
// from defaultConfig() and takes whatever fields it understands.
std::string encode(const Config& c);
Config decode(const std::string& s);

// Clamp a bracket angle to what the camera can show.
double clampMeasureDeg(double deg, double hfov_deg);

}  // namespace gcs::hud
