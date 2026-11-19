#include "check.hpp"
#include "core/geo.hpp"
#include "core/hud_config.hpp"

#include <cmath>
#include <string>

using namespace gcs;
using namespace gcs::hud;

static void testPresets() {
    const Config d = defaultConfig();
    CHECK(matchingPreset(d) == Preset::Standard);
    // STANDARD: a clear picture with the crosshair, heading, status and target
    CHECK(d.shows(Element::HeadingTape));
    CHECK(d.shows(Element::StatusLine));
    CHECK(d.shows(Element::TargetBox));
    CHECK(!d.shows(Element::Ladder));
    CHECK(!d.shows(Element::RollScale));
    CHECK(!d.shows(Element::SpeedTape));
    CHECK(!d.shows(Element::SensorFx));
    CHECK(!d.isCollapsed(Box::Target));
    CHECK(d.isCollapsed(Box::Aircraft));

    const Config full = presetConfig(Preset::Full);
    for (int i = 0; i < kElements; ++i) {
        CHECK(full.show[static_cast<size_t>(i)]);
    }
    CHECK(matchingPreset(full) == Preset::Full);
    const Config clean = presetConfig(Preset::Clean);
    CHECK(matchingPreset(clean) == Preset::Clean);

    // a preset keeps the operator's crosshair, line and colour
    Config styled = d;
    styled.crosshair = Crosshair::Duplex;
    styled.line = 2;
    styled.color = 3;
    const Config f2 = presetConfig(Preset::Full, styled);
    CHECK(f2.crosshair == Crosshair::Duplex);
    CHECK(f2.line == 2);
    CHECK(f2.color == 3);
    CHECK(matchingPreset(f2) == Preset::Full);  // style doesn't make it custom

    // one change: custom
    Config c = d;
    c.setShown(Element::Ladder, true);
    CHECK(matchingPreset(c) == Preset::Custom);
    CHECK(std::string(presetName(Preset::Custom)) == "CUSTOM");
}

static void testEncode() {
    Config c = presetConfig(Preset::Full);
    c.crosshair = Crosshair::Measure;
    c.line = 0;
    c.color = 5;
    c.setCollapsed(Box::Wind, true);
    c.measure_w_deg = 3.25;
    c.measure_h_deg = 1.5;
    const Config back = decode(encode(c));
    CHECK(back == c);
    CHECK_NEAR(back.measure_w_deg, 3.25, 1e-9);

    // garbage and partial strings: defaults, plus whatever was understood
    CHECK(decode("") == defaultConfig());
    CHECK(decode("nonsense;;=;x") == defaultConfig());
    const Config part = decode("v1;x=duplex;line=9;color=-3;mw=abc");
    CHECK(part.crosshair == Crosshair::Duplex);
    CHECK(part.line == 2);   // clamped
    CHECK(part.color == 0);  // clamped
    CHECK_NEAR(part.measure_w_deg, defaultConfig().measure_w_deg, 1e-9);
    // an older, shorter show list keeps the defaults for the rest
    const Config short_show = decode("show=11");
    CHECK(short_show.shows(Element::Ladder));
    CHECK(short_show.shows(Element::RollScale));
    CHECK(short_show.shows(Element::StatusLine));  // default
    CHECK(!short_show.shows(Element::SpeedTape));  // default

    CHECK_NEAR(Config{}.lineFactor(), 1.0, 1e-12);
    Config thin;
    thin.line = 0;
    CHECK(thin.lineFactor() < 1.0);
}

static void testMeasure() {
    // 10 m at 1000 m subtends 0.5730 deg, and back
    const double a = 2.0 * std::atan(5.0 / 1000.0) * geo::kRadToDeg;
    CHECK_NEAR(a, 0.5730, 1e-4);
    CHECK_NEAR(geo::sizeAtRange(a, 1000.0), 10.0, 1e-9);
    // the whole picture width is the field of view
    CHECK_NEAR(geo::pixelAngleDeg(1920.0, 1920.0, 70.0), 70.0, 1e-9);
    CHECK_NEAR(geo::anglePixels(70.0, 1920.0, 70.0), 1920.0, 1e-6);
    // round trip at an arbitrary span
    CHECK_NEAR(geo::anglePixels(geo::pixelAngleDeg(333.0, 1600.0, 60.0), 1600.0, 60.0), 333.0, 1e-6);
    // a centred span half the picture wide covers more than half the field of
    // view (pinhole: pixels near the centre span more angle): 2*atan(0.5*tan 35)
    CHECK_NEAR(geo::pixelAngleDeg(960.0, 1920.0, 70.0), 38.5907, 1e-3);
    CHECK_NEAR(geo::sizeAtRange(1.0, -5.0), 0.0, 1e-12);

    CHECK_NEAR(clampMeasureDeg(0.0, 70.0), 0.1, 1e-12);
    CHECK_NEAR(clampMeasureDeg(500.0, 70.0), 63.0, 1e-12);
}

int main() {
    testPresets();
    testEncode();
    testMeasure();
    return TEST_MAIN_RESULT();
}
