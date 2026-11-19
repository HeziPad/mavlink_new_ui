#include "check.hpp"
#include "core/geo.hpp"
#include "core/hud_config.hpp"

#include <cmath>
#include <string>

using namespace gcs;
using namespace gcs::hud;

static void testPresets() {
    const Config d = defaultConfig();
    CHECK(d.preset == Preset::Custom);
    // CUSTOM starts clear: the crosshair, heading, status and target
    CHECK(d.shows(Element::HeadingTape));
    CHECK(d.shows(Element::StatusLine));
    CHECK(d.shows(Element::TargetBox));
    CHECK(!d.shows(Element::Ladder));
    CHECK(!d.shows(Element::RollScale));
    CHECK(!d.shows(Element::SpeedTape));
    CHECK(!d.shows(Element::SensorFx));
    CHECK(!d.isCollapsed(Box::Target));
    CHECK(d.isCollapsed(Box::Aircraft));
    CHECK(d.custom_show == d.show);

    const Config full = presetConfig(Preset::Full);
    for (int i = 0; i < kElements; ++i) {
        CHECK(full.show[static_cast<size_t>(i)]);
    }
    CHECK(full.preset == Preset::Full);
    CHECK(presetConfig(Preset::Clean).preset == Preset::Clean);

    // a preset keeps the operator's crosshair, line and colour
    Config styled = d;
    styled.crosshair = Crosshair::Duplex;
    styled.line = 2;
    styled.color = 3;
    Config f2 = styled;
    applyPreset(f2, Preset::Full);
    CHECK(f2.preset == Preset::Full);
    CHECK(f2.crosshair == Crosshair::Duplex);
    CHECK(f2.line == 2);
    CHECK(f2.color == 3);

    // CUSTOM remembers the operator's mix across CLEAN / FULL
    Config c = d;
    editShown(c, Element::Ladder, true);
    editCollapsed(c, Box::Wind, false);
    CHECK(c.preset == Preset::Custom);
    applyPreset(c, Preset::Full);
    CHECK(c.shows(Element::SpeedTape));
    editCollapsed(c, Box::Target, true);  // a fold under FULL doesn't touch the slot
    CHECK(c.preset == Preset::Full);
    applyPreset(c, Preset::Clean);
    CHECK(!c.shows(Element::Ladder));
    applyPreset(c, Preset::Custom);
    CHECK(c.shows(Element::Ladder));
    CHECK(!c.shows(Element::SpeedTape));
    CHECK(!c.isCollapsed(Box::Wind));
    CHECK(!c.isCollapsed(Box::Target));

    // an edit under FULL starts a new CUSTOM mix from what is on screen
    Config e = d;
    applyPreset(e, Preset::Full);
    editShown(e, Element::SensorFx, false);
    CHECK(e.preset == Preset::Custom);
    CHECK(e.shows(Element::SpeedTape));
    CHECK(!e.shows(Element::SensorFx));
    CHECK(e.custom_show == e.show);
    CHECK(std::string(presetName(Preset::Custom)) == "CUSTOM");
}

static void testEncode() {
    Config c = defaultConfig();
    editShown(c, Element::Ladder, true);
    applyPreset(c, Preset::Full);
    c.crosshair = Crosshair::Measure;
    c.line = 0;
    c.color = 5;
    c.setCollapsed(Box::Wind, true);
    c.measure_w_deg = 3.25;
    c.measure_h_deg = 1.5;
    const Config back = decode(encode(c));
    CHECK(back == c);
    CHECK(back.preset == Preset::Full);
    CHECK(back.custom_show[static_cast<size_t>(Element::Ladder)]);
    CHECK_NEAR(back.measure_w_deg, 3.25, 1e-9);

    // v1 strings (13 elements, RULER REF at 11) migrate
    const Config v1std = decode("v1;show=0010001111100;x=cross;line=1;color=0;fold=0111;mw=4000;mh=2500");
    CHECK(v1std == defaultConfig());
    const Config v1full = decode("v1;show=1111111111111;x=fine;line=1;color=0;fold=0000;mw=4000;mh=2500");
    CHECK(v1full.preset == Preset::Full);
    CHECK(v1full.crosshair == Crosshair::Fine);
    CHECK(v1full.custom_show == defaultConfig().show);
    // the RULER REF bit is dropped without shifting SENSOR FX
    const Config v1fx = decode("v1;show=0010001111110;fold=0111");
    CHECK(v1fx.preset == Preset::Custom);
    CHECK(!v1fx.shows(Element::SensorFx));
    const Config v1fx2 = decode("v1;show=0010001111101;fold=0111");
    CHECK(v1fx2.shows(Element::SensorFx));
    CHECK(v1fx2.custom_show == v1fx2.show);

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
