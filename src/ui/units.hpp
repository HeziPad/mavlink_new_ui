#pragma once

#include <QString>

// Display units. Everything inside the app (core, link, MainWindow logic) stays
// in metres; only what the user reads or types through steppers is converted.
// Process-wide, set from the settings screen.
namespace gcs::ui::units {

enum class Alt { Meters = 0, Feet = 1 };

Alt altUnit();
void setAltUnit(Alt u);
bool feet();

constexpr double kFeetPerMeter = 1.0 / 0.3048;

double altToDisplay(double meters);
double altFromDisplay(double value);
QString altUnitLabel();                            // "m" / "ft"
QString altText(double meters, bool unit = true);  // "120 m" / "394 ft", rounded
// One stepper step: `meters_step` in metres mode; the matching round feet step
// (1 -> 5, 10 -> 50, 100 -> 500) in feet mode. Returned in metres.
double altStepMeters(double meters_step);
// Label of that step in the current unit, e.g. "+10" / "+50".
QString altStepLabel(double meters_step);

}  // namespace gcs::ui::units
