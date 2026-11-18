#pragma once

#include "core/coords.hpp"

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

// ---- speed -------------------------------------------------------------------------
enum class Speed { Ms = 0, Knots = 1 };
Speed speedUnit();
void setSpeedUnit(Speed u);
constexpr double kKnotsPerMs = 3600.0 / 1852.0;
double speedToDisplay(double mps);
QString speedUnitLabel();                                   // "m/s" / "kt"
QString speedText(double mps, int decimals = 0, bool unit = true);  // "18 m/s" / "35 kt"

// ---- coordinates ---------------------------------------------------------------------
coords::Fmt coordFormat();
void setCoordFormat(coords::Fmt f);
QString coordText(double lat, double lon);  // in the chosen format

// ---- distance and time ------------------------------------------------------------------
QString distanceText(double m);   // "850 m" / "2.4 km" (always metric)
QString durationText(double s);   // "1:05", "1:02:05"; "—" when s < 0

}  // namespace gcs::ui::units
