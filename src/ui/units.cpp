#include "units.hpp"

#include <cmath>

namespace gcs::ui::units {

namespace {
Alt g_alt = Alt::Meters;

double feetStep(double meters_step) {
    // round feet steps of about the same size: 1 m -> 5 ft, 10 m -> 50 ft
    return meters_step * 5.0;
}
}  // namespace

Alt altUnit() { return g_alt; }
void setAltUnit(Alt u) { g_alt = u; }
bool feet() { return g_alt == Alt::Feet; }

double altToDisplay(double meters) { return feet() ? meters * kFeetPerMeter : meters; }
double altFromDisplay(double value) { return feet() ? value / kFeetPerMeter : value; }

QString altUnitLabel() { return feet() ? QStringLiteral("ft") : QStringLiteral("m"); }

QString altText(double meters, bool unit) {
    const QString n = QString::number(std::lround(altToDisplay(meters)));
    return unit ? n + QLatin1Char(' ') + altUnitLabel() : n;
}

double altStepMeters(double meters_step) {
    return feet() ? feetStep(meters_step) / kFeetPerMeter : meters_step;
}

QString altStepLabel(double meters_step) {
    const double v = feet() ? feetStep(meters_step) : meters_step;
    const long n = std::lround(v);
    return n > 0 ? QStringLiteral("+%1").arg(n) : QString::number(n);
}

}  // namespace gcs::ui::units
