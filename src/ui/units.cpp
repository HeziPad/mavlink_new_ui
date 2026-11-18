#include "units.hpp"

#include <cmath>

namespace gcs::ui::units {

namespace {
Alt g_alt = Alt::Meters;
Speed g_speed = Speed::Ms;
coords::Fmt g_coord = coords::Fmt::Decimal;

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

Speed speedUnit() { return g_speed; }
void setSpeedUnit(Speed u) { g_speed = u; }
double speedToDisplay(double mps) { return g_speed == Speed::Knots ? mps * kKnotsPerMs : mps; }
QString speedUnitLabel() { return g_speed == Speed::Knots ? QStringLiteral("kt") : QStringLiteral("m/s"); }

QString speedText(double mps, int decimals, bool unit) {
    const double v = speedToDisplay(mps);
    const QString n = decimals <= 0 ? QString::number(std::lround(v)) : QString::number(v, 'f', decimals);
    return unit ? n + QLatin1Char(' ') + speedUnitLabel() : n;
}

coords::Fmt coordFormat() { return g_coord; }
void setCoordFormat(coords::Fmt f) { g_coord = f; }
QString coordText(double lat, double lon) { return QString::fromStdString(coords::format(lat, lon, g_coord)); }

QString distanceText(double m) {
    if (m >= 10000.0) {
        return QStringLiteral("%1 km").arg(m / 1000.0, 0, 'f', 1);
    }
    if (m >= 1000.0) {
        return QStringLiteral("%1 km").arg(m / 1000.0, 0, 'f', 2);
    }
    return QStringLiteral("%1 m").arg(std::lround(m));
}

QString durationText(double s) {
    if (s < 0.0) {
        return QStringLiteral("—");
    }
    const long t = std::lround(s);
    if (t >= 3600) {
        return QStringLiteral("%1:%2:%3").arg(t / 3600).arg((t / 60) % 60, 2, 10, QLatin1Char('0')).arg(t % 60, 2, 10, QLatin1Char('0'));
    }
    return QStringLiteral("%1:%2").arg(t / 60).arg(t % 60, 2, 10, QLatin1Char('0'));
}

}  // namespace gcs::ui::units
