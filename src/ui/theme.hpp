#pragma once

// Single source of truth for the "tactical HUD" look: colour tokens, fonts and
// the few drawing primitives every widget shares (chamfered panels, glow
// strokes, corner brackets, scanlines).

#include <QColor>
#include <QFont>
#include <QPainterPath>
#include <QRectF>
#include <QSizeF>
#include <QString>

class QPainter;
class QWidget;

namespace gcs::ui::theme {

inline const QColor kBg{0x07, 0x0b, 0x10};
inline const QColor kGlass{10, 20, 28, 184};
inline const QColor kGlassDeep{6, 12, 18, 214};
inline const QColor kCyan{0x3f, 0xe0, 0xff};
inline const QColor kAmber{0xff, 0xb5, 0x47};
inline const QColor kRed{0xff, 0x4d, 0x5e};
inline const QColor kGreen{0x5c, 0xff, 0x9d};
inline const QColor kText{0xe6, 0xf1, 0xf8};
inline const QColor kMuted{0x7d, 0x93, 0xa6};

// Returns `c` with its alpha multiplied by `a` (0..1).
QColor alpha(QColor c, double a);
QColor mix(const QColor& a, const QColor& b, double t);

// The two font families, bundled in resources/fonts and registered by
// loadFonts() at startup, so text looks and measures the same on Linux,
// Windows and macOS. Stylesheets name them literally ("DejaVu Sans",
// "DejaVu Sans Mono"); never name a font the OS may not have.
inline constexpr const char* kSansFamily = "DejaVu Sans";
inline constexpr const char* kMonoFamily = "DejaVu Sans Mono";
// Registers the bundled fonts; false if one is missing from the resources.
bool loadFonts();

QFont monoFont(double px, bool bold = false);     // numerics, tabular digits
QFont labelFont(double px, bool bold = true);     // small letter-spaced uppercase
QFont titleFont(double px);                       // banners

// 45 degree cuts on the top-left and bottom-right corners.
QPainterPath chamferPath(const QRectF& r, double cut);
QPainterPath hexPath(const QPointF& c, double radius);

// Two-pass stroke: wide translucent halo under a crisp line.
void drawGlowPath(QPainter& p, const QPainterPath& path, const QColor& c, double width = 1.2,
                  double glow = 6.0);
// Glass panel fill plus hairline edge.
void drawGlass(QPainter& p, const QPainterPath& path, const QColor& edge = kCyan,
               double edge_alpha = 0.35, const QColor& fill = kGlass);
// Four L-shaped corner brackets.
void drawBrackets(QPainter& p, const QRectF& r, double len, const QColor& c, double width = 1.5);
void drawScanlines(QPainter& p, const QRectF& r, double strength = 1.0);
// Text with letter spacing helpers that keep the baseline crisp.
void drawText(QPainter& p, const QRectF& r, int flags, const QString& text, const QFont& f,
              const QColor& c);

bool reducedMotion();
void setReducedMotion(bool on);

// ---- text size (Settings > DISPLAY) -------------------------------------------------
// Every size in the UI code is a logical pixel at Small. The chosen text size
// scales text *and* the boxes around it, applied in one of two ways (never
// both, or a font grows twice):
//  - widgets that are all HUD chrome (bar, dock, settings, alerts, preflight,
//    toasts) scale their painter by uiScale() and lay out in size()/uiScale()
//    (logicalSize(), toLogical());
//  - views (map, camera) keep their picture and scale only the HUD on top,
//    through sc(); stylesheets go through cssPx().
enum class TextSize { Small = 0, Medium = 1, Large = 2 };
double scaleFor(TextSize t);  // 1.0 / 1.25 / 1.5
double uiScale();
void setUiScale(double k);
inline double sc(double px) { return px * uiScale(); }
int sci(double px);                             // sc(), rounded, for widget/layout sizes
QString cssPx(double px);                       // "16px", scaled and rounded
// A stylesheet written at Small with every "NNpx" scaled (fonts, padding,
// min sizes, letter spacing); 1px hairlines stay 1px.
QString scaleCss(const QString& css);
// For QWidget panels built with layouts at Small: scales every layout's
// margins, spacing and fixed spacers, and every child's min / max size. The
// Small values are remembered on the first call, so it can run again on each
// text-size change (and after adding children). Sizes a panel recomputes from
// its scaled fonts later on (setInfo-style) simply get recomputed.
void scaleWidgetTree(QWidget* root);
QSizeF logicalSize(const QSizeF& physical);     // physical / uiScale()
QPointF toLogical(const QPointF& physical);     // a tap, in the scaled painter's coordinates

}  // namespace gcs::ui::theme
