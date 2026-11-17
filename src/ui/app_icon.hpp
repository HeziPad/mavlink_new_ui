#pragma once

#include <QIcon>
#include <QImage>

namespace gcs::ui {

// The application icon, painted in code (no SVG module needed): a top-down
// QuadPlane (wing, booms, four lift rotors, pusher) in HUD cyan on a dark
// chamfered tile. Crisp at every size.
QImage appIconImage(int size);
QIcon appIcon();

}  // namespace gcs::ui
