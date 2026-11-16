#include "plane_modes.hpp"

namespace gcs::plane {

const char* modeName(uint32_t mode) {
    switch (mode) {
    case MANUAL: return "MANUAL";
    case CIRCLE: return "CIRCLE";
    case STABILIZE: return "STABILIZE";
    case TRAINING: return "TRAINING";
    case ACRO: return "ACRO";
    case FBWA: return "FBWA";
    case FBWB: return "FBWB";
    case CRUISE: return "CRUISE";
    case AUTOTUNE: return "AUTOTUNE";
    case AUTO: return "AUTO";
    case RTL: return "RTL";
    case LOITER: return "LOITER";
    case TAKEOFF: return "TAKEOFF";
    case AVOID_ADSB: return "AVOID_ADSB";
    case GUIDED: return "GUIDED";
    case INITIALISING: return "INITIALISING";
    case QSTABILIZE: return "QSTABILIZE";
    case QHOVER: return "QHOVER";
    case QLOITER: return "QLOITER";
    case QLAND: return "QLAND";
    case QRTL: return "QRTL";
    case QAUTOTUNE: return "QAUTOTUNE";
    case QACRO: return "QACRO";
    case THERMAL: return "THERMAL";
    case LOITER_ALT_QLAND: return "LOITER\xE2\x86\x92QLAND";
    case AUTOLAND: return "AUTOLAND";
    default: return nullptr;
    }
}

bool isQMode(uint32_t mode) {
    switch (mode) {
    case QSTABILIZE:
    case QHOVER:
    case QLOITER:
    case QLAND:
    case QRTL:
    case QAUTOTUNE:
    case QACRO:
        return true;
    default:
        return false;
    }
}

}  // namespace gcs::plane
