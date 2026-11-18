#include "frame_layout.hpp"

namespace gcs {

namespace {
struct Def {
    double angle;
    bool ccw;
    int order;
};

FrameLayout build(const char* name, std::initializer_list<Def> defs, bool known = true) {
    FrameLayout f;
    f.known = known;
    f.name = name;
    int n = 1;
    for (const auto& d : defs) {
        f.motors.push_back({n++, d.angle, d.ccw, d.order});
    }
    return f;
}
}  // namespace

FrameLayout frameLayout(int frame_class, int frame_type) {
    // AP_MotorsMatrix::setup_quad_matrix / setup_hexa_matrix (angle, yaw factor, test order)
    if (frame_class == 1) {
        switch (frame_type) {
        case 0: return build("Quad +", {{90, true, 2}, {-90, true, 4}, {0, false, 1}, {180, false, 3}});
        case 1: return build("Quad X", {{45, true, 1}, {-135, true, 3}, {-45, false, 4}, {135, false, 2}});
        case 3: return build("Quad H", {{45, false, 1}, {-135, false, 3}, {-45, true, 4}, {135, true, 2}});
        default: break;
        }
    } else if (frame_class == 2) {
        switch (frame_type) {
        case 0:
            return build("Hexa +", {{0, false, 1}, {180, true, 4}, {-120, false, 5}, {60, true, 2}, {-60, true, 6},
                                    {120, false, 3}});
        case 1:
            return build("Hexa X", {{90, false, 2}, {-90, true, 5}, {-30, false, 6}, {150, true, 3}, {30, true, 1},
                                    {-150, false, 4}});
        default: break;
        }
    }
    return build("Quad X (assumed)", {{45, true, 1}, {-135, true, 3}, {-45, false, 4}, {135, false, 2}}, false);
}

}  // namespace gcs
