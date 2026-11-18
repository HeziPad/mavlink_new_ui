#pragma once

#include <string>
#include <vector>

namespace gcs {

// Lift-motor layout of a QuadPlane frame, mirroring AP_MotorsMatrix: which
// motor sits where, which way it spins, and its letter in the motor test
// (MAV_CMD_DO_MOTOR_TEST "sequence" order: A = 1, B = 2, ... clockwise from
// the front-right for X frames).
struct LiftMotor {
    int number = 0;       // MOTORn, output function 32 + n
    double angle = 0.0;   // degrees clockwise from the nose
    bool ccw = false;     // spins counter-clockwise (seen from above)
    int test_order = 0;   // 1-based; letter = 'A' + test_order - 1
    char letter() const { return static_cast<char>('A' + test_order - 1); }
};

struct FrameLayout {
    bool known = false;   // false: drawn as a quad X, with a warning
    std::string name;     // "Quad X"
    std::vector<LiftMotor> motors;
};

// Q_FRAME_CLASS (1 quad, 2 hexa, ...) and Q_FRAME_TYPE (0 plus, 1 X, 3 H, ...)
FrameLayout frameLayout(int frame_class, int frame_type);

}  // namespace gcs
