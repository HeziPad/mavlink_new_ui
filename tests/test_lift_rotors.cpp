#include "check.hpp"
#include "core/vehicle_state.hpp"

using namespace gcs;

namespace {
const Clock::time_point T0 = Clock::now();
// state long after any spin started: debounce satisfied
LiftRotors settled(VehicleState s) {
    updateAssistTimer(s, T0);
    return liftRotorState(s, T0 + std::chrono::seconds(5));
}
}  // namespace

namespace {

// SITL-like mapping: SERVO1-4 surfaces/throttle, SERVO5-8 lift motors.
VehicleState mapped() {
    VehicleState s;
    s.servo_function.fill(-1);
    s.servo_function[0] = 4;
    s.servo_function[2] = 70;
    for (int i = 4; i < 8; ++i) {
        s.servo_function[i] = kServoFnMotor1 + (i - 4);
    }
    s.servo_functions_known = 16;
    s.servo_map_final = true;
    s.has_servo_raw = true;
    s.servo_raw.fill(1000);
    s.servo_raw[2] = 1600;  // pusher at cruise throttle: not a lift motor
    s.has_heartbeat = true;
    s.has_ext_state = true;
    s.landed = LandedState::InAir;
    return s;
}

}  // namespace

static void testUnknownUntilMapped() {
    VehicleState s = mapped();
    s.servo_map_final = false;
    CHECK(settled(s) == LiftRotors::Unknown);
    s = mapped();
    s.has_servo_raw = false;
    CHECK(settled(s) == LiftRotors::Unknown);
    s = mapped();
    s.servo_function.fill(-1);  // no motor outputs at all
    CHECK(settled(s) == LiftRotors::Unknown);
}

static void testCruiseIdleIsOff() {
    VehicleState s = mapped();
    s.reported_vtol = VtolState::Cruise;
    CHECK(settled(s) == LiftRotors::Off);
}

static void testHoverIsActive() {
    VehicleState s = mapped();
    s.reported_vtol = VtolState::Hover;
    s.servo_raw[4] = s.servo_raw[5] = s.servo_raw[6] = s.servo_raw[7] = 1500;
    CHECK(settled(s) == LiftRotors::Active);
    s.reported_vtol = VtolState::TransitionToCruise;
    CHECK(settled(s) == LiftRotors::Active);
}

static void testCruiseSpinningIsAssist() {
    VehicleState s = mapped();
    s.reported_vtol = VtolState::Cruise;
    s.servo_raw[6] = kLiftSpinPwm;
    CHECK(settled(s) == LiftRotors::Assist);
}

static void testAssistReportedAsTransition() {
    // ArduPlane: Q_ASSIST in cruise is reported as TRANSITION_TO_FW
    VehicleState s = mapped();
    s.servo_raw[4] = 1400;
    s.reported_vtol = VtolState::Hover;
    updateCruiseLatch(s);
    s.reported_vtol = VtolState::TransitionToCruise;  // real takeoff transition
    updateCruiseLatch(s);
    CHECK(settled(s) == LiftRotors::Active);
    s.reported_vtol = VtolState::Cruise;
    updateCruiseLatch(s);
    s.reported_vtol = VtolState::TransitionToCruise;  // assist, after cruising
    updateCruiseLatch(s);
    CHECK(settled(s) == LiftRotors::Assist);
    s.reported_vtol = VtolState::TransitionToHover;   // landing: expected again
    updateCruiseLatch(s);
    CHECK(settled(s) == LiftRotors::Active);
    s.reported_vtol = VtolState::Cruise;
    updateCruiseLatch(s);
    s.landed = LandedState::OnGround;                 // landed resets the latch
    updateCruiseLatch(s);
    CHECK(!s.cruised_since_hover);
}

static void testSpinDownIsNotAssist() {
    VehicleState s = mapped();
    s.reported_vtol = VtolState::Cruise;
    s.servo_raw[4] = 1300;  // still spinning down right after the transition
    updateAssistTimer(s, T0);
    CHECK(liftRotorState(s, T0 + std::chrono::milliseconds(500)) == LiftRotors::Active);
    CHECK(liftRotorState(s, T0 + std::chrono::seconds(3)) == LiftRotors::Assist);
    s.servo_raw[4] = 1000;  // stopped: timer resets
    updateAssistTimer(s, T0 + std::chrono::seconds(4));
    CHECK(s.wingborne_spin_since == Clock::time_point{});
    CHECK(liftRotorState(s, T0 + std::chrono::seconds(4)) == LiftRotors::Off);
}

int main() {
    testSpinDownIsNotAssist();
    testAssistReportedAsTransition();
    testUnknownUntilMapped();
    testCruiseIdleIsOff();
    testHoverIsActive();
    testCruiseSpinningIsAssist();
    return TEST_MAIN_RESULT();
}
