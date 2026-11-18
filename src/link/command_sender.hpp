#pragma once

#include "mavlink_headers.hpp"

#include <array>
#include <chrono>
#include <functional>
#include <string>
#include <vector>

namespace gcs::link {

struct CommandRequest {
    uint16_t command = 0;
    std::array<float, 7> params{};
    // COMMAND_INT instead of COMMAND_LONG: positions as degE7 integers, so a
    // target isn't rounded to float precision (~0.5 m). params[4..6] unused.
    bool as_int = false;
    uint8_t frame = 0;
    int32_t x = 0;
    int32_t y = 0;
    float z = 0.0f;
    bool optional = false;   // a failure does not abort the sequence
    // Commands the vehicle executes before it ACKs (calibrations block for
    // seconds) must not be re-sent: a repeat would restart them.
    int max_attempts = 3;
    std::chrono::milliseconds ack_timeout{0};  // per attempt; 0 = the default retry interval
    std::string label;       // human readable, e.g. "ARM"
};

CommandRequest makeSetMode(uint32_t custom_mode, const std::string& label);
// force: ArduPilot's magic 21196 (skip checks). Used only to disarm on the
// ground after a GCS-driven test, where a refused disarm would leave it armed.
CommandRequest makeArm(bool arm, bool force = false);
CommandRequest makeVtolTakeoff(float alt_rel);
// MAV_CMD_DO_SET_MISSION_CURRENT: continue the mission from `seq`.
// reset: also restart the mission (jump counters, completed -> active)
CommandRequest makeSetMissionCurrent(uint16_t seq, bool reset = false);
// Hardware safety switch: safe = outputs cut. ArduPilot: MAV_CMD_DO_SET_SAFETY_SWITCH_STATE.
CommandRequest makeSetSafety(bool safe);
// MAV_CMD_DO_CHANGE_SPEED, airspeed (m/s); -2 = back to AIRSPEED_CRUISE.
// ArduPlane accepts AIRSPEED_MIN..AIRSPEED_MAX.
CommandRequest makeChangeSpeed(float airspeed);
// MAV_CMD_DO_REPOSITION (COMMAND_INT): switch to GUIDED and circle (lat, lon)
// at alt_rel with the given radius and direction. Re-sending with the same
// centre changes radius / altitude / direction in place.
CommandRequest makeReposition(double lat, double lon, float alt_rel, float radius_m, bool ccw);

// ---- preflight ------------------------------------------------------------------
// MAV_CMD_DO_MOTOR_TEST: lift motor `seq` (1-based test order A=1..), `pct` %
// throttle for `seconds`, `count` motors in sequence from seq. Disarmed only.
CommandRequest makeMotorTest(int seq, float pct, float seconds, int count = 1);
enum class Calibration { Gyro, Level, AccelSimple, AccelFull, BaroAirspeed };
CommandRequest makeCalibration(Calibration c);
// MAV_CMD_DO_START_MAG_CAL for all compasses; the vehicle saves on success
CommandRequest makeStartMagCal();
CommandRequest makeAcceptMagCal();
CommandRequest makeCancelMagCal();
// MAV_CMD_FIXED_MAG_CAL_YAW: the aircraft's nose points at `yaw_deg` true
CommandRequest makeFixedMagCalYaw(float yaw_deg);
// answer to the vehicle's ACCELCAL_VEHICLE_POS request: it is placed in `pos`
CommandRequest makeAccelCalPos(int pos);
CommandRequest makeRunPrearmChecks();
CommandRequest makeReboot();
// MAV_CMD_DO_MOUNT_CONTROL in MAVLink targeting: pitch / yaw in degrees
CommandRequest makeMountControl(float pitch_deg, float yaw_deg);
CommandRequest makeRequestMessage(uint32_t msg_id);

// COMMAND_LONG sender with COMMAND_ACK tracking and retransmission.
//
// One user-facing "action slot" runs a sequence of commands, each advancing only
// on MAV_RESULT_ACCEPTED. Starting a new sequence pre-empts the running one, so a
// safety action (Hold, QRTL) never waits behind a launch. Background commands
// (stream-rate requests, ...) are tracked separately and never reported.
class CommandSender {
public:
    using Clock = std::chrono::steady_clock;
    using SendFn = std::function<void(const mavlink_message_t&)>;

    struct Result {
        std::string tag;
        std::string label;     // label of the step this result is about
        int step = 0;          // 0-based
        int steps = 0;
        uint16_t command = 0;
        uint8_t mav_result = 0;
        bool final = false;    // sequence finished (success or failure)
        bool ok = false;
        bool cancelled = false;
        std::string text;      // short failure reason
    };
    using ResultFn = std::function<void(const Result&)>;

    static constexpr auto kRetryInterval = std::chrono::milliseconds(1000);
    static constexpr auto kInProgressTimeout = std::chrono::milliseconds(10000);
    static constexpr int kMaxAttempts = 3;

    CommandSender(SendFn send, ResultFn result);

    void setIds(const LinkIds& ids) { ids_ = ids; }

    void runSequence(const std::string& tag, std::vector<CommandRequest> steps, Clock::time_point now);
    void sendBackground(const CommandRequest& req, Clock::time_point now);
    void cancel(const std::string& reason);

    bool handle(const mavlink_message_t& msg, Clock::time_point now);
    void tick(Clock::time_point now);

    bool busy() const { return active_; }
    const std::string& activeTag() const { return tag_; }

    static const char* resultName(uint8_t mav_result);

private:
    struct Pending {
        CommandRequest req;
        Clock::time_point last_tx{};
        int attempts = 0;
        bool in_progress = false;
    };

    void transmit(Pending& p, Clock::time_point now);
    void startStep(Clock::time_point now);
    void stepFinished(uint8_t mav_result, const std::string& text, Clock::time_point now);

    SendFn send_;
    ResultFn result_;
    LinkIds ids_;

    bool active_ = false;
    std::string tag_;
    std::vector<CommandRequest> steps_;
    size_t index_ = 0;
    Pending current_;

    std::vector<Pending> background_;
};

}  // namespace gcs::link
