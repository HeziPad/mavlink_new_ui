#pragma once

#include "core/preflight.hpp"
#include "core/preflight_flow.hpp"
#include "link/command_sender.hpp"
#include "link/link_events.hpp"

#include <QString>

#include <array>
#include <functional>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace gcs::link {
class MavlinkClient;
}

namespace gcs::ui {

// Runs the preflight by itself: evaluates the automatic steps every frame,
// walks the run with `preflight::Flow`, starts each test when its row comes
// up and judges it from what the vehicle reports (servo outputs, current,
// gimbal attitude), and keeps the session. The screen is only a view: the
// operator ticks the look card, holds the moving-parts gate, and answers the
// result (SET IT UP, a calibration, TRY AGAIN).
class PreflightRunner {
public:
    enum class Style { Primary, Good, Bad, Neutral, Danger };

    explicit PreflightRunner(link::MavlinkClient* client);

    // ---- lifecycle -----------------------------------------------------------------
    void open(const VehicleState& s, bool turnaround);
    bool isOpen() const { return open_; }
    void close();
    void update(const VehicleState& s, Clock::time_point now, double video_age_s);
    void onEvent(const link::LinkEvent& ev);

    // ---- state for the view ------------------------------------------------------------
    preflight::Session& session() { return session_; }
    const preflight::Session& session() const { return session_; }
    const preflight::Profile& profile() const { return profile_; }
    const preflight::Flow& flow() const { return flow_; }
    preflight::Status status(const std::string& id) const;
    QString detail(const std::string& id) const;
    const VehicleState& vehicle() const { return s_; }
    const FrameLayout& frame() const { return frame_; }

    // ---- what the operator does ---------------------------------------------------------
    void setLook(const std::string& id, bool done);  // a look-card tick
    void lookContinue();                             // CONTINUE (every tick done)
    void goMoving();     // the hold before the moving parts: safety off, then the tests
    void rerun();        // TRY AGAIN: the scan again; failed tests run again
    void restart();      // a fresh run from the look card
    bool testRunning() const { return !running_.empty(); }
    void stopAll();      // STOP: ends the test running and the moving parts still to come

    // SET IT UP: writes every wrong setup parameter with its recommended value
    bool setupFixable() const;
    bool setupWriting() const { return !setup_pending_.empty(); }
    void fixSetup();
    // a parameter that only applies after a restart was written
    bool rebootPending() const { return reboot_pending_; }
    void reboot();

    // the calibration a problem offers (from the sensor that's unhealthy)
    enum class Cal { None, Compass, Accel };
    Cal calFor(const std::string& id) const;
    Cal calRunning() const { return cal_; }
    void startCal(Cal c);
    void cancelCal();
    QString calText() const;  // what to do now
    int calPercent() const;   // compass progress (0..100)

    // ---- what the pictures need ----------------------------------------------------------
    struct Art {
        int lit_motor = -1;       // index into frame().motors
        bool pusher_lit = false;
        double surf_cmd = 0.0;    // stick command being swept, -1..1
        double surf_out = 0.0;    // live output, -1..1 (NaN unknown)
        int axis = 0;             // 0 roll 1 pitch 2 yaw
        double gimbal_pitch = 0.0, gimbal_yaw = 0.0;
    };
    const Art& art() const { return art_; }

    // the user may launch: complete (or overridden) and still valid
    bool launchAllowed(const VehicleState& s, Clock::time_point now) const;
    QString launchBlockReason(const VehicleState& s, Clock::time_point now) const;
    void overrideLaunch(Clock::time_point now);
    void noteFlight();
    void setPrearmReasons(std::vector<std::string> r) { prearm_ = std::move(r); }

    // Settings -> BATTERY (the cell count comes from the pack)
    void setBattery(preflight::Chemistry chem, int capacity_mah, double endurance_min);

    // callbacks the window wires up
    std::function<void(bool make_safe)> onSafety;
    std::function<void(const QString& text, bool good)> onToast;
    std::function<void()> onComplete;

    void loadProfile();
    void saveProfile() const;
    QString saveReport() const;  // path written ("" on failure)

private:
    void setResult(const std::string& id, preflight::Status st, const std::string& detail, bool by_user = false,
                   const std::string& note = {});
    void send(const std::string& tag, std::vector<link::CommandRequest> steps);
    void startTest(const std::string& id);
    void endTest(const std::string& id, preflight::Status st, const std::string& detail);
    void abortTest(const std::string& why);  // stop the running test: it fails with `why`
    void tickTests(Clock::time_point now);
    void tickCal(Clock::time_point now);
    void restoreSafety();
    std::vector<int> surfaceChannels(const std::string& id) const;
    double outputNorm(int channel) const;  // servo output -1..1 from SERVOn_MIN/TRIM/MAX
    void restoreMode();
    void saveMode();
    void newSession(bool turnaround);

    link::MavlinkClient* client_;
    bool open_ = false;
    VehicleState s_;
    Clock::time_point now_{};
    double video_age_ = 1e9;
    preflight::Session session_;
    preflight::Profile profile_;
    preflight::Flow flow_;
    FrameLayout frame_;
    Art art_;
    bool fetched_ = false;
    bool passed_override_ = false;
    std::vector<std::string> prearm_;  // the vehicle's current PreArm reasons
    std::string started_row_;          // the row whose start was handled

    // test bookkeeping
    std::string running_;            // step id of the test in progress ("" none)
    int phase_ = 0;
    Clock::time_point phase_t_{};
    std::map<std::string, std::string> tag_result_;  // last CommandResult per tag ("ok" / reason)
    std::map<std::string, bool> tag_done_;
    // The mode before the first test that changed it. Kept until the vehicle is
    // confirmed back in it, so back-to-back tests don't save each other's modes.
    uint32_t mode_before_ = 0;
    bool mode_saved_ = false;
    bool restore_sent_ = false;
    bool safety_restore_ = false;    // the gate turned safety off: back on afterwards

    // per-test state
    std::map<int, std::pair<double, double>> swing_;  // surface channel -> (min, max) output seen
    int motor_idx_ = 0;              // lift test: position in motor_order_
    std::vector<int> motor_order_;   // frame_.motors indices sorted by test order
    std::vector<std::string> motor_faults_, motor_notes_;
    int spun_channel_ = 0;           // the output seen spinning
    double amps0_ = 0.0, amps_peak_ = 0.0;  // current before / during a spin
    double pitot_peak_ = 0.0;
    double gimbal_min_pitch_ = 0.0, gimbal_yaw_lo_ = 0.0, gimbal_yaw_hi_ = 0.0;
    bool gimbal_seen_ = false;

    // SET IT UP / restart
    std::map<std::string, std::pair<float, float>> fix_from_to_;  // param -> (from, to)
    std::set<std::string> setup_pending_;  // written, not confirmed yet
    std::vector<std::string> setup_done_;  // titles fixed, for the toast
    bool setup_failed_ = false;
    bool reboot_pending_ = false;
    int reboot_boot_count_ = 0;

    // calibration
    Cal cal_ = Cal::None;
    Clock::time_point cal_t_{};
};

}  // namespace gcs::ui
