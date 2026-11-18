#pragma once

#include "core/preflight.hpp"
#include "link/command_sender.hpp"
#include "link/link_events.hpp"

#include <QString>

#include <functional>
#include <map>
#include <string>
#include <vector>

namespace gcs::link {
class MavlinkClient;
}

namespace gcs::ui {

// Runs the preflight: evaluates the automatic steps every frame, drives the
// tests and calibrations through the link, and keeps the session. The screen
// is only a view of it: it asks for the buttons of a step and passes the
// pressed action back in.
class PreflightRunner {
public:
    enum class Style { Primary, Good, Bad, Neutral, Danger };
    struct Button {
        std::string action;
        QString label;
        Style style = Style::Primary;
        bool hold = false;     // needs a 1 s hold (spins things / changes the vehicle)
        bool enabled = true;
        QString reason;        // when disabled
    };

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
    preflight::Profile& profile() { return profile_; }
    preflight::Status status(const std::string& id) const;
    QString detail(const std::string& id) const;
    QString instruction(const std::string& id) const;  // what to do right now (phase-aware)
    std::vector<Button> buttons(const std::string& id) const;
    void act(const std::string& id, const std::string& action);
    bool testRunning() const;
    void stopAll();
    const VehicleState& vehicle() const { return s_; }
    const FrameLayout& frame() const { return frame_; }

    // ---- what the pictures need ----------------------------------------------------------
    struct Art {
        int lit_motor = -1;       // index into frame().motors
        bool pusher_lit = false;
        double surf_cmd = 0.0;    // stick command being swept, -1..1
        double surf_out = 0.0;    // live output, -1..1 (NaN unknown)
        int axis = 0;             // 0 roll 1 pitch 2 yaw
        int pose = 0;             // accel cal pose requested, 0 none
        std::array<bool, 8> stick_reached{};
        double gimbal_pitch = 0.0, gimbal_yaw = 0.0;
        bool orient_roll = false, orient_pitch = false, orient_yaw = false;
        int mode_positions_seen = 0;
        std::vector<QString> mode_list;  // "1: FBWA" ... with the live one marked
        int mode_live = -1;
    };
    const Art& art() const { return art_; }

    // the user may launch: complete (or overridden) and still valid
    bool launchAllowed(const VehicleState& s, Clock::time_point now) const;
    QString launchBlockReason(const VehicleState& s, Clock::time_point now) const;
    void overrideLaunch(Clock::time_point now);
    void noteFlight();
    void setPrearmReasons(std::vector<std::string> r) { prearm_ = std::move(r); }

    // callbacks the window wires up
    std::function<void(bool make_safe)> onSafety;
    std::function<void(const QString& text, bool good)> onToast;
    std::function<void()> onComplete;

    void loadProfile();
    void saveProfile() const;
    QString saveReport() const;  // path written ("" on failure)

private:
    void setResult(const std::string& id, preflight::Status st, const std::string& detail, bool by_user,
                   const std::string& note = {});
    void send(const std::string& tag, std::vector<link::CommandRequest> steps);
    void tickTests(Clock::time_point now);
    double outputNorm(int channel) const;  // servo output -1..1 from SERVOn_MIN/TRIM/MAX
    int rcChannel(const char* rcmap_param, int fallback) const;
    double rcNorm(int ch) const;
    void restoreMode();

    link::MavlinkClient* client_;
    bool open_ = false;
    VehicleState s_;
    Clock::time_point now_{};
    double video_age_ = 1e9;
    preflight::Session session_;
    preflight::Profile profile_;
    FrameLayout frame_;
    Art art_;
    std::map<std::string, preflight::Eval> live_;  // last automatic evaluation per step
    bool fetched_ = false;
    bool passed_override_ = false;
    bool was_complete_ = false;

    // generic test bookkeeping
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
    void saveMode();
    std::vector<std::string> prearm_;  // the vehicle's current PreArm reasons

    // per-test state
    int motor_idx_ = 0;              // lift test: which motor (index in frame_.motors order of test letters)
    std::vector<int> motor_order_;   // frame_.motors indices sorted by test order
    std::vector<bool> motor_ok_;
    int spun_channel_ = 0;           // the output seen spinning during a lift test
    double surf_right_sign_ = 0.0;   // aileron output change for a roll-right command (for the stabilise check)
    std::map<std::string, std::string> fix_pending_;  // step id -> parameter awaiting confirmation
    std::map<std::string, std::pair<float, float>> fix_from_to_;  // param -> (from, to)
    int rcfs_phase_ = 0;
    double pitot_peak_ = 0.0;
    double yaw0_ = 0.0;
    std::array<bool, 6> mode_seen_{};
    bool props_on_ = false;
    int compass_method_ = 0;         // 1 fixed-yaw, 2 onboard
    int fixed_yaw_deg_ = 0;
};

}  // namespace gcs::ui
