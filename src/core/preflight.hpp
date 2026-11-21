#pragma once

// Preflight: the step catalogue, automatic evaluators, parameter checks and a
// session that records results, decides when they stop being valid, and
// writes the report. Qt-free; the UI runs the tests and reports their
// outcome back into the session, and `preflight_flow` paces the run.
//
// The run is made for a simple operator: one "look at the aircraft" card
// (the things only a person can see), then every other step checks itself,
// one after another, and the result lists what is wrong in plain words.

#include "frame_layout.hpp"
#include "health.hpp"
#include "vehicle_state.hpp"

#include <map>
#include <optional>
#include <string>
#include <vector>

namespace gcs::preflight {

// the run's sections, in order
enum class Stage {
    Look,        // the "look at the aircraft" card
    Connection,
    Battery,
    Sensors,
    Setup,       // the vehicle's parameters
    Camera,
    Plan,
    Moving,      // surfaces and motors: behind a hold
    Final,
    Count
};
const char* stageName(Stage s);

enum class Kind {
    Auto,  // judged from telemetry / parameters
    Test,  // the GCS acts (or watches) and judges what it measured
    You,   // a physical check you confirm
};
const char* kindName(Kind k);

enum class Status { Pending, Waiting, Pass, Warn, Fail, Skipped, NotApplicable };
const char* statusName(Status s);
bool isDone(Status s);  // Pass, Warn, Skipped, NotApplicable

struct StepDef {
    std::string id;
    Stage stage = Stage::Look;
    Kind kind = Kind::You;
    std::string title;      // what it is, a few words
    std::string sentence;   // while it runs: "Checking the battery is charged"
    std::string how;        // what to do when it isn't right
    bool critical = false;    // a failure blocks Launch (otherwise it's a note)
    bool turnaround = false;  // part of the short after-landing preflight
    double wait_s = 10.0;     // how long a "still waiting" step is waited for
};
const std::vector<StepDef>& steps();
const StepDef* findStep(const std::string& id);

// ---- aircraft profile ----------------------------------------------------------------
enum class Chemistry { LiPo = 0, LiHV = 1, LiIon = 2 };
// Set once in Settings -> BATTERY; the cell count is read off each pack.
struct Profile {
    Chemistry chemistry = Chemistry::LiPo;
    int cells = 0;               // of the pack fitted now (guessCells), 0 = unknown
    int capacity_mah = 0;        // 0 = not entered
    double endurance_min = 0.0;  // typical flight time on a full pack (0 = unknown)
};
const char* chemistryName(Chemistry c);
double fullCellVolts(Chemistry c);      // "charged" threshold per cell
double storageCellVolts(Chemistry c);   // below this: clearly not charged
double lowCellVolts(Chemistry c);       // recommended BATT_LOW_VOLT per cell
double criticalCellVolts(Chemistry c);  // recommended BATT_CRT_VOLT per cell
int guessCells(double pack_volts, Chemistry c);

// ---- automatic evaluation -----------------------------------------------------------
struct Eval {
    Status status = Status::Pending;
    std::string detail;  // the measured values, in words
};
// For Kind::Auto steps (and the Auto part of others); nullopt for steps the
// UI must judge (tests, physical checks).
std::optional<Eval> evaluate(const std::string& id, const VehicleState& s, const Profile& p,
                             Clock::time_point now);

// ---- parameter checks (stage Setup) --------------------------------------------------
// Setup: how this app flies the aircraft and its failsafes. One SET IT UP tap
// writes every wrong one. Battery: the pack's warning levels; checked against
// the pack fitted, never written (packs change).
enum class ParamGroup { Setup, Battery };
struct ParamCheck {
    std::string id;     // "param.<NAME>"
    std::string name;   // the parameter written by a fix ("" = no single fix)
    std::string title;  // plain words, as the result shows it
    std::string why;
    ParamGroup group = ParamGroup::Setup;
    bool reboot = false;  // takes effect after a reboot
};
const std::vector<ParamCheck>& paramChecks();
struct ParamVerdict {
    bool known = false;  // all needed parameters read
    bool ok = false;
    float current = 0.0f;
    std::optional<float> recommended;  // a value a one-tap fix would write
    std::string detail;
};
ParamVerdict checkParam(const ParamCheck& c, const VehicleState& s, const Profile& p);
// every parameter the preflight reads
std::vector<std::string> paramsToFetch();

// The setup parameters that are wrong, with the value SET IT UP writes
// (`fix` empty when there's no single right value: a technician's job).
struct SetupItem {
    const ParamCheck* check = nullptr;
    ParamVerdict verdict;
};
struct SetupReport {
    bool known = false;  // every setup parameter read
    std::vector<SetupItem> wrong;
    bool fixable() const;  // at least one has a recommended value
};
SetupReport setupReport(const VehicleState& s, const Profile& p);

// ---- output map ---------------------------------------------------------------------
struct OutputMap {
    std::vector<int> lift;     // servo channel (1-based) per lift motor number 1..n (0 = missing)
    int throttle = 0;          // pusher
    int aileron = 0, elevator = 0, rudder = 0;  // 0 = not assigned
    int elevon_l = 0, elevon_r = 0, vtail_l = 0, vtail_r = 0;
    std::vector<int> ailerons;  // all channels with the aileron function
};
OutputMap outputMap(const VehicleState& s, int lift_motors);

// ---- session ------------------------------------------------------------------------
struct Result {
    Status status = Status::Pending;
    std::string detail;
    std::string note;        // a comment (e.g. "timed out")
    bool by_user = false;
    double at_s = 0.0;       // seconds since the session started
};

class Session {
public:
    // A fresh run; `turnaround` = the short after-landing list.
    void begin(const VehicleState& s, Clock::time_point now, bool turnaround);
    bool active() const { return active_; }
    bool turnaround() const { return turnaround_; }
    std::vector<const StepDef*> activeSteps() const;
    bool includes(const std::string& id) const;

    void set(const std::string& id, Status st, const std::string& detail, bool by_user, Clock::time_point now,
             const std::string& note = {});
    Result result(const std::string& id) const;

    int doneCount() const;
    int totalCount() const;
    int failCount() const;
    // every required step passed (or warned), every optional one decided
    bool complete() const;
    // the passed state no longer holds (empty = still valid)
    std::string invalidReason(const VehicleState& s, Clock::time_point now) const;
    void markPassed(const VehicleState& s, Clock::time_point now);
    bool passed() const { return passed_; }
    void invalidate() { passed_ = false; }
    void noteFlight() { flown_ = true; }
    void noteOverride(const std::string& why, Clock::time_point now);
    void noteParamWrite(const std::string& name, float from, float to, bool ok);

    // JSON report (wall-clock date/time supplied by the caller)
    std::string reportJson(const std::string& started_iso, const VehicleState& s, const Profile& p) const;

    static constexpr double kValidMinutes = 60.0;

private:
    bool active_ = false;
    bool turnaround_ = false;
    bool passed_ = false;
    bool flown_ = false;
    Clock::time_point start_{};
    Clock::time_point passed_at_{};
    int boot_count_ = 0;
    int params_changed_ = 0;
    double pack_volts_ = 0.0;
    std::map<std::string, Result> results_;
    struct ParamWrite {
        std::string name;
        float from, to;
        bool ok;
    };
    std::vector<ParamWrite> writes_;
    std::vector<std::string> overrides_;
};

}  // namespace gcs::preflight
