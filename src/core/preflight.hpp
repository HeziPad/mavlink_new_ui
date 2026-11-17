#pragma once

// Preflight: the step catalogue, automatic evaluators, parameter checks and a
// session that records results, decides when they stop being valid, and
// writes the report. Qt-free; the UI runs the tests and calibrations and
// reports their outcome back into the session.

#include "frame_layout.hpp"
#include "health.hpp"
#include "vehicle_state.hpp"

#include <map>
#include <optional>
#include <string>
#include <vector>

namespace gcs::preflight {

enum class Stage {
    Profile,
    Site,
    Airframe,
    Power,
    Links,
    Sensors,
    Surfaces,
    Motors,
    Payload,
    Params,
    Mission,
    Final,
    Count
};
const char* stageName(Stage s);

enum class Kind {
    Auto,  // judged from telemetry / parameters
    Test,  // the GCS acts, you confirm what you saw
    Cal,   // a calibration run
    You,   // a physical check you confirm
    Fix,   // a parameter that can be written
};
const char* kindName(Kind k);

enum class Status { Pending, Waiting, Pass, Warn, Fail, Skipped, NotApplicable };
const char* statusName(Status s);
bool isDone(Status s);  // Pass, Warn, Skipped, NotApplicable

struct StepDef {
    std::string id;
    Stage stage = Stage::Profile;
    Kind kind = Kind::You;
    std::string title;
    std::string why;    // one line: why it matters
    std::string how;    // what to do
    bool critical = false;    // can't be skipped
    bool turnaround = false;  // part of the short after-landing preflight
};
const std::vector<StepDef>& steps();
const StepDef* findStep(const std::string& id);

// ---- aircraft profile ----------------------------------------------------------------
enum class Chemistry { LiPo = 0, LiHV = 1, LiIon = 2 };
struct Profile {
    bool set = false;
    Chemistry chemistry = Chemistry::LiPo;
    int cells = 0;
    int capacity_mah = 0;
    double endurance_min = 0.0;  // typical flight time on a full pack (0 = unknown)
};
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

// ---- parameter checks (stage Params) ------------------------------------------------
struct ParamCheck {
    std::string id;     // step id: "param.<NAME>"
    std::string name;   // the parameter written by a fix ("" = no single fix)
    std::string title;
    std::string why;
    bool critical = false;
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
    std::string note;        // skip reason, user's comment
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
    // every step done, and no critical step failed or skipped
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
