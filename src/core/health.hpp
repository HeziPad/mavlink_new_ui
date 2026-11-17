#pragma once

// Plain-language vehicle health: turns EKF flags, SYS_STATUS sensor bits, GPS,
// battery, link state and the autopilot's STATUSTEXT messages into advisories
// a pilot can act on: what's wrong, why it matters, and what to try. Advice
// only; nothing here changes the vehicle.

#include "vehicle_state.hpp"

#include <optional>
#include <string>
#include <vector>

namespace gcs {

enum class Severity {
    Info = 0,      // FYI
    Caution = 1,   // worth a look; flying is fine
    Warning = 2,   // blocks arming or degrades a function
    Critical = 3,  // act now
};

struct Advisory {
    std::string id;        // stable key (dedupe, "is this new?")
    Severity severity = Severity::Caution;
    std::string title;     // a few words: "Position estimate not ready"
    std::string cause;     // what is happening and why it matters
    std::vector<std::string> suggestions;
    std::string detail;    // raw data for experts: "EKF flags 0x00a7"
};

// EKF_STATUS_REPORT innovation ratios, as Mission Planner colours them.
constexpr double kEkfVarCaution = 0.5;
constexpr double kEkfVarBad = 0.8;

// Everything currently wrong with the vehicle (state-derived), worst first.
// `ever_connected`: a lost link is only reported after it was once up.
std::vector<Advisory> assess(const VehicleState& s, Clock::time_point now, bool ever_connected);

// The autopilot's text messages translated. nullopt: not a known problem
// (show it verbatim). The advisory's detail is the original text.
std::optional<Advisory> explainStatusText(const std::string& text, int severity);

// Keeps advisories that come from text messages. ArduPilot repeats failing
// PreArm reasons every 30 s while disarmed, so each one lives kStickyLife
// after its last repeat and they all clear when the vehicle arms.
class AdvisoryBook {
public:
    static constexpr double kStickyLife = 35.0;  // s

    // Returns the advisory when the text is a known problem not already
    // listed (so the caller can announce it once).
    std::optional<Advisory> onStatusText(const std::string& text, int severity, Clock::time_point now);
    void update(const VehicleState& s, Clock::time_point now);
    std::vector<Advisory> current() const;

private:
    struct Entry {
        Advisory adv;
        Clock::time_point last;
        bool prearm = false;
    };
    std::vector<Entry> entries_;
    bool was_armed_ = false;
};

// Merge, de-duplicate by id and sort worst first.
std::vector<Advisory> mergeAdvisories(std::vector<Advisory> a, const std::vector<Advisory>& b);

const char* severityName(Severity s);

}  // namespace gcs
