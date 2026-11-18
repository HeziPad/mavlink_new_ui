#pragma once

#include "core/mission.hpp"

#include <string>
#include <vector>

namespace gcs::link {

// Posted by the worker thread, drained by the UI via MavlinkClient::pollEvents().
struct LinkEvent {
    enum class Type {
        StatusText,        // text, severity
        CommandStep,       // tag, text(label), ok — intermediate sequence progress
        CommandResult,     // tag, ok, text — sequence finished
        MissionProgress,   // upload flag, done/total
        MissionUploaded,   // items
        MissionDownloaded, // items
        MissionFailed,     // upload flag, text
        VehicleConnected,  // first heartbeat from a vehicle (sysid in done)
        Error,             // text
        ParamSet,          // tag = name, ok, text = value read back (or the reason)
    };

    Type type = Type::StatusText;
    std::string tag;
    std::string text;
    int severity = 6;  // MAV_SEVERITY_*
    bool ok = false;
    bool upload = true;
    int done = 0;
    int total = 0;
    std::vector<MissionItem> items;
};

}  // namespace gcs::link
