#pragma once

// Single include point for the generated C library. Channel status lives in one
// translation unit (mavlink_globals.cpp) so sequence numbers and parser state
// are shared by every file that packs or parses messages.
#ifndef MAVLINK_EXTERNAL_RX_STATUS
#define MAVLINK_EXTERNAL_RX_STATUS 1
#endif

#include <mavlink_types.h>

extern mavlink_status_t m_mavlink_status[MAVLINK_COMM_NUM_BUFFERS];

#include <ardupilotmega/mavlink.h>

namespace gcs::link {

struct LinkIds {
    uint8_t own_sys = 255;
    uint8_t own_comp = 190;  // MAV_COMP_ID_MISSIONPLANNER
    uint8_t target_sys = 1;
    uint8_t target_comp = 1;
};

}  // namespace gcs::link
