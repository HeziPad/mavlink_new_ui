#include "payload/adapter.hpp"

namespace gcs::payload {

// one factory per adapter file (src/payload/adapters/*.cpp)
std::unique_ptr<PayloadAdapter> makeNextVisionAdapter();

// ---- registry ---------------------------------------------------------------------
// A payload with deep integration (camera LOS on the map, tap-to-track...):
// add an adapter file and one line here. Every other ICD still gets the
// generic Payload panel from its JSON alone.
std::unique_ptr<PayloadAdapter> makeAdapter(const std::string& icd_id) {
    if (icd_id == "nextvision_trip") {
        return makeNextVisionAdapter();
    }
    return nullptr;
}

}  // namespace gcs::payload
