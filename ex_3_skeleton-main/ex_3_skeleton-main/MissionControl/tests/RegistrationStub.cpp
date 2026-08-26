// MissionControlImpl.cpp calls REGISTER_MISSION_CONTROL(...) at global scope,
// which needs MissionControlRegistration's constructor defined somewhere. In
// the real build that lives in the Simulator project; for a standalone
// MissionControl test binary we only need it to LINK, not register anything.
#include <Common/MissionControlRegistration.h>

namespace common {

MissionControlRegistration::MissionControlRegistration(MissionControlFactory) {}

} // namespace common
