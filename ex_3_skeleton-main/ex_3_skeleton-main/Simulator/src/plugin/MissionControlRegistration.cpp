#include <Common/MissionControlRegistration.h>

#include "PluginRegistrar.h"

#include <utility>

namespace common {

MissionControlRegistration::MissionControlRegistration(MissionControlFactory factory) {
    ::simulator::PluginRegistrar::instance().setMissionControlFactory(std::move(factory));
}

} // namespace common
