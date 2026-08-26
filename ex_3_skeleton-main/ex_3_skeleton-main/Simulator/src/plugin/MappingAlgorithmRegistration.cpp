#include <Common/MappingAlgorithmRegistration.h>

#include "PluginRegistrar.h"

#include <utility>

namespace common {

MappingAlgorithmRegistration::MappingAlgorithmRegistration(MappingAlgorithmFactory factory) {
    ::simulator::PluginRegistrar::instance().setAlgorithmFactory(std::move(factory));
}

} // namespace common
