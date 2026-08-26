#pragma once

#include <Common/MappingAlgorithmFactory.h>
#include <Common/MissionControlFactory.h>

#include <optional>
#include <utility>

namespace simulator {

// Singleton landing pad for factories registered by dynamically-loaded plugins.
// The registration constructors (defined in MappingAlgorithmRegistration.cpp /
// MissionControlRegistration.cpp) stash their factory here as a side effect of
// the static initialization that dlopen() triggers.
//
// Invariant this class relies on: exactly one .so is dlopen()'d at a time, and
// its matching pending slot is drained immediately afterward (see PluginLoader).
// No dlopen() call happens concurrently with another, so a single pending slot
// per registration type is sufficient -- no keying by handle/path is needed.
class PluginRegistrar {
public:
    static PluginRegistrar& instance() {
        static PluginRegistrar registrar;
        return registrar;
    }

    void setAlgorithmFactory(common::MappingAlgorithmFactory factory) {
        pending_algorithm_ = std::move(factory);
    }
    void setMissionControlFactory(common::MissionControlFactory factory) {
        pending_mission_control_ = std::move(factory);
    }

    [[nodiscard]] std::optional<common::MappingAlgorithmFactory> takeAlgorithmFactory() {
        auto out = std::move(pending_algorithm_);
        pending_algorithm_.reset();
        return out;
    }
    [[nodiscard]] std::optional<common::MissionControlFactory> takeMissionControlFactory() {
        auto out = std::move(pending_mission_control_);
        pending_mission_control_.reset();
        return out;
    }

private:
    PluginRegistrar() = default;

    std::optional<common::MappingAlgorithmFactory> pending_algorithm_;
    std::optional<common::MissionControlFactory> pending_mission_control_;
};

} // namespace simulator
