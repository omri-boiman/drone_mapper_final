#include "PluginLoader.h"

#include "PluginRegistrar.h"

#include <dlfcn.h>

namespace simulator {

std::optional<LoadedAlgorithmPlugin> loadAlgorithmPlugin(
    const std::filesystem::path& so_path, std::string& error) {
    ::dlerror();
    void* raw = ::dlopen(so_path.c_str(), RTLD_NOW | RTLD_LOCAL);
    if (!raw) {
        error = std::string("dlopen failed: ") + ::dlerror();
        return std::nullopt;
    }
    DynamicLibraryHandle handle(raw);

    auto factory = PluginRegistrar::instance().takeAlgorithmFactory();
    if (!factory) {
        error = "no REGISTER_MAPPING_ALGORITHM call found in " + so_path.string();
        return std::nullopt; // handle goes out of scope here -> dlclose; nothing was built from it.
    }

    return LoadedAlgorithmPlugin{so_path, std::move(handle), std::move(*factory)};
}

std::optional<LoadedMissionControlPlugin> loadMissionControlPlugin(
    const std::filesystem::path& so_path, std::string& error) {
    ::dlerror();
    void* raw = ::dlopen(so_path.c_str(), RTLD_NOW | RTLD_LOCAL);
    if (!raw) {
        error = std::string("dlopen failed: ") + ::dlerror();
        return std::nullopt;
    }
    DynamicLibraryHandle handle(raw);

    auto factory = PluginRegistrar::instance().takeMissionControlFactory();
    if (!factory) {
        error = "no REGISTER_MISSION_CONTROL call found in " + so_path.string();
        return std::nullopt;
    }

    return LoadedMissionControlPlugin{so_path, std::move(handle), std::move(*factory)};
}

} // namespace simulator
