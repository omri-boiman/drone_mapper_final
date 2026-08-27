#include "PluginLoader.h"

#include "PluginRegistrar.h"

#include <dlfcn.h>
#include <tuple>

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
    // Also drain (and discard) the OTHER slot: a .so that self-registers as
    // the wrong kind (e.g. a real Algorithm .so dlopen'd here because it was
    // mistakenly placed in a mission_control_folder) would otherwise leave a
    // stale pending MissionControlFactory sitting in the registrar -- a
    // std::function whose captured code lives inside THIS .so, about to be
    // dlclose()'d below. Left undrained, that stale factory later segfaults
    // whenever the registrar singleton is destroyed (at process exit) or the
    // slot is next read, since it references now-unmapped memory. Found via
    // test_multiplugin.cpp's Comparative_ValidSoMissingRegistration test.
    std::ignore = PluginRegistrar::instance().takeMissionControlFactory();

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
    // See the matching comment in loadAlgorithmPlugin above -- same fix,
    // opposite direction (an Algorithm .so mistakenly loaded here).
    std::ignore = PluginRegistrar::instance().takeAlgorithmFactory();

    if (!factory) {
        error = "no REGISTER_MISSION_CONTROL call found in " + so_path.string();
        return std::nullopt;
    }

    return LoadedMissionControlPlugin{so_path, std::move(handle), std::move(*factory)};
}

} // namespace simulator
