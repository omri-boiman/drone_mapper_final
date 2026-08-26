#pragma once

#include "DynamicLibraryHandle.h"

#include <Common/MappingAlgorithmFactory.h>
#include <Common/MissionControlFactory.h>

#include <filesystem>
#include <optional>
#include <string>

namespace simulator {

// Field order matters: C++ destroys members in reverse declaration order, so
// `handle` is declared FIRST (destroyed LAST, i.e. dlclose() runs last) and
// `factory` is declared SECOND (destroyed FIRST). `factory` is a std::function
// whose type-erased destroy/invoke code lives inside the .so's mapped memory --
// destroying it after dlclose() has already unmapped that code segfaults.
// See project_ex3_status.md for how this was confirmed with a throwaway repro.
struct LoadedAlgorithmPlugin {
    std::filesystem::path so_path;
    DynamicLibraryHandle handle;
    common::MappingAlgorithmFactory factory;
};

struct LoadedMissionControlPlugin {
    std::filesystem::path so_path;
    DynamicLibraryHandle handle;
    common::MissionControlFactory factory;
};

// Loads exactly one .so and drains the matching PluginRegistrar slot immediately
// afterward. Returns nullopt (with `error` filled in) if dlopen() fails or the
// .so never called the matching REGISTER_* macro.
[[nodiscard]] std::optional<LoadedAlgorithmPlugin> loadAlgorithmPlugin(
    const std::filesystem::path& so_path, std::string& error);

[[nodiscard]] std::optional<LoadedMissionControlPlugin> loadMissionControlPlugin(
    const std::filesystem::path& so_path, std::string& error);

} // namespace simulator
