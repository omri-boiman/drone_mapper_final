#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace simulator {

enum class Mode { Comparative, Competition };

struct ParsedArgs {
    Mode mode;
    std::filesystem::path simulation;
    std::filesystem::path mission_control_folder; // comparative only
    std::filesystem::path algorithm;               // comparative only
    std::filesystem::path mission_control;         // competition only
    std::filesystem::path algorithms_folder;       // competition only
    std::optional<unsigned> num_threads;
    bool verbose = false;
};

// Parses and fully validates argv per the assignment spec: any order, all
// non-optional args mandatory, unsupported/missing args detected and reported
// together, file/folder existence and folder-has-.so-files checked. On any
// failure, prints a usage line plus every applicable error to stderr and
// returns nullopt -- the caller should treat that as "print nothing else,
// exit with a non-zero code."
[[nodiscard]] std::optional<ParsedArgs> parseAndValidateArgs(
    const std::vector<std::string>& args, const char* program_name);

} // namespace simulator
