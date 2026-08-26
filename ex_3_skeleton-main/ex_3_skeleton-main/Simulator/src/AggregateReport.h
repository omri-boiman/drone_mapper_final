#pragma once

#include <filesystem>
#include <string>
#include <vector>

namespace simulator {

// Score/steps totals for one plugin's full run across the whole composition.
struct PluginTotals {
    std::string so_name;
    double total_score = 0.0;
    std::size_t total_steps = 0;
};

// "Agreeing" managers are grouped by identical (total_score, total_steps) --
// this is our chosen definition (the spec leaves it to us); documented in
// README.md. Groups sorted by size descending, per spec's YAML comment.
void writeComparativeReport(const std::filesystem::path& composition_file,
                            const std::filesystem::path& mission_control_folder,
                            const std::vector<PluginTotals>& totals,
                            const std::vector<std::string>& errors,
                            const std::filesystem::path& output_path);

// Sorted by score descending, then steps ascending, per spec.
void writeCompetitiveReport(const std::filesystem::path& composition_file,
                            const std::filesystem::path& mission_control_so,
                            const std::vector<PluginTotals>& totals,
                            const std::vector<std::string>& errors,
                            const std::filesystem::path& output_path);

} // namespace simulator
