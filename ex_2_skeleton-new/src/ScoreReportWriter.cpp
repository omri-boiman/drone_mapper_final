#include <drone_mapper/ScoreReportWriter.h>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <numeric>
#include <sstream>
#include <stdexcept>

namespace drone_mapper {

namespace {

std::string statusStr(types::MissionRunStatus s) {
    switch (s) {
        case types::MissionRunStatus::Completed: return "completed";
        case types::MissionRunStatus::MaxSteps:  return "max_steps";
        case types::MissionRunStatus::Error:     return "error";
    }
    return "error";
}

std::string resStatusStr(types::ResolutionRequestStatus s) {
    switch (s) {
        case types::ResolutionRequestStatus::Accepted:        return "ACCEPTED";
        case types::ResolutionRequestStatus::Ignored:         return "IGNORED";
        case types::ResolutionRequestStatus::IgnoredTooSmall: return "IGNORED TOO SMALL";
    }
    return "IGNORED";
}

} // namespace

void ScoreReportWriter::write(const types::SimulationManagerReport& report,
                               const std::filesystem::path& composition_file,
                               const YamlConfigParser::CompositionWithPaths& paths,
                               const std::filesystem::path& output_path) {
    std::filesystem::create_directories(output_path);
    const std::filesystem::path out_file = output_path / "simulation_output.yaml";

    // Summary stats
    std::vector<double> scored_scores;
    int error_count = 0;
    for (const auto& run : report.runs) {
        const bool is_error = (!run.mission_results.empty() &&
            run.mission_results[0].status == types::MissionRunStatus::Error)
            || run.mission_score < 0.0;
        if (is_error) {
            ++error_count;
        } else {
            scored_scores.push_back(run.mission_score);
        }
    }
    const int total_runs    = static_cast<int>(report.runs.size());
    const int scored_runs   = static_cast<int>(scored_scores.size());
    const double avg  = scored_scores.empty() ? 0.0
        : std::accumulate(scored_scores.begin(), scored_scores.end(), 0.0)
            / static_cast<double>(scored_scores.size());
    const double min_score = scored_scores.empty() ? 0.0
        : *std::min_element(scored_scores.begin(), scored_scores.end());
    const double max_score = scored_scores.empty() ? 0.0
        : *std::max_element(scored_scores.begin(), scored_scores.end());

    // Build YAML
    YAML::Node root;
    YAML::Node score_report;
    score_report["composition_file"] = composition_file.string();
    score_report["generated_at_utc"] = report.generated_at_utc;
    score_report["metric"]           = report.metric;
    score_report["score_range"]["min"] = 0;
    score_report["score_range"]["max"] = 100;
    score_report["error_score"] = report.error_score;

    YAML::Node summary;
    summary["total_runs"]   = total_runs;
    summary["scored_runs"]  = scored_runs;
    summary["error_runs"]   = error_count;
    {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(1) << avg;
        summary["average_score"] = ss.str();
    }
    {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(1) << min_score;
        summary["min_score"] = ss.str();
    }
    {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(1) << max_score;
        summary["max_score"] = ss.str();
    }
    score_report["summary"] = summary;

    // Build simulations section using the same nested iteration order as SimulationManager.
    // IMPORTANT: yaml-cpp's Node::operator=(Node()) resets NodeData in-place, so all
    // previously-pushed sequence references see the mutation. We avoid reusing YAML::Node
    // variables across iterations by declaring them fresh in each loop scope.
    const std::size_t n_d = paths.drone_paths.size();
    const std::size_t n_l = paths.lidar_paths.size();
    YAML::Node simulations_node(YAML::NodeType::Sequence);
    std::size_t run_idx = 0;

    for (std::size_t si = 0; si < paths.sim_paths.size(); ++si) {
        YAML::Node sim_node;
        sim_node["simulation_config"] = paths.sim_paths[si].string();

        const auto& missions = std::get<1>(paths.data.simulation_mission_groups[si]);
        const auto& mpaths   = si < paths.mission_paths_per_sim.size()
                                    ? paths.mission_paths_per_sim[si]
                                    : std::vector<std::filesystem::path>{};

        for (std::size_t mi = 0; mi < missions.size(); ++mi) {
            YAML::Node mission_node;
            mission_node["mission_config"] =
                mi < mpaths.size() ? mpaths[mi].string() : "";

            const auto& first_run = run_idx < report.runs.size()
                ? report.runs[run_idx] : types::SimulationResult{};
            mission_node["resolution_cm"] = static_cast<int>(
                first_run.output_map_config.resolution.force_numerical_value_in(cm));
            mission_node["resolution_request_status"] =
                resStatusStr(first_run.resolution_request_status);

            for (std::size_t di = 0; di < n_d; ++di) {
                for (std::size_t li = 0; li < n_l; ++li) {
                    if (run_idx >= report.runs.size()) break;
                    const auto& run = report.runs[run_idx++];

                    YAML::Node run_node;
                    run_node["drone_config"] = di < paths.drone_paths.size()
                        ? paths.drone_paths[di].string() : "";
                    run_node["lidar_config"] = li < paths.lidar_paths.size()
                        ? paths.lidar_paths[li].string() : "";

                    const auto& mr = run.mission_results.empty()
                        ? types::MissionRunResult{}
                        : run.mission_results[0];
                    run_node["status"] = statusStr(mr.status);
                    run_node["steps"]  = static_cast<int>(mr.steps);
                    {
                        std::ostringstream ss;
                        ss << std::fixed << std::setprecision(1) << run.mission_score;
                        run_node["score"] = ss.str();
                    }
                    if (mr.status == types::MissionRunStatus::Error && !mr.errors.empty()) {
                        run_node["error_ref"]["code"] = mr.errors[0].code;
                    }
                    mission_node["runs"].push_back(run_node);
                }
            }
            sim_node["missions"].push_back(mission_node);
        }
        simulations_node.push_back(sim_node);
    }

    score_report["simulations"] = simulations_node;
    root["score_report"] = score_report;

    std::ofstream f(out_file);
    if (!f)
        throw std::runtime_error("ScoreReportWriter: cannot open " + out_file.string());
    f << root;
    f.flush();
}

} // namespace drone_mapper
