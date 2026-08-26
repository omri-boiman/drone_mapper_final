#include "AggregateReport.h"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace simulator {

namespace {

std::string utcNow() {
    const auto now = std::chrono::system_clock::now();
    const auto tt  = std::chrono::system_clock::to_time_t(now);
    std::ostringstream oss;
    oss << std::put_time(std::gmtime(&tt), "%Y-%m-%dT%H:%M:%SZ");
    return oss.str();
}

YAML::Node stringSequence(const std::vector<std::string>& items) {
    YAML::Node node(YAML::NodeType::Sequence);
    for (const auto& s : items) node.push_back(s);
    return node;
}

} // namespace

void writeComparativeReport(const std::filesystem::path& composition_file,
                            const std::filesystem::path& mission_control_folder,
                            const std::vector<PluginTotals>& totals,
                            const std::vector<std::string>& errors,
                            const std::filesystem::path& output_path) {
    std::vector<std::vector<const PluginTotals*>> groups;
    for (const auto& t : totals) {
        bool placed = false;
        for (auto& g : groups) {
            if (g.front()->total_score == t.total_score && g.front()->total_steps == t.total_steps) {
                g.push_back(&t);
                placed = true;
                break;
            }
        }
        if (!placed) groups.push_back({&t});
    }
    std::stable_sort(groups.begin(), groups.end(),
        [](const auto& a, const auto& b) { return a.size() > b.size(); });

    YAML::Node root;
    YAML::Node comparative_report;
    comparative_report["composition_file"] = composition_file.string();
    comparative_report["mission_control_folder"] = mission_control_folder.string();
    comparative_report["generated_at_utc"] = utcNow();

    YAML::Node results_summary(YAML::NodeType::Sequence);
    for (const auto& g : groups) {
        std::vector<std::string> names;
        names.reserve(g.size());
        for (const auto* t : g) names.push_back(t->so_name);

        YAML::Node entry;
        entry["same_results"] = stringSequence(names);
        entry["total_score"] = g.front()->total_score;
        entry["total_steps"] = static_cast<int>(g.front()->total_steps);
        results_summary.push_back(entry);
    }
    comparative_report["results_summary"] = results_summary;
    comparative_report["errors"] = stringSequence(errors);

    root["comparative_report"] = comparative_report;

    const auto out_file = output_path / "comparative_report.yaml";
    std::ofstream f(out_file);
    if (!f) throw std::runtime_error("could not open " + out_file.string() + " for writing");
    f << root;
}

void writeCompetitiveReport(const std::filesystem::path& composition_file,
                            const std::filesystem::path& mission_control_so,
                            const std::vector<PluginTotals>& totals,
                            const std::vector<std::string>& errors,
                            const std::filesystem::path& output_path) {
    std::vector<PluginTotals> sorted = totals;
    std::stable_sort(sorted.begin(), sorted.end(), [](const PluginTotals& a, const PluginTotals& b) {
        if (a.total_score != b.total_score) return a.total_score > b.total_score;
        return a.total_steps < b.total_steps;
    });

    YAML::Node root;
    YAML::Node competitive_report;
    competitive_report["composition_file"] = composition_file.string();
    competitive_report["mission_control"] = mission_control_so.string();
    competitive_report["generated_at_utc"] = utcNow();

    YAML::Node results_summary(YAML::NodeType::Sequence);
    for (const auto& t : sorted) {
        YAML::Node entry;
        entry["algorithm"] = t.so_name;
        entry["total_score"] = t.total_score;
        entry["total_steps"] = static_cast<int>(t.total_steps);
        results_summary.push_back(entry);
    }
    competitive_report["results_summary"] = results_summary;
    competitive_report["errors"] = stringSequence(errors);

    root["competitive_report"] = competitive_report;

    const auto out_file = output_path / "competitive_report.yaml";
    std::ofstream f(out_file);
    if (!f) throw std::runtime_error("could not open " + out_file.string() + " for writing");
    f << root;
}

} // namespace simulator
