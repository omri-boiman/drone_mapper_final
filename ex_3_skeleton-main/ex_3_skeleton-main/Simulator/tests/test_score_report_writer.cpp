#include <gtest/gtest.h>

#include <Simulator/ScoreReportWriter.h>

#include <yaml-cpp/yaml.h>

#include <filesystem>

namespace sim = simulator;

class ScoreReportWriter : public ::testing::Test {};

TEST_F(ScoreReportWriter, MinScoreIsZeroNotMinusOneWhenAllRunsError) {
    // No run has a valid score, so scored_scores is empty. min_score must fall back
    // to 0.0 — the error sentinel is carried separately via error_score, not min_score.
    sim::types::SimulationManagerReport report;
    report.composition_file = "composition.yaml";
    report.generated_at_utc = "2026-01-01T00:00:00Z";
    report.metric           = "output_map_accuracy";
    report.score_range      = {0.0, 100.0};
    report.error_score      = -1;

    sim::types::SimulationResult err_result;
    err_result.mission_results = {{common::types::MissionRunStatus::Error, 0, {{"ERR", "ERR"}}}};
    err_result.mission_score   = -1.0;
    report.runs.push_back(err_result);

    sim::YamlConfigParser::CompositionWithPaths paths{};
    const std::filesystem::path out_dir = "/tmp/test_score_report_writer_all_error";
    sim::ScoreReportWriter::write(report, paths, out_dir);

    const YAML::Node root = YAML::LoadFile((out_dir / "simulation_output.yaml").string());
    const std::string min_score = root["score_report"]["summary"]["min_score"].as<std::string>();
    EXPECT_EQ(min_score, "0.0")
        << "min_score must be 0.0 (not -1.0) when there are no scored runs";
}
