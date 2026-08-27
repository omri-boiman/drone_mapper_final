#include <gtest/gtest.h>

#include "AggregateReport.h"

#include <yaml-cpp/yaml.h>

#include <filesystem>

// Unit tests for Simulator/src/AggregateReport.cpp -- previously zero
// coverage (see ex3-test-plan.md, section 2). writeComparativeReport/
// writeCompetitiveReport are pure functions of plain data (PluginTotals +
// errors + an output path) -- no plugin loading or simulation running is
// needed to test them; the written YAML is parsed back with yaml-cpp and
// asserted on structurally, not just "file exists".

namespace sim = simulator;

namespace {

std::filesystem::path freshTmpDir(const std::string& name) {
    const auto p = std::filesystem::temp_directory_path() / ("ex3_aggregate_report_" + name);
    std::filesystem::remove_all(p);
    std::filesystem::create_directories(p);
    return p;
}

} // namespace

class AggregateReport : public ::testing::Test {};

// ---- writeComparativeReport ----

TEST_F(AggregateReport, ComparativeReport_TopLevelFields) {
    const auto out = freshTmpDir("top_level");
    sim::writeComparativeReport("composition.yaml", "/some/mc_folder",
                                {{"a.so", 10.0, 5}}, {}, out);

    const auto root = YAML::LoadFile((out / "comparative_report.yaml").string());
    const auto report = root["comparative_report"];
    ASSERT_TRUE(report);
    EXPECT_EQ(report["composition_file"].as<std::string>(), "composition.yaml");
    EXPECT_EQ(report["mission_control_folder"].as<std::string>(), "/some/mc_folder");
    ASSERT_TRUE(report["generated_at_utc"]);
    // ISO-8601 UTC per the spec's example ("2026-05-30T23:31:10Z"): 20 chars,
    // ends in 'Z'.
    const auto ts = report["generated_at_utc"].as<std::string>();
    EXPECT_EQ(ts.size(), 20u) << ts;
    EXPECT_EQ(ts.back(), 'Z') << ts;
}

TEST_F(AggregateReport, ComparativeReport_GroupsMatchingTotals) {
    const auto out = freshTmpDir("grouping");
    sim::writeComparativeReport("c.yaml", "mc_folder", {
        {"mgr1.so", 495.0, 100},
        {"mgr2.so", 495.0, 100}, // matches mgr1 -> same group
        {"mgr3.so", 502.0, 124}, // distinct
    }, {}, out);

    const auto root = YAML::LoadFile((out / "comparative_report.yaml").string());
    const auto summary = root["comparative_report"]["results_summary"];
    ASSERT_EQ(summary.size(), 2u);

    // Group of 2 sorts before group of 1 (spec: sorted by number of agreeing
    // managers, descending).
    ASSERT_EQ(summary[0]["same_results"].size(), 2u);
    EXPECT_EQ(summary[0]["same_results"][0].as<std::string>(), "mgr1.so");
    EXPECT_EQ(summary[0]["same_results"][1].as<std::string>(), "mgr2.so");
    EXPECT_DOUBLE_EQ(summary[0]["total_score"].as<double>(), 495.0);
    EXPECT_EQ(summary[0]["total_steps"].as<int>(), 100);

    ASSERT_EQ(summary[1]["same_results"].size(), 1u);
    EXPECT_EQ(summary[1]["same_results"][0].as<std::string>(), "mgr3.so");
    EXPECT_DOUBLE_EQ(summary[1]["total_score"].as<double>(), 502.0);
    EXPECT_EQ(summary[1]["total_steps"].as<int>(), 124);
}

TEST_F(AggregateReport, ComparativeReport_GroupingRequiresBothScoreAndSteps) {
    // Same score but different steps (or vice versa) must NOT be grouped --
    // the chosen "agreeing managers" definition is the (score, steps) PAIR.
    const auto out = freshTmpDir("grouping_pair");
    sim::writeComparativeReport("c.yaml", "mc_folder", {
        {"mgrA.so", 100.0, 50},
        {"mgrB.so", 100.0, 51}, // same score, different steps -> separate group
    }, {}, out);

    const auto root = YAML::LoadFile((out / "comparative_report.yaml").string());
    const auto summary = root["comparative_report"]["results_summary"];
    ASSERT_EQ(summary.size(), 2u);
    EXPECT_EQ(summary[0]["same_results"].size(), 1u);
    EXPECT_EQ(summary[1]["same_results"].size(), 1u);
}

TEST_F(AggregateReport, ComparativeReport_ThreeWayGroupSizesSortDescending) {
    const auto out = freshTmpDir("three_way");
    // Deliberately scrambled input order: group sizes 1, 2, 3.
    sim::writeComparativeReport("c.yaml", "mc_folder", {
        {"solo.so", 1.0, 1},
        {"pairA.so", 2.0, 2},
        {"trioA.so", 3.0, 3},
        {"pairB.so", 2.0, 2},
        {"trioB.so", 3.0, 3},
        {"trioC.so", 3.0, 3},
    }, {}, out);

    const auto root = YAML::LoadFile((out / "comparative_report.yaml").string());
    const auto summary = root["comparative_report"]["results_summary"];
    ASSERT_EQ(summary.size(), 3u);
    EXPECT_EQ(summary[0]["same_results"].size(), 3u); // trio group first
    EXPECT_EQ(summary[1]["same_results"].size(), 2u); // pair group second
    EXPECT_EQ(summary[2]["same_results"].size(), 1u); // solo group last
}

TEST_F(AggregateReport, ComparativeReport_ErrorsListedSeparatelyFromResults) {
    const auto out = freshTmpDir("errors");
    sim::writeComparativeReport("c.yaml", "mc_folder",
        {{"good.so", 10.0, 5}}, {"bad1.so", "bad2.so"}, out);

    const auto root = YAML::LoadFile((out / "comparative_report.yaml").string());
    const auto report = root["comparative_report"];
    ASSERT_EQ(report["results_summary"].size(), 1u);
    EXPECT_EQ(report["results_summary"][0]["same_results"][0].as<std::string>(), "good.so");

    ASSERT_EQ(report["errors"].size(), 2u);
    EXPECT_EQ(report["errors"][0].as<std::string>(), "bad1.so");
    EXPECT_EQ(report["errors"][1].as<std::string>(), "bad2.so");
}

TEST_F(AggregateReport, ComparativeReport_NoSuccessesOnlyErrors) {
    const auto out = freshTmpDir("all_errors");
    ASSERT_NO_THROW(sim::writeComparativeReport("c.yaml", "mc_folder", {},
                                                {"bad1.so", "bad2.so"}, out));

    const auto root = YAML::LoadFile((out / "comparative_report.yaml").string());
    const auto report = root["comparative_report"];
    EXPECT_EQ(report["results_summary"].size(), 0u);
    EXPECT_EQ(report["errors"].size(), 2u);
}

TEST_F(AggregateReport, ComparativeReport_NoErrorsAtAll) {
    const auto out = freshTmpDir("no_errors");
    sim::writeComparativeReport("c.yaml", "mc_folder", {{"a.so", 1.0, 1}}, {}, out);

    const auto root = YAML::LoadFile((out / "comparative_report.yaml").string());
    const auto errors = root["comparative_report"]["errors"];
    ASSERT_TRUE(errors);
    EXPECT_TRUE(errors.IsSequence());
    EXPECT_EQ(errors.size(), 0u);
}

TEST_F(AggregateReport, ComparativeReport_SingleEntry_RegressionGuard) {
    // Locks down the exact single-plugin shape that was manually verified
    // against the real dlopen pipeline (see memory-ex3-status.md).
    const auto out = freshTmpDir("single");
    sim::writeComparativeReport("scenarios/scenario1/simulation.yaml", "/mc",
        {{"MissionControl_211781141_325049575.so", 97.58224336107808, 320}}, {}, out);

    const auto root = YAML::LoadFile((out / "comparative_report.yaml").string());
    const auto summary = root["comparative_report"]["results_summary"];
    ASSERT_EQ(summary.size(), 1u);
    EXPECT_EQ(summary[0]["same_results"].size(), 1u);
    EXPECT_EQ(summary[0]["same_results"][0].as<std::string>(),
              "MissionControl_211781141_325049575.so");
    EXPECT_DOUBLE_EQ(summary[0]["total_score"].as<double>(), 97.58224336107808);
    EXPECT_EQ(summary[0]["total_steps"].as<int>(), 320);
}

// ---- writeCompetitiveReport ----

TEST_F(AggregateReport, CompetitiveReport_TopLevelFields) {
    const auto out = freshTmpDir("comp_top_level");
    sim::writeCompetitiveReport("c.yaml", "mission_control.so",
                                {{"algo.so", 10.0, 5}}, {}, out);

    const auto root = YAML::LoadFile((out / "competitive_report.yaml").string());
    const auto report = root["competitive_report"];
    ASSERT_TRUE(report);
    EXPECT_EQ(report["composition_file"].as<std::string>(), "c.yaml");
    EXPECT_EQ(report["mission_control"].as<std::string>(), "mission_control.so");
    ASSERT_TRUE(report["generated_at_utc"]);
}

TEST_F(AggregateReport, CompetitiveReport_SortedByScoreDescending) {
    const auto out = freshTmpDir("comp_score_sort");
    sim::writeCompetitiveReport("c.yaml", "mc.so", {
        {"algorithm3.so", 490.0, 97},
        {"algorithm1.so", 495.0, 100},
        {"algorithm4.so", 490.0, 113},
    }, {}, out);

    const auto root = YAML::LoadFile((out / "competitive_report.yaml").string());
    const auto summary = root["competitive_report"]["results_summary"];
    ASSERT_EQ(summary.size(), 3u);
    EXPECT_EQ(summary[0]["algorithm"].as<std::string>(), "algorithm1.so"); // 495
    EXPECT_EQ(summary[1]["algorithm"].as<std::string>(), "algorithm3.so"); // 490, 97 steps
    EXPECT_EQ(summary[2]["algorithm"].as<std::string>(), "algorithm4.so"); // 490, 113 steps
}

TEST_F(AggregateReport, CompetitiveReport_TieBrokenByStepsAscending) {
    // Equal score requires the SECONDARY sort key (steps ascending) to
    // disambiguate -- this is the case the spec's own example illustrates
    // (algorithm3.so/algorithm4.so both score 490).
    const auto out = freshTmpDir("comp_tie_break");
    sim::writeCompetitiveReport("c.yaml", "mc.so", {
        {"slower.so", 100.0, 50},
        {"faster.so", 100.0, 20},
    }, {}, out);

    const auto root = YAML::LoadFile((out / "competitive_report.yaml").string());
    const auto summary = root["competitive_report"]["results_summary"];
    ASSERT_EQ(summary.size(), 2u);
    EXPECT_EQ(summary[0]["algorithm"].as<std::string>(), "faster.so");
    EXPECT_EQ(summary[1]["algorithm"].as<std::string>(), "slower.so");
}

TEST_F(AggregateReport, CompetitiveReport_ErrorsListedSeparately) {
    const auto out = freshTmpDir("comp_errors");
    sim::writeCompetitiveReport("c.yaml", "mc.so",
        {{"good.so", 10.0, 5}}, {"algorithm2.so", "algorithm5.so"}, out);

    const auto root = YAML::LoadFile((out / "competitive_report.yaml").string());
    const auto report = root["competitive_report"];
    ASSERT_EQ(report["results_summary"].size(), 1u);
    ASSERT_EQ(report["errors"].size(), 2u);
    EXPECT_EQ(report["errors"][0].as<std::string>(), "algorithm2.so");
    EXPECT_EQ(report["errors"][1].as<std::string>(), "algorithm5.so");
}

TEST_F(AggregateReport, CompetitiveReport_NoSuccessesOnlyErrors) {
    const auto out = freshTmpDir("comp_all_errors");
    ASSERT_NO_THROW(sim::writeCompetitiveReport("c.yaml", "mc.so", {},
                                                {"a.so", "b.so"}, out));

    const auto root = YAML::LoadFile((out / "competitive_report.yaml").string());
    const auto report = root["competitive_report"];
    EXPECT_EQ(report["results_summary"].size(), 0u);
    EXPECT_EQ(report["errors"].size(), 2u);
}
