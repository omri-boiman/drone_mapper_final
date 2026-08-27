#include <gtest/gtest.h>

#include <Common/IMap3D.h>
#include <Common/IMappingAlgorithm.h>

#include "AggregateReport.h"
#include "CliArgs.h"
#include "RunModes.h"
#include "plugin/PluginLoader.h"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <optional>
#include <string>
#include <vector>

// Exercises the REAL multi-plugin orchestration (runComparative/
// runCompetition, extracted out of main.cpp into RunModes.h specifically so
// it could be called directly here -- see test_partition_work.cpp's header
// comment) end to end: real dlopen'd .so's (both purpose-built fixtures --
// see fixtures/FixtureMissionControl.cpp and FixtureAlgorithm.cpp -- and the
// actual graded Algorithm/MissionControl .so's), real threading, real
// aggregate-report output on disk. See ex3-test-plan.md, sections 3, 4, 5.
//
// Only builds when linked with real fixture .so's + the real Algorithm/
// MissionControl .so's (see the Simulator/CMakeLists.txt guard around
// simulator_integration_real_test), since none of this is reachable through
// stub factories alone.

namespace sim = simulator;

namespace {

std::filesystem::path fixturesDir() {
    // Absolute, CWD-independent: __FILE__ is this test file's own path.
    return std::filesystem::path(__FILE__).parent_path() / "fixtures";
}

void copySo(const std::filesystem::path& src, const std::filesystem::path& dst) {
    std::filesystem::copy_file(src, dst, std::filesystem::copy_options::overwrite_existing);
}

std::filesystem::path freshDir(const std::string& name) {
    const auto p = std::filesystem::temp_directory_path() / ("ex3_multiplugin_" + name);
    std::filesystem::remove_all(p);
    std::filesystem::create_directories(p);
    return p;
}

// Finds the single subdirectory of `parent` whose name starts with `prefix`
// (i.e. the comparative_results_<time>/competition_<time> directory
// runComparative/runCompetition just created).
std::optional<std::filesystem::path> findResultsDir(const std::filesystem::path& parent,
                                                     const std::string& prefix) {
    std::optional<std::filesystem::path> found;
    for (const auto& e : std::filesystem::directory_iterator(parent)) {
        if (e.is_directory() && e.path().filename().string().rfind(prefix, 0) == 0) {
            found = e.path();
        }
    }
    return found;
}

} // namespace

class MultiPlugin : public ::testing::Test {};

// ---- Comparative mode: multi-MissionControl grouping ----

TEST_F(MultiPlugin, Comparative_ThreeMissionControls_GroupsAndSortsCorrectly) {
    const auto mc_folder = freshDir("comparative_grouping");
    copySo(FIXTURE_MC_A_SO_PATH, mc_folder / "fixture_mc_a.so"); // steps=5
    copySo(FIXTURE_MC_B_SO_PATH, mc_folder / "fixture_mc_b.so"); // steps=5 -> groups with A
    copySo(FIXTURE_MC_C_SO_PATH, mc_folder / "fixture_mc_c.so"); // steps=9 -> separate group

    sim::ParsedArgs args;
    args.mode = sim::Mode::Comparative;
    args.simulation = fixturesDir() / "composition.yaml";
    args.mission_control_folder = mc_folder;
    args.algorithm = REAL_ALGORITHM_SO_PATH; // must be real+loadable; these fixtures ignore it

    EXPECT_EQ(sim::runComparative(args), 0);

    const auto results_dir = findResultsDir(mc_folder, "comparative_results_");
    ASSERT_TRUE(results_dir.has_value());

    const auto root = YAML::LoadFile((*results_dir / "comparative_report.yaml").string());
    const auto summary = root["comparative_report"]["results_summary"];
    ASSERT_EQ(summary.size(), 2u);
    ASSERT_EQ(summary[0]["same_results"].size(), 2u); // bigger group sorts first
    EXPECT_EQ(summary[1]["same_results"].size(), 1u);

    std::vector<std::string> group0_names = {
        summary[0]["same_results"][0].as<std::string>(),
        summary[0]["same_results"][1].as<std::string>(),
    };
    std::sort(group0_names.begin(), group0_names.end());
    EXPECT_EQ(group0_names, (std::vector<std::string>{"fixture_mc_a.so", "fixture_mc_b.so"}));
    EXPECT_EQ(summary[1]["same_results"][0].as<std::string>(), "fixture_mc_c.so");

    // Per-plugin legacy YAML exists for all three, in the shared output folder
    // (spec point 7: "the name of the mission control added to the file
    // name... added to the same output folder").
    for (const std::string& name : {"fixture_mc_a.so", "fixture_mc_b.so", "fixture_mc_c.so"}) {
        EXPECT_TRUE(std::filesystem::exists(*results_dir / ("simulation_output_" + name + ".yaml")))
            << name;
        // Each plugin's own output/error-log subdirectory exists, isolated
        // from the others.
        EXPECT_TRUE(std::filesystem::exists(*results_dir / name / "output_results")) << name;
    }
}

TEST_F(MultiPlugin, Comparative_MixedSuccessAndCorruptedPlugin_ErrorIsolatedFromSuccess) {
    const auto mc_folder = freshDir("comparative_mixed_failure");
    copySo(FIXTURE_MC_A_SO_PATH, mc_folder / "fixture_mc_a.so");
    // A corrupted, non-ELF ".so": dlopen must fail cleanly for this one
    // without blocking the other (valid) plugin from running.
    { std::ofstream(mc_folder / "not_really_a_plugin.so") << "this is not an ELF file\n"; }

    sim::ParsedArgs args;
    args.mode = sim::Mode::Comparative;
    args.simulation = fixturesDir() / "composition.yaml";
    args.mission_control_folder = mc_folder;
    args.algorithm = REAL_ALGORITHM_SO_PATH;

    EXPECT_EQ(sim::runComparative(args), 0);

    const auto results_dir = findResultsDir(mc_folder, "comparative_results_");
    ASSERT_TRUE(results_dir.has_value());

    const auto root = YAML::LoadFile((*results_dir / "comparative_report.yaml").string());
    const auto report = root["comparative_report"];
    ASSERT_EQ(report["results_summary"].size(), 1u);
    EXPECT_EQ(report["results_summary"][0]["same_results"][0].as<std::string>(), "fixture_mc_a.so");
    ASSERT_EQ(report["errors"].size(), 1u);
    EXPECT_EQ(report["errors"][0].as<std::string>(), "not_really_a_plugin.so");
}

TEST_F(MultiPlugin, Comparative_ValidSoMissingRegistration_ReportedAsError) {
    // A genuinely valid ELF .so (the REAL Algorithm .so) placed where a
    // MissionControl .so is expected: dlopen() itself succeeds, but it never
    // called REGISTER_MISSION_CONTROL, so PluginLoader must report it as a
    // load failure -- not crash, not silently produce a null/garbage factory.
    const auto mc_folder = freshDir("comparative_wrong_kind");
    copySo(FIXTURE_MC_A_SO_PATH, mc_folder / "fixture_mc_a.so");
    copySo(REAL_ALGORITHM_SO_PATH, mc_folder / "algorithm_pretending_to_be_mc.so");

    sim::ParsedArgs args;
    args.mode = sim::Mode::Comparative;
    args.simulation = fixturesDir() / "composition.yaml";
    args.mission_control_folder = mc_folder;
    args.algorithm = REAL_ALGORITHM_SO_PATH;

    EXPECT_EQ(sim::runComparative(args), 0);

    const auto results_dir = findResultsDir(mc_folder, "comparative_results_");
    ASSERT_TRUE(results_dir.has_value());

    const auto root = YAML::LoadFile((*results_dir / "comparative_report.yaml").string());
    const auto report = root["comparative_report"];
    ASSERT_EQ(report["results_summary"].size(), 1u);
    EXPECT_EQ(report["results_summary"][0]["same_results"][0].as<std::string>(), "fixture_mc_a.so");
    ASSERT_EQ(report["errors"].size(), 1u);
    EXPECT_EQ(report["errors"][0].as<std::string>(), "algorithm_pretending_to_be_mc.so");
}

// ---- Competition mode: multi-Algorithm sort order ----

TEST_F(MultiPlugin, Competition_ThreeAlgorithms_SortedByStepsAscending) {
    // All three fixtures produce the same (baseline) mission_score, since
    // none of them ever issue a scan command -- see FixtureAlgorithm.cpp --
    // so this exercises the steps-ascending tie-break with an unambiguous,
    // fully-determined expected order.
    const auto algo_folder = freshDir("competition_sort");
    copySo(FIXTURE_ALGO_C_SO_PATH, algo_folder / "fixture_algo_c.so"); // steps=6
    copySo(FIXTURE_ALGO_A_SO_PATH, algo_folder / "fixture_algo_a.so"); // steps=2
    copySo(FIXTURE_ALGO_B_SO_PATH, algo_folder / "fixture_algo_b.so"); // steps=4

    sim::ParsedArgs args;
    args.mode = sim::Mode::Competition;
    args.simulation = fixturesDir() / "composition.yaml";
    // Must be the REAL MissionControl -- it has to actually drive each
    // algorithm's nextStep() loop for the differing FIXTURE_STEPS values to
    // show up as differing total_steps at all.
    args.mission_control = REAL_MISSION_CONTROL_SO_PATH;
    args.algorithms_folder = algo_folder;

    EXPECT_EQ(sim::runCompetition(args), 0);

    const auto results_dir = findResultsDir(algo_folder, "competition_");
    ASSERT_TRUE(results_dir.has_value());

    const auto root = YAML::LoadFile((*results_dir / "competitive_report.yaml").string());
    const auto summary = root["competitive_report"]["results_summary"];
    ASSERT_EQ(summary.size(), 3u);
    EXPECT_EQ(summary[0]["algorithm"].as<std::string>(), "fixture_algo_a.so"); // fewest steps
    EXPECT_EQ(summary[1]["algorithm"].as<std::string>(), "fixture_algo_b.so");
    EXPECT_EQ(summary[2]["algorithm"].as<std::string>(), "fixture_algo_c.so"); // most steps
}

// ---- Threading correctness (beyond wall-clock timing) ----

TEST_F(MultiPlugin, Comparative_ThreadedAndSingleThreaded_ProduceIdenticalTotals) {
    auto runOnce = [](std::optional<unsigned> num_threads) {
        const auto mc_folder = freshDir(num_threads ? "threaded" : "unthreaded");
        copySo(FIXTURE_MC_A_SO_PATH, mc_folder / "fixture_mc_a.so");
        copySo(FIXTURE_MC_B_SO_PATH, mc_folder / "fixture_mc_b.so");
        copySo(FIXTURE_MC_C_SO_PATH, mc_folder / "fixture_mc_c.so");

        sim::ParsedArgs args;
        args.mode = sim::Mode::Comparative;
        args.simulation = fixturesDir() / "composition.yaml";
        args.mission_control_folder = mc_folder;
        args.algorithm = REAL_ALGORITHM_SO_PATH;
        args.num_threads = num_threads;

        EXPECT_EQ(sim::runComparative(args), 0);

        const auto results_dir = findResultsDir(mc_folder, "comparative_results_");
        return YAML::LoadFile((*results_dir / "comparative_report.yaml").string());
    };

    const auto single = runOnce(std::nullopt);
    // 3 items, 3 threads requested -> extra_threads = min(3, 2) = 2 -> 3
    // total workers, i.e. all 3 plugins genuinely run concurrently.
    const auto threaded = runOnce(3u);

    const auto s_summary = single["comparative_report"]["results_summary"];
    const auto t_summary = threaded["comparative_report"]["results_summary"];
    ASSERT_EQ(s_summary.size(), t_summary.size());
    for (std::size_t i = 0; i < s_summary.size(); ++i) {
        EXPECT_DOUBLE_EQ(s_summary[i]["total_score"].as<double>(),
                        t_summary[i]["total_score"].as<double>());
        EXPECT_EQ(s_summary[i]["total_steps"].as<int>(), t_summary[i]["total_steps"].as<int>());
        EXPECT_EQ(s_summary[i]["same_results"].size(), t_summary[i]["same_results"].size());
    }
}

// ---- dlclose stress ----

TEST_F(MultiPlugin, RepeatedLoadAndUnload_RealAlgorithmSo_NoCrash) {
    // Loads and unloads the SAME real .so many times in one process, to
    // catch a use-after-free/resource-leak regression in the registration/
    // PluginRegistrar singleton that a single load+run wouldn't surface --
    // directly protects the invariant documented in PluginRegistrar.h.
    struct NullMap final : common::IMap3D {
        common::types::VoxelOccupancy atVoxel(const common::Position3D&) const override {
            return common::types::VoxelOccupancy::Empty;
        }
        common::types::MapConfig getMapConfig() const override { return {}; }
        bool isInBounds(const common::Position3D&) const override { return false; }
    } null_map;

    for (int i = 0; i < 50; ++i) {
        std::string error;
        auto plugin = sim::loadAlgorithmPlugin(REAL_ALGORITHM_SO_PATH, error);
        ASSERT_TRUE(plugin.has_value()) << "iteration " << i << ": " << error;

        auto instance = plugin->factory(common::MappingAlgorithmDependencies{
            common::types::MissionConfigData{}, common::types::LidarConfigData{},
            common::types::DroneConfigData{}, null_map});
        ASSERT_NE(instance, nullptr) << "iteration " << i;
        instance.reset(); // destroyed before `plugin` (and its .so handle) goes out of scope
    } // `plugin` destructs here each iteration -> dlclose, then reloaded next iteration
}
