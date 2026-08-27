#include <gtest/gtest.h>

#include "CliArgs.h"

#include <algorithm>
#include <filesystem>
#include <fstream>

// Unit tests for Simulator/src/CliArgs.cpp -- previously zero coverage (see
// ex3-test-plan.md, section 1). parseAndValidateArgs is a pure function of
// (argv, program_name), so these tests build small on-disk fixtures (a
// throwaway file for "exists and openable" checks, a throwaway folder with a
// dummy ".so"-suffixed file for the folder checks) rather than needing any
// real, buildable .so -- CliArgs itself never dlopens anything, it only
// checks existence/extension.

namespace sim = simulator;

namespace {

class CliArgs : public ::testing::Test {
protected:
    void SetUp() override {
        tmp_dir_ = std::filesystem::temp_directory_path() / "ex3_cli_args_test";
        std::filesystem::remove_all(tmp_dir_);
        std::filesystem::create_directories(tmp_dir_);

        valid_sim_ = writeFile("sim.yaml");
        valid_sim_alt_ = writeFile("sim_alt.yaml");
        valid_file_ = writeFile("some_file.so"); // used for algorithm=/mission_control=
        missing_path_ = tmp_dir_ / "does_not_exist";

        folder_with_so_ = tmp_dir_ / "folder_with_so";
        std::filesystem::create_directories(folder_with_so_);
        writeFile("folder_with_so/plugin.so");

        empty_folder_ = tmp_dir_ / "empty_folder";
        std::filesystem::create_directories(empty_folder_);

        folder_no_so_ = tmp_dir_ / "folder_no_so";
        std::filesystem::create_directories(folder_no_so_);
        writeFile("folder_no_so/readme.txt");
    }

    void TearDown() override {
        std::filesystem::remove_all(tmp_dir_);
    }

    std::filesystem::path writeFile(const std::string& relative) {
        const auto p = tmp_dir_ / relative;
        std::ofstream(p) << "placeholder\n";
        return p;
    }

    // Valid, complete argv for -comparative, as a vector so individual
    // entries can be dropped/overridden per test case.
    std::vector<std::string> comparativeArgs() const {
        return {
            "-comparative",
            "simulation=" + valid_sim_.string(),
            "mission_control_folder=" + folder_with_so_.string(),
            "algorithm=" + valid_file_.string(),
        };
    }

    std::vector<std::string> competitionArgs() const {
        return {
            "-competition",
            "simulation=" + valid_sim_.string(),
            "mission_control=" + valid_file_.string(),
            "algorithms_folder=" + folder_with_so_.string(),
        };
    }

    std::filesystem::path tmp_dir_;
    std::filesystem::path valid_sim_, valid_sim_alt_, valid_file_, missing_path_;
    std::filesystem::path folder_with_so_, empty_folder_, folder_no_so_;
};

} // namespace

// ---- Happy paths ----

TEST_F(CliArgs, HappyPath_Comparative) {
    const auto parsed = sim::parseAndValidateArgs(comparativeArgs(), "simulator");
    ASSERT_TRUE(parsed.has_value());
    EXPECT_EQ(parsed->mode, sim::Mode::Comparative);
    EXPECT_EQ(parsed->simulation, valid_sim_);
    EXPECT_EQ(parsed->mission_control_folder, folder_with_so_);
    EXPECT_EQ(parsed->algorithm, valid_file_);
    EXPECT_FALSE(parsed->num_threads.has_value());
    EXPECT_FALSE(parsed->verbose);
}

TEST_F(CliArgs, HappyPath_Competition) {
    const auto parsed = sim::parseAndValidateArgs(competitionArgs(), "simulator");
    ASSERT_TRUE(parsed.has_value());
    EXPECT_EQ(parsed->mode, sim::Mode::Competition);
    EXPECT_EQ(parsed->simulation, valid_sim_);
    EXPECT_EQ(parsed->mission_control, valid_file_);
    EXPECT_EQ(parsed->algorithms_folder, folder_with_so_);
}

TEST_F(CliArgs, ArgumentOrderIsIrrelevant) {
    auto args = comparativeArgs();
    std::reverse(args.begin(), args.end());
    const auto parsed = sim::parseAndValidateArgs(args, "simulator");
    ASSERT_TRUE(parsed.has_value());
    EXPECT_EQ(parsed->mode, sim::Mode::Comparative);
    EXPECT_EQ(parsed->simulation, valid_sim_);
    EXPECT_EQ(parsed->mission_control_folder, folder_with_so_);
    EXPECT_EQ(parsed->algorithm, valid_file_);
}

// ---- Mode-flag errors ----

TEST_F(CliArgs, MissingModeFlag_Rejected) {
    auto args = comparativeArgs();
    args.erase(args.begin()); // drop "-comparative"
    EXPECT_FALSE(sim::parseAndValidateArgs(args, "simulator").has_value());
}

TEST_F(CliArgs, BothModeFlags_Rejected) {
    auto args = comparativeArgs();
    args.push_back("-competition");
    EXPECT_FALSE(sim::parseAndValidateArgs(args, "simulator").has_value());
}

// ---- Missing required arguments ----

TEST_F(CliArgs, MissingSimulation_Rejected) {
    auto args = comparativeArgs();
    args.erase(args.begin() + 1); // drop "simulation="
    EXPECT_FALSE(sim::parseAndValidateArgs(args, "simulator").has_value());
}

TEST_F(CliArgs, MissingMissionControlFolder_Rejected) {
    auto args = comparativeArgs();
    args.erase(args.begin() + 2); // drop "mission_control_folder="
    EXPECT_FALSE(sim::parseAndValidateArgs(args, "simulator").has_value());
}

TEST_F(CliArgs, MissingAlgorithm_Rejected) {
    auto args = comparativeArgs();
    args.erase(args.begin() + 3); // drop "algorithm="
    EXPECT_FALSE(sim::parseAndValidateArgs(args, "simulator").has_value());
}

TEST_F(CliArgs, MultipleMissingArgs_AllNamedInOneCombinedError) {
    // Drop 2 of the 3 required comparative args -- spec requires the program
    // to detail ALL missing args together, not stop at the first.
    std::vector<std::string> args = {"-comparative", "simulation=" + valid_sim_.string()};
    testing::internal::CaptureStderr();
    const auto parsed = sim::parseAndValidateArgs(args, "simulator");
    const std::string err = testing::internal::GetCapturedStderr();
    EXPECT_FALSE(parsed.has_value());
    EXPECT_NE(err.find("mission_control_folder"), std::string::npos) << err;
    EXPECT_NE(err.find("algorithm"), std::string::npos) << err;
}

// ---- Unsupported arguments ----

TEST_F(CliArgs, UnsupportedArgument_Rejected) {
    auto args = comparativeArgs();
    args.push_back("foo=bar");
    testing::internal::CaptureStderr();
    const auto parsed = sim::parseAndValidateArgs(args, "simulator");
    const std::string err = testing::internal::GetCapturedStderr();
    EXPECT_FALSE(parsed.has_value());
    EXPECT_NE(err.find("foo=bar"), std::string::npos) << err;
}

TEST_F(CliArgs, MissingAndUnsupportedArgs_BothReportedTogether) {
    // Drop "algorithm=" (missing) and add "foo=bar" (unsupported) in the same
    // invocation -- both must appear in the combined error output.
    auto args = comparativeArgs();
    args.erase(args.begin() + 3); // drop "algorithm="
    args.push_back("foo=bar");
    testing::internal::CaptureStderr();
    const auto parsed = sim::parseAndValidateArgs(args, "simulator");
    const std::string err = testing::internal::GetCapturedStderr();
    EXPECT_FALSE(parsed.has_value());
    EXPECT_NE(err.find("algorithm"), std::string::npos) << err;
    EXPECT_NE(err.find("foo=bar"), std::string::npos) << err;
}

TEST_F(CliArgs, ComparativeOnlyArgInCompetitionMode_IsUnsupported) {
    // "algorithm=" is comparative-only; supplying it alongside a fully valid
    // -competition invocation should be flagged as unsupported, not silently
    // accepted or silently ignored.
    auto args = competitionArgs();
    args.push_back("algorithm=" + valid_file_.string());
    EXPECT_FALSE(sim::parseAndValidateArgs(args, "simulator").has_value());
}

TEST_F(CliArgs, EmptyKeyArgument_IsUnsupported) {
    auto args = comparativeArgs();
    args.push_back("=nokey");
    EXPECT_FALSE(sim::parseAndValidateArgs(args, "simulator").has_value());
}

// ---- Bad file / folder arguments ----

TEST_F(CliArgs, SimulationFileMissing_Rejected) {
    auto args = comparativeArgs();
    args[1] = "simulation=" + missing_path_.string();
    EXPECT_FALSE(sim::parseAndValidateArgs(args, "simulator").has_value());
}

TEST_F(CliArgs, AlgorithmFileMissing_Rejected) {
    auto args = comparativeArgs();
    args[3] = "algorithm=" + missing_path_.string();
    EXPECT_FALSE(sim::parseAndValidateArgs(args, "simulator").has_value());
}

TEST_F(CliArgs, MissionControlFileMissing_Rejected_Competition) {
    auto args = competitionArgs();
    args[2] = "mission_control=" + missing_path_.string();
    EXPECT_FALSE(sim::parseAndValidateArgs(args, "simulator").has_value());
}

TEST_F(CliArgs, SingleFileArg_PointingAtFolder_Rejected) {
    // algorithm= must name a FILE; a folder should fail the same check a
    // missing file would.
    auto args = comparativeArgs();
    args[3] = "algorithm=" + folder_with_so_.string();
    EXPECT_FALSE(sim::parseAndValidateArgs(args, "simulator").has_value());
}

TEST_F(CliArgs, MissionControlFolder_NonExistent_Rejected) {
    auto args = comparativeArgs();
    args[2] = "mission_control_folder=" + missing_path_.string();
    EXPECT_FALSE(sim::parseAndValidateArgs(args, "simulator").has_value());
}

TEST_F(CliArgs, MissionControlFolder_Empty_Rejected) {
    auto args = comparativeArgs();
    args[2] = "mission_control_folder=" + empty_folder_.string();
    EXPECT_FALSE(sim::parseAndValidateArgs(args, "simulator").has_value());
}

TEST_F(CliArgs, MissionControlFolder_NoSoFiles_Rejected) {
    auto args = comparativeArgs();
    args[2] = "mission_control_folder=" + folder_no_so_.string();
    EXPECT_FALSE(sim::parseAndValidateArgs(args, "simulator").has_value());
}

TEST_F(CliArgs, AlgorithmsFolder_NonExistent_Rejected_Competition) {
    auto args = competitionArgs();
    args[3] = "algorithms_folder=" + missing_path_.string();
    EXPECT_FALSE(sim::parseAndValidateArgs(args, "simulator").has_value());
}

TEST_F(CliArgs, AlgorithmsFolder_Empty_Rejected_Competition) {
    auto args = competitionArgs();
    args[3] = "algorithms_folder=" + empty_folder_.string();
    EXPECT_FALSE(sim::parseAndValidateArgs(args, "simulator").has_value());
}

TEST_F(CliArgs, AlgorithmsFolder_NoSoFiles_Rejected_Competition) {
    auto args = competitionArgs();
    args[3] = "algorithms_folder=" + folder_no_so_.string();
    EXPECT_FALSE(sim::parseAndValidateArgs(args, "simulator").has_value());
}

// ---- num_threads ----

TEST_F(CliArgs, NumThreads_AbsentIsNullopt) {
    const auto parsed = sim::parseAndValidateArgs(comparativeArgs(), "simulator");
    ASSERT_TRUE(parsed.has_value());
    EXPECT_FALSE(parsed->num_threads.has_value());
}

TEST_F(CliArgs, NumThreads_One) {
    auto args = comparativeArgs();
    args.push_back("num_threads=1");
    const auto parsed = sim::parseAndValidateArgs(args, "simulator");
    ASSERT_TRUE(parsed.has_value());
    ASSERT_TRUE(parsed->num_threads.has_value());
    EXPECT_EQ(*parsed->num_threads, 1u);
}

TEST_F(CliArgs, NumThreads_Zero_IsValidNonNegative) {
    auto args = comparativeArgs();
    args.push_back("num_threads=0");
    const auto parsed = sim::parseAndValidateArgs(args, "simulator");
    ASSERT_TRUE(parsed.has_value());
    ASSERT_TRUE(parsed->num_threads.has_value());
    EXPECT_EQ(*parsed->num_threads, 0u);
}

TEST_F(CliArgs, NumThreads_LargeValue) {
    auto args = comparativeArgs();
    args.push_back("num_threads=8");
    const auto parsed = sim::parseAndValidateArgs(args, "simulator");
    ASSERT_TRUE(parsed.has_value());
    ASSERT_TRUE(parsed->num_threads.has_value());
    EXPECT_EQ(*parsed->num_threads, 8u);
}

TEST_F(CliArgs, NumThreads_Negative_Rejected) {
    auto args = comparativeArgs();
    args.push_back("num_threads=-1");
    EXPECT_FALSE(sim::parseAndValidateArgs(args, "simulator").has_value());
}

TEST_F(CliArgs, NumThreads_NonNumeric_Rejected) {
    auto args = comparativeArgs();
    args.push_back("num_threads=abc");
    EXPECT_FALSE(sim::parseAndValidateArgs(args, "simulator").has_value());
}

TEST_F(CliArgs, NumThreads_TrailingGarbage_Rejected) {
    auto args = comparativeArgs();
    args.push_back("num_threads=5abc");
    EXPECT_FALSE(sim::parseAndValidateArgs(args, "simulator").has_value());
}

// ---- -verbose ----

TEST_F(CliArgs, VerboseFlag_SetsFieldTrue) {
    auto args = comparativeArgs();
    args.push_back("-verbose");
    const auto parsed = sim::parseAndValidateArgs(args, "simulator");
    ASSERT_TRUE(parsed.has_value());
    EXPECT_TRUE(parsed->verbose);
}

TEST_F(CliArgs, VerboseFlag_AbsentDefaultsFalse) {
    const auto parsed = sim::parseAndValidateArgs(comparativeArgs(), "simulator");
    ASSERT_TRUE(parsed.has_value());
    EXPECT_FALSE(parsed->verbose);
}

// ---- Duplicate argument ----

TEST_F(CliArgs, DuplicateArgument_LastValueWins) {
    auto args = comparativeArgs();
    // args[1] is "simulation=<valid_sim_>" -- append a second, different,
    // also-valid simulation= after it. Current implementation stores kv in a
    // std::map built by a single left-to-right pass, so the LAST occurrence
    // wins; this test locks that behavior down explicitly.
    args.push_back("simulation=" + valid_sim_alt_.string());
    const auto parsed = sim::parseAndValidateArgs(args, "simulator");
    ASSERT_TRUE(parsed.has_value());
    EXPECT_EQ(parsed->simulation, valid_sim_alt_);
}
