#include <gtest/gtest.h>

#include <sys/wait.h>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <optional>
#include <string>

// Black-box tests of the REAL, built simulator_<ids> executable: spawns it
// as a subprocess with a real argv and asserts on exit code + combined
// stdout/stderr text + resulting files. Everything about main.cpp's actual
// argv-to-exit-code/output behavior had previously only ever been checked by
// hand (see ex3-test-plan.md, section 6) -- the unit-level tests in
// test_cli_args.cpp exercise CliArgs::parseAndValidateArgs directly, but
// never through the real compiled binary, so a drift between the two (e.g.
// a main.cpp change that stops calling parseAndValidateArgs correctly) would
// not be caught by those tests alone.

namespace {

struct ProcessResult {
    int exit_code = -1;
    std::string output; // combined stdout+stderr
};

ProcessResult runProcess(const std::string& command) {
    ProcessResult result;
    FILE* pipe = popen((command + " 2>&1").c_str(), "r");
    if (!pipe) {
        ADD_FAILURE() << "popen failed for: " << command;
        return result;
    }
    char buf[4096];
    while (std::fgets(buf, sizeof(buf), pipe) != nullptr) {
        result.output += buf;
    }
    const int status = pclose(pipe);
    if (WIFEXITED(status)) result.exit_code = WEXITSTATUS(status);
    return result;
}

std::string quoted(const std::filesystem::path& p) {
    return "\"" + p.string() + "\"";
}

std::filesystem::path fixturesDir() {
    return std::filesystem::path(__FILE__).parent_path() / "fixtures";
}

std::filesystem::path freshDir(const std::string& name) {
    const auto p = std::filesystem::temp_directory_path() / ("ex3_cli_e2e_" + name);
    std::filesystem::remove_all(p);
    std::filesystem::create_directories(p);
    return p;
}

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

class CliEndToEnd : public ::testing::Test {};

TEST_F(CliEndToEnd, NoArguments_NonZeroExitAndUsagePrinted) {
    const auto result = runProcess(quoted(REAL_SIMULATOR_EXE_PATH));
    EXPECT_NE(result.exit_code, 0);
    EXPECT_NE(result.output.find("usage"), std::string::npos) << result.output;
}

TEST_F(CliEndToEnd, MissingRequiredArgument_NonZeroExit) {
    const std::string cmd = quoted(REAL_SIMULATOR_EXE_PATH) + " -comparative simulation=" +
        quoted(fixturesDir() / "composition.yaml");
    const auto result = runProcess(cmd);
    EXPECT_NE(result.exit_code, 0);
}

TEST_F(CliEndToEnd, BadSimulationFile_NonZeroExit) {
    const auto mc_folder = freshDir("bad_sim_file_mc");
    std::ofstream(mc_folder / "dummy.so") << "placeholder\n";

    const std::string cmd = quoted(REAL_SIMULATOR_EXE_PATH) +
        " -comparative simulation=" + quoted(fixturesDir() / "does_not_exist.yaml") +
        " mission_control_folder=" + quoted(mc_folder) +
        " algorithm=" + quoted(REAL_ALGORITHM_SO_PATH);
    const auto result = runProcess(cmd);
    EXPECT_NE(result.exit_code, 0);
}

TEST_F(CliEndToEnd, ValidComparativeRun_ExitZeroAndExpectedOutput) {
    const auto mc_folder = freshDir("valid_comparative_mc");
    std::filesystem::copy_file(REAL_MISSION_CONTROL_SO_PATH, mc_folder / "MissionControl.so",
                               std::filesystem::copy_options::overwrite_existing);

    const std::string cmd = quoted(REAL_SIMULATOR_EXE_PATH) +
        " -comparative simulation=" + quoted(fixturesDir() / "composition.yaml") +
        " mission_control_folder=" + quoted(mc_folder) +
        " algorithm=" + quoted(REAL_ALGORITHM_SO_PATH);
    const auto result = runProcess(cmd);

    EXPECT_EQ(result.exit_code, 0) << result.output;
    EXPECT_NE(result.output.find("Comparative run:"), std::string::npos) << result.output;
    EXPECT_TRUE(findResultsDir(mc_folder, "comparative_results_").has_value());
}

TEST_F(CliEndToEnd, ValidCompetitionRun_ExitZeroAndExpectedOutput) {
    const auto algo_folder = freshDir("valid_competition_algo");
    std::filesystem::copy_file(REAL_ALGORITHM_SO_PATH, algo_folder / "Algorithm.so",
                               std::filesystem::copy_options::overwrite_existing);

    const std::string cmd = quoted(REAL_SIMULATOR_EXE_PATH) +
        " -competition simulation=" + quoted(fixturesDir() / "composition.yaml") +
        " mission_control=" + quoted(REAL_MISSION_CONTROL_SO_PATH) +
        " algorithms_folder=" + quoted(algo_folder);
    const auto result = runProcess(cmd);

    EXPECT_EQ(result.exit_code, 0) << result.output;
    EXPECT_NE(result.output.find("Competitive run:"), std::string::npos) << result.output;
    EXPECT_TRUE(findResultsDir(algo_folder, "competition_").has_value());
}

TEST_F(CliEndToEnd, ValidRunWithVerboseFlag_StillExitsZero) {
    const auto mc_folder = freshDir("verbose_mc");
    std::filesystem::copy_file(REAL_MISSION_CONTROL_SO_PATH, mc_folder / "MissionControl.so",
                               std::filesystem::copy_options::overwrite_existing);

    const std::string cmd = quoted(REAL_SIMULATOR_EXE_PATH) +
        " -comparative simulation=" + quoted(fixturesDir() / "composition.yaml") +
        " mission_control_folder=" + quoted(mc_folder) +
        " algorithm=" + quoted(REAL_ALGORITHM_SO_PATH) + " -verbose";
    const auto result = runProcess(cmd);
    EXPECT_EQ(result.exit_code, 0) << result.output;
}
