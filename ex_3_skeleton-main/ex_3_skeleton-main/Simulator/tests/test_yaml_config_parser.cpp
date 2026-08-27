#include <gtest/gtest.h>

#include <Simulator/YamlConfigParser.h>

#include <filesystem>
#include <fstream>

// YamlConfigParser previously had no dedicated test file -- it was only
// incidentally exercised as a helper inside test_score_report_writer.cpp.
// See ex3-test-plan.md, section 7: the drone_configs/lidar_configs rename and
// composition_file's move onto SimulationManagerReport (both called out as
// ex2->ex3 port traps in memory-ex3-status.md) have no regression test
// guarding them specifically until now.
//
// Note: parseSingleSimulation only stores `map_filename` as a path string --
// it never opens the map file itself (that happens later, inside
// SimulationRunFactoryImpl/Map3DImpl) -- so these tests use a placeholder
// map_filename and never need a real .npy fixture.

namespace sim = simulator;

namespace {

class YamlConfigParserTest : public ::testing::Test {
protected:
    void SetUp() override {
        tmp_dir_ = std::filesystem::temp_directory_path() / "ex3_yaml_config_parser_test";
        std::filesystem::remove_all(tmp_dir_);
        std::filesystem::create_directories(tmp_dir_);
    }
    void TearDown() override { std::filesystem::remove_all(tmp_dir_); }

    std::filesystem::path write(const std::string& relative, const std::string& content) {
        const auto p = tmp_dir_ / relative;
        std::filesystem::create_directories(p.parent_path());
        std::ofstream(p) << content;
        return p;
    }

    std::filesystem::path tmp_dir_;
};

} // namespace

// ---- parseDrone / parseLidar / parseMission: happy path + defaults ----

TEST_F(YamlConfigParserTest, ParseDrone_AllFieldsPresent) {
    const auto p = write("drone.yaml", R"(
drone_config:
  dimensions_cm: 20
  max_rotate_deg: 30
  max_advance_cm: 40
  max_elevate_cm: 15
)");
    const auto drone = sim::YamlConfigParser::parseDrone(p);
    EXPECT_DOUBLE_EQ(drone.radius.force_numerical_value_in(common::cm), 10.0); // dimensions/2
    EXPECT_DOUBLE_EQ(drone.max_advance.force_numerical_value_in(common::cm), 40.0);
    EXPECT_DOUBLE_EQ(drone.max_elevate.force_numerical_value_in(common::cm), 15.0);
}

TEST_F(YamlConfigParserTest, ParseDrone_MissingKeys_FallBackToDefaults) {
    const auto p = write("drone_empty.yaml", "drone_config:\n");
    common::types::DroneConfigData drone;
    ASSERT_NO_THROW(drone = sim::YamlConfigParser::parseDrone(p));
    EXPECT_DOUBLE_EQ(drone.radius.force_numerical_value_in(common::cm), 15.0); // default 30/2
}

TEST_F(YamlConfigParserTest, ParseLidar_AllFieldsPresent) {
    const auto p = write("lidar.yaml", R"(
lidar_config:
  z_min_cm: 1
  z_max_cm: 60
  d_cm: 1.5
  fov_circles: 4
)");
    const auto lidar = sim::YamlConfigParser::parseLidar(p);
    EXPECT_DOUBLE_EQ(lidar.z_min.force_numerical_value_in(common::cm), 1.0);
    EXPECT_DOUBLE_EQ(lidar.z_max.force_numerical_value_in(common::cm), 60.0);
    EXPECT_EQ(lidar.fov_circles, 4u);
}

TEST_F(YamlConfigParserTest, ParseMission_BoundariesAndDefaults) {
    const auto p = write("mission.yaml", R"(
mission_config:
  max_steps: 777
  gps_resolution_cm: 5
  output_mapping_resolution_factor: 2
  boundaries:
    x_boundary: {min_cm: 10, max_cm: 90}
    y_boundary: {min_cm: 0, max_cm: 200}
    height_boundary: {min_cm: 0, max_cm: 50}
)");
    const auto mission = sim::YamlConfigParser::parseMission(p);
    EXPECT_EQ(mission.max_steps, 777u);
    EXPECT_DOUBLE_EQ(mission.gps_resolution.force_numerical_value_in(common::cm), 5.0);
    EXPECT_DOUBLE_EQ(mission.mission_bounds.min_x.force_numerical_value_in(common::cm), 10.0);
    EXPECT_DOUBLE_EQ(mission.mission_bounds.max_x.force_numerical_value_in(common::cm), 90.0);
}

TEST_F(YamlConfigParserTest, ParseSingleSimulation_MapFilenameStoredAsIs) {
    // map_filename must NOT be resolved relative to the yaml's own directory
    // (unlike drone/lidar/mission sub-paths) -- confirmed CWD-relative
    // convention, see baseline-and-scoring-guide.md.
    const auto p = write("sim.yaml", R"(
simulation_config:
  map_filename: "some/relative/map.npy"
  map_resolution_cm: 10
  initial_drone_position: {x_cm: 5, y_cm: 6, height_cm: 7}
  initial_angle_deg: 90
  map_axes_offset: {x_offset: 0, y_offset: 0, height_offset: 0}
)");
    const auto sim_cfg = sim::YamlConfigParser::parseSingleSimulation(p);
    EXPECT_EQ(sim_cfg.map_filename, std::filesystem::path("some/relative/map.npy"));
    EXPECT_DOUBLE_EQ(sim_cfg.initial_drone_position.x.force_numerical_value_in(common::cm), 5.0);
    EXPECT_DOUBLE_EQ(sim_cfg.initial_angle.force_numerical_value_in(common::deg), 90.0);
}

// ---- parseCompositionWithPaths ----

namespace {

// Writes a minimal, fully self-consistent composition (1 drone, 1 lidar,
// 1 simulation with 1 mission) rooted at `dir`, and returns the path to the
// top-level composition.yaml.
std::filesystem::path writeMinimalComposition(const std::filesystem::path& dir) {
    const auto drone_path = dir / "drone.yaml";
    std::ofstream(drone_path) << "drone_config:\n  dimensions_cm: 10\n";
    const auto lidar_path = dir / "lidar.yaml";
    std::ofstream(lidar_path) << "lidar_config:\n  fov_circles: 2\n";
    const auto mission_path = dir / "mission.yaml";
    std::ofstream(mission_path) <<
        "mission_config:\n"
        "  max_steps: 50\n"
        "  boundaries:\n"
        "    x_boundary: {min_cm: 0, max_cm: 100}\n"
        "    y_boundary: {min_cm: 0, max_cm: 100}\n"
        "    height_boundary: {min_cm: 0, max_cm: 100}\n";
    const auto sim_path = dir / "simulation.yaml";
    std::ofstream(sim_path) <<
        "simulation_config:\n"
        "  map_filename: \"map.npy\"\n"
        "  map_resolution_cm: 10\n"
        "  initial_drone_position: {x_cm: 5, y_cm: 5, height_cm: 5}\n"
        "  initial_angle_deg: 0\n"
        "  map_axes_offset: {x_offset: 0, y_offset: 0, height_offset: 0}\n";

    const auto composition_path = dir / "composition.yaml";
    std::ofstream(composition_path) <<
        "simulation_compositions:\n"
        "  drone_configs:\n"
        "    - drone.yaml\n"
        "  lidar_configs:\n"
        "    - lidar.yaml\n"
        "  simulations:\n"
        "    - simulation_config: simulation.yaml\n"
        "      mission_configs:\n"
        "        - mission.yaml\n";
    return composition_path;
}

} // namespace

TEST_F(YamlConfigParserTest, ParseCompositionWithPaths_HappyPath) {
    const auto composition_path = writeMinimalComposition(tmp_dir_);

    const auto result = sim::YamlConfigParser::parseCompositionWithPaths(composition_path);

    EXPECT_EQ(result.data.composition_file, composition_path);
    ASSERT_EQ(result.data.drone_configs.size(), 1u);
    ASSERT_EQ(result.data.lidar_configs.size(), 1u);
    ASSERT_EQ(result.data.simulation_mission_groups.size(), 1u);
    EXPECT_EQ(std::get<1>(result.data.simulation_mission_groups[0]).size(), 1u);

    // Sub-config paths must resolve relative to the composition file's own
    // directory, not the caller's CWD.
    ASSERT_EQ(result.drone_paths.size(), 1u);
    EXPECT_EQ(result.drone_paths[0], tmp_dir_ / "drone.yaml");
    ASSERT_EQ(result.sim_paths.size(), 1u);
    EXPECT_EQ(result.sim_paths[0], tmp_dir_ / "simulation.yaml");
    ASSERT_EQ(result.mission_paths_per_sim.size(), 1u);
    ASSERT_EQ(result.mission_paths_per_sim[0].size(), 1u);
    EXPECT_EQ(result.mission_paths_per_sim[0][0], tmp_dir_ / "mission.yaml");
}

TEST_F(YamlConfigParserTest, ParseCompositionWithPaths_MissingDroneConfigs_Throws) {
    const auto dir = tmp_dir_;
    writeMinimalComposition(dir);
    const auto composition_path = dir / "composition_no_drones.yaml";
    std::ofstream(composition_path) <<
        "simulation_compositions:\n"
        "  lidar_configs:\n"
        "    - lidar.yaml\n"
        "  simulations:\n"
        "    - simulation_config: simulation.yaml\n"
        "      mission_configs:\n"
        "        - mission.yaml\n";

    EXPECT_THROW(sim::YamlConfigParser::parseCompositionWithPaths(composition_path),
                std::runtime_error);
}

TEST_F(YamlConfigParserTest, ParseCompositionWithPaths_MissingLidarConfigs_Throws) {
    const auto dir = tmp_dir_;
    writeMinimalComposition(dir);
    const auto composition_path = dir / "composition_no_lidars.yaml";
    std::ofstream(composition_path) <<
        "simulation_compositions:\n"
        "  drone_configs:\n"
        "    - drone.yaml\n"
        "  simulations:\n"
        "    - simulation_config: simulation.yaml\n"
        "      mission_configs:\n"
        "        - mission.yaml\n";

    EXPECT_THROW(sim::YamlConfigParser::parseCompositionWithPaths(composition_path),
                std::runtime_error);
}

TEST_F(YamlConfigParserTest, ParseCompositionWithPaths_ReferencedFileMissing_Throws) {
    const auto dir = tmp_dir_;
    writeMinimalComposition(dir);
    const auto composition_path = dir / "composition_bad_ref.yaml";
    std::ofstream(composition_path) <<
        "simulation_compositions:\n"
        "  drone_configs:\n"
        "    - does_not_exist.yaml\n"
        "  lidar_configs:\n"
        "    - lidar.yaml\n"
        "  simulations:\n"
        "    - simulation_config: simulation.yaml\n"
        "      mission_configs:\n"
        "        - mission.yaml\n";

    EXPECT_THROW(sim::YamlConfigParser::parseCompositionWithPaths(composition_path),
                std::exception);
}

TEST_F(YamlConfigParserTest, ParseCompositionWithPaths_NonExistentFile_Throws) {
    EXPECT_THROW(sim::YamlConfigParser::parseCompositionWithPaths(tmp_dir_ / "nope.yaml"),
                std::exception);
}

// ---- parseComparisonConfig ----

TEST_F(YamlConfigParserTest, ParseComparisonConfig_OriginAndTarget) {
    const auto p = write("comparison.yaml", R"(
comparison_config:
  original:
    map_res_cm: 5
    map_offset: {x_offset: 1, y_offset: 2, height_offset: 3}
  target:
    map_res_cm: 10
    map_offset: {x_offset: 0, y_offset: 0, height_offset: 0}
)");
    const auto [origin, target] = sim::YamlConfigParser::parseComparisonConfig(p);
    EXPECT_DOUBLE_EQ(origin.map_config.resolution.force_numerical_value_in(common::cm), 5.0);
    EXPECT_DOUBLE_EQ(target.map_config.resolution.force_numerical_value_in(common::cm), 10.0);
}
