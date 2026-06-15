#pragma once

#include <drone_mapper/IMappingAlgorithm.h>

#include <deque>
#include <map>
#include <set>
#include <utility>
#include <vector>

namespace drone_mapper {

class MappingAlgorithmImpl final : public IMappingAlgorithm {
public:
    using IMappingAlgorithm::IMappingAlgorithm;
    [[nodiscard]] types::MappingStepCommand nextStep(const types::DroneState& state,
                                                     const types::LidarScanResult* latest_scan) override;

private:
    struct GridCell2D {
        int x = 0, y = 0;
        bool operator==(const GridCell2D& o) const noexcept { return x == o.x && y == o.y; }
        bool operator<(const GridCell2D& o) const noexcept {
            return x != o.x ? x < o.x : y < o.y;
        }
    };

    using Cell3D = std::pair<GridCell2D, int>;

    // Populated on first nextStep() call from _drone_config and _output_map
    double nav_step_cm_    = 50.0;
    double max_rotate_deg_ = 45.0;
    double max_elevate_cm_ = 40.0;
    double min_x_cm_ = 0.0, max_x_cm_ = 500.0;
    double min_y_cm_ = 0.0, max_y_cm_ = 500.0;
    double min_height_cm_ = 0.0, max_height_cm_ = 300.0;

    // Algorithm state
    bool initialized_ = false;
    bool done_        = false;
    bool needs_scan_  = true;

    std::set<Cell3D> visited_3d_;
    std::set<Cell3D> frontier_3d_;

    std::map<std::pair<GridCell2D, int>, bool> obstacle_cache_;
    std::set<std::pair<int,int>> fine_wall_xy_;
    std::deque<types::MovementCommand> pending_commands_;

    std::vector<double> height_levels_cm_;
    std::size_t         height_level_index_ = 0;
    int pending_level_idx_ = -1;

    // Helpers
    void initialize(const types::DroneState& state);
    void applyFiltered(const std::vector<types::MappedVoxel>& voxels);
    GridCell2D worldToGrid(const Position3D& pos) const;
    Position3D gridToWorld(const GridCell2D& cell, double height_cm) const;
    bool isInBounds(const GridCell2D& cell) const;

    bool isWalkable(const GridCell2D& cell, int level_idx) const;
    bool hasClearance(const GridCell2D& from, const GridCell2D& to,
                      int level_idx, double height_cm) const;
    Cell3D findNearest3DFrontier(const GridCell2D& from, int from_level) const;
    void   expandFrontier(const GridCell2D& cell, int level_idx, double height_cm);
    std::vector<GridCell2D> findPath(const GridCell2D& from, const GridCell2D& to,
                                     int level_idx, double height_cm);

    void enqueueNavigationTo(const GridCell2D& from, const GridCell2D& to,
                              int level_idx, double height_cm, double& sim_heading_deg);
    void enqueueRotateToAngle(double target_deg, double& current_deg);
};

} // namespace drone_mapper
