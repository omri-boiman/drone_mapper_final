#pragma once

#include <drone_mapper/IMappingAlgorithm.h>

#include <deque>
#include <functional>
#include <unordered_map>
#include <unordered_set>
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
    struct GridCell2DHash {
        std::size_t operator()(const GridCell2D& c) const noexcept {
            return std::hash<long long>{}((static_cast<long long>(c.x) << 32) |
                                         static_cast<unsigned int>(c.y));
        }
    };

    using Cell3D     = types::GridCell3D;
    using Cell3DHash = std::hash<types::GridCell3D>;

    struct PairHash {
        std::size_t operator()(const std::pair<int,int>& p) const noexcept {
            return std::hash<long long>{}((static_cast<long long>(p.first) << 32) |
                                         static_cast<unsigned int>(p.second));
        }
    };

    // Populated on first nextStep() call from drone_config_ and output_map_
    PhysicalLength  nav_step_    = 50.0 * cm;
    HorizontalAngle max_rotate_  = 45.0 * horizontal_angle[deg];
    PhysicalLength  max_elevate_ = 40.0 * cm;
    XLength min_x_ = 0.0 * x_extent[cm];
    XLength max_x_ = 500.0 * x_extent[cm];
    YLength min_y_ = 0.0 * y_extent[cm];
    YLength max_y_ = 500.0 * y_extent[cm];
    ZLength min_height_ = 0.0 * z_extent[cm];
    ZLength max_height_ = 300.0 * z_extent[cm];

    // Algorithm state
    bool initialized_ = false;
    bool done_        = false;
    bool needs_scan_  = true;

    std::unordered_set<Cell3D, Cell3DHash> visited_3d_;
    std::unordered_set<Cell3D, Cell3DHash> frontier_3d_;

    std::unordered_map<Cell3D, bool, Cell3DHash> obstacle_cache_;
    std::unordered_set<std::pair<int,int>, PairHash> fine_wall_xy_;
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
