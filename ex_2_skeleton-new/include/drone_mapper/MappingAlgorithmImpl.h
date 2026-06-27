#pragma once

#include <drone_mapper/IMappingAlgorithm.h>

#include <deque>
#include <unordered_map>
#include <vector>

namespace drone_mapper {

class MappingAlgorithmImpl final : public IMappingAlgorithm {
public:
    using IMappingAlgorithm::IMappingAlgorithm;
    [[nodiscard]] types::MappingStepCommand nextStep(const types::DroneState& state,
                                                     const types::LidarScanResult* latest_scan) override;

private:
    using Cell     = types::GridCell3D;
    using CellHash = std::hash<types::GridCell3D>;

    types::MapConfig map_cfg_;
    double           res_cm_        = 1.0;
    int              radius_voxels_ = 1;     // ceiling, loop bounds only
    double           phys_r2_       = 0.25; // (radius_cm/res_cm)^2, used for sphere test
    HorizontalAngle  max_rotate_    = 45.0 * horizontal_angle[deg];
    PhysicalLength   max_advance_   = 50.0 * cm;
    PhysicalLength   max_elevate_   = 40.0 * cm;

    double           los_L_         = 3.0; // neighborhood radius for gain/structure counts
    double           w_struct_      = 4.0; // structure weight — zero for small drones (no escape risk)
    double           beta_          = 0.15; // distance-penalty coefficient

    bool initialized_      = false;
    bool done_             = false;
    bool needs_scan_       = true;
    int  stuck_scan_count_ = 0;
    std::unordered_map<Cell, int, CellHash> frontier_try_count_;

    std::deque<types::MovementCommand> pending_commands_;
    std::deque<Cell>                   recent_targets_;  // anti-oscillation guard (max 8)

    void       initialize(const types::DroneState& state);
    Cell       worldToVoxel(const Position3D& pos) const;
    Position3D centerOf(const Cell& c) const;
    types::VoxelOccupancy stateOf(const Cell& c) const;
    bool       navigable(const Cell& c) const;           // loose: Unmapped footprint ok (for BFS/frontier)
    bool       clearForBodyKnown(const Cell& c) const;  // sphere only: rejects known obstacles, allows Unmapped face-neighbours
    bool       clearForBody(const Cell& c) const;        // strict: clearForBodyKnown + no Unmapped face-neighbours (intermediate path cells)
    bool       isFrontier(const Cell& c) const;
    Cell       selectBestFrontier(const Cell& from) const;
    std::vector<Cell> findPath3D(const Cell& from, const Cell& to) const;
    void       enqueueNavigationTo(const Cell& from, const Cell& to, double& heading_deg);
    void       enqueueRotateToAngle(double target_deg, double& current_deg);
};

} // namespace drone_mapper
