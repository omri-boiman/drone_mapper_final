#include <drone_mapper/MapsComparison.h>

#include <algorithm>
#include <cmath>
#include <set>
#include <tuple>

namespace drone_mapper {

namespace {

using Key = std::tuple<int, int, int>;

Key posToKey(double x, double y, double z, double step) {
    return {
        static_cast<int>(std::floor(x / step)),
        static_cast<int>(std::floor(y / step)),
        static_cast<int>(std::floor(z / step)),
    };
}

double compareOne(const IMap3D& origin, const IMap3D& target) {
    const auto cfg = origin.getMapConfig();
    const double res = cfg.resolution.force_numerical_value_in(cm);
    if (res <= 0.0) return -1.0;

    const double x_min = cfg.boundaries.min_x.force_numerical_value_in(cm);
    const double x_max = cfg.boundaries.max_x.force_numerical_value_in(cm);
    const double y_min = cfg.boundaries.min_y.force_numerical_value_in(cm);
    const double y_max = cfg.boundaries.max_y.force_numerical_value_in(cm);
    const double z_min = cfg.boundaries.min_height.force_numerical_value_in(cm);
    const double z_max = cfg.boundaries.max_height.force_numerical_value_in(cm);

    std::set<Key> set_origin, set_target;

    for (double x = x_min; x < x_max; x += res) {
        for (double y = y_min; y < y_max; y += res) {
            for (double z = z_min; z < z_max; z += res) {
                const Position3D pos{
                    x * x_extent[cm],
                    y * y_extent[cm],
                    z * z_extent[cm],
                };
                const auto vo = origin.atVoxel(pos);
                const auto vt = target.atVoxel(pos);

                if (vo == types::VoxelOccupancy::Occupied)
                    set_origin.insert(posToKey(x, y, z, res));
                if (vt == types::VoxelOccupancy::Occupied)
                    set_target.insert(posToKey(x, y, z, res));
            }
        }
    }

    if (set_origin.empty() && set_target.empty()) return 100.0;
    if (set_origin.empty() || set_target.empty()) return 0.0;

    // F1 = 2*TP / (2*TP + FP + FN) * 100
    int tp = 0, fp = 0, fn = 0;
    for (const auto& k : set_target) (set_origin.count(k) ? tp : fp)++;
    for (const auto& k : set_origin) if (!set_target.count(k)) fn++;

    const int denom = 2 * tp + fp + fn;
    return (denom > 0) ? (200.0 * tp / denom) : 100.0;
}

} // namespace

std::vector<double> MapsComparison::compare(const IMap3D& origin,
                                             const std::vector<IMap3D*> targets) {
    std::vector<double> scores;
    scores.reserve(targets.size());
    for (const IMap3D* target : targets) {
        if (target == nullptr) {
            scores.push_back(-1.0);
        } else {
            scores.push_back(compareOne(origin, *target));
        }
    }
    return scores;
}

} // namespace drone_mapper
