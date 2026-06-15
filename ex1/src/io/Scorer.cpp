#include "io/Scorer.h"

#include <cmath>
#include <set>
#include <tuple>

namespace drone {

namespace {

using Key = std::tuple<int, int, int>;

Key makeKey(const MapCell& c, double resXYCm, double resHCm)
{
    return {
        static_cast<int>(std::floor(c.x.numerical_value_in(cm) / resXYCm)),
        static_cast<int>(std::floor(c.y.numerical_value_in(cm) / resXYCm)),
        static_cast<int>(std::floor(c.z.numerical_value_in(cm) / resHCm))
    };
}

} // namespace

double ComputeF1Score(const std::vector<MapCell>& mapped,
                      const std::vector<MapCell>& groundTruth,
                      double resXYCm,
                      double resHCm)
{
    std::set<Key> groundTruthSet;
    for (const auto& c : groundTruth)
        groundTruthSet.insert(makeKey(c, resXYCm, resHCm));

    std::set<Key> mappedSet;
    for (const auto& c : mapped)
        mappedSet.insert(makeKey(c, resXYCm, resHCm));

    int tp = 0, fp = 0, fn = 0;
    for (const auto& k : mappedSet)
        (groundTruthSet.count(k) ? tp : fp)++;
    for (const auto& k : groundTruthSet)
        if (!mappedSet.count(k)) fn++;

    const int denom = 2 * tp + fp + fn;
    return (denom > 0) ? (200.0 * tp / denom) : 100.0;
}

} // namespace drone
