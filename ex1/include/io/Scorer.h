#pragma once

#include <vector>
#include "io/MapIO.h"

namespace drone {

// Computes F1 score (0-100) comparing mapped occupied cells against ground truth.
// F1 = 2*TP / (2*TP + FP + FN) * 100
// TP: cells mapped as Occupied that are in ground truth
// FP: cells mapped as Occupied that are NOT in ground truth
// FN: ground truth cells NOT mapped as Occupied
double ComputeF1Score(const std::vector<MapCell>& mapped,
                      const std::vector<MapCell>& groundTruth,
                      double resXYCm = 1.0,
                      double resHCm  = 1.0);

} // namespace drone
