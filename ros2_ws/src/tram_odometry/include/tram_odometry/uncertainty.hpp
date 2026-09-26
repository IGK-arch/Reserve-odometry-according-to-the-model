#pragma once

#include <algorithm>

namespace tram_odometry {

// A shared wheel-scale error persists throughout a run and makes along-track
// uncertainty grow with traveled distance, even when both wheels agree.
inline double commonScaleDistanceVariance(double distance_m,
                                          double anchor_distance_m,
                                          double scale_sigma) {
  const double traveled_m = std::max(0.0, distance_m - anchor_distance_m);
  const double drift_sigma_m = scale_sigma * traveled_m;
  return drift_sigma_m * drift_sigma_m;
}

}  // namespace tram_odometry
