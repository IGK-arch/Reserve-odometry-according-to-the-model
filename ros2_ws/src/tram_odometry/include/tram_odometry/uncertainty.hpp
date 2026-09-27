#pragma once

#include <algorithm>
#include <array>
#include <cmath>

#include "tram_odometry/navigation.hpp"

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

struct OdometryStatistics {
  double velocity_mps = 0.0;
  double scale_distance_variance = 0.0;
  std::array<double, 36> pose_covariance{};
  std::array<double, 36> twist_covariance{};
};

// Pose/yaw already contain the configured output transform. Twist is expressed
// in the child frame; a length scale affects linear means and variances only.
inline OdometryStatistics odometryStatistics(const NavigationOutput& output,
                                             double output_scale,
                                             double anchor_distance_m,
                                             double wheel_common_scale_sigma) {
  const auto& state = output.estimate;
  OdometryStatistics result;
  result.velocity_mps = output_scale * state.velocity_mps;
  result.scale_distance_variance = commonScaleDistanceVariance(
    state.distance_m, anchor_distance_m, wheel_common_scale_sigma);
  const double along_var = std::max(0.01, state.distance_variance) +
    result.scale_distance_variance +
    (output.anchor_source == "master" ? 4.0 :
     output.anchor_source == "rover_fallback" ? 144.0 : 25.0);
  const double cross_var = output.mapped ? 9.0 : 100.0;
  const double scale_sq = output_scale * output_scale;
  const double c = std::cos(output.yaw), s = std::sin(output.yaw);
  auto& pose = result.pose_covariance;
  pose[0] = scale_sq * (along_var * c * c + cross_var * s * s);
  pose[1] = scale_sq * (along_var - cross_var) * c * s;
  pose[6] = pose[1];
  pose[7] = scale_sq * (along_var * s * s + cross_var * c * c);
  pose[14] = scale_sq * (output.mapped ? 16.0 : 100.0);
  pose[21] = 1e3;
  pose[28] = 1e3;
  pose[35] = output.mapped ? 0.1 : 10.0;
  auto& twist = result.twist_covariance;
  twist[0] = scale_sq * std::max(0.0001, state.velocity_variance);
  twist[7] = scale_sq * 1e3;
  twist[14] = scale_sq * 1e3;
  twist[21] = 1e3;
  twist[28] = 1e3;
  twist[35] = 1e3;
  return result;
}

}  // namespace tram_odometry
