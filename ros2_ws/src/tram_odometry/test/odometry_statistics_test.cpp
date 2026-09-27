#include "tram_odometry/navigation.hpp"
#include "tram_odometry/uncertainty.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>

namespace {
void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}
bool near(double a, double b) { return std::abs(a - b) < 1e-8; }
}

int main() {
  using namespace tram_odometry;
  NavigationOutput output;
  output.estimate.velocity_mps = 5;
  output.estimate.velocity_variance = 0.04;
  output.estimate.distance_m = 5100;
  output.estimate.distance_variance = 0.25;
  output.yaw = std::acos(-1.) / 4;
  output.mapped = true;
  output.anchor_source = "master";
  const auto stats = odometryStatistics(output, 2, 100, 0.01);
  require(near(stats.velocity_mps, 10), "published twist must use the pose length scale");
  require(near(stats.scale_distance_variance, 2500), "drift is measured since the startup anchor in core metres");
  // A 45-degree rotation of diag(2504.25, 9), followed by a factor-two
  // length change, has eigenvalues 10017 and 36 and equal diagonals.
  require(near(stats.pose_covariance[0], 5026.5) &&
          near(stats.pose_covariance[7], 5026.5) &&
          near(stats.pose_covariance[1], 4990.5) &&
          near(stats.pose_covariance[6], 4990.5),
          "pose covariance must rotate to the published frame and scale quadratically");
  require(near(stats.pose_covariance[14], 64), "vertical position variance must use the length scale");
  require(near(stats.twist_covariance[0], 0.16) &&
          near(stats.twist_covariance[7], 4000) &&
          near(stats.twist_covariance[14], 4000),
          "all linear twist variances must use the length scale");
  require(near(stats.pose_covariance[21], 1000) &&
          near(stats.pose_covariance[28], 1000) && near(stats.pose_covariance[35], 0.1) &&
          near(stats.twist_covariance[21], 1000) &&
          near(stats.twist_covariance[28], 1000) && near(stats.twist_covariance[35], 1000),
          "a length change must preserve angular variances");

  NavigationConfig base_config;
  base_config.use_startup_gnss = false;
  base_config.output_projection = "enu";
  base_config.relative_heading_rad = 0.31;
  NavigationConfig scaled_config = base_config;
  scaled_config.output_scale = 0.5;
  scaled_config.output_rotation_rad = -0.7;
  scaled_config.output_offset_x_m = 3;
  scaled_config.output_offset_y_m = -2;
  scaled_config.output_offset_z_m = 8;
  Navigation base(EstimatorConfig{}, base_config), scaled(EstimatorConfig{}, scaled_config);
  for (int i = 0; i < 30; ++i) {
    const double stamp = 100 + 0.1 * i;
    for (auto* nav : {&base, &scaled}) {
      nav->submitVehicle('F', 18, stamp);
      nav->submitVehicle('R', 18, stamp);
      nav->submitVehicle('C', 0, stamp + 0.001);
    }
  }
  const auto& a = base.output();
  const auto& b = scaled.output();
  require(a.estimate.velocity_mps == b.estimate.velocity_mps &&
          a.estimate.distance_m == b.estimate.distance_m,
          "publication transforms must leave estimator means unchanged");
  require(near(b.position.x, 0.5 * (std::cos(-0.7) * a.position.x - std::sin(-0.7) * a.position.y) + 3) &&
          near(b.position.y, 0.5 * (std::sin(-0.7) * a.position.x + std::cos(-0.7) * a.position.y) - 2) &&
          near(b.position.z, 0.5 * a.position.z + 8),
          "shared navigation must apply the same output transform as publication statistics");
  const auto base_stats = odometryStatistics(a, 1, base.anchorDistanceM(), 0.01);
  const auto scaled_stats = odometryStatistics(b, 0.5, scaled.anchorDistanceM(), 0.01);
  require(near(scaled_stats.velocity_mps, 0.5 * base_stats.velocity_mps),
          "shared estimator output must produce proportionally scaled twist");
  // Rotation preserves trace and determinant; the length scale multiplies
  // each planar covariance eigenvalue by 1/4, including the relative mode.
  const auto& p = base_stats.pose_covariance;
  const auto& q = scaled_stats.pose_covariance;
  require(near(q[0] + q[7], 0.25 * (p[0] + p[7])) &&
          near(q[0] * q[7] - q[1] * q[6], (p[0] * p[7] - p[1] * p[6]) / 16),
          "combined rotation and scaling must preserve covariance eigenvalue ratios");
  require(near(q[14], 25) && near(q[35], 10), "relative mode must retain its own vertical and angular uncertainty");

  const auto path = std::filesystem::temp_directory_path() / "tram_statistics_anchor_test.csv";
  { std::ofstream f(path); f << "direction,s,x,y,z\nout,0,0,0,0\nout,1000,1000,0,0\n"; }
  NavigationConfig config;
  config.map_file = path.string();
  config.output_projection = "enu";
  config.route_direction = "out";
  config.enable_gnss_corrections = false;
  Navigation anchored(EstimatorConfig{}, config);
  geo::EnuDatum datum({config.map_datum_lat_deg, config.map_datum_lon_deg, config.map_datum_alt_m});
  for (int i = 0; i <= 10; ++i) {
    anchored.submitVehicle('F', 18, 100 + i * 0.1);
    anchored.submitVehicle('R', 18, 100 + i * 0.1);
  }
  require(anchored.anchorDistanceM() == 0, "unanchored drift starts at the run origin");
  double median_distance = 0;
  for (int i = 0; i < 3; ++i) {
    const double stamp = 101.1 + 0.1 * i;
    anchored.submitVehicle('F', 18, stamp);
    anchored.submitVehicle('R', 18, stamp);
    if (i == 1) median_distance = anchored.state().distance_m;
    const auto llh = datum.fromEnu({anchored.state().distance_m, 0, 0});
    anchored.submitFix(false, llh.latitude_deg, llh.longitude_deg, llh.height_m, 2, stamp);
  }
  anchored.submitVehicle('C', 0, 101.4);
  require(anchored.output().anchored && median_distance > 5,
          "fixture must acquire an anchor after motion");
  require(near(anchored.anchorDistanceM(), median_distance),
          "publication uncertainty must use the navigation startup median distance");
  const auto anchored_stats = odometryStatistics(anchored.output(), 1, anchored.anchorDistanceM(), 0.01);
  require(anchored_stats.scale_distance_variance > 0 && anchored_stats.scale_distance_variance < 0.001,
          "pre-anchor travel must not inflate anchored uncertainty");
  anchored.submitVehicle('C', 15, 10);
  require(!anchored.output().anchored && anchored.anchorDistanceM() == 0,
          "replay reset must discard the previous uncertainty anchor");
  require(anchored.state().notch == 15,
          "replay reset must allow a controller header from the new run");
  require(odometryStatistics(anchored.output(), 1, anchored.anchorDistanceM(), 0.01).scale_distance_variance == 0,
          "a replay reset must discard accumulated common-scale drift");
  std::filesystem::remove(path);
  std::cout << "PASS: output scale, covariance, shared navigation anchor and replay reset\n";
}
