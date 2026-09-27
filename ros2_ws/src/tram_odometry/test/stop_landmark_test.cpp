#include "tram_odometry/navigation.hpp"

#include <cassert>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <string>

using tram_odometry::Navigation;
using tram_odometry::NavigationConfig;
using tram_odometry::EstimatorConfig;

namespace {

void step(Navigation & navigation, double t, double raw_kmh, int notch) {
  navigation.submitVehicle('C', notch, t);
  navigation.submitVehicle('F', raw_kmh, t + 0.01);
  navigation.submitVehicle('R', raw_kmh, t + 0.02);
}

void trace(Navigation & navigation, int stop_notch = 0) {
  for (int i = 0; i < 120; ++i) step(navigation, 1000.0 + i * 0.1, 0.0, 0);
  assert(navigation.output().stop_corrections == 0); // startup dwell is not a landmark
  for (int i = 0; i < 480; ++i) step(navigation, 1012.0 + i * 0.1, 10.8, 0);
  for (int i = 0; i < 150; ++i) step(navigation, 1060.0 + i * 0.1, 0.0, stop_notch);
}

void traceWithMissingRear(Navigation & navigation) {
  for (int i = 0; i < 120; ++i) step(navigation, 1000.0 + i * 0.1, 0.0, 0);
  for (int i = 0; i < 480; ++i) step(navigation, 1012.0 + i * 0.1, 10.8, 0);
  for (int i = 0; i < 150; ++i) {
    const double stamp = 1060.0 + i * 0.1;
    navigation.submitVehicle('C', 0, stamp);
    navigation.submitVehicle('F', 0.0, stamp + 0.01);
    if (i < 20) navigation.submitVehicle('R', 0.0, stamp + 0.02);
  }
}

void traceWithStaleWheels(Navigation & navigation) {
  for (int i = 0; i < 120; ++i) step(navigation, 1000.0 + i * 0.1, 0.0, 0);
  for (int i = 0; i < 480; ++i) step(navigation, 1012.0 + i * 0.1, 10.8, 0);
  for (int i = 0; i < 150; ++i)
    navigation.submitVehicle('C', 0, 1060.0 + i * 0.1);
}

} // namespace

int main(int argc, char ** argv) {
  assert(argc == 2);
  const auto map = std::string(argv[1]);
  EstimatorConfig estimator = EstimatorConfig::forVehicle(30618);
  estimator.enable_drive_table = false;
  NavigationConfig config;
  config.map_file = map;
  config.use_startup_gnss = false;
  config.use_map_without_gnss = true;
  config.route_direction = "out";
  Navigation baseline(estimator, config);
  trace(baseline);
  const double distance = baseline.output().estimate.distance_m;
  assert(distance > 100.0);

  const auto name = "stop_landmark_test_" + std::to_string(
      std::chrono::steady_clock::now().time_since_epoch().count()) + ".csv";
  const auto catalog = std::filesystem::current_path() / name;
  {
    std::ofstream file(catalog);
    file << "direction,s,bag_count\n";
    file << "out," << std::fixed << distance - 5.0 << ",10\n";
  }
  config.enable_stop_landmarks = true;
  config.stop_landmarks_file = catalog.string();
  Navigation corrected(estimator, config);
  trace(corrected);
  const auto & result = corrected.output();
  assert(result.stop_corrections == 1);
  assert(result.mapped && baseline.output().mapped);
  assert(std::abs(result.estimate.distance_m - distance) < 1e-8);
  assert(std::abs(result.estimate.velocity_mps - baseline.output().estimate.velocity_mps) < 1e-8);
  const auto dx = result.position.x - baseline.output().position.x;
  const auto dy = result.position.y - baseline.output().position.y;
  assert(std::hypot(dx, dy) > 2.0);

  Navigation commanded(estimator, config);
  trace(commanded, 5);
  assert(commanded.output().stop_corrections == 0);
  Navigation missing_rear(estimator, config);
  traceWithMissingRear(missing_rear);
  assert(missing_rear.output().stop_corrections == 0);
  Navigation stale_wheels(estimator, config);
  traceWithStaleWheels(stale_wheels);
  assert(stale_wheels.output().stop_corrections == 0);
  NavigationConfig relative_config = config;
  relative_config.map_file = "missing-route-map.csv";
  Navigation relative(estimator, relative_config);
  trace(relative);
  assert(relative.output().stop_corrections == 0);
  assert(!relative.output().mapped);
  NavigationConfig other_vehicle_config = config;
  other_vehicle_config.stop_landmarks_file = "missing-stops.csv";
  Navigation other_vehicle(EstimatorConfig::forVehicle(30639), other_vehicle_config);
  trace(other_vehicle);
  assert(other_vehicle.output().stop_corrections == 0);
  std::filesystem::remove(catalog);
}
