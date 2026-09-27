#include "tram_odometry/estimator.hpp"
#include <cmath>
#include <iostream>
#include <string>

int main(int argc, char** argv) {
  int failures = 0;
  for (int vehicle : {30618, 30639}) {
    for (bool reverse : {false, true}) {
      for (bool adaptation : {false, true}) {
        for (double stationary_noise : {0.01, 0.04, 0.054}) {
          auto config = tram_odometry::EstimatorConfig::forVehicle(vehicle);
          config.enable_adaptation = adaptation;
          if (vehicle == 30618 && argc > 1) {
            config.enable_drive_table = true;
            config.drive_table_path = argv[1];
          }
          tram_odometry::Estimator estimator(config);
          estimator.reset(100.0, 0.0);
          for (int i = 1; i <= 100; ++i) {
            const double t = 100.0 + i * 0.1;
            estimator.submitDriverPosition(0, t);
            const double f = stationary_noise / (config.wheel_scale * config.front_scale);
            const double r = stationary_noise / (config.wheel_scale * config.rear_scale);
            if (reverse) {
              estimator.submitRearWheel(r, t);
              estimator.submitFrontWheel(f, t);
            } else {
              estimator.submitFrontWheel(f, t);
              estimator.submitRearWheel(r, t);
            }
          }
          const double stopped_distance = estimator.state().distance_m;
          bool stayed_stopped = estimator.state().velocity_mps == 0.0;
          for (int i = 1; i <= 100; ++i) {
            estimator.submitDriverPosition(0, 110.0 + i * 0.1);
            stayed_stopped &= estimator.state().velocity_mps < 0.005 &&
                std::abs(estimator.state().distance_m - stopped_distance) < 0.005;
          }
          if (!stayed_stopped) {
            std::cerr << "FAIL: authoritative standstill must erase hidden acceleration; vehicle="
                      << vehicle << " reverse=" << reverse << " adaptation=" << adaptation
                      << " noise=" << stationary_noise << " final_v="
                      << estimator.state().velocity_mps << " extra_distance="
                      << estimator.state().distance_m - stopped_distance << '\n';
            ++failures;
          }
        }
      }
    }
  }
  if (failures) return 1;
  std::cout << "PASS: dual-wheel standstill preserves zero speed/distance through dropout\n";
}
