// Standalone receive-order replay of the exact C++ estimator used by ROS.
// Build on Windows/Linux without ROS; input CSV on stdin:
// receive_ns,header_stamp_ns,C|F|R,value
// where F/R wheel values are the recorded km/h numbers.

#include "tram_odometry/estimator.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

int main(int argc, char** argv) {
  const int vehicle_id = argc > 1 ? std::atoi(argv[1]) : 30618;
  auto config = tram_odometry::EstimatorConfig::forVehicle(vehicle_id);
  if (argc > 2 && argv[2][0] != '\0') {
    config.enable_drive_table = true;
    config.drive_table_path = argv[2];
  }
  tram_odometry::Estimator estimator(config);
  std::cout << "stamp_ns,receive_ns,velocity_mps,distance_m,model_accel_mps2,"
               "front_weight,rear_weight,front_slip,rear_slip,model_only,"
               "drive_table_active,drive_table_used\n";
  std::cout << std::setprecision(17);
  std::string line;
  while (std::getline(std::cin, line)) {
    if (line.empty()) continue;
    std::istringstream fields(line);
    std::string recv_text, stamp_text, topic_text, value_text;
    if (!std::getline(fields, recv_text, ',') ||
        !std::getline(fields, stamp_text, ',') ||
        !std::getline(fields, topic_text, ',') ||
        !std::getline(fields, value_text, ',')) {
      continue;
    }
    try {
      const long long receive_ns = std::stoll(recv_text);
      const long long stamp_ns = std::stoll(stamp_text);
      const double value = std::stod(value_text);
      const double stamp_s = static_cast<double>(stamp_ns) * 1e-9;
      if (!std::isfinite(value) || !std::isfinite(stamp_s)) continue;
      if (topic_text == "F") {
        estimator.submitFrontWheel(value, stamp_s);
      } else if (topic_text == "R") {
        estimator.submitRearWheel(value, stamp_s);
      } else if (topic_text == "C") {
        estimator.submitDriverPosition(static_cast<int>(std::lrint(value)),
                                       stamp_s);
        const auto state = estimator.state();
        if (!state.initialized || !std::isfinite(state.stamp_s) ||
            std::abs(state.stamp_s - stamp_s) > 1e-4) {
          continue;
        }
        std::cout << stamp_ns << ',' << receive_ns << ','
                  << state.velocity_mps << ',' << state.distance_m << ','
                  << state.model_acceleration_mps2 << ','
                  << state.front_weight << ',' << state.rear_weight << ','
                  << state.front_slip << ',' << state.rear_slip << ','
                  << state.model_only << ',' << state.drive_table_active << ','
                  << state.drive_table_used << '\n';
      }
    } catch (const std::exception&) {
      continue;
    }
  }
  return 0;
}
