#include "tram_odometry/estimator.hpp"

#include <cmath>
#include <iostream>

namespace {
int failures = 0;
void require(bool condition, const char* message, double dt) {
  if (!condition) {
    std::cerr << "FAIL dt=" << dt << ": " << message << '\n';
    ++failures;
  }
}
void wheels(tram_odometry::Estimator& estimator, double speed, double t,
            bool rear_first) {
  if (rear_first) {
    estimator.submitRearWheel(speed * 3.6, t);
    estimator.submitFrontWheel(speed * 3.6, t);
  } else {
    estimator.submitFrontWheel(speed * 3.6, t);
    estimator.submitRearWheel(speed * 3.6, t);
  }
}
}  // namespace

int main() {
  for (double dt : {0.1, 0.05, 0.04, 0.02, 0.01}) {
    for (bool rear_first : {false, true}) {
      for (double bad_speed : {1.0, 9.0}) {
        tram_odometry::Estimator estimator;
        estimator.reset(0.0, 5.0);
        wheels(estimator, 5.0, 0.0, rear_first);
        bool all_rejected = true;
        for (int i = 1; i <= std::lround(3.0 / dt); ++i) {
          const double t = i * dt;
          estimator.submitDriverPosition(0, t);
          wheels(estimator, bad_speed, t, rear_first);
          const auto state = estimator.state();
          all_rejected &= state.model_only && state.front_slip &&
                          state.rear_slip &&
                          std::abs(state.velocity_mps - 5.0) < 0.15;
        }
        require(all_rejected, "common strong jump remains rejected at every rate", dt);
        wheels(estimator, 5.0, 3.0 + dt, rear_first);
        require(!estimator.state().model_only &&
                    std::abs(estimator.state().velocity_mps - 5.0) < 0.05,
                "inverse step to accepted history promptly restores correct pair", dt);
      }

      for (double bad_speed : {1.0, 9.0}) {
        tram_odometry::Estimator burst;
        burst.reset(10.0, 5.0);
        wheels(burst, 5.0, 10.0, rear_first);
        burst.submitDriverPosition(0, 10.2);
        wheels(burst, 5.0, 10.1, rear_first);
        // Ordered sensor samples share one effective controller frontier.
        wheels(burst, bad_speed, 10.1 + dt, rear_first);
        require(burst.state().model_only &&
                    std::abs(burst.state().velocity_mps - 5.0) < 0.1,
                "late ordered burst uses sensor time to reject raw jump", dt);
        for (int i = 1; i <= 30; ++i) {
          const double t = 10.2 + i * 0.1;
          burst.submitDriverPosition(0, t);
          wheels(burst, bad_speed, t, rear_first);
        }
        require(burst.state().model_only,
                "late burst jump remains rejected over its full episode", dt);
        wheels(burst, 5.0, 13.3, rear_first);
        require(!burst.state().model_only &&
                    std::abs(burst.state().velocity_mps - 5.0) < 0.05,
                "correct readings promptly return after late burst fault", dt);
      }

      tram_odometry::Estimator reordered;
      reordered.reset(10.0, 5.0);
      wheels(reordered, 5.0, 10.0, rear_first);
      reordered.submitDriverPosition(0, 10.2);
      wheels(reordered, 5.0, 10.19, rear_first);
      wheels(reordered, 5.0, 10.11, rear_first);
      wheels(reordered, 5.7, 10.2, rear_first);
      require(reordered.state().model_only &&
                  std::abs(reordered.state().velocity_mps - 5.0) < 0.1,
              "late older callbacks cannot move raw adjacency history backward", dt);

      for (double acceleration : {-2.0, 0.0, 2.0}) {
        tram_odometry::Estimator clean;
        clean.reset(0.0, 8.0);
        wheels(clean, 8.0, 0.0, rear_first);
        bool all_healthy = true;
        for (int i = 1; i <= std::lround(2.0 / dt); ++i) {
          const double t = i * dt;
          clean.submitDriverPosition(0, t);
          // Deterministic bounded 0.08m/s measurement noise: instantaneous
          // slopes can be high at100Hz without crossing the strong-step budget.
          const double noise = 0.08 * std::sin(i * 0.7);
          wheels(clean, 8.0 + acceleration * t + noise, t, rear_first);
          all_healthy &= !clean.state().model_only &&
                         !clean.state().front_slip &&
                         !clean.state().rear_slip &&
                         std::abs(clean.state().velocity_mps -
                                  (8.0 + acceleration * t)) < 0.12;
        }
        require(all_healthy, "plausible acceleration with bounded noise stays healthy", dt);
      }
    }
  }
  if (failures) return 1;
  std::cout << "PASS: common-jump rejection and healthy motion at10-100Hz\n";
  return 0;
}
