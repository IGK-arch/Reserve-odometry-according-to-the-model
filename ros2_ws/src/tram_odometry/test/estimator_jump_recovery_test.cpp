#include "tram_odometry/estimator.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {

void require(bool condition, const char* description) {
  if (!condition) {
    std::cerr << "FAIL: " << description << '\n';
    std::exit(EXIT_FAILURE);
  }
}

void wheels(tram_odometry::Estimator& estimator, double speed, double stamp,
            bool rear_first = false) {
  const auto& config = estimator.config();
  const double front = speed / (config.wheel_scale * config.front_scale);
  const double rear = speed / (config.wheel_scale * config.rear_scale);
  if (rear_first) {
    estimator.submitRearWheel(rear, stamp);
    estimator.submitFrontWheel(front, stamp);
  } else {
    estimator.submitFrontWheel(front, stamp);
    estimator.submitRearWheel(rear, stamp);
  }
}

}  // namespace

int main() {
  using tram_odometry::Estimator;
  using tram_odometry::EstimatorConfig;
  // A moderate exceedance of the conservative acceleration prior is not
  // enough evidence for persistent rejection. Unmodelled emergency braking
  // can exceed that prior even when controller messages indicate neutral.
  Estimator emergency;
  emergency.reset(0.0, 12.0);
  wheels(emergency, 12.0, 0.0);
  for (int i = 1; i <= 20; ++i) {
    const double t = 0.1 * i;
    emergency.submitDriverPosition(0, t);
    wheels(emergency, 12.0 - 5.0 * t, t);
    if (i >= 7) {
      require(std::abs(emergency.state().velocity_mps - (12.0 - 5.0 * t)) < 3.0,
              "hard braking cannot remain latched to the neutral force model");
    }
  }

  // Tests use SI motion and invert each vehicle's calibrated input scaling.
  for (int vehicle : {30618, 30639}) {
    for (bool rear_first : {false, true}) {
      for (double faulty_speed : {1.0, 9.0}) {
        Estimator estimator(EstimatorConfig::forVehicle(vehicle));
        estimator.reset(0.0, 5.0);
        wheels(estimator, 5.0, 0.0, rear_first);
        const double bias_before = estimator.state().model_bias_mps2;
        // A sustained +/-4 m/s step is independently impossible at 10 Hz.
        // Repeating it must not turn two suspect channels into corroboration.
        for (int i = 1; i <= 30; ++i) {
          const double t = 0.1 * i;
          estimator.submitDriverPosition(0, t);
          wheels(estimator, faulty_speed, t, rear_first);
          const auto state = estimator.state();
          require(state.model_only && state.front_slip && state.rear_slip,
                  "sustained common jump remains rejected after slope window");
          require(std::abs(state.velocity_mps - 5.0) < 0.15,
                  "persistent common jump cannot corrupt model prediction");
          require(state.model_bias_mps2 == bias_before,
                  "rejected common jump cannot train the adaptive model");
        }
        // Correct readings return while the short slope window is long gone.
        wheels(estimator, 5.0, 3.1, rear_first);
        require(!estimator.state().model_only &&
                    !estimator.state().front_slip &&
                    !estimator.state().rear_slip,
                "return near independently predicted speed restores trust");
        require(std::abs(estimator.state().velocity_mps - 5.0) < 0.05,
                "correct pair quickly recovers true speed");

        // A fresh trusted peer is continuing independent evidence; time alone
        // cannot free the still incorrect front channel.
        for (int i = 32; i <= 180; ++i) {
          const double t = 0.1 * i;
          estimator.submitDriverPosition(0, t);
          estimator.submitFrontWheel(
              faulty_speed / (estimator.config().wheel_scale *
                              estimator.config().front_scale), t);
          estimator.submitRearWheel(
              5.0 / (estimator.config().wheel_scale *
                     estimator.config().rear_scale), t);
          require(estimator.state().front_slip &&
                      estimator.state().rear_weight > 0.0 &&
                      std::abs(estimator.state().velocity_mps - 5.0) < 0.1,
                  "healthy peer preserves rejection of sustained lone jump");
        }
        estimator.reset(20.0, 9.0);
        wheels(estimator, 9.0, 20.0, rear_first);
        require(!estimator.state().front_slip &&
                    !estimator.state().rear_slip &&
                    std::abs(estimator.state().velocity_mps - 9.0) < 1.0e-9,
                "nonzero reset clears independent jump evidence");
      }

      // With no independent speed input, old rejection evidence eventually
      // becomes inconclusive under acceleration-model uncertainty. The same
      // samples could now be a corrected sensor or a sustained shared fault.
      Estimator uncertain(EstimatorConfig::forVehicle(vehicle));
      uncertain.reset(0.0, 5.0);
      wheels(uncertain, 5.0, 0.0, rear_first);
      for (int i = 1; i <= 100; ++i) {
        const double t = 0.1 * i;
        uncertain.submitDriverPosition(0, t);
        wheels(uncertain, 9.0, t, rear_first);
      }
      require(!uncertain.state().model_only &&
                  std::abs(uncertain.state().velocity_mps - 9.0) < 0.1,
              "growing model uncertainty prevents permanent pair rejection");
      require(uncertain.state().front_tentative &&
                  uncertain.state().rear_tentative,
              "uncertainty-only pair acceptance stays visible as tentative");

      // Tentative acceptance must not turn an unresolved common fault into
      // a fresh independent anchor against the correct inverse transition.
      for (int i = 101; i <= 108; ++i) {
        const double t = 0.1 * i;
        uncertain.submitDriverPosition(0, t);
        wheels(uncertain, 5.0, t, rear_first);
      }
      require(!uncertain.state().model_only &&
                  std::abs(uncertain.state().velocity_mps - 5.0) < 0.1,
              "correct return after tentative acceptance recovers promptly");

      for (int i = 109; i <= 400; ++i) {
        const double t = 0.1 * i;
        uncertain.submitDriverPosition(0, t);
        wheels(uncertain, 5.0, t, rear_first);
      }
      require(!uncertain.state().front_tentative &&
                  !uncertain.state().rear_tentative,
              "exhausted recovery evidence retires the tentative diagnostic");
      for (int i = 401; i <= 410; ++i) {
        const double t = 0.1 * i;
        uncertain.submitDriverPosition(0, t);
        wheels(uncertain, 9.0, t, rear_first);
        require(uncertain.state().model_only,
                "exhausted tentative history does not disable future jump protection");
      }

      Estimator dropout(EstimatorConfig::forVehicle(vehicle));
      dropout.reset(0.0, 5.0);
      wheels(dropout, 5.0, 0.0, rear_first);
      wheels(dropout, 9.0, 0.1, rear_first);
      for (int i = 2; i <= 80; ++i) {
        dropout.submitDriverPosition(0, 0.1 * i);
      }
      wheels(dropout, 8.0, 8.0, rear_first);
      wheels(dropout, 8.0, 8.1, rear_first);
      require(!dropout.state().model_only &&
                  std::abs(dropout.state().velocity_mps - 8.0) < 0.1,
              "correct readings reacquire after jump followed by dropout");

      Estimator startup(EstimatorConfig::forVehicle(vehicle));
      wheels(startup, 9.0, 100.0, rear_first);
      require(!startup.state().model_only &&
                  std::abs(startup.state().velocity_mps - 9.0) < 1.0e-9,
              "first moving pair has no invented discontinuity history");

      for (double acceleration : {-2.0, 2.0}) {
        Estimator clean(EstimatorConfig::forVehicle(vehicle));
        clean.reset(0.0, 8.0);
        wheels(clean, 8.0, 0.0, rear_first);
        // Intentional model mismatch: neutral controller and physical motion
        // on an unknown grade, with 40 ms wheel receive latency.
        for (int i = 1; i <= 60; ++i) {
          const double t = 0.05 * i;
          const double wheel_stamp = t - 0.04;
          clean.submitDriverPosition(0, t);
          wheels(clean, 8.0 + acceleration * wheel_stamp,
                 wheel_stamp, rear_first);
          require(!clean.state().front_slip && !clean.state().rear_slip,
                  "plausible clean acceleration and braking stay trusted");
          require(std::abs(clean.state().velocity_mps -
                           (8.0 + acceleration * t)) < 0.15,
                  "healthy wheels tolerate a mismatched force-model prior");
        }
      }
    }
  }
  std::cout << "PASS: estimator persistent jump rejection and bounded recovery\n";
  return EXIT_SUCCESS;
}
