#include "tram_odometry/estimator.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {
void require(bool condition, const char* message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(EXIT_FAILURE);
  }
}

void wheels(tram_odometry::Estimator& estimator, double speed_mps,
            double stamp_s) {
  const auto& c = estimator.config();
  estimator.submitFrontWheel(speed_mps / (c.wheel_scale * c.front_scale),
                             stamp_s);
  estimator.submitRearWheel(speed_mps / (c.wheel_scale * c.rear_scale),
                            stamp_s);
}
}  // namespace

int main() {
  using tram_odometry::Estimator;
  using tram_odometry::EstimatorConfig;
  for (int vehicle : {30618, 30639}) {
    // Repeated equal readings during a real constant-speed cruise contain no
    // evidence of a fault, so the pair must remain usable.
    Estimator cruise(EstimatorConfig::forVehicle(vehicle));
    cruise.reset(0.0, 5.0);
    for (int i = 0; i <= 120; ++i) {
      const double t = i * 0.1;
      cruise.submitDriverPosition(4, t);
      wheels(cruise, 5.0, t);
    }
    require(!cruise.state().model_only,
            "constant speed cannot trigger a shared freeze alarm");
    require(!cruise.state().pair_freeze_active &&
                cruise.state().pair_freeze_distance_correction_total_m == 0.0,
            "cruise cannot receive a false retrospective distance correction");

    Estimator accelerating(EstimatorConfig::forVehicle(vehicle));
    accelerating.reset(0.0, 5.0);
    wheels(accelerating, 5.0, 0.0);
    for (int i = 1; i <= 50; ++i) {
      const double t = i * 0.1;
      accelerating.submitDriverPosition(4, t);
      wheels(accelerating, 5.0 + 0.5 * t, t);
    }
    for (int i = 51; i <= 130; ++i) {
      const double t = i * 0.1;
      accelerating.submitDriverPosition(4, t);
      wheels(accelerating, 7.5, t);
    }
    const auto during = accelerating.state();
    require(during.model_only && during.front_slip && during.rear_slip &&
                during.pair_freeze_active,
            "shared exact flatline under traction must be quarantined");
    require(during.velocity_mps > 8.5,
            "model prediction must advance while both wheels are frozen");

    // Both channels resume with a common, physically plausible speed. The
    // return is a large raw step, but it should not be misread as a new slip.
    accelerating.submitDriverPosition(4, 13.1);
    wheels(accelerating, 11.55, 13.1);
    accelerating.submitDriverPosition(4, 13.2);
    wheels(accelerating, 11.6, 13.2);
    const auto recovered = accelerating.state();
    require(!recovered.model_only && !recovered.front_slip &&
                !recovered.rear_slip,
            "both resumed wheel streams must be reacquired");
    require(std::abs(recovered.velocity_mps - 11.6) < 0.2,
            "paired recovery must quickly restore speed");
    require(!recovered.pair_freeze_active &&
                recovered.pair_freeze_distance_correction_total_m > 1.0 &&
                recovered.pair_freeze_distance_correction_total_m <= 5.0,
            "endpoint evidence must yield a bounded distance correction");
    require(recovered.pair_freeze_distance_correction_pending_m > 0.5 &&
                recovered.distance_m > during.distance_m &&
                recovered.distance_m - during.distance_m < 3.0,
            "distance correction must be applied gradually after recovery");
  }
  std::cout << "PASS: paired flatline detection and reacquisition\n";
  return EXIT_SUCCESS;
}
