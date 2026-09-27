#include "tram_odometry/estimator.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

namespace {
int failures = 0;
void require(bool condition, const char* message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    ++failures;
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
void wheel(tram_odometry::Estimator& estimator, double speed_mps,
           double stamp_s, bool front) {
  const auto& c = estimator.config();
  if (front) estimator.submitFrontWheel(speed_mps / (c.wheel_scale * c.front_scale), stamp_s);
  else estimator.submitRearWheel(speed_mps / (c.wheel_scale * c.rear_scale), stamp_s);
}

void freeze(tram_odometry::Estimator& estimator, int hz = 10, bool reverse = false) {
  estimator.reset(0.0, 5.0);
  for (int i = 0; i <= 13 * hz; ++i) {
    const double t = double(i) / hz;
    estimator.submitDriverPosition(4, t);
    const double v = t <= 5.0 ? 5.0 + 0.5 * t : 7.5;
    wheel(estimator, v, t, !reverse);
    wheel(estimator, v, t, reverse);
  }
  require(estimator.state().pair_freeze_active, "setup must detect the paired freeze at every sample rate");
}

void recoveryRegressions(tram_odometry::EstimatorConfig config) {
  using tram_odometry::Estimator;
  for (bool front_first : {true, false}) {
    Estimator lone(config);
    freeze(lone);
    for (int i = 131; i <= 250; ++i) {
      const double t = i * 0.1;
      lone.submitDriverPosition(4, t);
      wheel(lone, 8.0, t, front_first);
    }
    require(!lone.state().model_only && std::abs(lone.state().velocity_mps - 8.0) < 0.1,
            "a lone resumed sensor must recover through its independent gate");
    require(lone.state().pair_freeze_distance_correction_total_m == 0.0,
            "single sensor recovery cannot justify retrospective correction");
    wheel(lone, 7.5, 25.0, !front_first);
    require(front_first ? lone.state().rear_slip : lone.state().front_slip,
            "a still-frozen companion must remain quarantined after lone recovery");
    for (int i = 251; i <= 265; ++i) {
      const double t = i * 0.1;
      lone.submitDriverPosition(4, t);
      wheel(lone, 8.0, t, front_first);
      wheel(lone, 8.0, t, !front_first);
    }
    require(!lone.state().front_slip && !lone.state().rear_slip &&
                lone.state().pair_freeze_distance_correction_total_m == 0.0,
            "the second channel can later recover without a paired backfill");

    Estimator fresh_frozen_peer(config);
    freeze(fresh_frozen_peer);
    for (int i = 131; i <= 250; ++i) {
      const double t = i * 0.1;
      fresh_frozen_peer.submitDriverPosition(4, t);
      wheel(fresh_frozen_peer, 8.0, t, front_first);
      wheel(fresh_frozen_peer, 7.5, t, !front_first);
      if (i == 132) {
        require(fresh_frozen_peer.state().pair_freeze_active,
                "single changed channel must allow a bounded paired recovery grace");
      }
      if (i == 140) {
        require(!fresh_frozen_peer.state().pair_freeze_active,
                "fresh frozen companion cannot extend paired recovery grace indefinitely");
      }
    }
    require(!fresh_frozen_peer.state().model_only &&
                std::abs(fresh_frozen_peer.state().velocity_mps - 8.0) < 0.1,
            "lone recovery must work when companion still publishes frozen payloads");
    require(front_first ? fresh_frozen_peer.state().rear_slip :
                          fresh_frozen_peer.state().front_slip,
            "fresh frozen companion remains quarantined after grace expires");
    require(fresh_frozen_peer.state().pair_freeze_distance_correction_total_m == 0.0,
            "lone recovery with a fresh frozen peer cannot backfill distance");

    Estimator duplicate(config);
    freeze(duplicate);
    wheel(duplicate, 11.6, 13.0, front_first);
    wheel(duplicate, 11.6, 13.0, !front_first);
    require(duplicate.state().pair_freeze_active && duplicate.state().model_only &&
                duplicate.state().pair_freeze_distance_correction_total_m == 0.0,
            "changed payloads on duplicate sensor headers cannot form a recovery pair");
    wheel(duplicate, 11.6, 13.1, front_first);
    require(duplicate.state().pair_freeze_active,
            "first valid recovery sample must wait for its coherent companion");
    wheel(duplicate, 11.6, 13.1, !front_first);
    require(!duplicate.state().pair_freeze_active &&
                duplicate.state().pair_freeze_distance_correction_total_m > 0.0,
            "genuine ordered samples can recover after rejected duplicate headers");

    Estimator stale(config);
    freeze(stale);
    wheel(stale, 11.5, 13.1, front_first);
    for (int i = 132; i <= 180; ++i) stale.submitDriverPosition(4, i * 0.1);
    wheel(stale, 11.5, 18.0, !front_first);
    require(stale.state().pair_freeze_distance_correction_total_m == 0.0,
            "a stale companion cannot justify paired retrospective correction");

    // Both are inside wheel_stale_s, but their sensor stamps exceed the
    // narrower pair window. Callback-effective times must not hide this.
    Estimator asynchronous(config);
    freeze(asynchronous);
    wheel(asynchronous, 11.5, 13.1, front_first);
    asynchronous.submitDriverPosition(4, 13.4);
    wheel(asynchronous, 11.5, 13.4, !front_first);
    require(asynchronous.state().pair_freeze_distance_correction_total_m == 0.0,
            "asynchronous companion cannot justify paired correction");

    Estimator reordered(config);
    freeze(reordered);
    reordered.submitDriverPosition(4, 13.3);
    wheel(reordered, 7.5, 13.3, front_first);
    // A newer callback with an older header must not overwrite the evidence
    // used to unlock both channels.
    wheel(reordered, 11.5, 13.2, front_first);
    wheel(reordered, 11.5, 13.3, !front_first);
    require(reordered.state().pair_freeze_distance_correction_total_m == 0.0,
            "out-of-order recovery must not manufacture a paired endpoint");

    Estimator changed(config);
    freeze(changed);
    changed.submitDriverPosition(0, 13.1);
    wheels(changed, 7.5, 13.1);
    changed.submitDriverPosition(4, 13.2);
    wheel(changed, 11.6, 13.2, front_first);
    wheel(changed, 11.6, 13.2, !front_first);
    require(!changed.state().pair_freeze_active && !changed.state().model_only,
            "command changes must still permit coherent wheel recovery");
    require(changed.state().pair_freeze_distance_correction_total_m == 0.0,
            "+4 to 0 to +4 invalidates constant-demand correction");
    changed.reset(0.0, 5.0);
    require(!changed.state().pair_freeze_active &&
                changed.state().pair_freeze_distance_correction_pending_m == 0.0,
            "reset clears freeze and correction state");
    freeze(changed);
    wheels(changed, 11.6, 13.2);
    require(changed.state().pair_freeze_distance_correction_total_m > 0.0,
            "reset clears the command-change latch for a later valid interval");
  }
  for (int hz : {10, 20, 25, 50, 100}) {
    Estimator rate(config);
    freeze(rate, hz, hz == 25);
  }
  Estimator plateau(config);
  plateau.reset(0.0, 5.0);
  for (int i = 0; i <= 120; ++i) {
    const double t = i * 0.1;
    plateau.submitDriverPosition(t < 3.0 ? 0 : 4, t);
    wheels(plateau, 5.0, t);
  }
  require(!plateau.state().model_only && !plateau.state().pair_freeze_active &&
              std::abs(plateau.state().velocity_mps - 5.0) < 0.05 &&
              std::abs(plateau.state().distance_m - 60.0) < 0.2,
          "a command change alone cannot invalidate healthy constant-speed wheels");
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
  for (int vehicle : {30618, 30639}) {
    recoveryRegressions(EstimatorConfig::forVehicle(vehicle));
  }
  auto deployed = EstimatorConfig::forVehicle(30618);
  deployed.enable_drive_table = true;
  const std::string source = __FILE__;
  deployed.drive_table_path = source.substr(0, source.find_last_of("/")) +
                             "/../assets/drive_accel_table.csv";
  require(Estimator(deployed).state().drive_table_active,
          "deployed 30618 regression must load the train-only drive table");
  recoveryRegressions(deployed);
  if (failures) return EXIT_FAILURE;
  std::cout << "PASS: paired flatline detection and reacquisition\n";
  return EXIT_SUCCESS;
}
