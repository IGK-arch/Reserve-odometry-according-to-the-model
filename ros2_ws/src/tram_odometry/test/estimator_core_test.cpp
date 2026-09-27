#include "tram_odometry/estimator.hpp"

#include <cmath>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>

namespace {

void require(bool condition, const char* description) {
  if (!condition) {
    std::cerr << "FAIL: " << description << '\n';
    std::exit(EXIT_FAILURE);
  }
}

bool near(double actual, double expected, double tolerance) {
  return std::abs(actual - expected) <= tolerance;
}

}  // namespace

int main() {
  using tram_odometry::Estimator;

  // Recorded numeric wheel speed is km/h, while the core state is m/s.
  Estimator conversion;
  conversion.submitFrontWheel(18.0, 100.0);
  conversion.submitRearWheel(18.0, 100.0);
  require(conversion.state().initialized, "wheel data initializes filter");
  require(near(conversion.state().velocity_mps, 5.0, 1.0e-9),
          "wheel speed conversion km/h to m/s");
  require(!conversion.state().model_only, "wheel measurement is trusted");

  // Receive order can lag the source header by one command cycle.
  Estimator late;
  late.submitDriverPosition(0, 100.0);
  late.submitFrontWheel(18.0, 100.0);
  late.submitDriverPosition(0, 100.10);
  late.submitRearWheel(18.0, 100.04);
  require(near(late.state().stamp_s, 100.10, 1.0e-9),
          "late wheel never moves state time backwards");
  require(!late.state().rear_stale, "slightly late wheel is still accepted");
  late.submitFrontWheel(100.0, 99.0);
  require(near(late.state().velocity_mps, 5.0, 0.2),
          "large backward header jump is ignored");
  late.submitFrontWheel(100.0, 103.0);
  require(near(late.state().stamp_s, 100.10, 1.0e-9),
          "isolated future wheel header cannot poison event time");

  // Rear bogie spin must not drag the estimated tram speed upward.
  Estimator spin;
  spin.reset(0.0, 5.0);
  spin.submitDriverPosition(10, 0.0);
  spin.submitFrontWheel(18.0, 0.0);
  spin.submitRearWheel(18.0, 0.0);
  spin.submitRearWheel(39.6, 0.10);
  require(spin.state().rear_slip, "rear acceleration spike is flagged");
  require(spin.state().rear_weight == 0.0, "rear spin is downweighted");
  require(spin.state().velocity_mps < 5.4,
          "rear spin does not create tram speed spike");
  spin.submitFrontWheel(18.18, 0.10);
  require(spin.state().front_weight > 0.0,
          "healthy front bogie remains available");
  spin.submitRearWheel(18.18, 0.20);
  require(!spin.state().rear_slip, "bogie recovers when it agrees again");

  // Braking slide makes one wheel report a speed lower than body speed.
  Estimator slide;
  slide.reset(0.0, 5.0);
  slide.submitDriverPosition(-15, 0.0);
  slide.submitFrontWheel(18.0, 0.0);
  slide.submitRearWheel(18.0, 0.0);
  slide.submitRearWheel(0.0, 0.10);
  require(slide.state().rear_slip, "locked rear wheel is flagged");
  require(slide.state().velocity_mps > 4.6,
          "locked rear wheel does not collapse body speed");

  // One sensor can disappear for much longer than the stale timeout.
  Estimator gap;
  gap.reset(0.0, 5.0);
  gap.submitFrontWheel(18.0, 0.0);
  gap.submitRearWheel(18.0, 0.0);
  for (int i = 1; i <= 50; ++i) {
    const double t = 0.1 * i;
    gap.submitDriverPosition(0, t);
    gap.submitFrontWheel(3.6 * (5.0 + 0.2 * t), t);
  }
  require(gap.state().rear_stale, "missing rear channel becomes stale");
  require(!gap.state().model_only, "front channel covers rear dropout");
  require(near(gap.state().velocity_mps, 6.0, 0.2),
          "healthy front speed is tracked during dropout");

  // Regression: a rear tachometer frozen at zero while the tram starts
  // smoothly must not quarantine the moving front channel before rear drops.
  Estimator frozen_rear;
  frozen_rear.reset(0.0);
  for (int i = 0; i <= 20; ++i) {
    const double t = 0.1 * i;
    frozen_rear.submitDriverPosition(0, t);
    frozen_rear.submitFrontWheel(0.0, t);
    frozen_rear.submitRearWheel(0.0, t);
  }
  for (int i = 1; i <= 5; ++i) {
    const double t = 2.0 + 0.1 * i;
    frozen_rear.submitDriverPosition(4, t);
    const double late_stamp = t - (0.06 + 0.02 * (i % 3));
    frozen_rear.submitFrontWheel(3.6 * 0.18 * i, late_stamp);
    frozen_rear.submitRearWheel(0.0, late_stamp);
  }
  require(frozen_rear.state().front_weight > 0.0,
          "smoothly moving front sensor survives frozen rear");
  require(frozen_rear.state().rear_weight == 0.0,
          "frozen rear sensor is quarantined");
  for (int i = 6; i <= 30; ++i) {
    const double t = 2.0 + 0.1 * i;
    frozen_rear.submitDriverPosition(4, t);
    const double late_stamp = t - (0.06 + 0.02 * (i % 3));
    frozen_rear.submitFrontWheel(3.6 * 0.18 * i, late_stamp);
  }
  require(!frozen_rear.state().model_only,
          "front continues to cover prolonged rear loss");
  require(near(frozen_rear.state().velocity_mps, 5.4, 0.35),
          "front speed is followed after rear disappears");

  // With both sensors absent, bounded physics propagates state and uncertainty.
  Estimator model_only;
  model_only.reset(0.0, 5.0);
  model_only.submitDriverPosition(15, 0.0);
  model_only.advance(2.0);
  require(model_only.state().model_only, "no wheels means model-only mode");
  require(model_only.state().velocity_mps > 5.0,
          "traction model predicts forward acceleration");
  require(model_only.state().distance_m > 9.0,
          "model-only mode integrates position");
  require(model_only.state().velocity_variance > 0.01,
          "model-only mode increases uncertainty");

  // Two simultaneously suspicious wheels must regain trust after convergence.
  Estimator recovery;
  recovery.reset(0.0, 5.0);
  recovery.submitFrontWheel(18.0, 0.0);
  recovery.submitRearWheel(18.0, 0.0);
  recovery.submitFrontWheel(0.0, 0.10);
  recovery.submitRearWheel(0.0, 0.10);
  require(recovery.state().model_only,
          "simultaneous impossible wheel jump is quarantined");
  recovery.submitFrontWheel(18.0, 0.20);
  recovery.submitRearWheel(18.0, 0.20);
  require(!recovery.state().model_only,
          "converged wheels restore sensor fusion");

  // A 5 m/s cruise over ten seconds must integrate to approximately 50 m.
  Estimator integral;
  integral.reset(0.0, 5.0);
  for (int i = 1; i <= 100; ++i) {
    const double t = 0.1 * i;
    integral.submitDriverPosition(0, t);
    integral.submitFrontWheel(18.0, t);
    integral.submitRearWheel(18.0, t);
  }
  require(near(integral.state().distance_m, 50.0, 0.5),
          "distance remains close to 50 m at 5 m/s for 10 s");

  // Healthy, agreeing wheels should correct a weak drive-model prior. The
  // driver can hold a traction notch at constant speed on a grade. Replay
  // 10 Hz wheels received 40 ms late against 20 Hz controller/output events;
  // keeping the published state at current event time must not retain an
  // excessive velocity bias from the imperfect force model.
  Estimator steady_with_model_mismatch;
  steady_with_model_mismatch.reset(0.0, 5.0);
  double steady_squared_error = 0.0;
  int steady_samples = 0;
  for (int i = 0; i <= 200; ++i) {
    const double t = 0.05 * i;
    steady_with_model_mismatch.submitDriverPosition(4, t);
    const auto published = steady_with_model_mismatch.state();
    if (i > 0 && i % 2 == 0) {
      steady_with_model_mismatch.submitFrontWheel(18.0, t - 0.04);
      steady_with_model_mismatch.submitRearWheel(18.0, t - 0.04);
    }
    if (t >= 1.0) {
      require(near(published.stamp_s, t, 1.0e-9),
              "healthy wheel correction preserves current output time");
      require(!published.front_slip && !published.rear_slip,
              "steady agreeing wheels stay trusted despite model mismatch");
      const double error = published.velocity_mps - 5.0;
      steady_squared_error += error * error;
      ++steady_samples;
    }
  }
  require(std::sqrt(steady_squared_error / steady_samples) < 0.045,
          "agreeing late wheels limit steady-speed model bias");

  // The optional train-only table changes prediction inside a supported
  // speed/notch cell and leaves unsupported speeds on the physics model.
  const auto fixture = std::filesystem::temp_directory_path() /
      ("tram_drive_table_test_" +
       std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) +
       ".csv");
  {
    std::ofstream output(fixture);
    require(static_cast<bool>(output), "drive table test fixture can be created");
    output << "vehicle,notch,speed_bin,accel,uncertainty,count\n"
           << "30618,7,3-5,1.5,0.2,100\n";
  }
  tram_odometry::EstimatorConfig table_config;
  table_config.enable_drive_table = true;
  table_config.drive_table_path = fixture.string();
  Estimator with_table(table_config);
  Estimator physics_only;
  with_table.reset(0.0, 4.0);
  physics_only.reset(0.0, 4.0);
  with_table.submitDriverPosition(7, 0.0);
  physics_only.submitDriverPosition(7, 0.0);
  with_table.advance(0.2);
  physics_only.advance(0.2);
  require(with_table.state().drive_table_active,
          "valid train-only drive table loads");
  require(with_table.state().drive_table_used,
          "supported notch and speed activate drive residual");
  require(with_table.state().velocity_mps > physics_only.state().velocity_mps + 0.005,
          "drive table changes short model prediction");
  with_table.reset(0.0, 0.5);
  physics_only.reset(0.0, 0.5);
  with_table.submitDriverPosition(7, 0.0);
  physics_only.submitDriverPosition(7, 0.0);
  with_table.advance(0.2);
  physics_only.advance(0.2);
  require(!with_table.state().drive_table_used,
          "unsupported speed does not extrapolate a table row");
  require(near(with_table.state().velocity_mps,
               physics_only.state().velocity_mps, 1.0e-12),
          "unsupported speed follows unchanged physics");
  std::filesystem::remove(fixture);

  Estimator stop;
  stop.reset(0.0, 0.08);
  stop.submitFrontWheel(0.0, 0.0);
  stop.submitRearWheel(0.0, 0.0);
  require(stop.state().velocity_mps == 0.0,
          "agreed standstill clamps velocity to zero");

  Estimator invalid;
  invalid.reset(0.0, 3.0);
  invalid.submitFrontWheel(std::numeric_limits<double>::quiet_NaN(), 0.1);
  require(invalid.state().front_stale,
          "non-finite wheel sample cannot enter the filter");
  require(std::isfinite(invalid.state().velocity_mps),
          "invalid input does not make state non-finite");

  std::cout << "PASS: estimator core causal conversion, slip, dropout and bounds\n";
  return EXIT_SUCCESS;
}
