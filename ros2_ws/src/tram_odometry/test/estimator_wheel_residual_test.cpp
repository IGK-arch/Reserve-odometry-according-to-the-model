#include "tram_odometry/estimator.hpp"

#include <cmath>
#include <iostream>

namespace {
int failures = 0;
void require(bool condition, const char* message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    ++failures;
  }
}
void wheels(tram_odometry::Estimator& estimator, double speed, double stamp) {
  estimator.submitFrontWheel(speed * 3.6, stamp);
  estimator.submitRearWheel(speed * 3.6, stamp);
}
void warmup(tram_odometry::Estimator& estimator) {
  for (int i = 0; i <= 200; ++i) {
    const double t = 10.0 + i * 0.05;
    estimator.submitDriverPosition(4, t);
    if (i % 2 == 0) wheels(estimator, 5.0, t);
  }
}
void requirePureModelDuringGap(tram_odometry::Estimator& estimator,
                               double start, int notch) {
  for (int i = 1; i <= 20; ++i) {
    estimator.submitDriverPosition(notch, start + i * 0.05);
    const auto state = estimator.state();
    require(std::abs(state.acceleration_mps2 -
                     state.model_acceleration_mps2) < 1.0e-9,
            "unresolved fault evidence cannot contribute a fallback residual");
  }
}
}  // namespace

int main() {
  using tram_odometry::Estimator;
  using tram_odometry::EstimatorConfig;
  EstimatorConfig config;
  config.enable_adaptation = false;

  // An unknown grade cancels the weak traction prior; agreeing past wheels
  // provide short-term evidence when both streams disappear. The 0.35 budget
  // requires >20% reduction from the recovery-only error of 0.432643 m/s.
  Estimator estimator(config);
  warmup(estimator);
  for (int i = 1; i <= 20; ++i)
    estimator.submitDriverPosition(4, 20.0 + i * 0.05);
  const double error = std::abs(estimator.state().velocity_mps - 5.0);
  std::cout << "paired dropout one-second error: " << error << '\n';
  require(error < 0.35 && estimator.state().model_only,
          "recent healthy wheel acceleration reduces paired dropout error");
  for (int i = 1; i <= 5; ++i) {
    const double t = 21.0 + i * 0.1;
    estimator.submitDriverPosition(4, t);
    wheels(estimator, 5.0, t);
  }
  require(std::abs(estimator.state().velocity_mps - 5.0) < 0.01 &&
              !estimator.state().model_only,
          "fresh wheels recover promptly after residual-assisted fallback");
  Estimator fresh(config);
  estimator.reset(40.0, 5.0);
  fresh.reset(40.0, 5.0);
  for (int i = 0; i <= 20; ++i) {
    const double t = 40.0 + i * 0.05;
    estimator.submitDriverPosition(4, t);
    fresh.submitDriverPosition(4, t);
    require(std::abs(estimator.state().velocity_mps -
                     fresh.state().velocity_mps) < 1.0e-12,
            "reset removes learned wheel residual and derivative history");
  }

  // A rejected common jump invalidates the old residual and BOTH derivative
  // histories. One subsequent clean pair cannot reuse a derivative spanning
  // the fault, even though its pending jump flags can already clear.
  Estimator returned(config);
  warmup(returned);
  returned.submitDriverPosition(4, 20.1);
  wheels(returned, 9.0, 20.1);
  require(returned.state().front_slip && returned.state().rear_slip,
          "fault injection establishes independent jump evidence");
  returned.submitDriverPosition(4, 20.2);
  wheels(returned, 5.0, 20.2);
  require(!returned.state().model_only,
          "correct pair returns before derivative-history poisoning check");
  requirePureModelDuringGap(returned, 20.2, 4);

  // Invalidation also clears the propagation acceleration cached for late
  // callbacks. A tiny positive-time advance must not materially change the
  // projection of an otherwise identical next wheel reading.
  Estimator cached(config);
  warmup(cached);
  for (int i = 1; i <= 20; ++i)
    cached.submitDriverPosition(4, 20.0 + i * 0.05);
  Estimator advanced = cached;
  cached.submitFrontWheel(std::numeric_limits<double>::quiet_NaN(), 21.0);
  advanced.submitFrontWheel(std::numeric_limits<double>::quiet_NaN(), 21.0);
  advanced.advance(21.00001);
  cached.submitRearWheel(18.0, 20.9);
  advanced.submitRearWheel(18.0, 20.9);
  require(std::abs(cached.state().velocity_mps -
                   advanced.state().velocity_mps) < 1.0e-4,
          "fault invalidation clears cached late-wheel projection acceleration");

  // Agreement accepted only through growing model uncertainty is tentative,
  // not a new independent acceleration observation. Let that pair vary slowly
  // before losing both channels, so a missing guard would learn its derivative.
  Estimator tentative(config);
  tentative.reset(0.0, 5.0);
  wheels(tentative, 5.0, 0.0);
  for (int i = 1; i <= 100; ++i) {
    const double t = i * 0.1;
    tentative.submitDriverPosition(0, t);
    wheels(tentative, 9.0 + 0.2 * std::max(0.0, t - 6.0), t);
  }
  require(tentative.state().front_tentative &&
              tentative.state().rear_tentative,
          "uncertainty-accepted common pair remains tentative");
  requirePureModelDuringGap(tentative, 10.0, 0);

  if (failures) return 1;
  std::cout << "PASS: causal wheel residual fallback, return, reset and fault isolation\n";
  return 0;
}
