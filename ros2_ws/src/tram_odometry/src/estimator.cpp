#include "tram_odometry/estimator.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <utility>

namespace tram_odometry {
namespace {

double clamp(double x, double lo, double hi) {
  return std::max(lo, std::min(x, hi));
}

}  // namespace

EstimatorConfig EstimatorConfig::forVehicle(int vehicle_id) {
  EstimatorConfig cfg;
  cfg.vehicle_id = vehicle_id;
  if (vehicle_id == 30618) {
    // Wheel/GNSS scale fitted on clean training sessions only.
    cfg.front_scale = 1.000295;
    cfg.rear_scale = 1.000195;
  }
  if (vehicle_id == 30639) {
    // Train-split-only robust wheel/GNSS ratio calibration. These very small
    // scales correct a persistent vehicle-specific radius/tachometer bias.
    cfg.front_scale = 1.003512;
    cfg.rear_scale = 1.003187;
    // Both vehicles are the same model. The dynamics below remain weak priors,
    // rather than a claimed identification of the second vehicle.
    cfg.mass_kg = 27500.0;
    cfg.motor_torque_max_nm = 590.0;
    cfg.wheel_sigma_mps = 0.085;
  }
  return cfg;
}

Estimator::Estimator(EstimatorConfig config) : config_(std::move(config)) {
  config_.mass_kg = std::max(1000.0, config_.mass_kg);
  config_.effective_wheel_radius_m =
      std::max(0.1, config_.effective_wheel_radius_m);
  config_.wheel_scale = std::max(1.0e-6, config_.wheel_scale);
  config_.max_speed_mps = std::max(1.0, config_.max_speed_mps);
  config_.wheel_stale_s = std::max(0.05, config_.wheel_stale_s);
  config_.wheel_pair_max_age_s = std::max(0.0, config_.wheel_pair_max_age_s);
  config_.max_out_of_order_s = std::max(0.0, config_.max_out_of_order_s);
  config_.max_wheel_header_lead_s =
      std::max(0.0, config_.max_wheel_header_lead_s);
  config_.wheel_sigma_mps = std::max(0.005, config_.wheel_sigma_mps);
  config_.process_accel_sigma_mps2 =
      std::max(0.01, config_.process_accel_sigma_mps2);
  config_.outlier_hold_s = std::max(0.0, config_.outlier_hold_s);
  if (config_.enable_drive_table && !config_.drive_table_path.empty()) {
    drive_table_loaded_ = loadDriveTable(config_.drive_table_path);
  }
}

void Estimator::reset(double stamp_s, double velocity_mps, double distance_m) {
  if (!std::isfinite(stamp_s) || !std::isfinite(velocity_mps) ||
      !std::isfinite(distance_m)) {
    return;
  }
  front_ = WheelState{};
  rear_ = WheelState{};
  stamp_s_ = stamp_s;
  velocity_mps_ = clamp(velocity_mps, 0.0, config_.max_speed_mps);
  distance_m_ = distance_m;
  acceleration_mps2_ = 0.0;
  model_acceleration_mps2_ = 0.0;
  velocity_variance_ = 0.01;
  distance_variance_ = 0.0;
  drive_state_ = 0.0;
  adaptive_bias_mps2_ = 0.0;
  drive_table_used_ = false;
  last_command_stamp_s_ = std::numeric_limits<double>::quiet_NaN();
  command_change_stamp_s_ = std::numeric_limits<double>::quiet_NaN();
  last_accepted_wheel_stamp_s_ = std::numeric_limits<double>::quiet_NaN();
  notch_ = 0;
  initialized_ = true;
}

bool Estimator::validStamp(double stamp_s) const {
  return std::isfinite(stamp_s) &&
         (!std::isfinite(stamp_s_) ||
          stamp_s >= stamp_s_ - config_.max_out_of_order_s);
}

void Estimator::submitDriverPosition(int notch, double stamp_s) {
  if (!validStamp(stamp_s) || notch < -15 || notch > 15) {
    return;
  }
  advance(stamp_s);
  const double effective_stamp_s = stamp_s_;
  if (notch != notch_) {
    command_change_stamp_s_ = effective_stamp_s;
  }
  notch_ = notch;
  last_command_stamp_s_ = effective_stamp_s;
}

void Estimator::submitFrontWheel(double raw_kmh, double stamp_s) {
  submitWheel(front_, rear_, raw_kmh, config_.front_scale, stamp_s);
}

void Estimator::submitRearWheel(double raw_kmh, double stamp_s) {
  submitWheel(rear_, front_, raw_kmh, config_.rear_scale, stamp_s);
}

double Estimator::modelAcceleration(double speed_mps,
                                    double drive_state) const {
  const double demand = clamp(drive_state, -1.0, 1.0);
  double force_n = 0.0;
  if (demand > 0.0) {
    // Four motor torques pass through a gear reduction and limited power.
    // The notch response is nonlinear and the force falls with speed.
    const double torque_force =
        config_.powered_axles * config_.motor_torque_max_nm *
        config_.gear_ratio * config_.drivetrain_efficiency /
        config_.effective_wheel_radius_m;
    const double rolloff =
        1.0 / (1.0 + std::pow(speed_mps /
                                  std::max(1.0, config_.traction_speed_rolloff_mps),
                              2.0));
    const double power_force =
        config_.traction_power_limit_w / std::max(1.5, speed_mps);
    force_n = std::pow(demand,
                       std::max(0.1, config_.traction_notch_exponent)) *
              std::min(torque_force * rolloff, power_force);
  } else if (demand < 0.0) {
    const double low_speed_factor =
        0.5 + 0.5 * clamp(speed_mps / 1.5, 0.0, 1.0);
    force_n = -config_.brake_force_max_n * low_speed_factor *
              std::pow(-demand,
                       std::max(0.1, config_.brake_notch_exponent));
  }
  const double drag =
      speed_mps > 0.05
          ? config_.rolling_resistance_accel_mps2 +
                config_.quadratic_drag_accel_per_mps2 * speed_mps * speed_mps
          : 0.0;
  return clamp(force_n / config_.mass_kg - drag +
                   config_.grade_accel_mps2 + adaptive_bias_mps2_,
               -4.0, 3.0);
}

bool Estimator::loadDriveTable(const std::string& path) {
  std::ifstream file(path);
  if (!file) {
    return false;
  }
  std::string line;
  if (!std::getline(file, line)) {
    return false;
  }
  if (!line.empty() && line.back() == '\r') {
    line.pop_back();
  }
  if (line != "vehicle,notch,speed_bin,accel,uncertainty,count") {
    return false;
  }
  size_t loaded = 0;
  while (std::getline(file, line)) {
    std::istringstream row(line);
    std::array<std::string, 6> field;
    bool complete = true;
    for (auto& value : field) {
      if (!std::getline(row, value, ',')) {
        complete = false;
        break;
      }
    }
    if (!complete) {
      continue;
    }
    try {
      if (std::stoi(field[0]) != config_.vehicle_id) {
        continue;
      }
      const int notch = std::stoi(field[1]);
      const auto separator = field[2].find('-');
      if (notch < -15 || notch > 15 || separator == std::string::npos) {
        continue;
      }
      const double lo = std::stod(field[2].substr(0, separator));
      const double hi = std::stod(field[2].substr(separator + 1));
      const double accel = std::stod(field[3]);
      const double uncertainty = std::stod(field[4]);
      const int count = std::stoi(field[5]);
      if (!std::isfinite(lo) || !std::isfinite(hi) ||
          !std::isfinite(accel) || !std::isfinite(uncertainty) ||
          lo < 1.0 || hi > 20.0 || hi <= lo || uncertainty < 0.0 ||
          count < 80) {
        continue;
      }
      drive_table_[static_cast<size_t>(notch + 15)].push_back(
          {lo, hi, accel, uncertainty});
      ++loaded;
    } catch (const std::exception&) {
      continue;
    }
  }
  for (auto& cells : drive_table_) {
    std::sort(cells.begin(), cells.end(),
              [](const DriveCell& a, const DriveCell& b) {
                return a.speed_lo_mps < b.speed_lo_mps;
              });
  }
  return loaded > 0;
}

bool Estimator::tableAcceleration(int notch, double speed_mps,
                                  double& acceleration,
                                  double& uncertainty) const {
  if (!drive_table_loaded_ || notch < -15 || notch > 15) {
    return false;
  }
  const auto& cells = drive_table_[static_cast<size_t>(notch + 15)];
  for (size_t i = 0; i < cells.size(); ++i) {
    const DriveCell& center = cells[i];
    const bool in_bin = speed_mps >= center.speed_lo_mps &&
                        (speed_mps < center.speed_hi_mps ||
                         (i + 1 == cells.size() &&
                          speed_mps == center.speed_hi_mps));
    if (!in_bin) {
      continue;
    }
    acceleration = center.accel_mps2;
    uncertainty = center.uncertainty_mps2;
    const double center_speed =
        0.5 * (center.speed_lo_mps + center.speed_hi_mps);
    const DriveCell* adjacent = nullptr;
    if (speed_mps < center_speed && i > 0 &&
        std::abs(cells[i - 1].speed_hi_mps - center.speed_lo_mps) <
            1.0e-6) {
      adjacent = &cells[i - 1];
    } else if (speed_mps > center_speed && i + 1 < cells.size() &&
               std::abs(center.speed_hi_mps - cells[i + 1].speed_lo_mps) <
                   1.0e-6) {
      adjacent = &cells[i + 1];
    }
    if (adjacent != nullptr) {
      const double adjacent_speed =
          0.5 * (adjacent->speed_lo_mps + adjacent->speed_hi_mps);
      const double t = clamp(
          (speed_mps - center_speed) /
              (adjacent_speed - center_speed),
          0.0, 1.0);
      acceleration += t * (adjacent->accel_mps2 - acceleration);
      uncertainty =
          std::max(uncertainty, adjacent->uncertainty_mps2);
    }
    return true;
  }
  return false;
}

void Estimator::predict(double dt_s) {
  if (dt_s <= 0.0) {
    return;
  }
  const bool command_fresh =
      std::isfinite(last_command_stamp_s_) &&
      stamp_s_ - last_command_stamp_s_ <= config_.command_stale_s;
  const double desired = command_fresh ? static_cast<double>(notch_) / 15.0 : 0.0;
  const double tau = desired < drive_state_
                         ? config_.brake_time_constant_s
                         : config_.traction_time_constant_s;
  drive_state_ +=
      (desired - drive_state_) *
      (1.0 - std::exp(-dt_s / std::max(0.02, tau)));

  const double before = velocity_mps_;
  model_acceleration_mps2_ = modelAcceleration(before, drive_state_);
  drive_table_used_ = false;
  if (drive_table_loaded_ && command_fresh) {
    double observed_accel = 0.0;
    double uncertainty = 1.0;
    if (tableAcceleration(notch_, before, observed_accel, uncertainty)) {
      const double dwell_s =
          std::isfinite(command_change_stamp_s_)
              ? std::max(0.0, stamp_s_ - command_change_stamp_s_)
              : 0.0;
      const double confidence =
          clamp((0.75 - uncertainty) / 0.55, 0.0, 1.0) *
          (1.0 - std::exp(-dwell_s / 0.5));
      if (confidence > 1.0e-6) {
        const double physics_at_notch =
            modelAcceleration(before, static_cast<double>(notch_) / 15.0) -
            adaptive_bias_mps2_;
        model_acceleration_mps2_ =
            clamp(model_acceleration_mps2_ +
                      confidence * (observed_accel - physics_at_notch),
                  -4.0, 3.0);
        drive_table_used_ = true;
      }
    }
  }
  velocity_mps_ =
      clamp(before + model_acceleration_mps2_ * dt_s, 0.0,
            config_.max_speed_mps);
  distance_m_ += 0.5 * (before + velocity_mps_) * dt_s;
  acceleration_mps2_ =
      (velocity_mps_ - before) / dt_s;

  const double process_var =
      std::pow(config_.process_accel_sigma_mps2 * dt_s, 2.0);
  distance_variance_ =
      std::min(1.0e12,
               distance_variance_ + velocity_variance_ * dt_s * dt_s +
                   0.25 * process_var * dt_s * dt_s);
  velocity_variance_ = std::min(1.0e6, velocity_variance_ + process_var);
  clampState();
}

Estimate Estimator::advance(double stamp_s) {
  if (!validStamp(stamp_s)) {
    return state();
  }
  if (!std::isfinite(stamp_s_)) {
    stamp_s_ = stamp_s;
    return state();
  }
  const double delta_s = stamp_s - stamp_s_;
  if (delta_s <= 1.0e-6) {
    stamp_s_ = std::max(stamp_s_, stamp_s);
    return state();
  }

  // A discontinuous timestamp cannot justify unbounded model extrapolation.
  // The uncertainty still records the entire unresolved gap.
  const double modeled_s = std::min(delta_s, 60.0);
  double left_s = modeled_s;
  while (left_s > 1.0e-9) {
    const double step_s = std::min(left_s, 0.1);
    stamp_s_ += step_s;
    predict(step_s);
    left_s -= step_s;
  }
  if (delta_s > modeled_s) {
    const double unmodeled_s = delta_s - modeled_s;
    distance_variance_ =
        std::min(1.0e12, distance_variance_ +
                               velocity_variance_ * unmodeled_s * unmodeled_s);
    velocity_variance_ =
        std::min(1.0e6, velocity_variance_ +
                               std::pow(config_.process_accel_sigma_mps2 *
                                            unmodeled_s,
                                        2.0));
  }
  stamp_s_ = stamp_s;
  return state();
}

bool Estimator::fresh(const WheelState& wheel) const {
  return wheel.present && std::isfinite(stamp_s_) &&
         std::isfinite(wheel.stamp_s) &&
         stamp_s_ - wheel.stamp_s <= config_.wheel_stale_s + 1.0e-9;
}

bool Estimator::trusted(const WheelState& wheel) const {
  return fresh(wheel) && stamp_s_ >= wheel.slip_until_s;
}

void Estimator::submitWheel(WheelState& wheel, WheelState& other,
                            double raw_kmh, double sensor_scale,
                            double stamp_s) {
  if (!validStamp(stamp_s)) {
    return;
  }
  // A single tachometer header may jump into the future while the controller
  // stream keeps normal event time. Let the controller set the time frontier
  // instead of allowing that one malformed wheel header to poison it.
  if (std::isfinite(stamp_s_) &&
      stamp_s - stamp_s_ > config_.max_wheel_header_lead_s &&
      std::isfinite(last_command_stamp_s_) &&
      stamp_s_ - last_command_stamp_s_ <= config_.command_stale_s) {
    return;
  }
  advance(stamp_s);
  const double effective_stamp_s = stamp_s_;
  const double late_by_s = std::max(0.0, effective_stamp_s - stamp_s);
  const double physical_raw_mps =
      raw_kmh * config_.wheel_scale * sensor_scale;
  if (!std::isfinite(physical_raw_mps) || physical_raw_mps < -0.5 ||
      physical_raw_mps > config_.max_speed_mps + 5.0) {
    wheel.present = false;
    wheel.slip_until_s = effective_stamp_s + config_.outlier_hold_s;
    return;
  }
  const double converted =
      physical_raw_mps + model_acceleration_mps2_ * late_by_s;
  const double measured_mps = clamp(converted, 0.0, config_.max_speed_mps);
  const double predicted_mps = velocity_mps_;
  const double old_good_mps = wheel.last_good_speed_mps;
  const double old_good_stamp_s = wheel.last_good_stamp_s;
  const bool had_good = std::isfinite(old_good_stamp_s);
  const bool same_as_previous =
      wheel.present && std::isfinite(wheel.stamp_s) &&
      effective_stamp_s - wheel.stamp_s <= 0.5 &&
      std::abs(physical_raw_mps - wheel.raw_speed_mps) <= 0.002;
  if (!same_as_previous || !std::isfinite(wheel.unchanged_since_s)) {
    wheel.unchanged_since_s = effective_stamp_s;
  }
  wheel.raw_speed_mps = physical_raw_mps;
  wheel.speed_mps = measured_mps;
  wheel.stamp_s = effective_stamp_s;
  wheel.present = true;

  if (!initialized_) {
    velocity_mps_ = measured_mps;
    velocity_variance_ = std::pow(config_.wheel_sigma_mps, 2.0);
    wheel.last_good_speed_mps = measured_mps;
    wheel.last_good_stamp_s = effective_stamp_s;
    last_accepted_wheel_stamp_s_ = effective_stamp_s;
    initialized_ = true;
    return;
  }

  const bool other_recent =
      other.present && std::isfinite(other.stamp_s) &&
      effective_stamp_s - other.stamp_s <= config_.wheel_pair_max_age_s;
  const bool other_available =
      other_recent && effective_stamp_s >= other.slip_until_s;
  const double other_projected_mps =
      other.speed_mps +
      model_acceleration_mps2_ *
          (other_recent ?
               std::max(0.0, effective_stamp_s - other.stamp_s) : 0.0);
  const double pair_difference =
      other_recent ? measured_mps - other_projected_mps : 0.0;
  const bool pair_consistent =
      other_recent && std::abs(pair_difference) <
                          0.5 * config_.disagreement_gate_mps;
  const bool pair_recovery =
      pair_consistent &&
      (effective_stamp_s < wheel.slip_until_s ||
       effective_stamp_s < other.slip_until_s);
  bool current_bad = false;
  bool other_bad = false;

  // A rapid sensor-only acceleration is the strongest single-channel slip
  // cue. Ignore very short intervals, where quantization magnifies dv/dt.
  if (had_good) {
    const double good_dt_s = effective_stamp_s - old_good_stamp_s;
    if (good_dt_s >= 0.045 && good_dt_s <= 0.5 &&
        std::abs(measured_mps - old_good_mps) > 0.20 &&
        std::abs((measured_mps - old_good_mps) / good_dt_s) >
            config_.implausible_wheel_accel_mps2) {
      current_bad = true;
    }
  }

  if (other_available &&
      std::abs(pair_difference) > config_.disagreement_gate_mps) {
    const double this_innovation = std::abs(measured_mps - predicted_mps);
    const double other_innovation =
        std::abs(other_projected_mps - predicted_mps);
    const bool current_frozen =
        std::isfinite(wheel.unchanged_since_s) &&
        effective_stamp_s - wheel.unchanged_since_s > 0.8;
    const bool other_frozen =
        std::isfinite(other.unchanged_since_s) &&
        effective_stamp_s - other.unchanged_since_s > 0.8;
    if (current_frozen && !other_frozen) {
      current_bad = true;
    } else if (other_frozen && !current_frozen) {
      other_bad = true;
    } else if (this_innovation > other_innovation + 0.10) {
      current_bad = true;
    } else if (other_innovation > this_innovation + 0.10) {
      other_bad = true;
    } else if (notch_ > 0) {
      // Powered wheels usually overspeed when adhesion is lost.
      current_bad = pair_difference > 0.0;
      other_bad = !current_bad;
    } else if (notch_ < 0) {
      // A locked/braking wheel usually under-reports tram speed.
      current_bad = pair_difference < 0.0;
      other_bad = !current_bad;
    } else {
      current_bad = this_innovation >= other_innovation;
      other_bad = !current_bad;
    }
  }

  // If both measured bogies reconverge, their mutual agreement can rescue a
  // model-only filter whose prediction has drifted far from the real tram.
  // A current physically implausible jump still remains suspect.
  if (pair_recovery && !current_bad) {
    wheel.slip_until_s = -std::numeric_limits<double>::infinity();
    other.slip_until_s = -std::numeric_limits<double>::infinity();
  }
  if (effective_stamp_s < wheel.slip_until_s &&
      !pair_recovery &&
      !(!other_available &&
        std::abs(measured_mps - predicted_mps) < 0.20)) {
    current_bad = true;
  }
  if (current_bad) {
    wheel.slip_until_s = effective_stamp_s + config_.outlier_hold_s;
    return;
  }
  if (other_bad) {
    other.slip_until_s = effective_stamp_s + config_.outlier_hold_s;
  }
  wheel.slip_until_s = -std::numeric_limits<double>::infinity();

  // Trust wheel measurements strongly on nominal sections; cap a large
  // correction when the model is uncertain and no second sensor agrees.
  const double innovation = measured_mps - velocity_mps_;
  double corrected_innovation = innovation;
  if (!other_available && !pair_consistent &&
      std::abs(innovation) >
                              config_.innovation_gate_mps) {
    corrected_innovation = clamp(innovation, -0.8, 0.8);
  }
  const double r = std::pow(config_.wheel_sigma_mps *
                                (other_available ? 1.0 : 1.3),
                            2.0);
  const double kalman_gain = pair_recovery
                                 ? 0.90
                                 : clamp(velocity_variance_ /
                                             (velocity_variance_ + r),
                                         0.40, 0.88);
  const double before_update = velocity_mps_;
  velocity_mps_ =
      clamp(velocity_mps_ + kalman_gain * corrected_innovation, 0.0,
            config_.max_speed_mps);
  if (std::isfinite(last_accepted_wheel_stamp_s_)) {
    const double interval_s =
        clamp(effective_stamp_s - last_accepted_wheel_stamp_s_, 0.0, 0.25);
    // The prediction integrated its pre-update velocity over this interval.
    // A causal trapezoid places half of the measured velocity correction in
    // the traversed distance, without revising any earlier published output.
    distance_m_ += 0.5 * (velocity_mps_ - before_update) * interval_s;
  }
  last_accepted_wheel_stamp_s_ = effective_stamp_s;
  velocity_variance_ =
      std::max(1.0e-5, (1.0 - kalman_gain) * velocity_variance_);
  if (had_good) {
    const double good_dt_s = effective_stamp_s - old_good_stamp_s;
    if (good_dt_s > 0.045 && good_dt_s < 0.5) {
      const double observed_accel =
          (measured_mps - old_good_mps) / good_dt_s;
      acceleration_mps2_ = clamp(0.65 * observed_accel +
                                    0.35 * model_acceleration_mps2_,
                                -4.0, 3.0);
    } else {
      acceleration_mps2_ = model_acceleration_mps2_;
    }
  } else {
    acceleration_mps2_ = model_acceleration_mps2_;
  }

  wheel.last_good_speed_mps = measured_mps;
  wheel.last_good_stamp_s = effective_stamp_s;

  if (config_.enable_adaptation && had_good && trusted(other) &&
      std::abs(measured_mps - other_projected_mps) < 0.12 &&
      std::isfinite(command_change_stamp_s_) &&
      effective_stamp_s - command_change_stamp_s_ > 0.7 &&
      measured_mps > 0.5) {
    const double dt_s = effective_stamp_s - old_good_stamp_s;
    if (dt_s >= 0.08 && dt_s <= 0.3) {
      const double observed_accel =
          (measured_mps - old_good_mps) / dt_s;
      const double residual = observed_accel - model_acceleration_mps2_;
      if (std::abs(residual) < 1.5) {
        adaptive_bias_mps2_ =
            clamp(adaptive_bias_mps2_ +
                      config_.adaptive_bias_rate_per_s * dt_s * residual,
                  -config_.adaptive_bias_limit_mps2,
                  config_.adaptive_bias_limit_mps2);
      }
    }
  }

  // A dual-bogie standstill is more reliable than the model's rolling term.
  if (trusted(other) && measured_mps < 0.055 &&
      other_projected_mps < 0.055 && velocity_mps_ < 0.15) {
    velocity_mps_ = 0.0;
    acceleration_mps2_ = std::min(0.0, acceleration_mps2_);
    velocity_variance_ = std::min(velocity_variance_, 0.0025);
  }
  clampState();
}

void Estimator::clampState() {
  if (!std::isfinite(velocity_mps_)) {
    velocity_mps_ = 0.0;
  }
  velocity_mps_ = clamp(velocity_mps_, 0.0, config_.max_speed_mps);
  if (!std::isfinite(distance_m_)) {
    distance_m_ = 0.0;
    distance_variance_ = 1.0e12;
  }
  if (!std::isfinite(velocity_variance_) || velocity_variance_ < 0.0) {
    velocity_variance_ = 1.0e6;
  }
  if (!std::isfinite(distance_variance_) || distance_variance_ < 0.0) {
    distance_variance_ = 1.0e12;
  }
}

Estimate Estimator::state() const {
  Estimate output;
  output.stamp_s = stamp_s_;
  output.velocity_mps = velocity_mps_;
  output.distance_m = distance_m_;
  output.acceleration_mps2 = acceleration_mps2_;
  output.model_acceleration_mps2 = model_acceleration_mps2_;
  output.model_bias_mps2 = adaptive_bias_mps2_;
  output.velocity_variance = velocity_variance_;
  output.distance_variance = distance_variance_;
  output.front_slip = std::isfinite(stamp_s_) &&
                      stamp_s_ < front_.slip_until_s;
  output.rear_slip = std::isfinite(stamp_s_) &&
                     stamp_s_ < rear_.slip_until_s;
  output.front_stale = !fresh(front_);
  output.rear_stale = !fresh(rear_);
  output.front_weight = trusted(front_) ? 1.0 : 0.0;
  output.rear_weight = trusted(rear_) ? 1.0 : 0.0;
  output.model_only = output.front_weight + output.rear_weight == 0.0;
  output.command_stale =
      !std::isfinite(last_command_stamp_s_) ||
      !std::isfinite(stamp_s_) ||
      stamp_s_ - last_command_stamp_s_ > config_.command_stale_s;
  output.drive_table_active = drive_table_loaded_;
  output.drive_table_used = drive_table_used_;
  output.initialized = initialized_;
  output.notch = notch_;
  return output;
}

}  // namespace tram_odometry
