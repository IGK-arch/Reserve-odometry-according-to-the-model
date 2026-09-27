#pragma once

#include <array>
#include <limits>
#include <string>
#include <vector>

namespace tram_odometry {

// All speeds and distances inside this module are SI. The recorded wheel topic is
// numerically km/h, despite its message definition saying m/s.
struct EstimatorConfig {
  int vehicle_id = 30618;
  double wheel_scale = 1.0 / 3.6;
  double front_scale = 1.0;
  double rear_scale = 1.0;
  double max_speed_mps = 25.0;

  // Grey-box motor/longitudinal model. The force values are deliberately
  // conservative defaults; offline calibration can replace them per vehicle.
  double mass_kg = 27000.0;
  double effective_wheel_radius_m = 0.34;
  double powered_axles = 4.0;
  double motor_torque_max_nm = 600.0;
  double gear_ratio = 6.0;
  double drivetrain_efficiency = 0.84;
  double traction_power_limit_w = 200000.0;
  double brake_force_max_n = 30000.0;
  double traction_speed_rolloff_mps = 23.0;
  double traction_notch_exponent = 0.75;
  double brake_notch_exponent = 0.72;
  double traction_time_constant_s = 0.45;
  double brake_time_constant_s = 0.27;
  double rolling_resistance_accel_mps2 = 0.018;
  double quadratic_drag_accel_per_mps2 = 0.00055;
  double grade_accel_mps2 = 0.0;

  // Robust sensor fusion and adaptation.
  double wheel_stale_s = 0.35;
  double wheel_pair_max_age_s = 0.22;
  double max_out_of_order_s = 0.15;
  double max_wheel_header_lead_s = 0.50;
  double disagreement_gate_mps = 0.35;
  double innovation_gate_mps = 0.42;
  double implausible_wheel_accel_mps2 = 4.0;
  double outlier_hold_s = 0.35;
  double wheel_sigma_mps = 0.075;
  double process_accel_sigma_mps2 = 0.8;
  double command_stale_s = 1.0;
  double adaptive_bias_rate_per_s = 0.035;
  double adaptive_bias_limit_mps2 = 0.25;
  bool enable_adaptation = true;
  // Train-only notch/speed acceleration residual. The pure core defaults to
  // physics; the ROS node enables this for vehicle 30618 after replay ablation.
  bool enable_drive_table = false;
  std::string drive_table_path;

  static EstimatorConfig forVehicle(int vehicle_id);
};

struct Estimate {
  double stamp_s = std::numeric_limits<double>::quiet_NaN();
  double velocity_mps = 0.0;
  double distance_m = 0.0;
  double acceleration_mps2 = 0.0;
  double model_acceleration_mps2 = 0.0;
  double model_bias_mps2 = 0.0;
  double velocity_variance = 1.0;
  double distance_variance = 0.0;
  double front_weight = 0.0;
  double rear_weight = 0.0;
  bool front_slip = false;
  bool rear_slip = false;
  // Accepted through model uncertainty, still unsuitable as independent
  // evidence or as a source for learning the acceleration model.
  bool front_tentative = false;
  bool rear_tentative = false;
  bool front_stale = true;
  bool rear_stale = true;
  bool model_only = true;
  bool command_stale = true;
  bool drive_table_active = false;
  bool drive_table_used = false;
  bool initialized = false;
  int notch = 0;
};

// Single-threaded, event-time estimator. Supply messages in receipt order with
// their header timestamps. Slightly late messages are projected to current
// time; larger backward timestamp jumps are ignored. Each submit method
// advances before applying its input.
class Estimator {
 public:
  explicit Estimator(EstimatorConfig config = EstimatorConfig{});

  void reset(double stamp_s, double velocity_mps = 0.0,
             double distance_m = 0.0);
  void submitDriverPosition(int notch, double stamp_s);
  void submitFrontWheel(double raw_kmh, double stamp_s);
  void submitRearWheel(double raw_kmh, double stamp_s);
  Estimate advance(double stamp_s);
  Estimate state() const;
  const EstimatorConfig& config() const { return config_; }

 private:
  struct WheelState {
    double raw_speed_mps = 0.0;
    // Monotonic raw sensor history is distinct from callback-effective time.
    double ordered_raw_speed_mps = 0.0;
    double ordered_raw_stamp_s = std::numeric_limits<double>::quiet_NaN();
    double last_good_raw_mps = 0.0;
    double last_good_sensor_stamp_s = std::numeric_limits<double>::quiet_NaN();
    double speed_mps = 0.0;
    double stamp_s = std::numeric_limits<double>::quiet_NaN();
    double last_good_speed_mps = 0.0;
    double last_good_stamp_s = std::numeric_limits<double>::quiet_NaN();
    double unchanged_since_s = std::numeric_limits<double>::quiet_NaN();
    double slip_until_s = -std::numeric_limits<double>::infinity();
    // An impossible jump is independent evidence, unlike pair disagreement.
    // Retain it until an independent prediction can support the measurement.
    bool jump_pending = false;
    bool jump_tentative = false;
    bool present = false;
  };
  struct DriveCell {
    double speed_lo_mps = 0.0;
    double speed_hi_mps = 0.0;
    double accel_mps2 = 0.0;
    double uncertainty_mps2 = 1.0;
  };

  void submitWheel(WheelState& wheel, WheelState& other, double raw_kmh,
                   double sensor_scale, double stamp_s);
  void predict(double dt_s);
  void invalidateWheelResidual();
  double modelAcceleration(double speed_mps, double drive_state) const;
  bool loadDriveTable(const std::string& path);
  bool tableAcceleration(int notch, double speed_mps, double& acceleration,
                         double& uncertainty) const;
  bool fresh(const WheelState& wheel) const;
  bool trusted(const WheelState& wheel) const;
  bool validStamp(double stamp_s) const;
  void clampState();

  EstimatorConfig config_;
  WheelState front_;
  WheelState rear_;
  double stamp_s_ = std::numeric_limits<double>::quiet_NaN();
  double velocity_mps_ = 0.0;
  double distance_m_ = 0.0;
  double acceleration_mps2_ = 0.0;
  double model_acceleration_mps2_ = 0.0;
  double propagation_acceleration_mps2_ = 0.0;
  double wheel_accel_residual_mps2_ = 0.0;
  double wheel_residual_stamp_s_ = std::numeric_limits<double>::quiet_NaN();
  double velocity_variance_ = 1.0;
  double distance_variance_ = 0.0;
  double drive_state_ = 0.0;
  double adaptive_bias_mps2_ = 0.0;
  std::array<std::vector<DriveCell>, 31> drive_table_;
  bool drive_table_loaded_ = false;
  bool drive_table_used_ = false;
  double last_command_stamp_s_ = std::numeric_limits<double>::quiet_NaN();
  double command_change_stamp_s_ = std::numeric_limits<double>::quiet_NaN();
  double last_accepted_wheel_stamp_s_ =
      std::numeric_limits<double>::quiet_NaN();
  double last_independent_wheel_stamp_s_ =
      std::numeric_limits<double>::quiet_NaN();
  int notch_ = 0;
  bool initialized_ = false;
};

}  // namespace tram_odometry
