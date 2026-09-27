#pragma once
#include <functional>
#include <deque>
#include <string>
#include "tram_odometry/estimator.hpp"
#include "tram_odometry/route_map.hpp"
#include "tram_odometry/elevation_profile.hpp"
namespace tram_odometry {
struct NavigationConfig {
std::string map_file;
std::string alternate_map_file;
std::string stop_landmarks_file;
bool enable_stop_landmarks = false;
double stop_landmark_gain = 0.8;
double stop_landmark_gate_m = 12.0;
double stop_landmark_max_step_m = 10.0;
std::string elevation_file;
std::string output_frame_id = "mgrs_37UCB";
std::string relative_frame_id = "odom";
std::string output_projection = "mgrs37ucb";
std::string child_frame_id = "base_link";
std::string route_direction = "auto";
double start_s_m = 0;
bool use_startup_gnss = true;
bool use_map_without_gnss = false;
bool use_rover_fallback = true;
double startup_gnss_window_s = 5;
int startup_min_fixes = 3;
double rover_fallback_delay_s = 1.5;
double rover_to_master_s_m = -12.436;
double map_match_max_distance_m = 50;
double auto_out_terminal_min_x_m = -200;
double auto_return_terminal_max_x_m = -4400;
double anchor_residual_decay_m = 100;
double route_longitudinal_offset_m = 9.873;
// Master-to-bogie-midpoint distance: 9.873 - 7.55/2, from organizer geometry.
double body_heading_lookahead_m = 6.098;
double antenna_to_base_z_m = -3;
double relative_heading_rad = 0;
double initial_x_m = 0;
double initial_y_m = 0;
double initial_z_m = 0;
double output_rotation_rad = 0;
double output_scale = 1;
double output_offset_x_m = 0;
double output_offset_y_m = 0;
double output_offset_z_m = 0;
double map_datum_lat_deg = 55.810367065;
double map_datum_lon_deg = 37.462266845;
double map_datum_alt_m = 168.3794;
double reset_on_large_time_jump_s = 60;
bool enable_gnss_corrections = true;
double gnss_correction_max_age_s = 0.5;
double gnss_correction_gate_m = 40;
double gnss_correction_lateral_gate_m = 8;
double gnss_correction_gain = 0.5;
double gnss_correction_max_step_m = 3;
};
struct NavigationOutput {
 Estimate estimate;
 Point3 position;
 double yaw = 0;
 bool position_valid = false;
 bool mapped = false;
 bool clamped = false;
 bool anchored = false;
 std::string frame_id;
 std::string direction;
 std::string anchor_source;
 std::string heading_source;
 std::string branch = "primary";
 size_t gnss_corrections = 0;
 size_t gnss_rejected = 0;
 size_t branch_switches = 0;
 size_t stop_corrections = 0;
};
class Navigation {
 public:
 Navigation(EstimatorConfig estimator_config, NavigationConfig config);
 bool submitVehicle(char type, double value, double stamp_s);
 void submitFix(bool rover, double latitude, double longitude, double altitude,
                int status, double stamp_s);
 const NavigationOutput& output() const { return output_; }
 Estimate state() const { return estimator_.state(); }
 const EstimatorConfig& estimatorConfig() const { return estimator_.config(); }
 bool mapReady() const { return has_map_; }
 // Read-only publication uncertainty origin; navigation owns anchor/reset state.
 double anchorDistanceM() const { return has_gnss_anchor_ ? anchor_distance_m_ : 0.0; }
 private:
 struct Fix { double latitude,longitude,altitude; int status; double stamp_s; };
 struct Correction { double stamp_s, innovation; };
 struct PairFix { Point3 p; double stamp_s=0; bool valid=false; };
 struct FixMotion { Point3 p; double distance=0, start=0, last=0; bool valid=false; };
 struct StopLandmark { std::string direction; double s=0; bool used=false; };
 struct WheelStopSample { double speed_mps=0, stamp_s=-INFINITY; };
 void prepareInput(double stamp_s);
 void collectStartupGnss(const Fix* message, bool rover);
 void tryStartupAnchor(double stamp_s);
 void finalizeStartupAnchor(bool rover, bool rtk);
 void correctGnss(const Fix& message, bool rover);
 void updateStopLandmark(char type, double value, const Estimate& state);
 PoseResult poseFromDistance(double distance_m) const;
 NavigationConfig config_;
 NavigationOutput output_;
 ElevationProfile elevation_;
 RouteMap alternate_map_;
 bool has_alternate_ = false;
 bool alternate_active_ = false;
 bool branch_rule_map_compatible_ = false;
 bool branch_behavior_used_ = false;
 double branch_highspeed_begin_s_ = NAN;
 double branch_transition_distance_m_ = NAN;
 std::array<double,2> last_fix_stamp_{{-INFINITY,-INFINITY}};
 std::array<std::deque<Correction>,2> correction_windows_;
 std::array<FixMotion,2> fix_motion_;
 std::array<PairFix,2> pair_fixes_;
 std::deque<Correction> branch_evidence_;
 size_t corrections_ = 0, rejected_ = 0, branch_switches_ = 0;
 std::vector<StopLandmark> stop_landmarks_;
 std::array<WheelStopSample,2> stop_wheels_;
 double stop_begin_s_ = NAN;
 double stop_approach_peak_mps_ = 0.0;
 double stop_approach_speed_mps_ = 0.0;
 double last_stop_landmark_distance_m_ = -INFINITY;
 size_t stop_corrections_ = 0;
 bool stop_landmark_applied_ = false;
  Estimator estimator_;
  EnuProjection projection_;
  RouteMap map_;
  bool has_map_ = false;
  bool use_startup_gnss_ = true;
  bool use_map_without_gnss_ = false;
  bool use_rover_fallback_ = true;
  bool has_gnss_anchor_ = false;
  double run_start_s_ = std::numeric_limits<double>::quiet_NaN();
  double last_published_stamp_s_ = -std::numeric_limits<double>::infinity();
  double startup_gnss_window_s_ = 5.0;
  int startup_min_fixes_ = 3;
  std::vector<StartupFix> startup_fixes_;
  std::vector<StartupFix> rover_startup_fixes_;
  // Keep RTK evidence and timestamp ordering independent of lower-quality fixes.
  std::array<std::vector<StartupFix>,2> startup_rtk_fixes_;
  std::array<double,2> startup_rtk_last_stamp_{{-INFINITY,-INFINITY}};
  double rover_fallback_delay_s_ = 1.5;
  double rover_to_master_s_m_ = -12.44;
  std::string anchor_source_ = "none";
  std::string heading_source_ = "configured";
  double map_match_max_distance_m_ = 50.0;
  double auto_out_terminal_min_x_m_ = -200.0;
  double auto_return_terminal_max_x_m_ = -4400.0;
  double anchor_residual_decay_m_ = 100.0;
  double anchor_distance_m_ = 0.0;
  double anchor_heading_delta_rad_ = 0.0;
  Point3 anchor_residual_;
  double configured_start_s_m_ = 0.0;
  double start_s_m_ = 0.0;
  double route_longitudinal_offset_m_ = 0.0;
  double antenna_to_base_z_m_ = -3.0;
  double relative_heading_rad_ = 0.0;
  double configured_relative_heading_rad_ = 0.0;
  double output_rotation_rad_ = 0.0;
  double output_scale_ = 1.0;
  double reset_on_large_time_jump_s_ = 60.0;
  Point3 initial_pose_;
  Point3 configured_initial_pose_;
  Point3 output_offset_;
  std::string output_frame_id_;
  std::string relative_frame_id_;
  std::string output_projection_;
  std::string child_frame_id_;
  std::string configured_direction_;
  std::string selected_direction_;

};
}
