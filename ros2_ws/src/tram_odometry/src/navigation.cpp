#include "tram_odometry/navigation.hpp"
#include "tram_odometry/startup_output_gate.hpp"
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <utility>
namespace tram_odometry {
Navigation::Navigation(EstimatorConfig ec, NavigationConfig c)
 : config_(c), estimator_(std::move(ec)),
 projection_(c.map_datum_lat_deg,c.map_datum_lon_deg,c.map_datum_alt_m) {
  output_frame_id_ = c.output_frame_id;
  relative_frame_id_ = c.relative_frame_id;
  output_projection_ = c.output_projection;
  child_frame_id_ = c.child_frame_id;
  start_s_m_ = c.start_s_m;
  use_startup_gnss_ = c.use_startup_gnss;
  use_map_without_gnss_ = c.use_map_without_gnss;
  use_rover_fallback_ = c.use_rover_fallback;
  startup_gnss_window_s_ = c.startup_gnss_window_s;
  startup_min_fixes_ = c.startup_min_fixes;
  rover_fallback_delay_s_ = c.rover_fallback_delay_s;
  rover_to_master_s_m_ = c.rover_to_master_s_m;
  map_match_max_distance_m_ = c.map_match_max_distance_m;
  auto_out_terminal_min_x_m_ = c.auto_out_terminal_min_x_m;
  auto_return_terminal_max_x_m_ = c.auto_return_terminal_max_x_m;
  anchor_residual_decay_m_ = c.anchor_residual_decay_m;
  route_longitudinal_offset_m_ = c.route_longitudinal_offset_m;
  antenna_to_base_z_m_ = c.antenna_to_base_z_m;
  relative_heading_rad_ = c.relative_heading_rad;
  output_rotation_rad_ = c.output_rotation_rad;
  output_scale_ = c.output_scale;
  reset_on_large_time_jump_s_ = c.reset_on_large_time_jump_s;
  configured_direction_ = c.route_direction;
  selected_direction_ = c.route_direction == "auto" ? "out" : c.route_direction;
  start_s_m_ = configured_start_s_m_ = c.start_s_m;
  configured_relative_heading_rad_ = c.relative_heading_rad;
  initial_pose_ = configured_initial_pose_ = {c.initial_x_m,c.initial_y_m,c.initial_z_m};
  output_offset_ = {c.output_offset_x_m,c.output_offset_y_m,c.output_offset_z_m};
  if (c.output_projection != "mgrs37ucb" && c.output_projection != "enu")
    throw std::invalid_argument("output_projection must be mgrs37ucb or enu");
  if (c.route_direction != "out" && c.route_direction != "return" && c.route_direction != "auto")
    throw std::invalid_argument("route_direction must be out, return or auto");
  if (c.use_map_without_gnss && c.route_direction == "auto")
    throw std::invalid_argument("use_map_without_gnss requires direction");
  if (!std::isfinite(c.output_scale) || c.output_scale <= 0)
    throw std::invalid_argument("output_scale must be positive");
  if (!std::isfinite(c.body_heading_lookahead_m) || c.body_heading_lookahead_m < 0 ||
      c.body_heading_lookahead_m > 20)
    throw std::invalid_argument("body_heading_lookahead_m must be finite and within [0,20]");
  has_map_ = map_.load(c.map_file);
  has_alternate_ = !c.alternate_map_file.empty() && alternate_map_.load(c.alternate_map_file);
  if (c.enable_stop_landmarks && estimator_.config().vehicle_id == 30618 && has_map_) {
    if (c.stop_landmarks_file.empty())
      throw std::invalid_argument("Stop landmarks require a catalog file");
    std::ifstream input(c.stop_landmarks_file);
    if (!input) throw std::invalid_argument("Cannot load stop landmarks: " + c.stop_landmarks_file);
    std::string line;
    std::getline(input, line); // direction,s,... header
    if (line.rfind("direction,s,", 0) != 0)
      throw std::invalid_argument("Invalid stop landmark header");
    while (std::getline(input, line)) {
      if (line.empty()) continue;
      std::stringstream row(line);
      std::string direction, coordinate;
      if (!std::getline(row, direction, ',') || !std::getline(row, coordinate, ','))
        throw std::invalid_argument("Malformed stop landmark row");
      const double s = std::stod(coordinate);
      if ((direction != "out" && direction != "return") || !std::isfinite(s) || s < 0)
        throw std::invalid_argument("Invalid stop landmark coordinate");
      stop_landmarks_.push_back({direction, s, false});
    }
    if (stop_landmarks_.empty()) throw std::invalid_argument("Empty stop landmark catalog");
  }
  if (!c.elevation_file.empty() && !elevation_.load(c.elevation_file))
    throw std::invalid_argument("Cannot load elevation_file: " + c.elevation_file);
  for (const double value : {c.gnss_correction_max_age_s,c.gnss_correction_gate_m,
       c.gnss_correction_lateral_gate_m,c.gnss_correction_gain,c.gnss_correction_max_step_m})
    if (!std::isfinite(value)) throw std::invalid_argument("GNSS correction parameters must be finite");
  if (c.gnss_correction_max_age_s <= 0 || c.gnss_correction_gate_m <= 0 ||
      c.gnss_correction_lateral_gate_m <= 0 || c.gnss_correction_gain <= 0 ||
      c.gnss_correction_gain > 1 || c.gnss_correction_max_step_m <= 0)
    throw std::invalid_argument("Invalid GNSS correction gates or gain");
  if (!std::isfinite(c.stop_landmark_gain) || c.stop_landmark_gain <= 0 ||
      c.stop_landmark_gain > 1 || !std::isfinite(c.stop_landmark_gate_m) ||
      c.stop_landmark_gate_m <= 0 || !std::isfinite(c.stop_landmark_max_step_m) ||
      c.stop_landmark_max_step_m <= 0)
    throw std::invalid_argument("Invalid stop landmark gates or gain");
}

bool Navigation::submitVehicle(char type, double value, double stamp_s) {
  if (!std::isfinite(value) || !std::isfinite(stamp_s) || stamp_s <= 0) return false;
  if (type != 'F' && type != 'R' && type != 'C') return false;
  if (type == 'C' && (value < -15 || value > 15 || value != std::trunc(value))) return false;
  prepareInput(stamp_s);
  if(type=='F') estimator_.submitFrontWheel(value,stamp_s);
  if(type=='R') estimator_.submitRearWheel(value,stamp_s);
  if(type=='C') estimator_.submitDriverPosition(static_cast<int>(value),stamp_s);
  const auto state=estimator_.state();
  if(!state.initialized || !std::isfinite(state.stamp_s) ||
     state.stamp_s<=last_published_stamp_s_+1e-7 || std::abs(state.stamp_s-stamp_s)>1e-5) return false;
  updateStopLandmark(type, value, state);
  last_published_stamp_s_=state.stamp_s;
  output_.estimate=state;
  output_.position_valid=!shouldHoldPositionForStartup(use_startup_gnss_,has_gnss_anchor_,
    state.stamp_s,run_start_s_,startup_gnss_window_s_);
  const auto pose=poseFromDistance(state.distance_m);
  output_.position=pose.p; output_.yaw=pose.yaw; output_.mapped=pose.mapped;
  output_.clamped=pose.clamped; output_.anchored=has_gnss_anchor_;
  output_.frame_id=(!has_gnss_anchor_&&!pose.mapped)?relative_frame_id_:output_frame_id_;
  output_.direction=selected_direction_; output_.anchor_source=anchor_source_;
  output_.heading_source=heading_source_;
  output_.gnss_corrections=corrections_; output_.gnss_rejected=rejected_;
  output_.branch_switches=branch_switches_;
  output_.stop_corrections=stop_corrections_;
  output_.branch=alternate_active_?"alternate":"primary";
  return true;
}

void Navigation::submitFix(bool rover,double latitude,double longitude,double altitude,int status,double stamp_s) {
  if(!use_startup_gnss_ || (rover && !use_rover_fallback_)) return;
  if(!std::isfinite(stamp_s)||stamp_s<=0||status<0||!std::isfinite(latitude)||
     !std::isfinite(longitude)||std::abs(latitude)>90||std::abs(longitude)>180) {++rejected_;return;}
  const size_t sensor=rover?1:0;
  const auto state=estimator_.state();
  if (state.initialized && (stamp_s > state.stamp_s + 0.3 ||
      stamp_s < state.stamp_s - config_.gnss_correction_max_age_s)) {++rejected_;return;}
  auto & last_stamp = !has_gnss_anchor_ && status == 2 ?
    startup_rtk_last_stamp_[sensor] : last_fix_stamp_[sensor];
  if(stamp_s<=last_stamp) {++rejected_;return;}
  last_stamp=stamp_s;
  Fix fix{latitude,longitude,altitude,status,stamp_s};
  if(has_gnss_anchor_) {
    if(config_.enable_gnss_corrections) correctGnss(fix,rover);
  } else collectStartupGnss(&fix,rover);
}

void Navigation::prepareInput(double stamp_s) {
    if (!std::isfinite(run_start_s_)) run_start_s_ = stamp_s;
    if (stamp_s < estimator_.state().stamp_s - reset_on_large_time_jump_s_) {
      estimator_.reset(stamp_s);
      last_published_stamp_s_ = -std::numeric_limits<double>::infinity();
      run_start_s_ = stamp_s;
      has_gnss_anchor_ = false;
      startup_fixes_.clear();
      rover_startup_fixes_.clear();
      for (auto & fixes : startup_rtk_fixes_) fixes.clear();
      startup_rtk_last_stamp_.fill(-std::numeric_limits<double>::infinity());
      anchor_source_ = "none";
      anchor_residual_ = {};
      anchor_distance_m_ = 0.0;
      anchor_heading_delta_rad_ = 0.0;
      start_s_m_ = configured_start_s_m_;
      initial_pose_ = configured_initial_pose_;
      relative_heading_rad_ = configured_relative_heading_rad_;
      heading_source_ = "configured";
      selected_direction_ = configured_direction_ == "auto" ? "out" : configured_direction_;
      last_fix_stamp_.fill(-std::numeric_limits<double>::infinity());
      for (auto & window : correction_windows_) window.clear();
      fix_motion_={};
      pair_fixes_={};
      branch_evidence_.clear();
      corrections_=rejected_=branch_switches_=0;
      stop_wheels_ = {};
      stop_begin_s_ = NAN;
      stop_approach_peak_mps_ = stop_approach_speed_mps_ = 0.0;
      last_stop_landmark_distance_m_ = -INFINITY;
      stop_corrections_ = 0;
      stop_landmark_applied_ = false;
      for (auto & landmark : stop_landmarks_) landmark.used = false;
      if (alternate_active_) std::swap(map_,alternate_map_);
      alternate_active_=false;
    }
    tryStartupAnchor(stamp_s);
  }

void Navigation::tryStartupAnchor(double stamp_s) {
    if (has_gnss_anchor_) return;
    const auto & master = startup_rtk_fixes_[0];
    const auto & rover = startup_rtk_fixes_[1];
    const auto minimum = static_cast<size_t>(std::max(1, startup_min_fixes_));
    const bool expired = stamp_s > run_start_s_ + startup_gnss_window_s_;
    if (master.size() >= minimum) {
      finalizeStartupAnchor(false, true);
    } else if (rover.size() >= minimum &&
               (expired || (master.empty() && stamp_s > run_start_s_ + rover_fallback_delay_s_))) {
      finalizeStartupAnchor(true, true);
    } else if (expired) {
      // The deadline also preserves sparse-fix startup. Never mix qualities in
      // a median or let a single RTK fix promote a usable window before then.
      if (!master.empty()) finalizeStartupAnchor(false, true);
      else if (!rover.empty()) finalizeStartupAnchor(true, true);
      else if (!startup_fixes_.empty()) finalizeStartupAnchor(false, false);
      else if (!rover_startup_fixes_.empty()) finalizeStartupAnchor(true, false);
    }
  }

void Navigation::collectStartupGnss(const Fix * message, bool rover) {
    if (has_gnss_anchor_ || !std::isfinite(message->stamp_s) || message->stamp_s <= 0 ||
        message->status < 0 || !std::isfinite(message->latitude) ||
        !std::isfinite(message->longitude) ||
        std::abs(message->latitude) > 90.0 || std::abs(message->longitude) > 180.0) return;
    const double stamp_s = message->stamp_s;
    if (!std::isfinite(run_start_s_)) run_start_s_ = stamp_s;
    if (stamp_s > run_start_s_ + startup_gnss_window_s_) {
      tryStartupAnchor(stamp_s);
      return;
    }
    const double altitude_m = std::isfinite(message->altitude) ?
      message->altitude : projection_.datumAltitude();
    const Point3 gnss_enu = projection_.project(message->latitude, message->longitude,
                                                 altitude_m);
    const Estimate estimate = estimator_.state();
    const double stamp_delta_s = estimate.initialized && std::isfinite(estimate.stamp_s) ?
      std::clamp(stamp_s - estimate.stamp_s, -0.3, 0.3) : 0.0;
    const double distance_m = estimate.distance_m + estimate.velocity_mps * stamp_delta_s;
    if (has_map_) {
      const RouteMatch match = map_.nearest(gnss_enu, configured_direction_);
      if (match.direction.empty() || match.distance_m > map_match_max_distance_m_) return;
    }
    auto & fixes = message->status == 2 ? startup_rtk_fixes_[rover ? 1 : 0] :
      (rover ? rover_startup_fixes_ : startup_fixes_);
    fixes.push_back({gnss_enu, distance_m, stamp_s});
    tryStartupAnchor(stamp_s);
    // Position anchoring is independent of the velocity estimator.
  }

void Navigation::correctGnss(const Fix & fix, bool rover) {
  const auto state=estimator_.state();
  if (!has_map_ || !state.initialized || !std::isfinite(fix.altitude)) {++rejected_;return;}
  const double age=state.stamp_s-fix.stamp_s;
  if (age < -0.3 || age > config_.gnss_correction_max_age_s) {++rejected_;return;}
  const double distance=state.distance_m-state.velocity_mps*age;
  const double prior=start_s_m_+distance;
  const auto query=projection_.project(fix.latitude,fix.longitude,fix.altitude);
  // Check the independent rigid antenna geometry before either route correction
  // or branch evidence. A coherent bias can pass a single-antenna history gate.
  const size_t sensor=rover?1:0;
  pair_fixes_[sensor]={query,fix.stamp_s,true};
  const auto & other=pair_fixes_[1-sensor];
  const double pair_age=std::abs(fix.stamp_s-other.stamp_s);
  if (other.valid && pair_age <= 0.2) {
    const auto delta=query-other.p;
    const double separation=std::sqrt(delta.x*delta.x+delta.y*delta.y+delta.z*delta.z);
    // The maximum allowed vehicle speed bounds unobserved travel between the
    // antenna timestamps without assuming that the current wheel speed is right.
    // Keep a loose 3 m allowance because these fixes may have no covariance.
    const double tolerance=3.0+estimator_.config().max_speed_mps*pair_age;
    if (std::abs(separation-std::abs(rover_to_master_s_m_)) > tolerance) {
      for (auto & pending : correction_windows_) pending.clear();
      branch_evidence_.clear(); ++rejected_; return;
    }
  }
  auto & motion=fix_motion_[sensor];
  if (!motion.valid || fix.stamp_s-motion.last > 1 ||
      std::hypot(query.x-motion.p.x,query.y-motion.p.y) > 0.05)
    motion={query,distance,fix.stamp_s,fix.stamp_s,true};
  motion.last=fix.stamp_s;
  // A repeated position payload with fresh stamps cannot cancel motion measured
  // by two healthy wheels. Stationary fixes remain useful for position correction.
  if (fix.stamp_s-motion.start >= 0.2 && std::abs(distance-motion.distance) > 0.5 &&
      !state.front_stale && !state.rear_stale && !state.front_slip && !state.rear_slip &&
      !state.front_tentative && !state.rear_tentative) {
    correction_windows_[rover?1:0].clear(); branch_evidence_.clear(); ++rejected_; return;
  }
  const double offset=rover ? -rover_to_master_s_m_ : 0;
  auto match=map_.nearestAntenna(query,selected_direction_,offset,prior,config_.gnss_correction_gate_m,config_.body_heading_lookahead_m);
  // Evaluate the alternative BEFORE the current branch's lateral gate.
  // In a real fork the correct observation is necessarily far from the old branch.
  if (has_alternate_ && selected_direction_=="out") {
    const auto alternative=alternate_map_.nearestAntenna(query,selected_direction_,offset,
                                                        prior,config_.gnss_correction_gate_m,config_.body_heading_lookahead_m);
    if (!alternative.direction.empty() && alternative.distance_m < 3 &&
        std::abs(alternative.s-prior) < config_.gnss_correction_gate_m-0.01 &&
        match.distance_m > 8 && match.distance_m-alternative.distance_m > 5) {
      const double alternative_anchor=alternative.s-distance;
      if (!branch_evidence_.empty() && (fix.stamp_s-branch_evidence_.back().stamp_s > 1 ||
          std::abs(alternative_anchor-branch_evidence_.back().innovation) > 2))
        branch_evidence_.clear();
      // Evidence must progress in time even when both antennas have the same stamp.
      if (branch_evidence_.empty() || fix.stamp_s > branch_evidence_.back().stamp_s+1e-6)
        branch_evidence_.push_back({fix.stamp_s,alternative_anchor});
      if (branch_evidence_.size() >= 3 && fix.stamp_s-branch_evidence_.front().stamp_s >= 0.19) {
        std::swap(map_,alternate_map_); alternate_active_=!alternate_active_;
        start_s_m_=alternative.s-distance;
        anchor_residual_={}; anchor_heading_delta_rad_=0;
        ++branch_switches_;
        branch_evidence_.clear();
        for (auto & window : correction_windows_) window.clear();
        return;
      }
    } else branch_evidence_.clear();
  }
  auto & window=correction_windows_[rover?1:0];
  if (match.direction.empty() || match.distance_m > config_.gnss_correction_lateral_gate_m ||
      std::abs(match.s-prior) >= config_.gnss_correction_gate_m-0.01) {
    window.clear(); ++rejected_; return;
  }
  // Store absolute route anchor observations, not residuals invalidated by earlier updates.
  const double anchor=match.s-distance;
  if (!window.empty() && (fix.stamp_s-window.back().stamp_s > 1 ||
                         std::abs(anchor-window.back().innovation) > 2)) window.clear();
  window.push_back({fix.stamp_s,anchor});
  while (window.size()>3) window.pop_front();
  if (window.size()<3 || fix.stamp_s-window.front().stamp_s < 0.15) return;
  std::array<double,3> values{{window[0].innovation,window[1].innovation,window[2].innovation}};
  std::sort(values.begin(),values.end());
  const double correction=std::clamp(config_.gnss_correction_gain*(values[1]-start_s_m_),
    -config_.gnss_correction_max_step_m,config_.gnss_correction_max_step_m);
  start_s_m_+=correction;
  ++corrections_;
}

void Navigation::updateStopLandmark(char type, double value, const Estimate& state) {
  if (!config_.enable_stop_landmarks || !has_map_ || estimator_.config().vehicle_id != 30618 ||
      (!has_gnss_anchor_ && !use_map_without_gnss_) || alternate_active_) return;
  if (type == 'F' || type == 'R') {
    const size_t index = type == 'F' ? 0 : 1;
    const double scale = estimator_.config().wheel_scale *
      (index == 0 ? estimator_.config().front_scale : estimator_.config().rear_scale);
    const double speed = value * scale;
    if (std::isfinite(speed) && speed >= 0 && speed <= estimator_.config().max_speed_mps &&
        state.stamp_s >= stop_wheels_[index].stamp_s)
      stop_wheels_[index] = {speed, state.stamp_s};
  }
  const bool front_fresh = state.stamp_s - stop_wheels_[0].stamp_s <= 0.35;
  const bool rear_fresh = state.stamp_s - stop_wheels_[1].stamp_s <= 0.35;
  const bool stationary = (front_fresh || rear_fresh) &&
      (!front_fresh || stop_wheels_[0].speed_mps <= 0.11) &&
      (!rear_fresh || stop_wheels_[1].speed_mps <= 0.11) &&
      state.velocity_mps <= 0.2 && state.notch <= 0;
  if (!stationary) {
    stop_begin_s_ = NAN;
    stop_landmark_applied_ = false;
    stop_approach_peak_mps_ = std::max(stop_approach_peak_mps_, state.velocity_mps);
    return;
  }
  if (!std::isfinite(stop_begin_s_)) {
    stop_begin_s_ = state.stamp_s;
    stop_approach_speed_mps_ = stop_approach_peak_mps_;
    stop_approach_peak_mps_ = 0.0;
  }
  if (stop_landmark_applied_ || state.stamp_s - stop_begin_s_ < 8.0) return;
  stop_landmark_applied_ = true;
  if (stop_approach_speed_mps_ < 2.0 || state.stamp_s - run_start_s_ < 60.0 ||
      state.distance_m - anchor_distance_m_ < 100.0 ||
      state.distance_m - last_stop_landmark_distance_m_ < 30.0) return;
  const double predicted = start_s_m_ + state.distance_m;
  size_t best = stop_landmarks_.size();
  double best_error = INFINITY, second_error = INFINITY;
  for (size_t i = 0; i < stop_landmarks_.size(); ++i) {
    const auto & landmark = stop_landmarks_[i];
    if (landmark.direction != selected_direction_ || landmark.used) continue;
    const double error = std::abs(landmark.s - predicted);
    if (error < best_error) { second_error = best_error; best_error = error; best = i; }
    else if (error < second_error) second_error = error;
  }
  if (best == stop_landmarks_.size() || best_error > config_.stop_landmark_gate_m ||
      second_error - best_error < 6.0) return;
  const double correction = config_.stop_landmark_gain *
      (stop_landmarks_[best].s - predicted);
  if (std::abs(correction) > config_.stop_landmark_max_step_m) return;
  start_s_m_ += correction;
  stop_landmarks_[best].used = true;
  last_stop_landmark_distance_m_ = state.distance_m;
  ++stop_corrections_;
}

void Navigation::finalizeStartupAnchor(bool rover, bool rtk) {
    auto & master_fixes = rtk ? startup_rtk_fixes_[0] : startup_fixes_;
    auto & rover_fixes = rtk ? startup_rtk_fixes_[1] : rover_startup_fixes_;
    auto & fixes = rover ? rover_fixes : master_fixes;
    if (fixes.empty() || has_gnss_anchor_) return;
    auto median = [&fixes](const std::function<double(const StartupFix &)> & field) {
      std::vector<double> values;
      values.reserve(fixes.size());
      for (const auto & fix : fixes) values.push_back(field(fix));
      std::sort(values.begin(), values.end());
      return values[values.size() / 2];
    };
    const Point3 gnss_enu{median([](const StartupFix & f) {return f.enu.x;}),
                          median([](const StartupFix & f) {return f.enu.y;}),
                          median([](const StartupFix & f) {return f.enu.z;})};
    const double initial_distance_m = median([](const StartupFix & f) {
      return f.distance_m;
    });
    double paired_heading_rad = 0.0;
    bool paired_heading_valid = false;
    if (!master_fixes.empty() && !rover_fixes.empty()) {
      double sum_x = 0.0;
      double sum_y = 0.0;
      int valid_pairs = 0;
      for (const auto & master : master_fixes) {
        const StartupFix * nearest_rover = nullptr;
        double best_dt = 0.20;
        for (const auto & candidate : rover_fixes) {
          const double dt = std::abs(candidate.stamp_s - master.stamp_s);
          if (dt < best_dt) {
            nearest_rover = &candidate;
            best_dt = dt;
          }
        }
        if (!nearest_rover) continue;
        const double dx = nearest_rover->enu.x - master.enu.x;
        const double dy = nearest_rover->enu.y - master.enu.y;
        const double baseline_m = std::hypot(dx, dy);
        if (baseline_m < 8.0 || baseline_m > 17.0) continue;
        sum_x += dx / baseline_m;
        sum_y += dy / baseline_m;
        ++valid_pairs;
      }
      if (valid_pairs > 0 && std::hypot(sum_x, sum_y) > 0.7 * valid_pairs) {
        paired_heading_rad = std::atan2(sum_y, sum_x);
        paired_heading_valid = true;
        relative_heading_rad_ = paired_heading_rad;
        heading_source_ = "dual_antenna";
      }
    }
    if (has_map_) {
      std::string match_direction = configured_direction_;
      if (configured_direction_ == "auto") {
        if (gnss_enu.x > auto_out_terminal_min_x_m_) {
          match_direction = "out";
        } else if (gnss_enu.x < auto_return_terminal_max_x_m_) {
          match_direction = "return";
        } else if (paired_heading_valid) {
          const RouteMatch out = map_.nearest(gnss_enu, "out");
          const RouteMatch back = map_.nearest(gnss_enu, "return");
          const double out_alignment = out.direction.empty() ? -2.0 :
            std::cos(map_.bodyYaw("out", out.s, config_.body_heading_lookahead_m) - paired_heading_rad);
          const double back_alignment = back.direction.empty() ? -2.0 :
            std::cos(map_.bodyYaw("return", back.s, config_.body_heading_lookahead_m) - paired_heading_rad);
          match_direction = out_alignment >= back_alignment ? "out" : "return";
        } else {
        }
      }
      RouteMatch match = map_.nearest(gnss_enu, match_direction);
      if (rover && !match.direction.empty()) {
        match=map_.nearestAntenna(gnss_enu,match.direction,-rover_to_master_s_m_,
          match.s+rover_to_master_s_m_,map_match_max_distance_m_+std::abs(rover_to_master_s_m_),config_.body_heading_lookahead_m);
      }
      if (match.direction.empty() || match.distance_m > map_match_max_distance_m_) {
        fixes.clear();
        return;
      }
      selected_direction_ = match.direction;
      start_s_m_ = match.s - initial_distance_m;
      anchor_distance_m_ = initial_distance_m;
      anchor_residual_ = gnss_enu - map_.antennaPosition(selected_direction_, match.s,
        rover ? -rover_to_master_s_m_ : 0,config_.body_heading_lookahead_m);
      const double route_yaw = map_.bodyYaw(selected_direction_, match.s, config_.body_heading_lookahead_m);
      // Both antenna-to-master and master-to-base offsets use the measured
      // body course. Keep the map arm's vertical component on graded track.
      if (paired_heading_valid && rover) {
        const Point3 arm = map_.antennaPosition(
            selected_direction_, match.s, -rover_to_master_s_m_,
            config_.body_heading_lookahead_m) - map_.sample(selected_direction_, match.s).p;
        const double delta = paired_heading_rad - route_yaw;
        anchor_residual_.x += arm.x - (std::cos(delta) * arm.x - std::sin(delta) * arm.y);
        anchor_residual_.y += arm.y - (std::sin(delta) * arm.x + std::cos(delta) * arm.y);
      }
      if (paired_heading_valid) {
        anchor_heading_delta_rad_ = std::atan2(
          std::sin(paired_heading_rad - route_yaw),
          std::cos(paired_heading_rad - route_yaw));
      } else {
        relative_heading_rad_ = route_yaw;
        heading_source_ = "route_body_heading";
      }
    } else {
      const double shift_m = initial_distance_m - (rover ? rover_to_master_s_m_ : 0.0);
      initial_pose_ = gnss_enu - Point3{shift_m * std::cos(relative_heading_rad_),
                                        shift_m * std::sin(relative_heading_rad_), 0.0};
    }
    has_gnss_anchor_ = true;
    anchor_source_ = rover ? "rover_fallback" : "master";
    startup_fixes_.clear();
    rover_startup_fixes_.clear();
    for (size_t sensor = 0; sensor < startup_rtk_fixes_.size(); ++sensor) {
      startup_rtk_fixes_[sensor].clear();
      last_fix_stamp_[sensor] = std::max(last_fix_stamp_[sensor], startup_rtk_last_stamp_[sensor]);
    }
  }

PoseResult Navigation::poseFromDistance(double distance_m) const {
    PoseResult pose;
    if (has_map_ && (has_gnss_anchor_ || use_map_without_gnss_) &&
        map_.has(selected_direction_)) {
      const double master_s_m = start_s_m_ + distance_m;
      const auto route_pose = map_.sample(selected_direction_, master_s_m);
      const Point3 tangent = map_.bodyDirection(selected_direction_, master_s_m,
                                                  config_.body_heading_lookahead_m);
      double anchor_weight = 0.0;
      if (has_gnss_anchor_ && anchor_residual_decay_m_ > 0.0) {
        const double progressed_m = std::max(0.0, distance_m - anchor_distance_m_);
        anchor_weight = std::exp(-progressed_m / anchor_residual_decay_m_);
      }
      // The startup pair corrects body yaw, so it must rotate the same rigid
      // antenna-to-base lever arm. Rotate around Z to preserve the map grade.
      const double heading_delta = anchor_heading_delta_rad_ * anchor_weight;
      const double ch = std::cos(heading_delta), sh = std::sin(heading_delta);
      const Point3 body{ch * tangent.x - sh * tangent.y,
                        sh * tangent.x + ch * tangent.y, tangent.z};
      // GNSS anchors remain master-antenna route coordinates. Apply the body
      // extrinsic once, then the independent decaying master-position residual.
      pose = {route_pose.p + body * route_longitudinal_offset_m_ +
                Point3{anchor_residual_.x * anchor_weight,
                       anchor_residual_.y * anchor_weight, 0.0},
              std::atan2(tangent.y, tangent.x) + heading_delta, route_pose.clamped, true};
    } else {
      const double longitudinal_m = distance_m +
        (has_gnss_anchor_ ? route_longitudinal_offset_m_ : 0.0);
      pose.p = initial_pose_ + Point3{longitudinal_m * std::cos(relative_heading_rad_),
                                      longitudinal_m * std::sin(relative_heading_rad_), 0.0};
      pose.yaw = relative_heading_rad_;
    }
    // Both the map and GNSS startup fix describe an antenna at z=+3 m in the
    // body frame. Relative unanchored odometry already starts at base_link.
    if (pose.mapped || has_gnss_anchor_) {
      if (output_projection_ == "mgrs37ucb") {
        const Point3 ahead_enu = pose.p + Point3{std::cos(pose.yaw),
                                                 std::sin(pose.yaw), 0.0};
        const Point3 projected = projection_.mgrsFromEnu(pose.p);
        const Point3 projected_ahead = projection_.mgrsFromEnu(ahead_enu);
        pose.yaw = std::atan2(projected_ahead.y - projected.y,
                              projected_ahead.x - projected.x);
        pose.p = projected;
      }
      pose.p.z += antenna_to_base_z_m_;
      if (pose.mapped && output_projection_ == "mgrs37ucb")
        pose.p.z = elevation_.sample(pose.p.x,pose.p.y,pose.p.z);
    }
    const double c = std::cos(output_rotation_rad_);
    const double s = std::sin(output_rotation_rad_);
    const double x = pose.p.x;
    const double y = pose.p.y;
    pose.p.x = output_scale_ * (c * x - s * y) + output_offset_.x;
    pose.p.y = output_scale_ * (s * x + c * y) + output_offset_.y;
    pose.p.z = output_scale_ * pose.p.z + output_offset_.z;
    pose.yaw += output_rotation_rad_;
    return pose;
  }


}
