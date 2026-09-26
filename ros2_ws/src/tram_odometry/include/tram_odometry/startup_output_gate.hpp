#pragma once

#include <cmath>

namespace tram_odometry {

// Hold only position while a GNSS startup anchor may still arrive. The
// longitudinal estimator and velocity publisher continue to run normally.
inline bool shouldHoldPositionForStartup(bool use_startup_gnss, bool has_anchor,
                                         double event_stamp_s, double run_start_s,
                                         double startup_window_s) {
  return use_startup_gnss && !has_anchor && std::isfinite(event_stamp_s) &&
         std::isfinite(run_start_s) && std::isfinite(startup_window_s) &&
         startup_window_s > 0.0 &&
         event_stamp_s <= run_start_s + startup_window_s;
}

}  // namespace tram_odometry
