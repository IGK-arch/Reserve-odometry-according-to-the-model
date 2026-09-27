#ifndef TRAM_ODOMETRY__ELEVATION_PROFILE_HPP_
#define TRAM_ODOMETRY__ELEVATION_PROFILE_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace tram_odometry {

// Surveyed base_link altitude in the final MGRS frame. This profile supplies
// height only: XY geometry and terminal loops remain in the trained route map.
class ElevationProfile {
 public:
  bool load(const std::string & path) {
    points_.clear();
    std::ifstream file(path);
    std::string line;
    if (!file || !std::getline(file, line)) return false;
    stripCarriageReturn(line);
    if (line != "x,y,z") return false;
    std::vector<Point> parsed;
    while (std::getline(file, line)) {
      stripCarriageReturn(line);
      if (line.empty()) continue;
      std::stringstream row(line);
      std::string x, y, z;
      if (!std::getline(row, x, ',') || !std::getline(row, y, ',') ||
          !std::getline(row, z, ',') || row.rdbuf()->in_avail() != 0 ||
          line.back() == ',') return false;
      Point point;
      if (!parseFinite(x, point.x) || !parseFinite(y, point.y) ||
          !parseFinite(z, point.z)) return false;
      if (!parsed.empty() &&
          std::hypot(point.x - parsed.back().x, point.y - parsed.back().y) <= 1e-8) {
        return false;
      }
      parsed.push_back(point);
    }
    if (file.bad() || parsed.size() < 2) return false;
    points_ = std::move(parsed);
    return true;
  }

  bool empty() const {return points_.empty();}
  std::size_t size() const {return points_.size();}

  // Projection and interpolation use the surveyed segment, not its nearest
  // vertex. At 5..10 m lateral distance, fade to the existing map height. The
  // same distance gate makes the missing terminal extensions fall back safely.
  // Do not apply an antenna-to-base offset after this returned height.
  double sample(double x, double y, double fallback_z) const {
    if (points_.size() < 2 || !std::isfinite(x) || !std::isfinite(y)) {
      return fallback_z;
    }
    double best_squared = 100.0;
    double surveyed_z = fallback_z;
    for (std::size_t i = 1; i < points_.size(); ++i) {
      const Point & a = points_[i - 1];
      const Point & b = points_[i];
      const double dx = b.x - a.x;
      const double dy = b.y - a.y;
      const double length_squared = dx * dx + dy * dy;
      const double t = std::clamp(((x - a.x) * dx + (y - a.y) * dy) /
                                  length_squared, 0.0, 1.0);
      const double ex = x - (a.x + t * dx);
      const double ey = y - (a.y + t * dy);
      const double squared = ex * ex + ey * ey;
      if (squared < best_squared) {
        best_squared = squared;
        surveyed_z = a.z + t * (b.z - a.z);
      }
    }
    if (best_squared >= 100.0) return fallback_z;
    const double weight = std::clamp((10.0 - std::sqrt(best_squared)) / 5.0, 0.0, 1.0);
    if (weight >= 1.0) return surveyed_z;
    return fallback_z + weight * (surveyed_z - fallback_z);
  }

 private:
  struct Point {double x = 0.0; double y = 0.0; double z = 0.0;};

  static void stripCarriageReturn(std::string & line) {
    if (!line.empty() && line.back() == '\r') line.pop_back();
  }

  static bool parseFinite(const std::string & text, double & value) {
    try {
      std::size_t used = 0;
      value = std::stod(text, &used);
      return used == text.size() && std::isfinite(value);
    } catch (...) {
      return false;
    }
  }

  std::vector<Point> points_;
};

}  // namespace tram_odometry

#endif  // TRAM_ODOMETRY__ELEVATION_PROFILE_HPP_
