#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>
#include "tram_odometry/geo_projection.hpp"
namespace tram_odometry {
struct Point3 {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

inline Point3 operator-(const Point3 & a, const Point3 & b) {
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}

inline Point3 operator+(const Point3 & a, const Point3 & b) {
  return {a.x + b.x, a.y + b.y, a.z + b.z};
}

inline Point3 operator*(const Point3 & a, double t) {
  return {a.x * t, a.y * t, a.z * t};
}

// WGS84 geodetic -> ECEF -> ENU. The datum must equal the offline map datum.
class EnuProjection {
 public:
  EnuProjection(double latitude_deg, double longitude_deg, double altitude_m)
  : datum_({latitude_deg, longitude_deg, altitude_m}) {}

  double datumAltitude() const {return datum_.datumAltitude();}

  Point3 project(double latitude_deg, double longitude_deg, double altitude_m) const {
    const auto p = datum_.toEnu({latitude_deg, longitude_deg, altitude_m});
    return {p.x, p.y, p.z};
  }

  Point3 mgrsFromEnu(const Point3 & enu) const {
    const auto p = datum_.enuToMgrs37Ucb({enu.x, enu.y, enu.z});
    return {p.x, p.y, p.z};
  }

 private:
  geo::EnuDatum datum_;
};

struct RoutePoint {
  double s = 0.0;
  Point3 p;
};

struct RouteSample {
  Point3 p;
  double yaw = 0.0;
  bool clamped = false;
};

struct RouteMatch {
  std::string direction;
  double s = 0.0;
  double distance_m = std::numeric_limits<double>::infinity();
};

class RouteMap {
 public:
  bool load(const std::string & path) {
    std::ifstream file(path);
    if (!file) return false;
    out_.clear();
    return_.clear();
    std::string line;
    if (!std::getline(file, line)) return false;
    if (!line.empty() && line.back() == '\r') line.pop_back();
    if (line != "direction,s,x,y,z") return false;
    while (std::getline(file, line)) {
      if (line.empty()) continue;
      std::stringstream stream(line);
      std::array<std::string, 5> fields;
      bool complete = true;
      for (auto & field : fields) {
        if (!std::getline(stream, field, ',')) {complete = false; break;}
      }
      if (!complete) continue;
      std::vector<RoutePoint> * target = nullptr;
      if (fields[0] == "out") target = &out_;
      if (fields[0] == "return") target = &return_;
      if (!target) continue;
      try {
        const RoutePoint point{std::stod(fields[1]),
                               {std::stod(fields[2]), std::stod(fields[3]),
                                std::stod(fields[4])}};
        if (!std::isfinite(point.s) || !std::isfinite(point.p.x) ||
            !std::isfinite(point.p.y) || !std::isfinite(point.p.z)) continue;
        if (!target->empty() && point.s <= target->back().s) continue;
        target->push_back(point);
      } catch (const std::exception &) {
        continue;
      }
    }
    return out_.size() >= 2 || return_.size() >= 2;
  }

  bool has(const std::string & direction) const {
    return points(direction).size() >= 2;
  }

  RouteSample sample(const std::string & direction, double s) const {
    const auto & route = points(direction);
    if (route.size() < 2) return {};
    const bool clamped = s < route.front().s || s > route.back().s;
    const double clamped_s = std::clamp(s, route.front().s, route.back().s);
    auto upper = std::upper_bound(route.begin(), route.end(), clamped_s,
      [](double value, const RoutePoint & point) {return value < point.s;});
    const size_t index = upper == route.begin() ? 0u :
      std::min(static_cast<size_t>(upper - route.begin() - 1), route.size() - 2);
    const auto & a = route[index];
    const auto & b = route[index + 1];
    const double t = (clamped_s - a.s) / (b.s - a.s);
    const Point3 delta = b.p - a.p;
    return {a.p + delta * t, std::atan2(delta.y, delta.x), clamped};
  }

  RouteMatch nearest(const Point3 & query, const std::string & direction) const {
    RouteMatch best;
    if (direction == "out" || direction == "auto") nearestIn(query, "out", out_, best);
    if (direction == "return" || direction == "auto") {
      nearestIn(query, "return", return_, best);
    }
    return best;
  }

  // The map traces the rear master antenna. On a turn its trajectory tangent
  // differs from the body heading. Sampling near the bogie midpoint gives a
  // quasi-steady rigid-body approximation; straight tracks are unchanged.
  Point3 bodyDirection(const std::string & direction, double s,
                       double heading_lookahead_m = 0.0) const {
    const auto & route = points(direction);
    if (route.size() < 2) return {1, 0, 0};
    const double center = std::clamp(s + heading_lookahead_m,
                                     route.front().s, route.back().s);
    const auto pose = sample(direction, center);
    const Point3 delta = sample(direction, center + 1).p -
                         sample(direction, center - 1).p;
    const double length = std::sqrt(delta.x*delta.x + delta.y*delta.y + delta.z*delta.z);
    return length > 1e-8 ? delta * (1/length) :
                          Point3{std::cos(pose.yaw), std::sin(pose.yaw), 0};
  }

  double bodyYaw(const std::string & direction, double s,
                 double heading_lookahead_m = 0.0) const {
    const auto body = bodyDirection(direction, s, heading_lookahead_m);
    return std::atan2(body.y, body.x);
  }

  // Match an antenna rigidly displaced along the local body approximation.
  // The returned s always belongs to the MASTER route, including for rover.
  RouteMatch nearestAntenna(const Point3 & query, const std::string & direction,
                            double offset, double prior_s, double radius,
                            double heading_lookahead_m = 0.0) const {
    RouteMatch best;
    const auto & route = points(direction);
    if (route.size() < 2 || !std::isfinite(prior_s) || radius <= 0) return best;
    auto antenna = [&](const RoutePoint & point) {
      return antennaPosition(direction, point.s, offset, heading_lookahead_m);
    };
    for (size_t i=0; i+1<route.size(); ++i) {
      if (route[i+1].s < prior_s-radius || route[i].s > prior_s+radius) continue;
      const auto a=antenna(route[i]), b=antenna(route[i+1]);
      const double dx=b.x-a.x, dy=b.y-a.y, norm=dx*dx+dy*dy;
      if (norm<1e-8) continue;
      const double ds=route[i+1].s-route[i].s;
      const double lo=std::max(0.0,(prior_s-radius-route[i].s)/ds);
      const double hi=std::min(1.0,(prior_s+radius-route[i].s)/ds);
      const double t=std::clamp(((query.x-a.x)*dx+(query.y-a.y)*dy)/norm,lo,hi);
      const double error=std::hypot(query.x-a.x-t*dx,query.y-a.y-t*dy);
      if (error<best.distance_m) best={direction,route[i].s+t*ds,error};
    }
    return best;
  }

  Point3 antennaPosition(const std::string & direction, double s, double offset,
                         double heading_lookahead_m = 0.0) const {
    return sample(direction, s).p + bodyDirection(direction, s, heading_lookahead_m)*offset;
  }

 private:
  const std::vector<RoutePoint> & points(const std::string & direction) const {
    return direction == "return" ? return_ : out_;
  }

  static void nearestIn(const Point3 & query, const std::string & direction,
                        const std::vector<RoutePoint> & route, RouteMatch & best) {
    if (route.size() < 2) return;
    for (size_t i = 0; i + 1 < route.size(); ++i) {
      const auto & a = route[i];
      const auto & b = route[i + 1];
      const double dx = b.p.x - a.p.x;
      const double dy = b.p.y - a.p.y;
      const double length_squared = dx * dx + dy * dy;
      if (length_squared <= 1e-8) continue;
      const double t = std::clamp(
        ((query.x - a.p.x) * dx + (query.y - a.p.y) * dy) / length_squared,
        0.0, 1.0);
      const double error = std::hypot(query.x - (a.p.x + t * dx),
                                      query.y - (a.p.y + t * dy));
      if (error < best.distance_m) {
        best.direction = direction;
        best.s = a.s + t * (b.s - a.s);
        best.distance_m = error;
      }
    }
  }

  std::vector<RoutePoint> out_;
  std::vector<RoutePoint> return_;
};

struct PoseResult {
  Point3 p;
  double yaw = 0.0;
  bool clamped = false;
  bool mapped = false;
};

struct StartupFix {
  Point3 enu;
  double distance_m = 0.0;
  double stamp_s = 0.0;
};


}
