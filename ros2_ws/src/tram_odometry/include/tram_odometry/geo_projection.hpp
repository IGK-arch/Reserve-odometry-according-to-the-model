#pragma once

#include <cmath>

namespace tram_odometry::geo {

constexpr double pi = 3.1415926535897932384626433832795;
constexpr double deg_to_rad = pi / 180.0;
constexpr double rad_to_deg = 180.0 / pi;
constexpr double wgs84_a_m = 6378137.0;
constexpr double wgs84_f = 1.0 / 298.257223563;
constexpr double wgs84_e2 = wgs84_f * (2.0 - wgs84_f);

struct Point3 {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct Geodetic {
  double latitude_deg = 0.0;
  double longitude_deg = 0.0;
  double height_m = 0.0;
};

inline Point3 geodeticToEcef(const Geodetic & position) {
  const double latitude = position.latitude_deg * deg_to_rad;
  const double longitude = position.longitude_deg * deg_to_rad;
  const double sin_latitude = std::sin(latitude);
  const double cos_latitude = std::cos(latitude);
  const double n = wgs84_a_m / std::sqrt(1.0 - wgs84_e2 * sin_latitude * sin_latitude);
  return {(n + position.height_m) * cos_latitude * std::cos(longitude),
          (n + position.height_m) * cos_latitude * std::sin(longitude),
          (n * (1.0 - wgs84_e2) + position.height_m) * sin_latitude};
}

inline Geodetic ecefToGeodetic(const Point3 & ecef) {
  const double longitude = std::atan2(ecef.y, ecef.x);
  const double radius_xy = std::hypot(ecef.x, ecef.y);
  double latitude = std::atan2(ecef.z, radius_xy * (1.0 - wgs84_e2));
  double height = 0.0;
  for (int i = 0; i < 6; ++i) {
    const double sin_latitude = std::sin(latitude);
    const double n = wgs84_a_m / std::sqrt(1.0 - wgs84_e2 * sin_latitude * sin_latitude);
    height = radius_xy / std::cos(latitude) - n;
    latitude = std::atan2(ecef.z,
      radius_xy * (1.0 - wgs84_e2 * n / (n + height)));
  }
  const double sin_latitude = std::sin(latitude);
  const double n = wgs84_a_m / std::sqrt(1.0 - wgs84_e2 * sin_latitude * sin_latitude);
  height = radius_xy / std::cos(latitude) - n;
  return {latitude * rad_to_deg, longitude * rad_to_deg, height};
}

// WGS84 Transverse Mercator, UTM zone 37 North. Returns continuous coordinates
// inside the 100-km MGRS square 37UCB: x=UTM easting-300000,
// y=UTM northing-6100000. This deliberately does not wrap at square edges.
inline Point3 geodeticToMgrs37Ucb(const Geodetic & position) {
  const double latitude = position.latitude_deg * deg_to_rad;
  const double longitude = position.longitude_deg * deg_to_rad;
  const double central_meridian = 39.0 * deg_to_rad;
  const double e4 = wgs84_e2 * wgs84_e2;
  const double e6 = e4 * wgs84_e2;
  const double ep2 = wgs84_e2 / (1.0 - wgs84_e2);
  const double sin_latitude = std::sin(latitude);
  const double cos_latitude = std::cos(latitude);
  const double tan_latitude = std::tan(latitude);
  const double n = wgs84_a_m / std::sqrt(1.0 - wgs84_e2 * sin_latitude * sin_latitude);
  const double t = tan_latitude * tan_latitude;
  const double c = ep2 * cos_latitude * cos_latitude;
  const double a = cos_latitude * (longitude - central_meridian);
  const double a2 = a * a;
  const double a3 = a2 * a;
  const double a4 = a2 * a2;
  const double a5 = a4 * a;
  const double a6 = a3 * a3;
  const double meridian_arc = wgs84_a_m * (
    (1.0 - wgs84_e2 / 4.0 - 3.0 * e4 / 64.0 - 5.0 * e6 / 256.0) * latitude
    - (3.0 * wgs84_e2 / 8.0 + 3.0 * e4 / 32.0 + 45.0 * e6 / 1024.0)
      * std::sin(2.0 * latitude)
    + (15.0 * e4 / 256.0 + 45.0 * e6 / 1024.0) * std::sin(4.0 * latitude)
    - 35.0 * e6 / 3072.0 * std::sin(6.0 * latitude));
  constexpr double k0 = 0.9996;
  const double easting = 500000.0 + k0 * n * (
    a + (1.0 - t + c) * a3 / 6.0 +
    (5.0 - 18.0 * t + t * t + 72.0 * c - 58.0 * ep2) * a5 / 120.0);
  const double northing = k0 * (meridian_arc + n * tan_latitude * (
    a2 / 2.0 + (5.0 - t + 9.0 * c + 4.0 * c * c) * a4 / 24.0 +
    (61.0 - 58.0 * t + t * t + 600.0 * c - 330.0 * ep2) * a6 / 720.0));
  return {easting - 300000.0, northing - 6100000.0, position.height_m};
}

class EnuDatum {
 public:
  explicit EnuDatum(const Geodetic & origin)
  : origin_(origin), origin_ecef_(geodeticToEcef(origin)),
    sin_lat_(std::sin(origin.latitude_deg * deg_to_rad)),
    cos_lat_(std::cos(origin.latitude_deg * deg_to_rad)),
    sin_lon_(std::sin(origin.longitude_deg * deg_to_rad)),
    cos_lon_(std::cos(origin.longitude_deg * deg_to_rad)) {}

  Point3 toEnu(const Geodetic & position) const {
    const Point3 p = geodeticToEcef(position);
    const double dx = p.x - origin_ecef_.x;
    const double dy = p.y - origin_ecef_.y;
    const double dz = p.z - origin_ecef_.z;
    return {-sin_lon_ * dx + cos_lon_ * dy,
            -sin_lat_ * cos_lon_ * dx - sin_lat_ * sin_lon_ * dy + cos_lat_ * dz,
            cos_lat_ * cos_lon_ * dx + cos_lat_ * sin_lon_ * dy + sin_lat_ * dz};
  }

  Geodetic fromEnu(const Point3 & enu) const {
    const Point3 ecef{
      origin_ecef_.x - sin_lon_ * enu.x - sin_lat_ * cos_lon_ * enu.y +
        cos_lat_ * cos_lon_ * enu.z,
      origin_ecef_.y + cos_lon_ * enu.x - sin_lat_ * sin_lon_ * enu.y +
        cos_lat_ * sin_lon_ * enu.z,
      origin_ecef_.z + cos_lat_ * enu.y + sin_lat_ * enu.z};
    return ecefToGeodetic(ecef);
  }

  Point3 enuToMgrs37Ucb(const Point3 & enu) const {
    return geodeticToMgrs37Ucb(fromEnu(enu));
  }

  double datumAltitude() const {return origin_.height_m;}

 private:
  Geodetic origin_;
  Point3 origin_ecef_;
  double sin_lat_;
  double cos_lat_;
  double sin_lon_;
  double cos_lon_;
};

}  // namespace tram_odometry::geo
