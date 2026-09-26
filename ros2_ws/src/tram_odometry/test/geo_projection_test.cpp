#include <cassert>
#include <cmath>
#include <iomanip>
#include <iostream>

#include "tram_odometry/geo_projection.hpp"

int main() {
  using tram_odometry::geo::EnuDatum;
  using tram_odometry::geo::Geodetic;
  using tram_odometry::geo::geodeticToMgrs37Ucb;
  const Geodetic official{55.8088325462547, 37.4602768500852, 167.4109};
  const auto projected = geodeticToMgrs37Ucb(official);
  std::cout << std::fixed << std::setprecision(6)
            << projected.x << ' ' << projected.y << ' ' << projected.z << '\n';
  // Official answer: MGRS 37UCB035858, x=103501.6309, y=85876.1201.
  assert(std::abs(projected.x - 103501.6309) < 0.02);
  assert(std::abs(projected.y - 85876.1201) < 0.02);
  const EnuDatum datum({55.810367065, 37.462266845, 168.3794});
  const auto enu = datum.toEnu(official);
  const auto round_trip = datum.fromEnu(enu);
  assert(std::abs(round_trip.latitude_deg - official.latitude_deg) < 1e-9);
  assert(std::abs(round_trip.longitude_deg - official.longitude_deg) < 1e-9);
  assert(std::abs(round_trip.height_m - official.height_m) < 0.001);
  const auto projected_from_map = datum.enuToMgrs37Ucb(enu);
  assert(std::abs(projected_from_map.x - projected.x) < 0.001);
  assert(std::abs(projected_from_map.y - projected.y) < 0.001);
  // The output must stay continuous at the 400000 m UTM easting boundary:
  // MGRS square letters may change, but the judge uses the fixed CB origin.
  const auto west = geodeticToMgrs37Ucb({55.8088325462547, 37.39, 167.4109});
  const auto east = geodeticToMgrs37Ucb({55.8088325462547, 37.46, 167.4109});
  assert(west.x < 100000.0 && east.x > 100000.0);
  assert(east.x - west.x > 3000.0 && east.x - west.x < 6000.0);
  std::cout << "PASS: official MGRS control and ENU round trip\n";
}
