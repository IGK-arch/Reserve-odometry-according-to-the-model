#include <array>
#include <cassert>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

#include "tram_odometry/geo_projection.hpp"

namespace {
struct Row {
  double s = 0.0;
  tram_odometry::geo::Point3 p;
};
}

int main(int argc, char ** argv) {
  const std::string path = argc > 1 ? argv[1] :
    "ros2_ws/src/tram_odometry/assets/route_map.csv";
  std::ifstream file(path);
  assert(file.good());
  std::string line;
  std::getline(file, line);
  std::array<Row, 2> out, back;
  int out_count = 0;
  int back_count = 0;
  while (std::getline(file, line)) {
    std::stringstream stream(line);
    std::array<std::string, 5> fields;
    for (auto & field : fields) std::getline(stream, field, ',');
    Row row{std::stod(fields[1]),
      {std::stod(fields[2]), std::stod(fields[3]), std::stod(fields[4])}};
    if (fields[0] == "out" && out_count < 2) out[out_count++] = row;
    if (fields[0] == "return" && back_count < 2) back[back_count++] = row;
    if (out_count == 2 && back_count == 2) break;
  }
  assert(out_count == 2 && back_count == 2);
  const tram_odometry::geo::EnuDatum datum({55.810367065, 37.462266845, 168.3794});
  auto print = [&datum](const char * label, const std::array<Row, 2> & route,
                        double expected_x, double expected_y, double expected_z) {
    const auto & a = route[0].p;
    const auto & b = route[1].p;
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double dz = b.z - a.z;
    const double length = std::sqrt(dx * dx + dy * dy + dz * dz);
    assert(length > 0.1);
    const tram_odometry::geo::Point3 base{
      a.x + 9.873 * dx / length,
      a.y + 9.873 * dy / length,
      a.z + 9.873 * dz / length - 3.0};
    const auto fixed = datum.enuToMgrs37Ucb(base);
    assert(std::abs(fixed.x - expected_x) < 0.01);
    assert(std::abs(fixed.y - expected_y) < 0.01);
    assert(std::abs(fixed.z - expected_z) < 0.01);
    std::cout << label << ": ENU " << std::fixed << std::setprecision(6)
              << base.x << ' ' << base.y << ' ' << base.z
              << "; CB " << fixed.x << ' ' << fixed.y << ' ' << fixed.z << '\n';
  };
  // Offline pyproj + route-map helper reference, using the same training map.
  print("out s=0", out, 103635.91506676021, 86057.51173933782, 165.3058973606676);
  print("return s=0", back, 99015.55252565537, 84949.12727011368, 171.3448272291571);
}
