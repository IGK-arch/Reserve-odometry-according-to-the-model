#include "tram_odometry/navigation.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>

namespace {
using namespace tram_odometry;
void require(bool value, const char* message) {
  if (!value) throw std::runtime_error(message);
}
void fix(Navigation& nav, const NavigationConfig& config, bool rover,
         const Point3& point, double time) {
  geo::EnuDatum datum({config.map_datum_lat_deg, config.map_datum_lon_deg,
                       config.map_datum_alt_m});
  const auto llh = datum.fromEnu({point.x, point.y, point.z});
  nav.submitFix(rover, llh.latitude_deg, llh.longitude_deg, llh.height_m, 2, time);
}

void checkGradedTrack(double grade, double heading, int vehicle) {
  const auto path = std::filesystem::temp_directory_path() / "tram_rover_anchor_grade.csv";
  const double horizontal = std::sqrt(1 - grade * grade);
  {
    std::ofstream file(path);
    file << std::setprecision(17)
         << "direction,s,x,y,z\nout,0,0,0,3\nout,250,"
         << 250 * horizontal << ",0," << 3 + 250 * grade << '\n';
  }
  NavigationConfig config;
  config.map_file = path.string();
  config.route_direction = "out";
  config.output_projection = "enu";
  config.enable_gnss_corrections = false;
  Navigation nav(EstimatorConfig::forVehicle(vehicle), config);
  const Point3 master{60 * horizontal, 0, 3 + 60 * grade};
  const Point3 body{horizontal * std::cos(heading), horizontal * std::sin(heading), grade};
  const Point3 rover = master + body * 12.436;
  nav.submitVehicle('F', 0, 100);
  fix(nav, config, false, master, 100);
  fix(nav, config, true, rover, 100);
  for (int i = 1; i <= 51; ++i) {
    const double time = 100 + .1 * i;
    nav.submitVehicle('F', 0, time);
    nav.submitVehicle('R', 0, time);
    nav.submitVehicle('C', 0, time);
    fix(nav, config, true, rover, time);
  }
  nav.submitVehicle('C', 0, 105.2);
  const auto output = nav.output();
  require(output.anchor_source == "rover_fallback", "graded case must select rover");
  require(output.heading_source == "dual_antenna", "graded case needs dual-RTK course");
  const Point3 expected = rover + body * (config.route_longitudinal_offset_m - 12.436)
                         + Point3{0, 0, -3};
  const double error = std::hypot(output.position.x - expected.x, output.position.y - expected.y);
  std::filesystem::remove(path);
  std::cout << vehicle << " grade " << grade << " heading " << heading
            << " XY error " << error << " m\n";
  require(error < 1e-6, "graded antenna offsets must preserve observed body course");
  require(nav.state().velocity_mps == 0 && nav.state().distance_m == 0,
          "geometry must preserve longitudinal state");
}
}

int main() {
  using namespace tram_odometry;
  const auto path = std::filesystem::temp_directory_path() / "tram_rover_anchor_curve.csv";
  constexpr double radius = 32, behind = 6.098, angle = 1.2;
  const double master_radius = std::hypot(radius, behind);
  {
    std::ofstream file(path);
    file << "direction,s,x,y,z\n" << std::setprecision(17);
    for (int i = 0; i <= 1800; ++i) {
      const double a = .002 * i;
      file << "out," << master_radius * a << ','
           << radius * std::cos(a) + behind * std::sin(a) << ','
           << radius * std::sin(a) - behind * std::cos(a) << ",3\n";
    }
  }
  NavigationConfig config;
  config.map_file = path.string();
  config.route_direction = "out";
  config.output_projection = "enu";
  config.enable_gnss_corrections = false;
  const Point3 master{radius * std::cos(angle) + behind * std::sin(angle),
                      radius * std::sin(angle) - behind * std::cos(angle), 3};
  const double heading = angle + std::acos(-1.) / 2 + .35;
  const Point3 forward{std::cos(heading), std::sin(heading), 0};
  const Point3 rover = master + forward * 12.436;
  const Point3 expected = master + forward * 9.873 + Point3{0, 0, -3};
  Navigation nav(EstimatorConfig{}, config);
  nav.submitVehicle('F', 0, 100);
  // One early RTK master blocks an early rover fallback, but cannot itself
  // satisfy startup_min_fixes. At the deadline the full rover window wins.
  fix(nav, config, false, master, 100.1);
  for (int i = 0; i < 3; ++i) {
    const double time = 100 + .1 * i;
    nav.submitVehicle('C', 0, time);
    fix(nav, config, true, rover, time);
  }
  nav.submitVehicle('C', 0, 104.99);
  require(!nav.output().anchored, "sparse master must retain bounded startup waiting");
  nav.submitVehicle('C', 0, 105.001);
  const auto output = nav.output();
  require(output.anchored && output.anchor_source == "rover_fallback",
          "complete rover window must be selected at the deadline");
  require(output.heading_source == "dual_antenna", "coherent RTK pair supplies course");
  const auto delta = output.position - expected;
  const double error = std::sqrt(delta.x * delta.x + delta.y * delta.y + delta.z * delta.z);
  std::filesystem::remove(path);
  std::cout << "RTK rover rigid-body startup error: " << error << " m\n";
  require(error < .002, "rover-to-master and master-to-base must use the same observed body course");
  for (double grade : {0., .6})
    for (double course : {-.35, .25})
      for (int vehicle : {30618, 30639}) checkGradedTrack(grade, course, vehicle);
}
