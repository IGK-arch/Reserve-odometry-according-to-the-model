#include "tram_odometry/navigation.hpp"
#include <cassert>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>

int main() {
  using namespace tram_odometry;
  const auto path = std::filesystem::temp_directory_path() / "tram_navigation_test.csv";
  { std::ofstream f(path); f << "direction,s,x,y,z\nout,0,0,0,0\nout,1000,1000,0,0\n"; }
  NavigationConfig c;
  c.map_file = path.string(); c.output_projection = "enu";
  c.route_direction = "out"; c.enable_gnss_corrections = true;
  Navigation nav(EstimatorConfig{}, c);
  geo::EnuDatum datum({c.map_datum_lat_deg,c.map_datum_lon_deg,c.map_datum_alt_m});
  auto fix = [&](double x, double stamp) {
    auto p = datum.fromEnu({x,0,0});
    nav.submitFix(false,p.latitude_deg,p.longitude_deg,p.height_m,2,stamp);
  };
  assert(nav.submitVehicle('F',0,100));
  assert(!nav.output().position_valid);
  for(int i=0;i<3;++i) {
    nav.submitVehicle('C',0,100+.1*i); fix(0,100+.1*i);
  }
  nav.submitVehicle('C',0,100.3);
  assert(nav.output().position_valid);
  assert(std::abs(nav.output().position.x-9.873)<1e-3);
  assert(std::abs(nav.output().position.z+3)<1e-3);
  const double before = nav.output().position.x;
  // An isolated outlier must not drag the published pose along the route.
  nav.submitVehicle('C',0,101); fix(250,101);
  nav.submitVehicle('C',0,101.1);
  assert(std::abs(nav.output().position.x-before)<.01);
  // A coherent short GNSS window corrects accumulated along-track drift.
  for(int i=0;i<5;++i) {
    nav.submitVehicle('C',0,102+.1*i);
    nav.submitVehicle('F',0,102+.1*i); fix(8,102+.1*i);
  }
  nav.submitVehicle('C',0,102.6);
  assert(nav.output().gnss_corrections>0);
  assert(nav.output().position.x>before+1);
  assert(nav.output().position.x<before+8.01);
  assert(nav.output().estimate.velocity_mps==0);
  const auto corrections=nav.output().gnss_corrections;
  fix(10,100.1); fix(10,102.4); // stale/replayed fixes
  nav.submitVehicle('C',0,102.7);
  assert(nav.output().gnss_corrections==corrections);
  // Startup-only mode must ignore the same later window.
  c.enable_gnss_corrections=false;
  Navigation startup(EstimatorConfig{},c);
  startup.submitVehicle('F',0,100);
  for(int i=0;i<3;++i) {
    auto p=datum.fromEnu({0,0,0});
    startup.submitFix(false,p.latitude_deg,p.longitude_deg,p.height_m,2,100+.1*i);
  }
  for(int i=0;i<5;++i) {
    startup.submitVehicle('C',0,102+.1*i);
    auto p=datum.fromEnu({8,0,0});
    startup.submitFix(false,p.latitude_deg,p.longitude_deg,p.height_m,2,102+.1*i);
  }
  startup.submitVehicle('C',0,102.6);
  assert(startup.output().gnss_corrections==0);
  assert(std::abs(startup.output().position.x-9.873)<1e-3);
  std::filesystem::remove(path);
  std::cout << "PASS: navigation anchor, bounded GNSS correction, outliers, stale fixes\n";
}
