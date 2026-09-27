#include "tram_odometry/navigation.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
using namespace tram_odometry;
void require(bool value, const std::string& message) {
  if (!value) throw std::runtime_error(message);
}
void near(double actual, double expected, double tolerance, const std::string& message) {
  require(std::isfinite(actual) && std::abs(actual - expected) < tolerance,
          message + ": actual=" + std::to_string(actual) + " expected=" + std::to_string(expected));
}
struct Fixture {
  std::filesystem::path directory = std::filesystem::temp_directory_path() / "tram_gnss_robustness";
  NavigationConfig config;
  Fixture() {
    std::filesystem::create_directories(directory);
    const auto primary = directory / "primary.csv";
    const auto alternate = directory / "alternate.csv";
    std::ofstream(primary) << "direction,s,x,y,z\nout,0,0,0,0\nout,2000,2000,0,0\n";
    std::ofstream(alternate) << "direction,s,x,y,z\nout,0,0,20,0\nout,2000,2000,20,0\n";
    config.map_file = primary.string(); config.alternate_map_file = alternate.string();
    config.output_projection = "enu"; config.route_direction = "out";
    config.anchor_residual_decay_m = 0;
  }
  ~Fixture() { std::filesystem::remove_all(directory); }
  void fix(Navigation& nav, bool rover, double master_x, double stamp, double y=0) const {
    geo::EnuDatum datum({config.map_datum_lat_deg, config.map_datum_lon_deg, config.map_datum_alt_m});
    const auto p = datum.fromEnu({master_x + (rover ? 12.436 : 0), y, 0});
    nav.submitFix(rover, p.latitude_deg, p.longitude_deg, p.height_m, 2, stamp);
  }
  void anchor(Navigation& nav, double start=100) const {
    nav.submitVehicle('F', 0, start);
    for (int i=0; i<3; ++i) {
      nav.submitVehicle('C', 0, start + .1*i);
      fix(nav, false, 50, start + .1*i);
    }
    nav.submitVehicle('C', 0, start + .3);
    require(nav.output().anchored, "fixture must anchor");
    near(nav.output().position.x, 59.873, .001, "startup position");
  }
  void pair(Navigation& nav, double master, double rover_master, double stamp,
            bool rover_first=false, double master_y=0, double rover_y=0) const {
    nav.submitVehicle('C', 0, stamp);
    fix(nav, rover_first, rover_first ? rover_master : master, stamp,
        rover_first ? rover_y : master_y);
    fix(nav, !rover_first, rover_first ? master : rover_master, stamp,
        rover_first ? master_y : rover_y);
    nav.submitVehicle('C', 0, stamp+.001);
  }
};
}

int main() {
  Fixture f;
  std::vector<std::pair<std::string,std::function<void()>>> tests;
  for (bool corrupt_rover : {false,true}) for (bool rover_first : {false,true}) {
    tests.push_back({std::string("inconsistent pair rejects biased ") + (corrupt_rover?"rover":"master") +
                     (rover_first?" rover first":" master first"), [&,corrupt_rover,rover_first] {
      Navigation nav(EstimatorConfig{}, f.config); f.anchor(nav);
      const auto state = nav.state();
      for(int i=0;i<20;++i)
        f.pair(nav, corrupt_rover?50:62, corrupt_rover?62:50, 102+.1*i, rover_first);
      near(nav.output().position.x, 59.873, .01, "inconsistent antenna pair must not move pose");
      near(nav.state().velocity_mps, state.velocity_mps, 1e-12, "GNSS velocity isolation");
      near(nav.state().distance_m, state.distance_m, 1e-12, "GNSS distance isolation");
      require(nav.output().gnss_rejected>0, "inconsistent pair must be diagnosed");
    }});
  }
  tests.push_back({"pair conflict cannot select a contradictory branch", [&] {
    Navigation nav(EstimatorConfig{}, f.config); f.anchor(nav);
    for(int i=0;i<10;++i) f.pair(nav, 50, 50, 102+.1*i, false, 0, 20);
    require(nav.output().branch_switches==0, "inconsistent pair must not identify a branch");
    near(nav.output().position.y, 0, .001, "contradictory branch preserves primary");
  }});
  tests.push_back({"consistent pair still corrects a large longitudinal innovation", [&] {
    Navigation nav(EstimatorConfig{}, f.config); f.anchor(nav);
    for(int i=0;i<20;++i) f.pair(nav, 70, 70, 102+.1*i, i%2==0);
    near(nav.output().position.x, 79.873, .02, "20m genuine correction must remain available");
    near(nav.state().distance_m, 0, 1e-12, "correction must not enter wheel distance");
  }});
  tests.push_back({"healthy pair and lone antenna recover after pair conflict", [&] {
    for(bool paired : {false,true}) {
      Navigation nav(EstimatorConfig{}, f.config); f.anchor(nav);
      for(int i=0;i<8;++i) f.pair(nav, 50, 62, 102+.1*i);
      for(int i=0;i<20;++i) {
        if(paired) f.pair(nav, 58, 58, 104+.1*i);
        else {
          nav.submitVehicle('C',0,104+.1*i); f.fix(nav,true,58,104+.1*i);
          nav.submitVehicle('C',0,104+.1*i+.001);
        }
      }
      near(nav.output().position.x, 67.873, .02, "fresh fixes must recover after conflict");
    }
  }});
  tests.push_back({"consistent pair retains late branch selection", [&] {
    Navigation nav(EstimatorConfig{}, f.config); f.anchor(nav);
    for(int i=0;i<4;++i) f.pair(nav,50,50,102+.1*i,false,20,20);
    require(nav.output().branch_switches==1, "consistent alternate pair must switch");
    near(nav.output().position.y,20,.001,"alternate geometry");
  }});
  tests.push_back({"single antenna repairs accumulated wheel scale drift", [&] {
    auto ec=EstimatorConfig{}; ec.front_scale=1.02; ec.rear_scale=1.02;
    Navigation corrected(ec,f.config), blackout(ec,f.config);
    f.anchor(corrected); f.anchor(blackout);
    double truth=50, previous_speed=0;
    for(int i=1;i<=1200;++i) {
      const double stamp=100.3+.1*i;
      const double speed=std::min(5.0,.05*i);
      for(auto* nav:{&corrected,&blackout}) {
        nav->submitVehicle('F',3.6*speed,stamp); nav->submitVehicle('R',3.6*speed,stamp);
        nav->submitVehicle('C',0,stamp+.001);
      }
      truth+=.05*(previous_speed+speed); previous_speed=speed;
      if(i>1180) f.fix(corrected,true,truth,stamp);
    }
    corrected.submitVehicle('C',0,220.401); blackout.submitVehicle('C',0,220.401);
    require(std::abs(blackout.output().position.x-(truth+9.873))>5,"fixture must accumulate drift");
    require(std::abs(corrected.output().position.x-(truth+9.873))<1,
            "late lone-antenna window must repair genuine accumulated drift");
    near(corrected.state().velocity_mps,blackout.state().velocity_mps,1e-12,"same wheel speed");
    near(corrected.state().distance_m,blackout.state().distance_m,1e-12,"same wheel distance");
  }});
  tests.push_back({"tentative shared wheel fault cannot veto stationary GNSS", [&] {
    Navigation nav(EstimatorConfig{}, f.config); f.anchor(nav);
    for(int i=1;i<=10;++i) {
      const double stamp=100.3+.1*i;
      nav.submitVehicle('C',0,stamp);
      nav.submitVehicle('F',0,stamp); nav.submitVehicle('R',0,stamp);
    }
    for(int i=1;i<=80;++i) {
      const double stamp=101.3+.1*i;
      nav.submitVehicle('C',0,stamp);
      nav.submitVehicle('F',14.4,stamp); nav.submitVehicle('R',14.4,stamp);
    }
    require(nav.state().front_tentative && nav.state().rear_tentative,
            "persistent common jump must enter explicitly tentative recovery");
    const auto before=nav.output().gnss_corrections;
    for(int i=1;i<=20;++i) {
      const double stamp=109.3+.1*i;
      nav.submitVehicle('C',0,stamp);
      nav.submitVehicle('F',14.4,stamp); nav.submitVehicle('R',14.4,stamp);
      f.fix(nav,true,50,stamp); // Real tram is stationary; wheel pair is wrong.
      nav.submitVehicle('C',0,stamp+.001);
    }
    require(nav.output().gnss_corrections>=before+5,
            "uncertain wheels cannot classify unchanged true GNSS as frozen");
    near(nav.output().position.x,59.873,2.,"stationary GNSS still controls route anchor");
  }});
  tests.push_back({"timestamp skew during fast motion does not look like pair distortion", [&] {
    Navigation nav(EstimatorConfig{}, f.config);
    nav.submitVehicle('F',72,100);
    for(int i=0;i<3;++i) {
      const double stamp=100+.1*i;
      nav.submitVehicle('F',72,stamp); nav.submitVehicle('R',72,stamp);
      f.fix(nav,false,50+nav.state().distance_m,stamp);
    }
    for(int i=1;i<=30;++i) {
      const double stamp=100.2+.1*i;
      nav.submitVehicle('F',72,stamp); nav.submitVehicle('R',72,stamp);
      f.fix(nav,false,50+nav.state().distance_m,stamp);
      if(i>2) f.fix(nav,true,50+nav.state().distance_m-3.8,stamp-.19);
      nav.submitVehicle('C',0,stamp+.001);
    }
    require(nav.output().gnss_corrections>0,"healthy skewed observations must still correct");
    near(nav.output().position.x,59.873+nav.state().distance_m,.2,"skewed fixes preserve travel");
  }});
  int failed=0;
  for(const auto& test:tests) {
    try {test.second(); std::cout<<"PASS: "<<test.first<<'\n';}
    catch(const std::exception& e) {++failed; std::cerr<<"FAIL: "<<test.first<<": "<<e.what()<<'\n';}
  }
  std::cout<<tests.size()-failed<<'/'<<tests.size()<<" GNSS robustness tests passed\n";
  return failed?1:0;
}
