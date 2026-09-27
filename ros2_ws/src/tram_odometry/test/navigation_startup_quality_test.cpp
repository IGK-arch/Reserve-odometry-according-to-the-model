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
void near(double actual, double expected, const std::string& message) {
  require(std::abs(actual-expected)<.002, message+": actual="+std::to_string(actual));
}
struct Fixture {
  std::filesystem::path map=std::filesystem::temp_directory_path()/"tram_startup_quality.csv";
  NavigationConfig config;
  Fixture() {
    std::ofstream(map)<<"direction,s,x,y,z\nout,0,0,0,3\nout,1000,1000,0,3\n";
    config.map_file=map.string(); config.output_projection="enu";
    config.route_direction="out"; config.enable_gnss_corrections=false;
  }
  ~Fixture() {std::filesystem::remove(map);}
  void fix(Navigation& nav, bool rover, int status, Point3 point, double stamp) const {
    geo::EnuDatum datum({config.map_datum_lat_deg,config.map_datum_lon_deg,config.map_datum_alt_m});
    const auto p=datum.fromEnu({point.x,point.y,point.z});
    nav.submitFix(rover,p.latitude_deg,p.longitude_deg,p.height_m,status,stamp);
  }
  void observe(Navigation& nav, bool rover, int status, Point3 point, double stamp) const {
    nav.submitVehicle('C',0,stamp); fix(nav,rover,status,point,stamp);
    nav.submitVehicle('C',0,stamp+.001);
  }
  void series(Navigation& nav, bool rover, int status, double x, double start) const {
    for(int i=0;i<3;++i) observe(nav,rover,status,{x,0,3},start+.1*i);
  }
};
}

int main() {
  Fixture f;
  std::vector<std::pair<std::string,std::function<void()>>> tests;
  tests.push_back({"RTK arriving after low-quality fixes owns the anchor", [&] {
    for(int status:{0,1}) {
      Navigation nav(EstimatorConfig{},f.config); nav.submitVehicle('F',0,100);
      f.series(nav,false,status,70,100);
      require(!nav.output().anchored&&!nav.output().position_valid,"usable fixes must wait for RTK");
      f.series(nav,false,2,50,101);
      require(nav.output().anchored&&nav.output().position_valid,"sufficient RTK must release startup early");
      near(nav.output().position.x,59.873,"low-quality samples must not enter RTK median");
    }
  }});
  tests.push_back({"absent RTK falls back at the unchanged five-second deadline", [&] {
    for(int status:{0,1}) for(bool rover:{false,true}) {
      Navigation nav(EstimatorConfig{},f.config); nav.submitVehicle('F',0,100);
      f.series(nav,rover,status,rover?62.436:50,100);
      nav.submitVehicle('C',0,104.99);
      require(!nav.output().anchored&&!nav.output().position_valid,"fallback must wait through RTK window");
      nav.submitVehicle('C',0,105.001);
      require(nav.output().anchored&&nav.output().position_valid,"usable fallback must end bounded hold");
      near(nav.output().position.x,59.873,"fallback position");
      require(nav.output().anchor_source==(rover?"rover_fallback":"master"),"fallback source");
    }
  }});
  tests.push_back({"one RTK fix cannot promote a lower-quality window early", [&] {
    Navigation nav(EstimatorConfig{},f.config); nav.submitVehicle('F',0,100);
    f.observe(nav,false,0,{70,0,3},100);
    f.observe(nav,false,1,{70,0,3},100.1);
    f.observe(nav,false,2,{50,0,3},100.2);
    require(!nav.output().anchored,"one RTK fix is insufficient for early anchor");
    f.observe(nav,false,2,{50,0,3},100.3);
    require(!nav.output().anchored,"two RTK fixes must still obey startup_min_fixes");
    f.observe(nav,false,2,{50,0,3},100.4);
    require(nav.output().anchored,"third RTK fix releases startup");
    near(nav.output().position.x,59.873,"RTK-only anchor");
  }});
  tests.push_back({"sparse RTK remains usable at the deadline without mixing qualities", [&] {
    Navigation nav(EstimatorConfig{},f.config); nav.submitVehicle('F',0,100);
    f.series(nav,false,0,70,100);
    f.observe(nav,false,2,{50,0,3},101);
    nav.submitVehicle('C',0,104.99);
    require(!nav.output().anchored,"sparse RTK must wait for the deadline");
    nav.submitVehicle('C',0,105.001);
    require(nav.output().anchored,"deadline preserves sparse RTK fallback");
    near(nav.output().position.x,59.873,"sparse RTK excludes lower-quality median");
  }});
  tests.push_back({"low-quality rover cannot rotate an RTK master anchor", [&] {
    Navigation nav(EstimatorConfig{},f.config); nav.submitVehicle('F',0,100);
    for(int i=0;i<3;++i) {
      const double t=100+.1*i;
      nav.submitVehicle('C',0,t);
      f.fix(nav,true,0,{50,12.436,3},t);
      f.fix(nav,false,2,{50,0,3},t);
    }
    nav.submitVehicle('C',0,100.3);
    require(nav.output().anchored,"RTK master must anchor without RTK rover");
    require(nav.output().heading_source=="route_body_heading","low-quality rover cannot supply RTK heading");
    near(nav.output().yaw,0,"route heading"); near(nav.output().position.x,59.873,"unrotated lever arm");
  }});
  tests.push_back({"coherent RTK pair survives denser low-quality timestamps", [&] {
    Navigation nav(EstimatorConfig{},f.config); nav.submitVehicle('F',0,100);
    for(int i=0;i<3;++i) {
      const double t=101+.1*i; nav.submitVehicle('C',0,t);
      // A lower-quality payload with a newer stamp must not suppress valid RTK.
      f.fix(nav,true,0,{62.436,0,3},t+.02);
      f.fix(nav,false,1,{70,0,3},t+.02);
      f.fix(nav,true,2,{50,12.436,3},t);
      f.fix(nav,false,2,{50,0,3},t);
    }
    nav.submitVehicle('C',0,101.3);
    require(nav.output().anchored,"RTK timestamps must progress independently of low-quality fixes");
    require(nav.output().heading_source=="dual_antenna","coherent RTK pair supplies heading");
    near(nav.output().yaw,std::acos(-1.)/2,"RTK pair heading");
    near(nav.output().position.x,50,"RTK master location");
    near(nav.output().position.y,9.873,"RTK heading lever arm");
  }});
  tests.push_back({"RTK rover fallback is not blocked by low-quality master", [&] {
    Navigation nav(EstimatorConfig{},f.config); nav.submitVehicle('F',0,100);
    f.series(nav,false,0,70,100);
    f.series(nav,true,2,62.436,102);
    require(nav.output().anchored&&nav.output().anchor_source=="rover_fallback","RTK rover wins after rover delay");
    near(nav.output().position.x,59.873,"RTK rover anchor");
  }});
  tests.push_back({"RTK rover respects its delay with either antenna arrival order", [&] {
    for(bool rover_first:{false,true}) {
      Navigation nav(EstimatorConfig{},f.config); nav.submitVehicle('F',0,100);
      for(int i=0;i<3;++i) {
        const double t=100+.1*i; nav.submitVehicle('C',0,t);
        for(bool rover:{rover_first,!rover_first})
          f.fix(nav,rover,rover?2:0,{rover?62.436:70,0,3},t);
      }
      nav.submitVehicle('C',0,101.49);
      require(!nav.output().anchored,"RTK rover still waits for configured fallback delay");
      nav.submitVehicle('C',0,101.501);
      require(nav.output().anchored&&nav.output().anchor_source=="rover_fallback","waiting RTK rover releases after delay");
      near(nav.output().position.x,59.873,"arrival order cannot change rover anchor");
      near(nav.output().yaw,0,"lower-quality master cannot supply RTK rover heading");
    }
  }});
  tests.push_back({"stale RTK pair cannot provide startup heading", [&] {
    Navigation nav(EstimatorConfig{},f.config); nav.submitVehicle('F',0,100);
    f.observe(nav,true,2,{50,12.436,3},100);
    f.series(nav,false,2,50,101);
    require(nav.output().anchored,"current RTK master must anchor");
    require(nav.output().heading_source=="route_body_heading","old RTK rover cannot provide heading");
    near(nav.output().yaw,0,"stale pair preserves route heading");
  }});
  tests.push_back({"reset restarts quality evidence and deadline", [&] {
    Navigation nav(EstimatorConfig{},f.config); nav.submitVehicle('F',0,200);
    f.series(nav,false,2,50,200);
    require(nav.output().anchored,"initial run must anchor");
    nav.submitVehicle('F',0,100);
    f.series(nav,false,0,70,100);
    f.observe(nav,false,2,{80,0,3},100.3);
    require(!nav.output().anchored&&!nav.output().position_valid,"reset must discard old RTK evidence");
    f.observe(nav,false,2,{80,0,3},100.4);
    f.observe(nav,false,2,{80,0,3},100.5);
    require(nav.output().anchored,"fresh run RTK must anchor");
    near(nav.output().position.x,89.873,"fresh run position");
  }});
  tests.push_back({"reset discards unfinished RTK and lower-quality fallback windows", [&] {
    Navigation nav(EstimatorConfig{},f.config); nav.submitVehicle('F',0,200);
    f.observe(nav,false,2,{50,0,3},200);
    f.observe(nav,false,2,{50,0,3},200.1);
    f.observe(nav,true,0,{62.436,0,3},200.2);
    nav.submitVehicle('F',0,100);
    f.observe(nav,false,2,{80,0,3},100);
    require(!nav.output().anchored,"unfinished old RTK must not complete a new window");
    nav.submitVehicle('C',0,105.001);
    require(nav.output().anchored,"new run sparse fallback at new deadline");
    near(nav.output().position.x,89.873,"old windows must not bias fresh fallback");
  }});
  tests.push_back({"moving startup prefers RTK without losing wheel travel", [&] {
    Navigation nav(EstimatorConfig{},f.config); nav.submitVehicle('F',18,100);
    for(int i=0;i<30;++i) {
      const double t=100+.1*i;
      nav.submitVehicle('F',18,t); nav.submitVehicle('R',18,t);
      if(i<3) f.fix(nav,false,0,{70+nav.state().distance_m,0,3},t);
      if(i>=20&&i<23) f.fix(nav,false,2,{50+nav.state().distance_m,0,3},t);
    }
    nav.submitVehicle('C',0,103);
    require(nav.output().anchored,"moving RTK must anchor");
    near(nav.output().position.x,59.873+nav.state().distance_m,"RTK anchor preserves integrated distance");
  }});
  tests.push_back({"stale and future RTK cannot satisfy startup evidence", [&] {
    Navigation nav(EstimatorConfig{},f.config); nav.submitVehicle('F',0,100);
    nav.submitVehicle('C',0,101);
    f.fix(nav,false,2,{70,0,3},100.1); f.fix(nav,false,2,{70,0,3},102);
    f.series(nav,false,2,50,101);
    require(nav.output().anchored,"valid RTK must survive rejected timestamps");
    near(nav.output().position.x,59.873,"invalid timestamps cannot enter anchor");
    require(nav.output().gnss_rejected>=2,"stale/future fixes diagnosed");
  }});
  tests.push_back({"no GNSS still releases relative position after five seconds", [&] {
    Navigation nav(EstimatorConfig{},f.config); nav.submitVehicle('F',0,100);
    nav.submitVehicle('C',0,105.001);
    require(nav.output().position_valid&&!nav.output().anchored&&!nav.output().mapped,"no-GNSS relative fallback");
    require(nav.output().frame_id=="odom","relative frame preserved");
  }});
  int failed=0;
  for(const auto& test:tests) {
    try {test.second(); std::cout<<"PASS: "<<test.first<<'\n';}
    catch(const std::exception& e) {++failed; std::cerr<<"FAIL: "<<test.first<<": "<<e.what()<<'\n';}
  }
  std::cout<<tests.size()-failed<<'/'<<tests.size()<<" startup quality tests passed\n";
  return failed?1:0;
}
