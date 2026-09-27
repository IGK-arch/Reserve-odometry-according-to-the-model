#include "tram_odometry/navigation.hpp"
#include "tram_odometry/geo_projection.hpp"
#include <cassert>
#include <cmath>
#include <iostream>

int main(int argc,char** argv) {
  assert(argc==3);
  using namespace tram_odometry;
  RouteMap route;
  assert(route.load(argv[1]));
  const auto antenna=route.sample("out",5388.0).p;
  geo::EnuDatum datum({55.810367065,37.462266845,168.3794});
  const auto geodetic=datum.fromEnu({antenna.x,antenna.y,antenna.z});
  NavigationConfig config;
  config.map_file=argv[1]; config.alternate_map_file=argv[2];
  config.route_direction="out"; config.enable_gnss_corrections=false;
  config.startup_min_fixes=3; config.enable_stop_landmarks=false;
  auto estimator_config=EstimatorConfig::forVehicle(30618);
  estimator_config.enable_drive_table=false;
  Navigation nav(estimator_config,config);
  const double base=1000.0;
  for(int i=0;i<5;++i) {
    const double t=base+0.05*i;
    nav.submitVehicle('C',0,t);
    nav.submitVehicle('F',0,t+0.001);
    nav.submitVehicle('R',0,t+0.002);
    if(i<3) nav.submitFix(false,geodetic.latitude_deg,geodetic.longitude_deg,
                         geodetic.height_m,2,t+0.003);
  }
  assert(nav.output().anchored);
  Point3 previous=nav.output().position;
  double previous_stamp=nav.output().estimate.stamp_s;
  int switches=0;
  double switch_step=INFINITY,max_step=0;
  for(int i=0;i<140;++i) {
    const double t=base+0.3+0.05*i;
    const double v=std::min(4.6,0.8+0.9*0.05*i);
    nav.submitVehicle('C',12,t);
    nav.submitVehicle('F',v*3.6,t+0.001);
    if(!nav.submitVehicle('R',v*3.6,t+0.002)) continue;
    const auto & out=nav.output();
    const double dt=out.estimate.stamp_s-previous_stamp;
    const auto delta=out.position-previous;
    const double step=std::sqrt(delta.x*delta.x+delta.y*delta.y+delta.z*delta.z);
    if(dt>0 && dt<0.06) max_step=std::max(max_step,step);
    if(out.branch_switches>static_cast<size_t>(switches)) {
      ++switches; switch_step=step;
    }
    previous=out.position;
    previous_stamp=out.estimate.stamp_s;
  }
  std::cout << "switches=" << switches << " switch_step=" << switch_step
            << " max_step=" << max_step << " branch=" << nav.output().branch
            << " distance=" << nav.output().estimate.distance_m << '\n';
  assert(switches==1);
  assert(nav.output().branch=="alternate");
  assert(switch_step<0.5);
  assert(max_step<0.5);
  nav.submitVehicle('F',0,base-100);
  assert(nav.output().branch=="primary");
  assert(nav.output().branch_switches==0);
  return 0;
}
