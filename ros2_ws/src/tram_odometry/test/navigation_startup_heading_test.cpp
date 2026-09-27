#include "tram_odometry/navigation.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>

namespace {
using namespace tram_odometry;
void require(bool condition,const char* message) {
  if(!condition) throw std::runtime_error(message);
}
double norm(const Point3& p) {return std::sqrt(p.x*p.x+p.y*p.y+p.z*p.z);}
void fix(Navigation& nav,const NavigationConfig& c,bool rover,const Point3& p,double stamp) {
  geo::EnuDatum datum({c.map_datum_lat_deg,c.map_datum_lon_deg,c.map_datum_alt_m});
  const auto llh=datum.fromEnu({p.x,p.y,p.z});
  nav.submitFix(rover,llh.latitude_deg,llh.longitude_deg,llh.height_m,2,stamp);
}
void anchor(Navigation& nav,const NavigationConfig& c,const Point3& master,
            const Point3& body,bool paired) {
  nav.submitVehicle('F',0,100);
  for(int i=0;i<3;++i) {
    const double t=100+.1*i;
    nav.submitVehicle('C',0,t);
    if(paired) fix(nav,c,true,master+body*12.436,t);
    fix(nav,c,false,master,t);
  }
  nav.submitVehicle('C',0,100.3);
  require(nav.output().anchored,"fixture must anchor");
  require(!paired||nav.output().heading_source=="dual_antenna","paired heading must be available");
}
Point3 impliedMaster(const NavigationOutput& r,double forward) {
  return r.position-Point3{forward*std::cos(r.yaw),forward*std::sin(r.yaw),-3};
}
}

int main() {
  using namespace tram_odometry;
  const auto dir=std::filesystem::temp_directory_path()/"tram_startup_heading_test";
  std::filesystem::create_directories(dir);
  const auto curve=dir/"curve.csv";
  constexpr double radius=32,behind=9.873-7.55/2,angle=1.2;
  const double master_radius=std::hypot(radius,behind);
  {
    std::ofstream file(curve);file<<"direction,s,x,y,z\n"<<std::setprecision(17);
    for(int i=0;i<=1800;++i) {
      const double a=.002*i;
      file<<"out,"<<master_radius*a<<','<<radius*std::cos(a)+behind*std::sin(a)<<','
          <<radius*std::sin(a)-behind*std::cos(a)<<",3\n";
    }
  }
  NavigationConfig c;c.map_file=curve.string();c.output_projection="enu";
  c.route_direction="out";c.anchor_residual_decay_m=20;c.enable_gnss_corrections=false;
  const Point3 on_map{radius*std::cos(angle)+behind*std::sin(angle),
                       radius*std::sin(angle)-behind*std::cos(angle),3};
  // A parked body need not share the learned route's local direction. The
  // independently specified pair tells us its actual heading and lever arm.
  const Point3 master=on_map+Point3{on_map.x/master_radius,on_map.y/master_radius,0}*1.75;
  const double heading=angle+std::acos(-1.)/2+.4;
  const Point3 body{std::cos(heading),std::sin(heading),0};
  Navigation nav(EstimatorConfig{},c);anchor(nav,c,master,body,true);
  const Point3 expected=master+body*c.route_longitudinal_offset_m+Point3{0,0,-3};
  const double stationary_error=norm(nav.output().position-expected);
  std::cout<<"paired stationary curve position error "<<stationary_error<<" m\n";
  require(stationary_error<.002,"paired startup heading must rotate the position lever arm as well as yaw");
  require(std::abs(std::atan2(std::sin(nav.output().yaw-heading),std::cos(nav.output().yaw-heading)))<1e-6,
          "startup yaw must equal observed paired heading");

  RouteMap map;require(map.load(curve.string()),"load known route");
  const auto match=map.nearest(master,"out");
  const auto residual=master-map.sample("out",match.s).p;
  const double initial_heading_error=std::atan2(std::sin(heading-map.bodyYaw("out",match.s,c.body_heading_lookahead_m)),
                                               std::cos(heading-map.bodyYaw("out",match.s,c.body_heading_lookahead_m)));
  for(int i=1;i<=300;++i) {
    const double t=100.3+.1*i;
    const double speed=std::min(2.,.02*i);
    nav.submitVehicle('F',3.6*speed,t);nav.submitVehicle('R',3.6*speed,t);
    nav.submitVehicle('C',0,t+.001);
    const auto& r=nav.output();const double distance=nav.state().distance_m;
    const double weight=std::exp(-distance/c.anchor_residual_decay_m);
    const Point3 expected_master=map.sample("out",match.s+distance).p+Point3{residual.x*weight,residual.y*weight,0};
    const Point3 offset=r.position-expected_master+Point3{0,0,3};
    require(std::abs(norm(offset)-c.route_longitudinal_offset_m)<.002,"rotation must preserve rigid lever-arm length");
    require(std::hypot(offset.x-c.route_longitudinal_offset_m*std::cos(r.yaw),
                       offset.y-c.route_longitudinal_offset_m*std::sin(r.yaw))<.002,
            "position and yaw must use the same decaying startup heading");
    const double desired_yaw=map.bodyYaw("out",match.s+distance,c.body_heading_lookahead_m)+initial_heading_error*weight;
    require(std::abs(std::atan2(std::sin(r.yaw-desired_yaw),std::cos(r.yaw-desired_yaw)))<1e-6,
            "heading correction must be applied once and decay with travelled distance");
  }
  require(nav.state().distance_m>40,"fixture must exercise heading decay over a travelled route");

  // GNSS observations remain absolute antenna measurements. The output
  // extrinsic must not shift their master-route anchor or wheel integration.
  c.enable_gnss_corrections=true;
  Navigation paired(EstimatorConfig{},c),single(EstimatorConfig{},c);
  anchor(paired,c,master,body,true);anchor(single,c,master,body,false);
  for(int i=0;i<20;++i) {
    const double t=102+.1*i;
    for(auto* n:{&paired,&single}) {
      n->submitVehicle('C',0,t);fix(*n,c,false,master,t);fix(*n,c,true,master+body*12.436,t);
      n->submitVehicle('C',0,t+.001);
    }
    require(norm(impliedMaster(paired.output(),c.route_longitudinal_offset_m)-
                 impliedMaster(single.output(),c.route_longitudinal_offset_m))<.002,
            "heading extrinsic must not be applied again to the master-route anchor");
    require(paired.state().velocity_mps==single.state().velocity_mps&&
            paired.state().distance_m==single.state().distance_m,
            "heading and GNSS corrections must not alter wheel state");
    require(paired.output().gnss_corrections==single.output().gnss_corrections,
            "paired startup heading must not change subsequent antenna-match acceptance");
  }

  require(paired.output().gnss_corrections>0,"matching isolation fixture must exercise accepted GNSS corrections");

  const auto grade=dir/"grade.csv";
  {std::ofstream file(grade);file<<"direction,s,x,y,z\nout,0,0,0,0\nout,250,200,0,150\n";}
  c.map_file=grade.string();c.enable_gnss_corrections=false;
  Navigation graded(EstimatorConfig{},c);
  const Point3 graded_master{40,0,30},graded_body{.8*std::cos(.3),.8*std::sin(.3),.6};
  anchor(graded,c,graded_master,graded_body,true);
  require(norm(graded.output().position-(graded_master+graded_body*c.route_longitudinal_offset_m+Point3{0,0,-3}))<.002,
          "yaw correction must preserve slope and apply antenna height once");
  std::filesystem::remove_all(dir);
  std::cout<<"PASS: paired startup heading, decaying lever arm, matching isolation and grade\n";
}
