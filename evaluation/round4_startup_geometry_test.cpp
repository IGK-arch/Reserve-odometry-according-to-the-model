// Regression first fails against baseline: sparse master RTK permits a rover
// anchor at the deadline, but both lever arms must use the observed body yaw.
#include "tram_odometry/navigation.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <cmath>
using namespace tram_odometry;
void require(bool b,const char* m){if(!b)throw std::runtime_error(m);}
void fix(Navigation& n,NavigationConfig& c,bool rover,Point3 p,double t){geo::EnuDatum d({c.map_datum_lat_deg,c.map_datum_lon_deg,c.map_datum_alt_m});auto q=d.fromEnu({p.x,p.y,p.z});n.submitFix(rover,q.latitude_deg,q.longitude_deg,q.height_m,2,t);}
int main(){for(double grade:{0.,.6})for(double heading:{-.35,.25})for(int vehicle:{30618,30639}){
 auto path=std::filesystem::path("/private/tmp/round4-startup/geometric_grade.csv");double h=std::sqrt(1-grade*grade);{std::ofstream f(path);f<<"direction,s,x,y,z\nout,0,0,0,3\nout,250,"<<250*h<<",0,"<<3+250*grade<<'\n';}
 NavigationConfig c;c.map_file=path.string();c.route_direction="out";c.output_projection="enu";c.enable_gnss_corrections=false;
 Navigation n(EstimatorConfig::forVehicle(vehicle),c);Point3 master{60*h,0,3+60*grade},body{h*std::cos(heading),h*std::sin(heading),grade};Point3 rover=master+body*12.436;
 n.submitVehicle('F',0,100);fix(n,c,false,master,100);fix(n,c,true,rover,100);
 for(int i=1;i<=51;++i){double t=100+.1*i;n.submitVehicle('F',0,t);n.submitVehicle('R',0,t);n.submitVehicle('C',0,t);fix(n,c,true,rover,t);}
 n.submitVehicle('C',0,105.2);const auto o=n.output();require(o.anchor_source=="rover_fallback","must exercise deadline rover selection with sparse RTK master");require(o.heading_source=="dual_antenna","must preserve strict dual-RTK course");Point3 expected=rover+body*(c.route_longitudinal_offset_m-12.436)+Point3{0,0,-3};double error=std::hypot(o.position.x-expected.x,o.position.y-expected.y);std::cout<<vehicle<<" grade "<<grade<<" heading "<<heading<<" XY error "<<error<<'\n';require(error<1e-6,"rover and base_link lever arms must use same measured body course");require(n.state().velocity_mps==0&&n.state().distance_m==0,"geometry must not modify longitudinal state");
 }
}
