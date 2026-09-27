// Independent geometric truth: no production map or bag-derived pose.
#include "tram_odometry/navigation.hpp"
#include <fstream>
#include <iostream>
#include <iomanip>
#include <filesystem>
using namespace tram_odometry;
void fix(Navigation& n,NavigationConfig& c,bool rover,Point3 p,int status,double t){geo::EnuDatum d({c.map_datum_lat_deg,c.map_datum_lon_deg,c.map_datum_alt_m});auto q=d.fromEnu({p.x,p.y,p.z});n.submitFix(rover,q.latitude_deg,q.longitude_deg,q.height_m,status,t);}
int main(int argc,char**argv){
 std::cout<<"geometry,rtk_sensor,scenario,position_error_m,heading_error_deg,heading_source,anchored,velocity,distance\n"<<std::setprecision(12);
 for(bool curve:{false,true}){
 auto path=std::filesystem::path("/private/tmp/round4-startup")/(curve?"synthetic_curve.csv":"synthetic_line.csv");
 {std::ofstream f(path);f<<"direction,s,x,y,z\n"<<std::setprecision(17);for(int i=0;i<=1800;++i){double s=i*.1,a=s/40;f<<"out,"<<s<<','<<(curve?40*std::sin(a):s)<<','<<(curve?40*(1-std::cos(a)):0)<<",3\n";}}
 for(bool rtk_rover:{false,true})for(std::string scenario:{"correct0","correct15","correct30","bad90","baseline9","baseline16","inconsistent","asynchronous","coherent4bias","common2bias","moving","no_map","sparse_master"}){
 NavigationConfig c;c.map_file=scenario=="no_map"?"":path.string();c.output_projection="enu";c.route_direction="out";c.enable_gnss_corrections=false;
 RouteMap map;map.load(path.string());double initial_s=60;Point3 initial_master=map.sample("out",initial_s).p;double route=map.bodyYaw("out",initial_s,c.body_heading_lookahead_m);
 double delta=(scenario=="correct30"?30:scenario=="correct0"?0:15)*std::acos(-1.)/180;double truth_yaw=route+delta;
 if(scenario=="no_map")c.relative_heading_rad=truth_yaw;
 Navigation n(EstimatorConfig{},c);double speed=scenario=="moving"?2:0;
 n.submitVehicle('F',3.6*speed,100);n.submitVehicle('R',3.6*speed,100);
 double anchor_time=0;Point3 truth;double error=0;
 for(int i=0;i<60;++i){double t=100+.1*i;n.submitVehicle('F',3.6*speed,t);n.submitVehicle('R',3.6*speed,t);n.submitVehicle('C',0,t);
 Point3 master=initial_master+Point3{std::cos(truth_yaw),std::sin(truth_yaw),0}*(speed*(t-100));double observed_yaw=truth_yaw;
 if(scenario=="bad90")observed_yaw=route+std::acos(-1.)/2;
 if(scenario=="coherent4bias")observed_yaw+=4*std::acos(-1.)/180;
 if(scenario=="inconsistent")observed_yaw+=(i%2?1:-1)*6*std::acos(-1.)/180;
 double length=scenario=="baseline9"?9:scenario=="baseline16"?16:12.436;
 Point3 rover=master+Point3{std::cos(observed_yaw),std::sin(observed_yaw),0}*length;
 // Keep selected RTK antenna truthful; the non-RTK companion is corrupted.
 if(rtk_rover){rover=master+Point3{std::cos(truth_yaw),std::sin(truth_yaw),0}*12.436;master=rover-Point3{std::cos(observed_yaw),std::sin(observed_yaw),0}*length;}
 if(scenario=="common2bias"){master.x+=2;rover.x+=2;}
 const double lowstamp=t-(scenario=="asynchronous"?.1:0);
 if(scenario=="sparse_master"){if(i==0)fix(n,c,false,master,2,t);fix(n,c,true,rover,2,t);}else if(rtk_rover){fix(n,c,false,master,0,lowstamp);fix(n,c,true,rover,2,t);}else{fix(n,c,true,rover,0,lowstamp);fix(n,c,false,master,2,t);}
 n.submitVehicle('C',0,t+.001);
 if(n.output().anchored){anchor_time=t;truth=initial_master+Point3{std::cos(truth_yaw),std::sin(truth_yaw),0}*(speed*(t+.001-100)+c.route_longitudinal_offset_m)+Point3{0,0,-3};break;}}
 auto o=n.output();error=std::hypot(o.position.x-truth.x,o.position.y-truth.y);double yaw_error=std::abs(std::atan2(std::sin(o.yaw-truth_yaw),std::cos(o.yaw-truth_yaw)))*180/std::acos(-1.);
 std::cout<<(curve?"curve":"line")<<','<<(rtk_rover?"rover":"master")<<','<<scenario<<','<<error<<','<<yaw_error<<','<<o.heading_source<<','<<o.anchored<<','<<o.estimate.velocity_mps<<','<<o.estimate.distance_m<<'\n';
 if(argc>1 && ((std::string(argv[1])=="mixed" && scenario=="correct15") || (std::string(argv[1])=="pure" && scenario=="sparse_master")) && error>.02)return 2;
 }
 }
}
