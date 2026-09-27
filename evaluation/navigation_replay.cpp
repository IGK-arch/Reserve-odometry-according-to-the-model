// Full causal pipeline replay. Only allowed input channels are accepted.
#include "tram_odometry/navigation.hpp"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <vector>
int main(int argc,char **argv) {
  try {
    tram_odometry::NavigationConfig config;
    int vehicle=30618; bool table=true;
    std::string input,output,table_path;
    for(int i=1;i<argc;++i) {
      const std::string key=argv[i];
      if (i+1>=argc) throw std::invalid_argument("Missing argument value: "+key);
      const std::string value=argv[++i];
      if(key=="--input") input=value;
      else if(key=="--output") output=value;
      else if(key=="--vehicle") vehicle=std::stoi(value);
      else if(key=="--map") config.map_file=value;
      else if(key=="--alternate-map") config.alternate_map_file=value;
      else if(key=="--stops") {config.stop_landmarks_file=value; config.enable_stop_landmarks=true;}
      else if(key=="--elevation") config.elevation_file=value;
      else if(key=="--drive-table") table_path=value;
      else if(key=="--table") table=std::stoi(value)!=0;
      else if(key=="--direction") config.route_direction=value;
      else if(key=="--body-heading-lookahead") config.body_heading_lookahead_m=std::stod(value);
      else if(key=="--gnss-mode") {
        if(value!="off" && value!="startup" && value!="corrections") throw std::invalid_argument("Invalid GNSS mode");
        config.use_startup_gnss=value!="off";
        config.enable_gnss_corrections=value=="corrections";
      } else throw std::invalid_argument("Unknown argument: "+key);
    }
    if(input.empty() || output.empty()) throw std::invalid_argument("--input and --output required");
    auto ec=tram_odometry::EstimatorConfig::forVehicle(vehicle);
    ec.enable_drive_table=table && vehicle==30618;
    if(ec.enable_drive_table && table_path.empty())
      throw std::invalid_argument("Provide --drive-table PATH or explicitly choose --table 0 (physics)");
    ec.drive_table_path=table_path;
    tram_odometry::Navigation navigation(ec,config);
    std::ifstream in(input); std::ofstream out(output);
    if(!in || !out) throw std::runtime_error("Cannot open input/output CSV");
    out << "stamp_ns,receive_ns,velocity_mps,distance_m,position_valid,x,y,z,front_slip,rear_slip,model_only,gnss_corrections,gnss_rejected,branch_switches,branch,clamped,frame_id,mapped,anchored,stop_corrections\n" << std::setprecision(17);
    std::string line; size_t number=0;
    while(std::getline(in,line)) {
      ++number; if(line.empty()) continue;
      std::stringstream stream(line); std::vector<std::string> f; std::string token;
      while(std::getline(stream,token,',')) f.push_back(token);
      if(number==1 && !f.empty() && f[0]=="receive_ns") continue;
      if(f.size()<4) throw std::runtime_error("Malformed input line "+std::to_string(number));
      const auto receive=std::stoll(f[0]),stamp=std::stoll(f[1]);
      const double time=static_cast<double>(stamp)*1e-9;
      if(f[2]=="MF" || f[2]=="RF") {
        if(f.size()!=7) throw std::runtime_error("Fix needs lat,lon,alt,status");
        navigation.submitFix(f[2]=="RF",std::stod(f[3]),std::stod(f[4]),std::stod(f[5]),std::stoi(f[6]),time);
      } else if(f[2]=="MV" || f[2]=="RV") {
        continue; // Permitted but currently unused; never substitute reference velocity.
      } else if(f[2]=="C" || f[2]=="F" || f[2]=="R") {
        if(!navigation.submitVehicle(f[2][0],std::stod(f[3]),time)) continue;
        const auto & r=navigation.output(); const auto & e=r.estimate;
        out << stamp << ',' << receive << ',' << e.velocity_mps << ',' << e.distance_m << ','
            << r.position_valid << ',' << r.position.x << ',' << r.position.y << ',' << r.position.z << ','
            << e.front_slip << ',' << e.rear_slip << ',' << e.model_only << ',' << r.gnss_corrections << ','
            << r.gnss_rejected << ',' << r.branch_switches << ',' << r.branch << ',' << r.clamped << ','
            << r.frame_id << ',' << r.mapped << ',' << r.anchored << ',' << r.stop_corrections << '\n';
      } else throw std::invalid_argument("Forbidden/unknown input channel: "+f[2]);
    }
    if(!out) throw std::runtime_error("Output write failed");
  } catch(const std::exception & e) {std::cerr<<e.what()<<'\n';return 1;}
}
