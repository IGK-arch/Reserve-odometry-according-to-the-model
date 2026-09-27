from pathlib import Path
import subprocess
root=Path('/private/tmp/gnss-round2-candidates')
for name in ['baseline','geometry']:
 p=root/(name+'_anchor_probe'); h=p/'include/tram_odometry/navigation.hpp'
 s=h.read_text().replace(' double diagnosticAnchor() const', ''' double diagnosticHeadingDelta() const { return anchor_heading_delta_rad_; }
 std::array<double,4> diagnosticFix(bool rover,double latitude,double longitude,double altitude,double stamp) const {
   const auto state=estimator_.state();
   const double distance=state.distance_m-state.velocity_mps*(state.stamp_s-stamp);
   const auto point=projection_.project(latitude,longitude,altitude);
   const auto match=map_.nearestAntenna(point,selected_direction_,rover?-rover_to_master_s_m_:0,
     start_s_m_+distance,config_.gnss_correction_gate_m''' + (',config_.body_heading_lookahead_m' if name=='geometry' else '') + ''');
   return {state.stamp_s,distance,match.s-distance,match.distance_m};
 }
 double diagnosticAnchor() const''')
 h.write_text(s)
 q=p/'replay.cpp'; s=q.read_text().replace('residual_y,direction\\n','residual_y,direction,heading_delta,yaw,heading_source\\n')
 s=s.replace("<< navigation.diagnosticResidual().y << ',' << r.direction << '\\n';", "<< navigation.diagnosticResidual().y << ',' << r.direction << ',' << navigation.diagnosticHeadingDelta() << ',' << r.yaw << ',' << r.heading_source << '\\n';")
 s=s.replace('        navigation.submitFix(f[2]=="RF",', '''        const auto diagnostic=navigation.diagnosticFix(f[2]=="RF",std::stod(f[3]),std::stod(f[4]),std::stod(f[5]),time);
        std::cout << std::setprecision(17) << stamp << ',' << f[2] << ',' << diagnostic[0] << ',' << diagnostic[1] << ',' << diagnostic[2] << ',' << diagnostic[3] << '\\n';
        navigation.submitFix(f[2]=="RF",''')
 q.write_text(s)
 nav=root/'geometry/navigation.cpp' if name=='geometry' else Path('/private/tmp/odometry-round2-baseline/ros2_ws/src/tram_odometry/src/navigation.cpp')
 subprocess.run(['c++','-std=c++17','-O2','-I',str(p/'include'),str(q),str(nav),'/private/tmp/odometry-round2-baseline/ros2_ws/src/tram_odometry/src/estimator.cpp','-o',str(p/'replay')],check=True)
