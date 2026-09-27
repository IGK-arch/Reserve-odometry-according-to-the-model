#!/usr/bin/env python3
"""Reproducible scratch startup variants from immutable baseline; no source install."""
from pathlib import Path
import argparse, subprocess, hashlib,json
ROOT=Path(__file__).resolve().parents[1]
BASE=Path('/private/tmp/odometry-round4-source')
OUT=Path('/private/tmp/round4-startup')
NAV='ros2_ws/src/tram_odometry/src/navigation.cpp'
MIXED=r'''
    // Experimental: position retains RTK priority. Mixed course requires
    // synchronized independent pairs, known length and temporal agreement.
    bool mixed_heading = false;
    if (rtk && !paired_heading_valid && has_map_) {
      const auto & hm = master_fixes.empty() ? startup_fixes_ : master_fixes;
      const auto & hr = rover_fixes.empty() ? rover_startup_fixes_ : rover_fixes;
      std::vector<double> angles, times;
      double last_rover = -INFINITY;
      for (const auto & m : hm) {
        const StartupFix * r = nullptr;
        double dt_best = 0.075;
        for (const auto & q : hr) {
          const double dt = std::abs(q.stamp_s-m.stamp_s);
          if (q.stamp_s > last_rover && dt < dt_best) {r=&q;dt_best=dt;}
        }
        if (!r) continue;
        last_rover=r->stamp_s;
        const double dx=r->enu.x-m.enu.x, dy=r->enu.y-m.enu.y;
        if (std::abs(std::hypot(dx,dy)-std::abs(rover_to_master_s_m_)) > 0.75) continue;
        angles.push_back(std::atan2(dy,dx));times.push_back(m.stamp_s);
      }
      if (angles.size() >= 3 && times.back()-times.front() >= 0.15) {
        double sx=0,sy=0;
        for(double a:angles) {sx+=std::cos(a);sy+=std::sin(a);}
        const double a=std::atan2(sy,sx);
        double spread=0;
        for(double q:angles) spread=std::max(spread,std::abs(std::atan2(std::sin(q-a),std::cos(q-a))));
        double alignment=INFINITY;
        for(const std::string d:{std::string("out"),std::string("return")}) {
          if(configured_direction_!="auto" && configured_direction_!=d) continue;
          auto match=map_.nearest(gnss_enu,d);
          if(match.direction.empty()) continue;
          if(rover) match=map_.nearestAntenna(gnss_enu,d,-rover_to_master_s_m_,
            match.s+rover_to_master_s_m_,map_match_max_distance_m_+std::abs(rover_to_master_s_m_),config_.body_heading_lookahead_m);
          const double delta=a-map_.bodyYaw(d,match.s,config_.body_heading_lookahead_m);
          alignment=std::min(alignment,std::abs(std::atan2(std::sin(delta),std::cos(delta))));
        }
        if(spread <= 3.0*std::acos(-1.0)/180 && alignment <= 35.0*std::acos(-1.0)/180) {
          paired_heading_rad=a;paired_heading_valid=true;mixed_heading=true;
          relative_heading_rad_=a;heading_source_="mixed_antenna_constrained";
        }
      }
    }
'''
RIGID=r'''
      // Preserve the measured rover rigid pose when observed course differs
      // from route course. The output lever is expressed from the master.
      if (mixed_heading && rover) {
        const Point3 arm=map_.antennaPosition(selected_direction_,match.s,-rover_to_master_s_m_,config_.body_heading_lookahead_m)-map_.sample(selected_direction_,match.s).p;
        const double delta=paired_heading_rad-route_yaw;
        anchor_residual_.x += arm.x-(std::cos(delta)*arm.x-std::sin(delta)*arm.y);
        anchor_residual_.y += arm.y-(std::sin(delta)*arm.x+std::cos(delta)*arm.y);
      }
'''
JOINT=r'''
      if (mixed_heading) {
        const auto & other = rover ? startup_fixes_ : rover_startup_fixes_;
        if(!other.empty()) {
          const auto & q=other[other.size()/2];
          // RTK variance 0.04 versus non-RTK 1.0 m^2, fixed a priori.
          const double weight=0.04/1.04;
          const double arm=rover ? -rover_to_master_s_m_ : rover_to_master_s_m_;
          const Point3 inferred=q.enu+Point3{arm*std::cos(paired_heading_rad),arm*std::sin(paired_heading_rad),0};
          anchor_residual_.x += weight*(inferred.x-gnss_enu.x);
          anchor_residual_.y += weight*(inferred.y-gnss_enu.y);
        }
      }
'''
def build():
 OUT.mkdir(exist_ok=True)
 src=(BASE/NAV).read_text();report={}
 for variant in ['mixed','rigid','joint','pure']:
  candidate=src.replace('    if (has_map_) {\n      std::string match_direction',MIXED+'    if (has_map_) {\n      std::string match_direction')
  if variant=='pure':candidate=src
  marker='      if (paired_heading_valid) {\n        anchor_heading_delta_rad_'
  if variant in ['rigid','joint']:candidate=candidate.replace(marker,RIGID+(JOINT if variant=='joint' else '')+marker)
  if variant=='pure':candidate=candidate.replace(marker,RIGID.replace('mixed_heading && rover','paired_heading_valid && rover')+marker)
  folder=OUT/variant;folder.mkdir(exist_ok=True)
  cpp=folder/'navigation.cpp';cpp.write_text(candidate)
  subprocess.run(['c++','-std=c++17','-O2','-I'+str(BASE/'ros2_ws/src/tram_odometry/include'),str(BASE/'ros2_ws/src/tram_odometry/src/estimator.cpp'),str(cpp),str(BASE/'evaluation/navigation_replay.cpp'),'-o',str(folder/'navigation_replay')],check=True)
  patch=subprocess.run(['diff','-u',str(BASE/NAV),str(cpp)],capture_output=True,text=True).stdout
  (ROOT/f'evaluation/round4_startup_{variant}.patch').write_text(patch)
  report[variant]={k:hashlib.sha256(p.read_bytes()).hexdigest() for k,p in [('source_sha256',cpp),('binary_sha256',folder/'navigation_replay')]}
 (ROOT/'evaluation/results/round4/startup_build.json').write_text(json.dumps(report,indent=2)+'\n')
if __name__=='__main__':build()
