#!/usr/bin/env python3
"""Generate isolated, position-only variants from immutable round4 source.

No estimator or startup algorithm change. Existing GNSS gates and 3-fix median
remain. Candidate KF accepts at most one median per0.5s across antennas; drift
is a bounded scale-like position bias per travelled metre, never published speed.
"""
from pathlib import Path
import json,shutil,subprocess,hashlib,difflib
S=Path('/private/tmp/odometry-round4-source');O=Path('/private/tmp/round4-position');ROOT=Path(__file__).resolve().parents[1]
VARIANTS={'scalar':(0.,0.),'drift_005':(.005,1e-8),'drift_015':(.015,1e-8)}
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def build():
 O.mkdir(parents=True,exist_ok=True);report={}
 for name,(bound,qdrift) in VARIANTS.items():
  D=O/name;D.mkdir(exist_ok=True);inc=D/'include';shutil.copytree(S/'ros2_ws/src/tram_odometry/include',inc,dirs_exist_ok=True)
  h=inc/'tram_odometry/navigation.hpp';t=h.read_text().replace(' void correctGnss(const Fix& message, bool rover);',' void correctGnss(const Fix& message, bool rover);\n void predictPositionBias();\n void resetPositionBias();\n void updatePositionBias(double anchor, double stamp, int status);\n double position_p00_=4, position_p01_=0, position_p11_=4e-6;\n double position_drift_=0, position_last_distance_=0, position_last_update_=-INFINITY;\n int position_windows_=0;');h.write_text(t)
  orig=(S/'ros2_ws/src/tram_odometry/src/navigation.cpp').read_text();t=orig
  needle='  const auto state=estimator_.state();\n  if(!state.initialized';assert needle in t;t=t.replace(needle,'  const auto state=estimator_.state();\n  predictPositionBias();\n  if(!state.initialized',1)
  t=t.replace('      has_gnss_anchor_ = false;','      has_gnss_anchor_ = false;\n      resetPositionBias();',1)
  t=t.replace('        start_s_m_=alternative.s-distance;','        start_s_m_=alternative.s-distance;\n        resetPositionBias();',1)
  a=t.index('  const double correction=std::clamp(config_.gnss_correction_gain*');b=t.index('\n}\n',a)
  t=t[:a]+'  updatePositionBias(values[1],fix.stamp_s,fix.status);'+t[b:]
  t=t.replace('    has_gnss_anchor_ = true;','    has_gnss_anchor_ = true;\n    resetPositionBias();',1)
  methods=r'''
void Navigation::resetPositionBias() {
  position_p00_=4; position_p01_=0; position_p11_=DRIFT_BOUND > 0 ? 4e-6 : 0;
  position_drift_=0; position_last_distance_=estimator_.state().distance_m;
  position_last_update_=-INFINITY; position_windows_=0;
}
void Navigation::predictPositionBias() {
  const auto state=estimator_.state();
  if(!has_gnss_anchor_ || !state.initialized) return;
  const double ds=std::max(0.,state.distance_m-position_last_distance_);
  position_last_distance_=state.distance_m;
  if(ds<=0) return;
  start_s_m_+=position_drift_*ds;
  position_p00_=std::min(400.,position_p00_+2*ds*position_p01_+ds*ds*position_p11_+.0025*ds);
  position_p01_+=ds*position_p11_;
  position_p11_=std::min(1e-4,position_p11_+Q_DRIFT*ds);
}
void Navigation::updatePositionBias(double anchor,double stamp,int status) {
  if(stamp-position_last_update_<.5) return;
  if(stamp-position_last_update_>10) ++position_windows_;
  position_last_update_=stamp;
  const double variance=status==2 ? .25 : 4.;
  const double innovation=anchor-start_s_m_;
  const double total=position_p00_+variance;
  const double k0=position_p00_/total, k1=position_windows_>=2 ? position_p01_/total : 0;
  const double used=std::clamp(innovation,-config_.gnss_correction_max_step_m/std::max(k0,1e-6),config_.gnss_correction_max_step_m/std::max(k0,1e-6));
  start_s_m_+=k0*used;
  position_drift_=std::clamp(position_drift_+k1*used,-DRIFT_BOUND,DRIFT_BOUND);
  const double p00=position_p00_,p01=position_p01_;
  position_p00_=std::max(.01,position_p00_-k0*p00);
  position_p01_-=k0*p01;
  position_p11_=std::max(0.,position_p11_-k1*p01);
  ++corrections_;
}
'''.replace('DRIFT_BOUND',repr(bound)).replace('Q_DRIFT',repr(qdrift))
  t=t.replace('\nvoid Navigation::finalizeStartupAnchor(',methods+'\nvoid Navigation::finalizeStartupAnchor(',1)
  cpp=D/'navigation.cpp';cpp.write_text(t)
  patch=''.join(difflib.unified_diff(orig.splitlines(True),t.splitlines(True),fromfile='a/navigation.cpp',tofile='b/navigation.cpp'))
  (D/'candidate.patch').write_text(patch)
  exe=D/'navigation_replay';subprocess.run(['c++','-std=c++17','-O2','-I'+str(inc),str(S/'evaluation/navigation_replay.cpp'),str(cpp),str(S/'ros2_ws/src/tram_odometry/src/estimator.cpp'),'-o',str(exe)],check=True)
  report[name]={'drift_bound_m_per_m':bound,'scale_random_walk_variance_per_m':qdrift,'offset_variance_growth_per_m':.0025,'minimum_update_spacing_s':.5,'measurement_variance_rtk_m2':.25,'measurement_variance_usable_m2':4.,'binary_sha256':sha(exe),'navigation_cpp_sha256':sha(cpp),'header_sha256':sha(h),'patch_sha256':sha(D/'candidate.patch')}
 (O/'build.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
if __name__=='__main__':build()
