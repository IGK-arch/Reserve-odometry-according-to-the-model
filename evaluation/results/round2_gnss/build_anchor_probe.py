from pathlib import Path
import shutil,subprocess
root=Path('/private/tmp/gnss-round2-candidates')
for name in ['baseline','geometry']:
 p=root/(name+'_anchor_probe'); (p/'include').mkdir(parents=True,exist_ok=True)
 src=root/'geometry' if name=='geometry' else Path('/private/tmp/odometry-round2-baseline/ros2_ws/src/tram_odometry')
 shutil.copytree(src/'include/tram_odometry',p/'include/tram_odometry',dirs_exist_ok=True)
 h=p/'include/tram_odometry/navigation.hpp'
 s=h.read_text().replace(' const NavigationOutput& output() const { return output_; }',' const NavigationOutput& output() const { return output_; }\n double diagnosticAnchor() const { return start_s_m_; }\n Point3 diagnosticResidual() const { return anchor_residual_; }')
 h.write_text(s)
 replay=(root/'geometry/navigation_replay.cpp').read_text()
 if name=='baseline': replay='\n'.join(line for line in replay.splitlines() if 'body-heading-lookahead' not in line)+'\n'
 replay=replay.replace('frame_id,mapped,anchored\\n','frame_id,mapped,anchored,anchor_s,residual_x,residual_y,direction\\n')
 replay=replay.replace("<< r.frame_id << ',' << r.mapped << ',' << r.anchored << '\\n';", "<< r.frame_id << ',' << r.mapped << ',' << r.anchored << ',' << navigation.diagnosticAnchor() << ',' << navigation.diagnosticResidual().x << ',' << navigation.diagnosticResidual().y << ',' << r.direction << '\\n';")
 (p/'replay.cpp').write_text(replay)
 nav=src/'navigation.cpp' if name=='geometry' else src/'src/navigation.cpp'
 subprocess.run(['c++','-std=c++17','-O2','-I',str(p/'include'),str(p/'replay.cpp'),str(nav),'/private/tmp/odometry-round2-baseline/ros2_ws/src/tram_odometry/src/estimator.cpp','-o',str(p/'replay')],check=True)
