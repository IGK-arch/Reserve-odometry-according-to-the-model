#!/usr/bin/env python3
"""Frozen structural standstill repair of long_guarded; no parameter changes."""
from pathlib import Path
import difflib,hashlib,json,subprocess,datetime
ROOT=Path(__file__).resolve().parents[1]
PARENT=Path('/private/tmp/round4-kalman/long_guarded')
DEST=Path('/private/tmp/round4-kalman/long_guarded_stop')
BASE=Path('/private/tmp/odometry-round4-source/ros2_ws/src/tram_odometry')
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
 assert sha(PARENT/'replay_cli')=='b37a73245bfc141b2975f4b65482ee6c736cd19e44ebdecf8c5e581e4da2c143'
 DEST.mkdir(exist_ok=False);(DEST/'include/tram_odometry').mkdir(parents=True)
 before=(PARENT/'estimator.cpp').read_text();hpp=(PARENT/'include/tram_odometry/estimator.hpp').read_text()
 marker='''      other_projected_mps < 0.055 && velocity_mps_ < 0.15) {
    velocity_mps_ = 0.0;'''
 assert before.count(marker)==1
 after=before.replace(marker,marker+'''
    // This authoritative standstill also constrains latent acceleration.
    kalman_residual_mps2_ = 0.0;
    kalman_va_ = 0.0;''')
 (DEST/'estimator.cpp').write_text(after);(DEST/'include/tram_odometry/estimator.hpp').write_text(hpp)
 (DEST/'standstill.patch').write_text(''.join(difflib.unified_diff(before.splitlines(True),after.splitlines(True),fromfile='a/estimator.cpp',tofile='b/estimator.cpp')))
 (DEST/'candidate.patch').write_text(''.join(difflib.unified_diff((BASE/'src/estimator.cpp').read_text().splitlines(True),after.splitlines(True),fromfile='a/estimator.cpp',tofile='b/estimator.cpp'))+''.join(difflib.unified_diff((BASE/'include/tram_odometry/estimator.hpp').read_text().splitlines(True),hpp.splitlines(True),fromfile='a/estimator.hpp',tofile='b/estimator.hpp')))
 cmd=['c++','-std=c++17','-O2','-Wall','-Wextra','-pedantic','-I'+str(DEST/'include'),str(DEST/'estimator.cpp'),'/private/tmp/odometry-round4-source/evaluation/replay_cli.cpp','-o',str(DEST/'replay_cli')];subprocess.run(cmd,check=True)
 report={'protocol':__doc__,'frozen_at_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'generator_sha256':sha(Path(__file__)),'parent':{f:sha(PARENT/f) for f in ['estimator.cpp','include/tram_odometry/estimator.hpp','replay_cli']},'candidate':{f:sha(DEST/f) for f in ['estimator.cpp','include/tram_odometry/estimator.hpp','replay_cli','candidate.patch','standstill.patch']},'compile':cmd,'parameters_unchanged':True,'public_result_not_used_for_selection':True}
 (DEST/'freeze.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
if __name__=='__main__':main()
