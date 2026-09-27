#!/usr/bin/env python3
"""Compile frozen sources without modifying them; save read-only review evidence."""
from pathlib import Path
import csv,json,subprocess,hashlib
D=Path(__file__).resolve().parent;O=Path('/private/tmp/round4-kalman-review');O.mkdir(exist_ok=True)
variants={'baseline':Path('/private/tmp/odometry-round4-source/ros2_ws/src/tram_odometry'),'long_guarded':Path('/private/tmp/round4-kalman/long_guarded'),'long_guarded_stop':Path('/private/tmp/round4-kalman/long_guarded_stop')}
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
report={'fixture_sha256':sha(D/'kalman_stop_probe.cpp'),'variants':{}}
for name,source in variants.items():
 cpp=source/('src/estimator.cpp' if name=='baseline' else 'estimator.cpp');header=source/'include/tram_odometry/estimator.hpp';exe=O/(name+'_stop_probe')
 subprocess.run(['c++','-std=c++17','-O2','-DHAS_KALMAN='+str(int(name!='baseline')),'-I'+str(source/'include'),str(D/'kalman_stop_probe.cpp'),str(cpp),'-o',str(exe)],check=True)
 r=subprocess.run([str(exe)],text=True,capture_output=True,check=True);(D/(name+'_stop_probe.csv')).write_text(r.stdout);rows=list(csv.DictReader(r.stdout.splitlines()));assert len(rows)==24
 summary={'cases':len(rows),'max_dropout_speed_mps':max(float(x['dropout_max_speed']) for x in rows),'max_dropout_distance_m':max(float(x['dropout_distance']) for x in rows),'negative_cov_observations':sum(int(x['negative_cov']) for x in rows),'nonfinite_cov_observations':sum(int(x['nonfinite_cov']) for x in rows),'minimum_cov_determinant':min(float(x['min_cov_determinant']) for x in rows),'max_standstill_residual':max(float(x['max_stop_residual']) for x in rows),'reset_residual_and_crosscov_zero':all(float(x['reset_residual'])==float(x['reset_cross'])==0 for x in rows)}
 report['variants'][name]={'source_sha256':sha(cpp),'header_sha256':sha(header),'probe_binary_sha256':sha(exe),'summary':summary,'cases':rows};print(name,summary,flush=True)
 if name=='long_guarded_stop':assert summary['max_dropout_speed_mps']<1e-9 and summary['max_dropout_distance_m']<1e-9 and summary['negative_cov_observations']==summary['nonfinite_cov_observations']==0
(D/'kalman_stop_results.json').write_text(json.dumps(report,indent=2)+'\n')
