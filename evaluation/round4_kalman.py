#!/usr/bin/env python3
"""Train-only scratch generator: coupled [velocity, acceleration residual] filter.
No GNSS or fused truth enters generated estimators. Source is immutable5a82712.
"""
import argparse,difflib,hashlib,json,subprocess
from pathlib import Path
VARIANTS={
 'short': {'tau_s':0.5,'jerk_psd':0.1},
 'medium': {'tau_s':1.5,'jerk_psd':0.4},
 'long': {'tau_s':3.0,'jerk_psd':1.0},
 'long_guarded': {'tau_s':3.0,'jerk_psd':1.0,'guarded':True},
}

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def replace(s,a,b):
 assert s.count(a)==1,(a[:80],s.count(a))
 return s.replace(a,b)

def generate(root,out,name):
 cfg=VARIANTS[name];base=root/'ros2_ws/src/tram_odometry';src=base/'src/estimator.cpp';hdr=base/'include/tram_odometry/estimator.hpp';cpp=src.read_text();hpp=hdr.read_text();original=cpp
 hpp=replace(hpp,'  double propagation_acceleration_mps2_ = 0.0;', '''  double propagation_acceleration_mps2_ = 0.0;
  // Research only: covariance of coupled velocity/acceleration residual.
  double kalman_residual_mps2_ = 0.0;
  double kalman_va_ = 0.0;
  double kalman_aa_ = 0.25;''')
 cpp=replace(cpp,'void Estimator::invalidateWheelResidual() {','''void Estimator::invalidateWheelResidual() {
  // Fault evidence erases learned acceleration; rejected wheels cannot train it.
  kalman_residual_mps2_ = 0.0;
  kalman_va_ = 0.0;
  kalman_aa_ = 0.25;''')
 a=cpp.index('  // Healthy prediction remains');b=cpp.index('  velocity_mps_ =\n      clamp(before + propagation_acceleration_mps2_',a)
 cpp=cpp[:a]+f'''  // Exact OU transition: residual acceleration decays toward the model.
  constexpr double kTau = {cfg['tau_s']};
  constexpr double kJerkPsd = {cfg['jerk_psd']};
  const double rho = std::exp(-dt_s / kTau);
  const double h = kTau * (1.0 - rho);
  propagation_acceleration_mps2_ = clamp(
      model_acceleration_mps2_ + kalman_residual_mps2_ * h / dt_s, -4.0, 3.0);
  kalman_residual_mps2_ *= rho;
  const double qvv = kJerkPsd * kTau * kTau *
      (dt_s - 2.0 * h + 0.5 * kTau * (1.0 - rho * rho));
  const double qva = 0.5 * kJerkPsd * h * h;
  const double qaa = 0.5 * kJerkPsd * kTau * (1.0 - rho * rho);
  velocity_variance_ = std::max(1.0e-8, velocity_variance_ +
      2.0 * h * kalman_va_ + h * h * kalman_aa_ + qvv);
  kalman_va_ = rho * (kalman_va_ + h * kalman_aa_) + qva;
  kalman_aa_ = rho * rho * kalman_aa_ + qaa;
''' +cpp[b:]
 cpp=replace(cpp,'  velocity_variance_ = std::min(1.0e6, velocity_variance_ + process_var);','  velocity_variance_ = std::min(1.0e6, velocity_variance_);')
 a=cpp.index('  const double kalman_gain =');b=cpp.index('  const double before_update =',a)
 cpp=cpp[:a]+'''  const bool kalman_healthy_pair =
      pair_consistent && trusted(other) && !other_bad &&
      !wheel.jump_pending && !other.jump_pending &&
      !wheel.jump_tentative && !other.jump_tentative;
  const double kalman_gain = kalman_healthy_pair
      ? velocity_variance_ / (velocity_variance_ + r)
      : ((pair_recovery || pair_consistent) ? 0.90
         : clamp(velocity_variance_ / (velocity_variance_ + r), 0.40, 0.88));
  if (kalman_healthy_pair) {
    const double accel_gain = kalman_va_ / (velocity_variance_ + r);
    kalman_residual_mps2_ = clamp(kalman_residual_mps2_ +
        accel_gain * clamp(corrected_innovation, -0.5, 0.5), -1.5, 1.5);
    kalman_aa_ = std::max(1.0e-8, kalman_aa_ - accel_gain * kalman_va_);
    kalman_va_ *= 1.0 - kalman_gain;
    if (measured_mps < 0.01 && velocity_mps_ < 0.05) {
      kalman_residual_mps2_ = 0.0;
      kalman_va_ = 0.0;
    }
  } else {
    kalman_va_ = 0.0;
  }
''' +cpp[b:]
 if cfg.get('guarded'):
  # Structural safety variant, motivated by pre-existing maneuver/fault tests:
  # allow the established physical acceleration range, preserve legacy process
  # uncertainty whenever wheel health is lost, broaden acceleration reset prior.
  hpp=hpp.replace('double kalman_aa_ = 0.25;', 'double kalman_aa_ = 1.0;')
  cpp=cpp.replace('kalman_aa_ = 0.25;', 'kalman_aa_ = 1.0;')
  cpp=cpp.replace('accel_gain * clamp(corrected_innovation, -0.5, 0.5), -1.5, 1.5);',
                  'accel_gain * clamp(corrected_innovation, -0.5, 0.5), -4.0, 4.0);')
  cpp=replace(cpp,'  velocity_variance_ = std::min(1.0e6, velocity_variance_);', '''  const bool model_fallback = !trusted(front_) || !trusted(rear_) ||
      front_.jump_tentative || rear_.jump_tentative;
  velocity_variance_ = std::min(1.0e6, velocity_variance_ +
      (model_fallback ? process_var : 0.0));''')
 dest=out/name;(dest/'include/tram_odometry').mkdir(parents=True,exist_ok=True)
 (dest/'estimator.cpp').write_text(cpp);(dest/'include/tram_odometry/estimator.hpp').write_text(hpp)
 patch=''.join(difflib.unified_diff(original.splitlines(True),cpp.splitlines(True),fromfile='a/estimator.cpp',tofile='b/estimator.cpp'))+''.join(difflib.unified_diff(hdr.read_text().splitlines(True),hpp.splitlines(True),fromfile='a/estimator.hpp',tofile='b/estimator.hpp'))
 (dest/'candidate.patch').write_text(patch)
 cmd=['c++','-std=c++17','-O2','-Wall','-Wextra','-pedantic','-I'+str(dest/'include'),str(dest/'estimator.cpp'),str(root/'evaluation/replay_cli.cpp'),'-o',str(dest/'replay_cli')]
 subprocess.run(cmd,check=True)
 return {'variant':name,'parameters':cfg,'build_command':cmd,'source_baseline':{str(src):sha(src),str(hdr):sha(hdr)},'artifacts':{x:sha(dest/x) for x in ['estimator.cpp','include/tram_odometry/estimator.hpp','candidate.patch','replay_cli']}}

def main():
 p=argparse.ArgumentParser();p.add_argument('--source',type=Path,default=Path('/private/tmp/odometry-round4-source'));p.add_argument('--out',type=Path,default=Path('/private/tmp/round4-kalman'));a=p.parse_args();a.out.mkdir(parents=True,exist_ok=True)
 report={'protocol':__doc__,'generator_sha256':sha(__file__),'variants':[generate(a.source,a.out,v) for v in VARIANTS]}
 (a.out/'provenance.json').write_text(json.dumps(report,indent=2)+'\n')
 print(json.dumps(report,indent=2))
if __name__=='__main__':main()
