#!/usr/bin/env python3
"""Round-2 causal nominal-speed experiment; immutable baseline, train-only choice.

Generates scratch estimator copies, never modifies production source. Use --split
train first; freeze the selected approach before running validation/holdout/public.
"""
from __future__ import annotations
import argparse, csv, hashlib, json, math, subprocess, sys
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor

VARIANTS = {
 'baseline': {},
 'gain98': {'gain':.98},
 'gain100': {'gain':1.0},
 'fast_bias': {'bias_rate':.5, 'bias_limit':.5},
 'accel025': {'kind':'accel', 'smooth':.25, 'fade':.25},
 'accel050': {'kind':'accel', 'smooth':.5, 'fade':.5},
 'residual025': {'kind':'residual', 'smooth':.25, 'fade':.25},
 'hold_fresh': {'kind':'hold', 'smooth':.25, 'fade':.25},
 'accel025_separate': {'kind':'accel', 'smooth':.25, 'fade':.25, 'separate':True},
 'accel050_separate': {'kind':'accel', 'smooth':.5, 'fade':.5, 'separate':True},
 'residual025_separate': {'kind':'residual', 'smooth':.25, 'fade':.25, 'separate':True},
 'residual_stale_only': {'kind':'residual', 'smooth':'config_.wheel_stale_s', 'fade':'config_.wheel_stale_s', 'separate':True, 'stale_only':True},
}

def sha(path): return hashlib.sha256(path.read_bytes()).hexdigest()

def generate(root, out, name):
    cfg=VARIANTS[name]; source=root/'ros2_ws/src/tram_odometry/src/estimator.cpp'
    header=root/'ros2_ws/src/tram_odometry/include/tram_odometry/estimator.hpp'
    cpp,hpp=source.read_text(),header.read_text()
    if 'gain' in cfg: cpp=cpp.replace('? 0.90',f"? {cfg['gain']}")
    if 'bias_rate' in cfg:
        hpp=hpp.replace('adaptive_bias_rate_per_s = 0.035',f"adaptive_bias_rate_per_s = {cfg['bias_rate']}")
        hpp=hpp.replace('adaptive_bias_limit_mps2 = 0.25',f"adaptive_bias_limit_mps2 = {cfg['bias_limit']}")
    if 'kind' in cfg:
        hpp=hpp.replace('double raw_speed_mps = 0.0;', '''double raw_speed_mps = 0.0;
    double last_good_raw_mps = 0.0;
    double last_good_sensor_stamp_s = std::numeric_limits<double>::quiet_NaN();''')
        hpp=hpp.replace('double adaptive_bias_mps2_ = 0.0;', '''double adaptive_bias_mps2_ = 0.0;
  double short_acceleration_mps2_ = 0.0;
  double short_accel_stamp_s_ = std::numeric_limits<double>::quiet_NaN();''')
        cpp=cpp.replace('adaptive_bias_mps2_ = 0.0;', '''adaptive_bias_mps2_ = 0.0;
  short_acceleration_mps2_ = 0.0;
  short_accel_stamp_s_ = std::numeric_limits<double>::quiet_NaN();''')
        addition='short_acceleration_mps2_' if cfg['kind']=='residual' else '(short_acceleration_mps2_ - model_acceleration_mps2_)'
        marker='  velocity_mps_ =\n      clamp(before + model_acceleration_mps2_ * dt_s, 0.0,'
        patch=f'''  // Short causal wheel acceleration fades into the command model after loss.
  if (std::isfinite(short_accel_stamp_s_) &&
      stamp_s_ >= front_.slip_until_s && stamp_s_ >= rear_.slip_until_s) {{
    const double age_s = std::max(0.0, stamp_s_ - short_accel_stamp_s_);
    const double weight = std::exp(-std::max(0.0, age_s - 0.15) / {cfg['fade']});
    model_acceleration_mps2_ = clamp(model_acceleration_mps2_ +
        weight * {addition}, -4.0, 3.0);
  }}
'''
        assert cpp.count(marker)==1
        cpp=cpp.replace(marker,patch+marker)
        marker='  wheel.last_good_speed_mps = measured_mps;\n  wheel.last_good_stamp_s = effective_stamp_s;'
        observed='0.0' if cfg['kind']=='hold' else '(physical_raw_mps - wheel.last_good_raw_mps) / raw_dt_s'
        # Residual uses raw physics/table acceleration from a zero-dt-independent
        # reconstruction is avoided: subtract previous measured prediction, then
        # add previous residual back to remove its already-applied contribution.
        if cfg['kind']=='residual':
            observed += ' - model_acceleration_mps2_ + short_acceleration_mps2_'
        patch=f'''  if (pair_consistent && trusted(other) &&
      std::isfinite(wheel.last_good_sensor_stamp_s)) {{
    const double raw_dt_s = stamp_s - wheel.last_good_sensor_stamp_s;
    if (raw_dt_s >= 0.045 && raw_dt_s <= 0.3) {{
      const double observed = {observed};
      if (std::abs(observed) <= 2.0) {{
        const double alpha = 1.0 - std::exp(-0.5 * raw_dt_s / {cfg['smooth']});
        short_acceleration_mps2_ = std::isfinite(short_accel_stamp_s_)
            ? short_acceleration_mps2_ + alpha * (observed - short_acceleration_mps2_)
            : observed;
        short_accel_stamp_s_ = effective_stamp_s;
      }}
    }}
  }}
  wheel.last_good_raw_mps = physical_raw_mps;
  wheel.last_good_sensor_stamp_s = stamp_s;
'''
        assert cpp.count(marker)==1
        cpp=cpp.replace(marker,patch+marker)
        if cfg.get('separate'):
            hpp=hpp.replace('double short_acceleration_mps2_ = 0.0;', 'double propagation_acceleration_mps2_ = 0.0;\n  double short_acceleration_mps2_ = 0.0;')
            cpp=cpp.replace('short_acceleration_mps2_ = 0.0;', 'propagation_acceleration_mps2_ = 0.0;\n  short_acceleration_mps2_ = 0.0;')
            cpp=cpp.replace('  // Short causal wheel acceleration', '  propagation_acceleration_mps2_ = model_acceleration_mps2_;\n  // Short causal wheel acceleration')
            cpp=cpp.replace('    model_acceleration_mps2_ = clamp(model_acceleration_mps2_ +', '    propagation_acceleration_mps2_ = clamp(model_acceleration_mps2_ +')
            cpp=cpp.replace('clamp(before + model_acceleration_mps2_ * dt_s', 'clamp(before + propagation_acceleration_mps2_ * dt_s')
            cpp=cpp.replace('physical_raw_mps + model_acceleration_mps2_ * late_by_s', 'physical_raw_mps + propagation_acceleration_mps2_ * late_by_s')
            cpp=cpp.replace('other.speed_mps +\n      model_acceleration_mps2_ *', 'other.speed_mps +\n      propagation_acceleration_mps2_ *')
            cpp=cpp.replace(' - model_acceleration_mps2_ + short_acceleration_mps2_', ' - model_acceleration_mps2_')
        if cfg.get('stale_only'):
            cpp=cpp.replace('if (std::isfinite(short_accel_stamp_s_) &&', 'if (!fresh(front_) && !fresh(rear_) &&\n      std::isfinite(short_accel_stamp_s_) &&')
            cpp=cpp.replace('age_s - 0.15', 'age_s - config_.wheel_stale_s')
    dest=out/name
    (dest/'include/tram_odometry').mkdir(parents=True,exist_ok=True)
    (dest/'include/tram_odometry/estimator.hpp').write_text(hpp)
    (dest/'estimator.cpp').write_text(cpp)
    exe=dest/'replay'
    command=['c++','-std=c++17','-O2','-Wall','-Wextra','-pedantic','-I',str(dest/'include'),str(dest/'estimator.cpp'),str(root/'evaluation/replay_cli.cpp'),'-o',str(exe)]
    subprocess.run(command,check=True)
    return exe,{'source':sha(dest/'estimator.cpp'),'header':sha(dest/'include/tram_odometry/estimator.hpp'),'binary':sha(exe),'command':command,'settings':cfg}

def rmse(errors): return math.sqrt(sum(v*v for v in errors)/len(errors)) if errors else None

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True)
    p.add_argument('--split',choices=['train','validation','holdout'],required=True)
    p.add_argument('--variants',nargs='+',default=list(VARIANTS))
    p.add_argument('--blackouts',action='store_true')
    p.add_argument('--baseline-exe',type=Path,help='Compare two already-built executables instead of generating variants')
    p.add_argument('--candidate-exe',type=Path)
    args=p.parse_args();root=args.root.resolve();out=args.out.resolve();out.mkdir(parents=True,exist_ok=True)
    sys.path.insert(0,str(root/'evaluation'))
    import causal_baseline as cb
    import core_benchmark as core
    import table_blackout_ablation as blackout
    import numpy as np
    manifest=json.loads((root/'tools/split_manifest.json').read_text())
    groups={x['representative']:x for x in manifest['groups']}
    binaries={};provenance={}
    if bool(args.baseline_exe) != bool(args.candidate_exe):
        p.error('--baseline-exe and --candidate-exe must be supplied together')
    if args.baseline_exe:
        args.variants=['baseline','candidate']
        binaries={'baseline':args.baseline_exe.resolve(),'candidate':args.candidate_exe.resolve()}
        provenance={name:{'path':str(exe),'binary':sha(exe)} for name,exe in binaries.items()}
    else:
        for name in args.variants: binaries[name],provenance[name]=generate(root,out/'candidates',name)
    table=root/'ros2_ws/src/tram_odometry/assets/drive_accel_table.csv'
    provenance['_evaluation']={'script_sha256':sha(Path(__file__)), 'manifest_sha256':sha(root/'tools/split_manifest.json'), 'table_sha256':sha(table), 'data_sha256':{b:groups[b]['sha256'] for b in manifest['representatives'][args.split]}}
    rows=[]; faults=[]
    for bag in manifest['representatives'][args.split]:
        events,truth=cb.read_run(bag);ref=cb.make_reference(truth)
        codes={cb.FRONT:'F',cb.REAR:'R',cb.CMD:'C'}
        text=''.join(f'{a},{b},{codes[c]},{d}\n' for a,b,c,d in events)
        selected_table=table if bag.startswith('30618') else None
        with ThreadPoolExecutor(max_workers=4) as pool:
            outputs=dict(zip(args.variants,pool.map(lambda n:blackout.replay(binaries[n],bag,text,selected_table),args.variants)))
        common=sorted(set.intersection(*(set(v) for v in outputs.values())))
        matched=[(s,v) for s in common if (v:=core.reference_at(ref,s)) is not None]
        mask=hashlib.sha256('\n'.join(str(s) for s,_ in matched).encode()).hexdigest()
        for name,values in outputs.items():
            errors=[values[s]['v']-v for s,v in matched]
            rows.append({'bag':bag,'vehicle':groups[bag]['vehicle'],'date':groups[bag]['date'],'variant':name,'n':len(errors),'rmse_mps':rmse(errors),'bias_mps':sum(errors)/len(errors) if errors else None,'outputs':len(values),'common_outputs':len(common),'mask_sha256':mask,'model_only':sum(v['model_only'] for v in values.values())})
        if args.blackouts:
            starts,t,g=blackout.select_windows(bag)
            if starts:
                text=blackout.filter_blackouts(events,starts)
                with ThreadPoolExecutor(max_workers=4) as pool:
                    outputs=dict(zip(args.variants,pool.map(lambda n:blackout.replay(binaries[n],bag,text,selected_table),args.variants)))
                for start in starts:
                    for horizon in (1.,3.,5.):
                        endpoints={n:(blackout.nearest(v,start),blackout.nearest(v,start+horizon)) for n,v in outputs.items()}
                        if any(a is None or b is None or not b[1]['model_only'] for a,b in endpoints.values()):continue
                        stamps={(a[0],b[0]) for a,b in endpoints.values()}
                        if len(stamps)!=1:raise RuntimeError('non-common blackout timestamps')
                        ss,es=next(iter(stamps));ss*=1e-9;es*=1e-9
                        ev=float(np.interp(es,t,g));ds=blackout.truth_distance(t,g,ss,es)
                        for name,(a,b) in endpoints.items():
                            faults.append({'bag':bag,'vehicle':groups[bag]['vehicle'],'date':groups[bag]['date'],'variant':name,'start_s':start,'horizon_s':horizon,'speed_error_mps':b[1]['v']-ev,'distance_error_m':b[1]['s']-a[1]['s']-ds})
        print(bag,' '.join(f"{r['variant']}={r['rmse_mps']:.5f}" for r in rows[-len(args.variants):] if r['n']),flush=True)
        (out/'results.json').write_text(json.dumps({'split':args.split,'provenance':provenance,'nominal':rows,'blackouts':faults},indent=2))
    summary=[]
    for vehicle,date in sorted({(r['vehicle'],r['date']) for r in rows}):
        for name in args.variants:
            rs=[r for r in rows if r['vehicle']==vehicle and r['date']==date and r['variant']==name and r['n']]
            n=sum(r['n'] for r in rs)
            item={'vehicle':vehicle,'date':date,'variant':name,'bags':len(rs),'n':n,'nominal_rmse_mps':math.sqrt(sum(r['n']*r['rmse_mps']**2 for r in rs)/n) if n else None}
            for h in (1.,3.,5.):
                fs=[r for r in faults if r['vehicle']==vehicle and r['date']==date and r['variant']==name and r['horizon_s']==h]
                item[f'blackout{h:g}']={'windows':len(fs),'speed_rmse_mps':rmse([r['speed_error_mps'] for r in fs]),'distance_rmse_m':rmse([r['distance_error_m'] for r in fs])}
            summary.append(item)
    (out/'summary.json').write_text(json.dumps(summary,indent=2))
    print(json.dumps(summary,indent=2),flush=True)
if __name__=='__main__':main()
