# Frozen pure rover correction: downstream evaluation

Selection was frozen before these runs. All tests compare immutable round4 baseline against the accepted pure geometry source, without further parameter or source changes.

## Full GNSS diagnostic study

Processed 46 bags × four modes: 33 train and 13 validation. Full output CSV hashes match in every mode: **True**. Lost absolute proxy matches: **0**; gained: **0**. Identical CSVs also prove identical publication stamps, validity masks, positions, speed, distance and flags.

| Split | Mode | Scored bags / total | Common matches | Baseline = candidate 3D proxy RMSE, m |
|---|---|---:|---:|---:|
| train | startup | 31/33 | 812023 | 14.130783710 |
| train | sparse | 31/33 | 812023 | 6.178909228 |
| train | delayed | 31/33 | 812023 | 14.130783710 |
| train | frozen | 31/33 | 812023 | 10.914712307 |
| validation | startup | 13/13 | 301243 | 2.798136789 |
| validation | sparse | 13/13 | 301243 | 1.840986507 |
| validation | delayed | 13/13 | 301243 | 2.798136789 |
| validation | frozen | 13/13 | 301243 | 1.752568257 |

The paired-GNSS proxy shares physical sensors with the inputs. It is a diagnostic, not independent fused truth. Off-map bags remain in total coverage and do not receive zero error. The full report retains each common-mask SHA-256, output/input hashes, per-bag errors, sessions and provenance. These 46 bags are the requested cached train/validation subset, not all 97 unique bags.

## Deterministic public reference

All four profile output CSVs are byte-identical. Each has 26,884 position matches. Publication and matched masks are identical, so no common-mask filtering can explain a difference.

| Profile | Baseline = candidate 3D RMSE, m |
|---|---:|
| startup | 5.676123255 |
| corrections | 5.641999085 |
| branch | 1.553040904 |
| full | 1.544900912 |

Full-profile velocity RMSE remains 0.048048137 m/s; final position error remains 0.100748673 m. This is nearest-reference matching within 50 ms, not ROS ATS. The public recording was previously studied and is not a new hidden test.

## Interpretation

The pure change corrects a constructed sparse-master RTK / rover-anchor geometry failure. It produces **no measured change** in these real-data evaluations, including the known public startup regression. It should be described as a specific geometric correction supported by independent synthetic truth, not as a demonstrated real-route accuracy improvement.

## Reproduction and provenance

Full study command:

```sh
python3 evaluation/round3_navigation_study.py --baseline-exe /private/tmp/odometry-round4-baseline/navigation_replay --candidate-exe /private/tmp/round4-startup/pure/navigation_replay --inputs-dir /private/tmp/gnss-round2-baseline --outdir evaluation/runs/round4_pure_navigation
```

Public command, run once for each frozen executable with a fresh output directory:

```sh
python3 evaluation/run_reference_ablation.py --bag /private/tmp/odometry-audit/check-code/bags/30618_88aea4d9 --exe EXECUTABLE --outdir FRESH_DIRECTORY
```

`pure_navigation_comparison.json` preserves the full GNSS report; `pure_navigation_summary.json` summarizes masks and parity. `pure_public_baseline_ablation.json`, `pure_public_candidate_ablation.json` and `pure_public_comparison.json` preserve public results and hashes. `pure_shared_assets_provenance.json` proves maps, elevation, drive table and projection header matched immutable baseline. Frozen source and executable hashes override incidental workspace-source hashes collected by the legacy evaluators.
