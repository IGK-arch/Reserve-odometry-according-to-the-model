# Position offset/scale uncertainty experiment — rejected

Three predefined scratch candidates were generated from immutable `5a82712` sources. Startup and the physical estimator were unchanged. All existing GNSS age, payload-freeze, pair-baseline, route/branch and three-fix-median gates remain. Only the Navigation along-track position anchor is updated.

The model is `a(d+Δd)=a(d)+b·Δd`, with anchor offset `a` in metres and scale-like drift `b` in m/m. Published wheel distance and velocity are never changed. Measurement is the gated median of `match.s − wheel_distance`. The scalar variant fixes b=0; drift variants bound |b| to0.005 or0.015. Initial covariance is Paa=4m², Pbb=4e-6; process variance grows0.0025m²/m and drift variance1e-8 per metre. R is0.25m² for status2 and4m² otherwise. Updates are at least0.5s apart across both antennas; drift gain remains zero until two GNSS windows separated by>10s. These are engineering assumptions, not measured GNSS covariance. Overlapping medians and common GNSS biases remain correlated.

## Synthetic behavior

Known straight track, 10m/s cruise,1% wheel scale (healthy control uses correct scale),GNSS first30s then2s windows every120s, stop after270s. XY RMS after30s:

| Case | Baseline | Scalar | Drift0.5% | Drift1.5% |
| --- | ---: | ---: | ---: | ---: |
| scale_sparse | 5.2046 | 6.8458 | 4.3595 | 3.3269 |
| scale_bad_shared | 20.8748 | 20.7991 | 23.1113 | 25.9813 |
| scale_frozen | 17.4061 | 17.6066 | 17.5972 | 17.5972 |
| scale_delayed | 17.4302 | 17.6168 | 17.6075 | 17.6075 |
| healthy_sparse | 0.1142 | 0.1190 | 0.0648 | 0.0648 |

All variants have exactly zero position range during the stopped GNSS blackout280–350s. Stopped position can subsequently change when a legitimate new fix arrives; that is a correction, not time-based drift. A common20m badGNSS window passes inherited single-antenna gates and is amplified by drift propagation. Frozen and delayed cases do not improve.

## Train-only results

33 unique30618 train bags × four input modes × baseline plus three candidates =528 Navigation replays. The initial30s GNSS period permits ongoing corrections; startup selection itself is unchanged. No validation, holdout or public fused-reference file was read. Proxy is the round3 cleaned paired-GNSS+physical-TF estimate sampled one in ten timestamps, nearest output within50ms. Absolute-valid masks, output stamps, velocity and wheel distance match exactly; no registration or time fitting. Proxy errors are diagnostics, not official scores.

| Mode | Baseline pooledXY RMS | Scalar | Drift0.5% | Drift1.5% | Samples |
| --- | ---: | ---: | ---: | ---: | ---: |
| startup | 14.9648 | 14.6456 | 14.6486 | 14.6486 | 34124 |
| sparse | 6.1793 | 6.8752 | 6.3990 | 6.1512 | 34124 |
| delayed | 14.9648 | 14.6456 | 14.6486 | 14.6486 | 34124 |
| frozen | 11.5765 | 11.7536 | 8.9387 | 8.0911 | 34124 |

The JSON retains each bag/session/mode, XY and XYZ RMS, p95, maximum and final error, coverage, hashes, and best/worst deltas. Do not choose solely by pooled RMS: bad GNSS and startup tails determine rejection.

## Session and failure-tail check

For the drift1.5% prototype, the pooled frozen-mode gain is dominated by one September run: `30618_defd0170` improves58.8407→29.4414m RMS. This does not establish broad robustness: its sparse mode worsens12.0985→14.2904m, while July frozen data worsen in aggregate.

| Session / mode | BaselineXY RMS | Drift1.5% |
| --- | ---: | ---: |
| 2026-07-27 / startup | 5.1271 | 5.1171 |
| 2026-07-27 / sparse | 4.6947 | 4.5300 |
| 2026-07-27 / delayed | 5.1271 | 5.1171 |
| 2026-07-27 / frozen | 4.3338 | 4.4086 |
| 2026-09-03 / startup | 37.6697 | 36.7917 |
| 2026-09-03 / sparse | 11.6528 | 11.9387 |
| 2026-09-03 / delayed | 37.6697 | 36.7917 |
| 2026-09-03 / frozen | 28.8224 | 18.5410 |

Drift1.5% improves>0.1m on3 sparse bags but worsens8 (worst+2.1919m). For frozen GNSS,7 improve and11 worsen (worst+1.5503m). The latter includes `30618_68d1748a`, whose RMS grows1.4611→3.0114m. Scalar correction has a much larger frozen failure,18.4711→41.9858m on `30618_0686195f`. These tails motivate rejection despite favorable pooled values.

## Numerical limitation and decision

The prototype covariance independently caps its diagonals while allowing the cross term to grow. In a standalone no-measurement propagation check its determinant becomes negative after5050m. This is not a production-safe covariance bound. No prototype is integrated; any future attempt requires PSD-preserving bounds/Joseph updates and a new train selection. Current rejection is specific to these implementations.

**Retain the baseline correction.** Constant wheel-scale errors are a favorable case for distance-domain drift; neither that synthetic gain nor occasional bag improvements outweigh the measured regressions and covariance limitation.

## Reproduce

```sh
python3 evaluation/round4_position_build.py
python3 evaluation/round4_position_synthetic.py
python3 evaluation/round4_position_train.py
python3 evaluation/round4_position_report.py
```

Frozen source and binaries are in `/private/tmp/odometry-round4-source` and `/private/tmp/odometry-round4-baseline`. Candidate source/executables and raw outputs are in `/private/tmp/round4-position`. Cached inputs are restricted by the train manifest before opening; the cache also contains validation files that this script does not use. Full candidate patches and executable/generator hashes are retained beside this note. Production sources and Git were not modified.
