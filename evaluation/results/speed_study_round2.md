# Round 2 speed study: causal fallback residual

Final integrated runtime evidence is in [speed_study_final.md](speed_study_final.md) and `speed_study_final.json`. This file preserves the isolated selection study and its rejected approaches.

The recommended integration candidate is **`residual_stale_only`**. It improves
paired-wheel dropout prediction while leaving healthy propagation and fusion
unchanged. This report covers a scratch experiment; applying the accompanying
patch to production remains a separate integration step.

The exact comparison is the pre-round-2 snapshot `/private/tmp/odometry-round2-baseline`,
including the previously accepted 0.90 healthy-wheel gain. It is not the original
Git HEAD. All replay is causal and uses C/F/R only. GNSS stays in the evaluator;
reference velocity never enters a candidate. Table mode is used for 30618 and
physics for 30639. Source/binary, table, manifest and data hashes, common masks,
full per-bag metrics, and whole-session results are in `speed_study_round2.json`.

## Investigation and selection

A known-motion test with a grade/force-prior mismatch, steady 5 m/s wheels,
10 Hz measurements received 40 ms late, and a 20 Hz controller reproduces
0.051556 m/s prediction RMSE even with adaptation disabled. Raising the healthy
wheel gain to 0.98/1.0 leaves 0.051145/0.051128: model propagation and late-wheel
projection dominate this particular error. Raw sensor timestamps are needed for
wheel acceleration; differentiating model-projected speed at callback time
mixes model error with the measurement derivative.

The first train-only sweep compared baseline, gain 0.98, gain 1.0, faster bounded
bias adaptation (0.5/s, bound 0.5 m/s²), short wheel acceleration with 0.25/0.50 s
smoothing and fade, a short model residual, and zero-order speed holding.
Stronger gain slightly regressed every complete train session. Zero-order
holding substantially regressed nominal and blackout scores. Faster bias
adaptation improved nominal scores but regressed July 27 five-second dropout
RMSE 0.60204 → 0.63388 m/s. Initial wheel-acceleration prototypes overwrote the
model acceleration used by the existing slow adaptation; they suppressed that
adaptation and regressed long dropouts. Those prototypes were rejected.

A second train-only comparison separated physical/table model acceleration from
propagation acceleration. All three variants improved nominal and dropout
errors in every train session. The best 30618 weighted nominal candidate,
`residual025_separate`, was frozen before validation/holdout/public evaluation:
train 0.060600 → 0.059439, validation 0.033146 → 0.031960 m/s. Its broad nominal
change regressed the public fused-reference diagnostic 0.047735 → 0.051199 and
30639 historical holdout 0.114213 → 0.116240. It remains a research candidate,
**not the default integration recommendation**. No parameters were changed
following those results.

The coordinator then explicitly requested a separate hypothesis: preserve all
healthy behavior and use the residual only during paired-wheel fallback. This
was predeclared before a new train run. There was no sweep: smoothing, activation
and fade all use the existing `wheel_stale_s = 0.35` parameter. It was frozen
after improving all three train sessions' 1/3/5-second dropout errors, then
checked on validation, historical holdout, synthetic faults, and the public bag.
The public result did not select a parameter.

## Recommended candidate

For a newly accepted wheel agreeing with its trusted partner, take the derivative
of its previous accepted raw speed using sensor header time, for intervals
0.045–0.30 s. Its residual is raw wheel acceleration minus the unmodified
physical/table acceleration. Reject residual magnitude above 2 m/s² and smooth
it with `alpha = 1 - exp(-0.5 * dt / wheel_stale_s)`; the factor 0.5 accounts for
the two alternating wheel streams. Keep the physical/table acceleration intact
for the original slow adaptation and diagnostics.

Only when **both wheels are stale**, and neither is currently quarantined,
add the last residual to propagation with weight
`exp(-max(0, age - wheel_stale_s) / wheel_stale_s)`. Healthy prediction,
measurement gain, and healthy late-wheel projection remain identical. No new
sensor, learned table, tuning parameter, or timestamp shift is introduced.
The residual and raw sensor history reset with the estimator. A gap without
prior healthy pair evidence uses the existing physical/table model.

## Whole-session results

Nominal RMSE in m/s; train July 27 has 29 scored 30618 bags, September 3 has 4,
30639 train has 9. Validation has 13 30618 bags. Historical holdout has 13 scored
30618 bags and 10 scored 30639 bags; other holdout bags lack GNSS and are retained
in the per-bag output/coverage record. These are whole session results, not
independent sample-level confidence intervals. Tables/maps were not refit per
session, so this is not session cross-validation.

| Session | Split | Baseline | Stale-only |
|---|---|---:|---:|
| 30618 / July 27 | Train | 0.05475810 | 0.05475772 |
| 30618 / September 3 | Train | 0.08799332 | 0.08799029 |
| 30639 / August 26 | Train | 0.04550370 | 0.04550370 |
| 30618 / August 26 | Validation | 0.03314565 | 0.03314565 |
| 30618 / August 10 | Historical holdout | 0.12182163 | 0.12182560 |
| 30639 / May 5 | Historical holdout | 0.11421285 | 0.11421262 |

The tiny nominal differences arise only around existing wheel gaps. Historical
holdout had already influenced earlier model choices and is not a new untouched
generalization claim. No bag was selected or excluded based on candidate score.

Paired-wheel blackout speed/distance RMSE for 284 identical validation windows:

| Horizon | Speed baseline → candidate (m/s) | Distance baseline → candidate (m) |
|---|---:|---:|
| 1 s | 0.16968 → 0.13654 | 0.09398 → 0.07753 |
| 3 s | 0.41009 → 0.38060 | 0.66569 → 0.58232 |
| 5 s | 0.55153 → 0.53388 | 1.59757 → 1.47283 |

All 1/3/5-second speed and distance aggregates also improve in each complete
train session and both historical holdout sessions. Example 30618 historical
five-second results: speed 0.65440 → 0.64110; distance 1.84237 → 1.72838.

On the public bag `30618_88aea4d9`, all nominal results are unchanged: fused
signed-x speed RMSE is 0.04773500545 m/s on 26,188 matches, sparse GNSS RMSE is
0.03710755357 on 1,045 matches. The wheel-mean baseline remains more accurate
against fused speed at 0.02683912016. This is offline nearest-within-50-ms
matching, not the official ROS checker's arrival-order ATS. A final integrated
1× official ROS run is still required.

## Known-motion evidence and limitations

The independent `wheel_fault_benchmark.py` uses analytic truth, deterministic
noise/jitter, 72 profile/fault/seed cases, and exact common output times. For the
stale-only candidate, all scenarios have 100% coverage. Clean, jumps, freezes,
slow bias, and single-wheel fault scores are unchanged; only paired dropout
behavior changes. See the JSON for profile-specific aggregate errors.

A proposed production regression requires a one-second paired dropout error
below 0.35 m/s under an intentionally imperfect command model. Frozen baseline
fails at 0.432643; the candidate passes at 0.313852 (27.5% lower) and passes
measurement-return and reset checks. The earlier exploratory 0.30 budget failed
at 0.313852; it is recorded rather than hidden. The production 0.35 requirement
was then written and rerun red on the immutable baseline before application to
production. Existing estimator core tests also pass for the scratch candidate.
The separate healthy-prediction regression belongs to the broad nominal
candidate and is **not** an acceptance test for stale-only behavior.

The residual derives from the same wheel sensors and is not independent truth.
Smooth common bias or a frozen pair may remain unobservable; this change does
not claim to detect them. It also does not fix persistent common-jump recovery,
which is a separate production change. Some individual blackout windows can
regress despite the consistent session aggregates. A wrong recent residual can
briefly bias fallback, but it fades with a 0.35 s time constant and never changes
healthy output merely to fit a delayed public reference.

## Reproduction

Run against the preserved baseline snapshot with its dataset symlink. Each
output directory contains the generated C++ source/header and compiler command.
All variants are defined in `speed_candidate_study.py`; explicitly run training
before frozen validation or diagnostic evaluation.

```bash
python3 evaluation/speed_candidate_study.py --root /path/to/frozen-baseline \
  --out /tmp/speed-train --split train --blackouts \
  --variants baseline residual_stale_only
python3 evaluation/speed_candidate_study.py --root /path/to/frozen-baseline \
  --out /tmp/speed-validation --split validation --blackouts \
  --variants baseline residual_stale_only
python3 evaluation/speed_candidate_study.py --root /path/to/frozen-baseline \
  --out /tmp/speed-holdout --split holdout --blackouts \
  --variants baseline residual_stale_only
python3 evaluation/speed_study_public.py --root /path/to/frozen-baseline \
  --bag /path/to/30618_88aea4d9 --before /path/to/frozen-baseline/build/replay_cli \
  --after /tmp/speed-train/candidates/residual_stale_only/replay \
  --out /tmp/speed-public.json
python3 evaluation/wheel_fault_benchmark.py \
  --before /path/to/frozen-baseline/build/replay_cli \
  --after /tmp/speed-train/candidates/residual_stale_only/replay \
  --table ros2_ws/src/tram_odometry/assets/drive_accel_table.csv \
  --out /tmp/speed-synthetic.json
```

The scratch proposal is `speed_study_stale_only.patch`; it should be integrated
with the separately owned recovery change and then revalidated. The source
fixture `speed_study_stale_regression.cpp` is intentionally outside production
tests until that integration.
