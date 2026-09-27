# Healthy wheel agreement: validation, 2026-09-27

The estimator now gives an accepted wheel that agrees with its recent partner
the existing recovery gain of 0.90. Previously only pair recovery used that
gain; normal pairs retained more of the imperfect model prediction. The slip
gates, causal timestamp handling, prediction, adaptation and fallback equations
are unchanged. This is a healthy-measurement accuracy change, not a new detector
for simultaneous wheel slip.

The candidate was selected on the frozen train split and then checked on
validation. The new reference bag was evaluated afterward, without adjusting
parameters or shifting timestamps. No holdout bags were used. Baseline source
was commit `b132f4a94ec5e30e0d21a21ad1e4816eaf9b70b6`.

## GNSS replay aggregates

All RMSE values below are m/s, weighted by matched sample count, using the
existing `core_benchmark.py` receive-order replay and independent GNSS proxy.
Each row uses identical common output stamps before and after the change.

| Split | Vehicle | Model | Matches | Before | After |
|---|---|---|---:|---:|---:|
| Train | 30618 | Physics | 755,782 | 0.06309183 | 0.06125555 |
| Train | 30618 | Table | 755,782 | 0.06187253 | 0.06059998 |
| Train | 30639 | Physics | 183,696 | 0.05036043 | 0.04550370 |
| Train | 30639 | Table | 183,696 | 0.04691153 | 0.04357136 |
| Train | All | Physics | 939,478 | 0.06081245 | 0.05851008 |
| Train | All | Table | 939,478 | 0.05924509 | 0.05766722 |
| Validation | 30618 (all) | Physics | 286,515 | 0.03591213 | 0.03387005 |
| Validation | 30618 (all) | Table | 286,515 | 0.03451910 | 0.03314565 |

For the deployed choice (table for 30618, physics for 30639), train aggregate
RMSE improves **0.05979613 → 0.05795829**: 39 of 42 bags improve, two tie, and
`30618_0686195f` regresses 0.09498705 → 0.09564179. All 13 validation bags improve
in both modes. Validation contains no independent 30639 session; the table
ablation for 30639 is not a recommendation to enable its table.

Slip/model-only output counts are unchanged: train 231/52 in either mode;
validation physics 198/28, table 198/29. Exact shared-header front/rear absolute
differences have median/95th percentile 0.00473/0.03497 m/s in train (441,088
pairs) and 0.00483/0.03451 in validation (134,089 pairs). Agreement supports
stronger nominal trust, but does not prove absence of common bias.

## Paired-wheel blackout validation

Fresh before/after runs of `table_blackout_ablation.py` use exactly the same 284
windows across 12 validation bags, deleting both wheels for 5.12 seconds.

| Horizon | Model | Speed RMSE before → after (m/s) | Distance RMSE before → after (m) |
|---:|---|---:|---:|
| 1 s | Physics | 0.19444 → 0.18758 | 0.10948 → 0.10223 |
| 1 s | Table | 0.17502 → 0.16968 | 0.09992 → 0.09398 |
| 3 s | Physics | 0.48051 → 0.47501 | 0.77906 → 0.75865 |
| 3 s | Table | 0.41377 → 0.41009 | 0.68163 → 0.66569 |
| 5 s | Physics | 0.67174 → 0.66707 | 1.91471 → 1.88481 |
| 5 s | Table | 0.55471 → 0.55153 | 1.61997 → 1.59757 |

The fallback benefits from its improved initial velocity. Its dynamics and
table activation are unchanged; table-used fractions at these horizons are
0.8380, 0.7782 and 0.7394 in both runs.

## External bag and reference timing

For new bag `30618_88aea4d9`, same-stamp nearest fused-reference matching within
50 ms gives these results; the wheel baseline remains better at 0.02683912.

| Reference | Model | Matches | RMSE before → after (m/s) |
|---|---|---:|---:|
| Fused signed x velocity | Physics | 26,188 | 0.04814323 → 0.04698142 |
| Fused signed x velocity | Table | 26,188 | 0.04910675 → 0.04773501 |
| Sparse GNSS | Physics | 1,045 | 0.03638844 → 0.03780519 |
| Sparse GNSS | Table | 1,045 | 0.03550061 → 0.03710755 |

The sparse external GNSS regression is retained explicitly. The change is not
a universal win across reference sources. Neither model flags slip or enters
model-only mode on this bag.

The ranking disagreement exists on the *same* 1,045 timestamps: before this
change, baseline/physics/table errors against GNSS are
0.05080/0.03639/0.03550, but against fused velocity are
0.01429/0.02363/0.02855. A timing diagnostic compares raw front speed at header
time t with fused(t): RMSE 0.05510; using fused(t+0.10 s) gives 0.02588. Rear
gives 0.05555 → 0.02617. This suggests reference latency, without establishing
an exact physical delay. **No such shift is applied to outputs or scores.**
Nearest matching remains distinct from the official ROS checker's one-to-one
approximate synchronizer.

## Regression and limits

The new core regression models healthy 10 Hz agreeing wheels received 40 ms
late, a 20 Hz controller, constant true speed 5 m/s, and notch 4 with an
imperfect force-model prior (for example, an unknown grade). It samples at
controller publication time before later wheel callbacks. After warmup,
RMSE is 0.0530379 before and 0.0380687 after. Its 0.045 m/s error budget failed
before implementation and passes afterward, as do the existing core tests.
The test also checks timestamp preservation and absence of spurious slip.

Smooth common-mode slip remains unobservable under the weak physical priors:
both wheels drifting from 5 to 7 m/s over four seconds can equally describe
real acceleration. Both frozen wheels can also describe true steady motion or
a tram held by brakes. Model disagreement can support a suspicion diagnostic,
but cannot safely guarantee rejection without another independent motion
source. Adaptation uses these same wheels and can absorb part of their shared
error; it is not independent evidence.

A separate existing recovery weakness remains: sustained simultaneous 5→9 m/s
wheel jumps are rejected for 0.5 s, then accepted at 0.6 s when the short-window
acceleration check expires and pair agreement clears quarantine. This requires
a separately designed recovery change; the healthy gain change does not claim
to fix it. Stronger gain can also expose genuine mutually consistent sensor
noise, so performance outside these measured noise conditions remains a limit.

## Reproduce

Run from the repository root with the dataset at `dataset/data`. The following
builds the frozen original and current estimator, reports full per-bag train
and validation metrics, and repeats blackout validation. It does not train on
the new fused bag or write into tracked result files.

```bash
speed_research_tmp=$(mktemp -d)
git show b132f4a94ec5e30e0d21a21ad1e4816eaf9b70b6:ros2_ws/src/tram_odometry/src/estimator.cpp > "$speed_research_tmp/before.cpp"
c++ -std=c++17 -O2 -Wall -Wextra -pedantic \
  -I ros2_ws/src/tram_odometry/include \
  "$speed_research_tmp/before.cpp" evaluation/replay_cli.cpp \
  -o "$speed_research_tmp/before"
c++ -std=c++17 -O2 -Wall -Wextra -pedantic \
  -I ros2_ws/src/tram_odometry/include \
  ros2_ws/src/tram_odometry/src/estimator.cpp evaluation/replay_cli.cpp \
  -o "$speed_research_tmp/after"
for version in before after; do
  for split in train validation; do
    python3 evaluation/core_benchmark.py --split "$split" \
      --exe "$speed_research_tmp/$version" \
      --out "$speed_research_tmp/${version}_${split}_physics.csv"
    python3 evaluation/core_benchmark.py --split "$split" \
      --exe "$speed_research_tmp/$version" \
      --drive-table ros2_ws/src/tram_odometry/assets/drive_accel_table.csv \
      --out "$speed_research_tmp/${version}_${split}_table.csv"
  done
  python3 evaluation/table_blackout_ablation.py \
    --exe "$speed_research_tmp/$version" \
    --out "$speed_research_tmp/${version}_blackout.json"
done
c++ -std=c++17 -O2 -Wall -Wextra -pedantic \
  -I ros2_ws/src/tram_odometry/include \
  ros2_ws/src/tram_odometry/src/estimator.cpp \
  ros2_ws/src/tram_odometry/test/estimator_core_test.cpp \
  -o "$speed_research_tmp/core_test"
"$speed_research_tmp/core_test"
```

The offline external fused check is reproducible using
`evaluation/reference_benchmark.py`: export permitted inputs with
`--export-input`, feed that CSV to each replay binary, then evaluate each
result CSV with `--candidate` against the original `--bag`. The replay binary
accepts C/F/R only; exported GNSS and reference never enter the speed filter.
