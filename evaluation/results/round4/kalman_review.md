# Independent review: frozen long_guarded and rover geometry

Reviewed `/private/tmp/round4-kalman/long_guarded` against immutable
`/private/tmp/odometry-round4-source`, without editing either implementation.
No validation/holdout outcomes were used for this review.

## Blocking: standstill can train fictitious acceleration

**P1 — clear the learned residual at the authoritative dual-wheel standstill.**
In `estimator.cpp:855–864`, the filter only clears acceleration residual and
cross-covariance for `measured_mps < 0.01 && velocity_mps_ < 0.05`. The later
existing dual-standstill rule (`:957–961`) accepts both wheel readings below
0.055 m/s and velocity below0.15 m/s, then forces velocity to zero. Readings
between these thresholds therefore keep supplying positive innovations to the
new acceleration state, despite authoritative stationary output.

Reproduction: neutral command, no adaptation or drive table, both wheels
report0.04 m/s at10Hz for10s after reset at zero; then both wheels stop publishing
while the neutral controller continues. The candidate learns about+0.36 m/s²
residual while its published speed remains zero. It then creates motion:

| Blackout duration | Candidate speed, m/s | Added distance, m | Baseline speed / added distance |
| --- | ---: | ---: | ---: |
| 1s | 0.299662 | 0.159896 | 0 / 0 |
| 2s | 0.506618 | 0.569220 | 0 / 0 |
| 5s | 0.811419 | 2.634778 | 0 / 0 |
| 10s | 0.889071 | 6.998858 | 0 / 0 |

This is an internally contradictory stationary clamp followed by invalid
residual carry, not an unobservable common wheel-slip trajectory. Both wheels
have already been explicitly accepted as standstill by the algorithm.

Scratch source `/private/tmp/round4-kalman-review/standstill.cpp` was compiled
against both frozen candidate and baseline estimator sources. Executables are
`standstill_candidate` and `standstill_baseline` in the same directory.

## Related covariance defect at the same clamp

**P2 — the standstill variance cap must also reset or consistently transform
the cross-covariance.** The existing `velocity_variance_=min(...,0.0025)` changes
only Pvv while retaining the new Pva. A stationary-noise sweep shows a negative
`Pvv·Paa−Pva²` even with the30639 default wheel sigma0.085:

- 10Hz: minimum determinant−3.644799e−5 (one negative step).
- 5Hz: minimum determinant−1.525259e−4 (one negative step).
- A supported larger sigma0.15 gives499 negative steps out of500 at10Hz.

Resetting the residual **and Pva** in the authoritative standstill branch
addresses this same root cause. Scratch diagnostic
`/private/tmp/round4-kalman-review/covariance.cpp` accesses the private covariance
for inspection without changing candidate code.

## Review of remaining paths

The exact OU transition and its Qvv/Qva/Qaa expressions are consistent with
integrated exponentially decaying acceleration noise. Healthy scalar-measurement
covariance updates have the expected algebra. Non-healthy accepted measurements
zero Pva; explicit invalid data, detected jump/slip/freeze and estimator reset
use `invalidateWheelResidual`, which clears residual and Pva and restores Paa=1.
The reset fixture confirms residual0 and covariance(Pvv,Pva,Paa)=(0.01,0,1).
Positive timestamp propagation is bounded by the existing60s extrapolation limit
and0.1s steps; nonpositive dt returns before the OU division. No additional
blocking timestamp/reset defect was established.

Identical repeated sensor headers still count as repeated accepted observations,
as in the baseline velocity update. They also reduce the new covariance: a
1000-pair duplicate burst at one timestamp reduced Pvv from0.001837 to its1e−5
floor and changed acceleration residual0.5190→0.5282 m/s². This is a measurement
independence limitation to document, not a newly demonstrated blocking speed
failure in this review. The covariance must not be advertised as calibrated
confidence without accounting for such correlations.

## Suggested production regression transfer

Transfer the public-API standstill fixture into an automatically discovered
`*_test.cpp`, for both30618 and30639:

1. Warm neutral paired wheels at nonzero readings inside the authoritative
   standstill band (0.04 m/s is the decisive fixture), and confirm speed zero.
2. Save current distance, remove both wheel streams, continue neutral controller
   timestamps for10s, and require speed and added distance remain near zero.
3. Include 5/10/50Hz and a genuine motion→standstill transition; retain existing
   moving-dropout and fault-recovery tests so the fix cannot disable prediction.
4. Use a scratch/internal covariance check to confirm finite nonnegative Pvv,
   Paa and determinant through standstill and reset, including sigma0.085.

A structural standstill repair should be frozen as a separate candidate; retain
all four original research variants and their benchmark hashes. Rerun behavioral
and train evidence before adoption. Do not silently overwrite long_guarded.

## Pure rover geometry review: passed

The separate Navigation change adjusts the rover anchor residual by
`arm − R(heading_delta)·arm`, preserving the vertical arm. Combined with the
already-rotated master→base arm, startup XY becomes
`rover + rotated_body·(9.873−12.436)`, which is the correct rigid-body relation.
The guard restricts the change to rover anchoring with valid paired heading;
position quality selection and longitudinal state remain unchanged.

Compiled `navigation_rover_anchor_test.cpp` with the current Navigation source
and immutable baseline estimator. The curve fixture passed with3D error
1.77198e−10 m. All eight grade/course/vehicle XY fixtures passed, maximum
8.12856e−10 m, with zero speed/distance. The same test compiled against frozen
baseline Navigation failed on the curve with4.30484 m error. These establish a
meaningful red/green regression rather than merely checking the implementation.
The graded fixtures test XY as intended; this review makes no new claim about
vertical GNSS-datum correctness. No actionable geometry defect was found.

Executables: `/private/tmp/round4-kalman-review/rover_candidate` and
`/private/tmp/round4-kalman-review/rover_baseline`. Production and candidate files
were not modified during this review.

Reviewed frozen hashes:

- `estimator.cpp`: `a62db61a8df442b8d89e703eb9cfc9e68ccc158db77e1319aa2a5c4b155b0c1b`
- `include/tram_odometry/estimator.hpp`: `4dff47c2540223bf107555f375b65c9eb97a52e94a2614a090cdce43f5b3ad5d`
- `replay_cli`: `b37a73245bfc141b2975f4b65482ee6c736cd19e44ebdecf8c5e581e4da2c143`
