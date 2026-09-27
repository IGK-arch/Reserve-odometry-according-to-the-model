# Fifth candidate: structural standstill fix and rejection of default adoption

`long_guarded_stop` differs from frozen `long_guarded` only by resetting the
latent acceleration residual and velocity–acceleration cross covariance when
the existing authoritative dual-wheel standstill clamp sets velocity to zero.
No noise, time constant, gain, gate or parameter was tuned. Earlier candidate
sources, executables and measurements were preserved. The fifth source and
binary hashes were frozen before train/validation/holdout replay.

The new public-API regression covers both vehicles, both wheel callback orders,
adaptation on/off, and stationary wheel noise 0.01/0.04/0.054 m/s. Legacy passes,
frozen fourth candidate fails 20 cases, and the fifth passes. Before the fix,
a 10-second stationary dropout could invent almost 10 m of motion. Five existing
estimator tests also pass; 84 synthetic cases per vehicle retain full coverage
and show no recovery-delay regression over 0.2 s against legacy.

| Dataset | Legacy nominal RMSE, m/s | Fifth nominal RMSE, m/s | Legacy 5 s blackout speed RMSE, m/s | Fifth 5 s blackout speed RMSE, m/s |
|---|---:|---:|---:|---:|
| Train | 0.05798233 | 0.05552023 | 0.66042345 | 0.56407363 |
| Validation | 0.03314562 | 0.03069494 | 0.53388278 | 0.38822965 |
| Historical holdout | 0.11807518 | 0.11699213 | 0.72327376 | 0.61656363 |

Nominal output counts/common masks and blackout endpoint sets are identical.
Train has 42 scored bags/939478 matches; validation has 13/286515; holdout has
23 scored bags out of 42/431768 matches, with the 19 missing-reference bags
explicitly retained in coverage reports. Blackout windows per horizon are
829/284/442 respectively. Per-session, vehicle, bag and window reports accompany
this summary; no rows were excluded based on candidate error size.

The fix leaves the general prediction tradeoff essentially unchanged. For
30618 holdout, 5 s speed absolute-error p95 worsens 1.000607→1.088767 m/s and
maximum worsens 3.912534→4.786080 m/s. A window in 30618_4d487b0d starting at
1786374990 s changes speed error −0.233230→−2.006331 m/s and distance error
−0.868246→−6.213260 m. The 30639 holdout nominal RMSE also worsens
0.11421036→0.11528526 m/s, while its dropout RMSE improves.

This tradeoff has a clear mechanism: a learned acceleration residual is helpful
when the recent model error persists through wheel loss, but hurts when that
error was transient. With the fixed 3 s decay, an erroneous residual b contributes
approximately 2.43*b m/s speed error and 7.70*b m distance error after 5 s.
A residual of 0.7 m/s² can therefore add roughly 1.7 m/s and 5.4 m. The stop
reset removes a specific inconsistency between visible velocity and hidden
state; it cannot make that broader persistence assumption universally valid.

Recommendation: **retain legacy as the production default**. The experimental
Kalman result is useful evidence of average-error gains and a reproducible
alternative, not a universal reliability improvement. A future structural
revision should study independent transient-error and command-transition
fixtures on train, not tune parameters to these exposed holdout windows. The
public-score context supplied by the parent was not used for selection or
parameter changes, and this task did not run a new public evaluation.

Portable regression source: `evaluation/round4_kalman_stop_test.cpp`, ready to
copy as `test/estimator_standstill_dropout_test.cpp`. Optional first argument is
the drive-table path. Reproducible generator: `evaluation/round4_kalman_stop.py`.
New executable SHA256:
`8eb8a3f548aff2bbafc1e6e7e7b0e6070e0cee490cee945a737e2c56ad0f91ad`.
Full provenance and all measured artifacts: `kalman_stop_decision.json`,
`kalman_stop_freeze.json`, `kalman_stop_*_results.json`, synthetic reports,
red/green logs and exact patches in this directory.
