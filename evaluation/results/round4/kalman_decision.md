# Round4 coupled velocity / acceleration-residual filter

Train-only selection frozen: **long_guarded** is recommended for the next validation stage. This is not approval to replace production. Production was not edited.

The filter state is velocity v and an acceleration residual b around the existing physics/table prediction. Residual acceleration follows db = -b/tau dt + sqrt(q) dW. With rho = exp(-dt/tau) and h = tau(1-rho), prediction uses v += a_model*dt + h*b, b *= rho; the full 2x2 covariance propagates with F=[[1,h],[0,rho]] and exact integrated OU process covariance. Accepted healthy wheel pairs update both v and b through their covariance. This is not the earlier fixed gain or differentiated-wheel-residual variant.

All original jump, slip, stale, plateau, timestamp and recovery gates remain. Fault invalidation clears learned b and cross covariance. The selected structural repair keeps the original velocity process uncertainty while wheel health is absent, allows residual acceleration within +/-4 m/s^2, and resets its variance to 1. Ordinary propagated acceleration keeps existing physical bounds. Runtime inputs are C/F/R only.

| Variant | tau, q | Train speed RMSE m/s | Existing estimator tests | Decision |
|---|---|---:|---|---|
| short | 0.5, 0.1 | 0.06023784 | 0/5 | reject |
| medium | 1.5, 0.4 | 0.05577779 | 3/5 | reject |
| long | 3.0, 1.0 | 0.05556304 | 3/5 | reject |
| long_guarded | 3.0, 1.0 | 0.05552554 | 5/5 | validation candidate |

Baseline train RMSE is 0.05798233 m/s. Selected candidate gives 0.05552554 m/s (-4.24%), with identical 42-bag coverage, 939478 matched outputs and common masks. Blackout windows: 829 at each horizon. Speed RMSE at 1/3/5 s is 0.20984080/0.42472187/0.56407546 m/s versus 0.23738673/0.49390895/0.66042345. Five-second distance RMSE is 1.67857571 m versus 1.97802988.

The first three variants were defined before measurement. All showed either accuracy or maneuver/recovery failures in existing tests. The fourth is a structural repair based on those pre-existing failure cases, not a fresh search over train-score parameters. Original failing and repaired passing test output is retained in kalman_existing_tests.json. Analytic constant-acceleration, command-step and stop fixtures (with and without dropout) were evaluated before real train data.

Both vehicles completed 84 synthetic cases each with full coverage. No recovery time worsened by more than 0.2 s. This does **not** mean every fault metric improved: trip freeze-transition speed RMSE, some braking-freeze errors, and long common-jump distance error increase. Per-case tails and all raw reports are retained. The classic gate tests caught failures that the 84-scenario suite did not, which is why the unguarded candidates were rejected despite better pooled train scores.

The bounded innovation and saturated residual make covariance only an approximation after a clamp. The existing distance uncertainty was not redesigned. True healthy plateau after acceleration remains observationally ambiguous. No validation, historical holdout, or public inputs were used to select this variant.

Runnable generator: evaluation/round4_kalman.py. Fixtures: evaluation/round4_kalman_fixtures.py. Report/selector: evaluation/round4_kalman_report.py. Scratch C++ and header plus executable are in /private/tmp/round4-kalman/long_guarded; the exact patch is kalman_long_guarded.patch. All four train per-bag/session reports, synthetic reports, source/build hashes, rejected decisions, and binary hashes accompany this report. Candidate replay SHA256: b37a73245bfc141b2975f4b65482ee6c736cd19e44ebdecf8c5e581e4da2c143.
