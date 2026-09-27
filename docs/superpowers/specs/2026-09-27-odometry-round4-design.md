# Round 4 design: measured experimental improvements

The user explicitly authorised continuing startup/heading work and trying
experimental approaches that could substantially improve performance. Existing
permission covers execution, validation and publishing; no additional approval
cycle is required for reversible research.

Freeze5a82712 (runtime1b4a395), including binaries and complete source tree.
Keep production files unchanged while three isolated experiments run:

1. Separate position quality from heading evidence. Compare strictly RTK-only
heading with physically constrained, temporally coherent mixed-quality heading
and joint rigid-body antenna anchoring. Avoid unrestricted use of a noisy
companion; reject malformed baselines, inconsistent angles and stale stamps.
Synthetic truth supplies bad-companion and curved parked-body cases. Train
startup replays test whether improved observability survives real data.
2. Compare a genuine velocity/acceleration Kalman predictor with the current
physical model plus short residual. Only causal wheel/control input is allowed;
GNSS and fused velocity remain evaluator-only. A candidate must improve both
healthy-data accuracy or dropout prediction without losing fault recovery.
3. Compare current fixed-gain position corrections with an uncertainty-aware
along-track offset/drift state. GNSS can update Navigation position uncertainty
and offset, but never the published longitudinal velocity/distance estimator.
All existing stale/frozen GNSS, physical-baseline and branch gates remain.

These are testable alternative hypotheses, not promised gains. Scratch branches
and bounded parameter sets are selected on train only. Freeze each recommendation
before inspecting validation/historical holdout/public reference. Report data
coverage, common masks, session/bag tails and failures, not merely pooled scores.
The already inspected public data is diagnostic, never a new independent test.

Integrate only a supported improvement with behavioral tests. A rejected idea
still produces reproducible code and a documented result. Independently review
any accepted combined patch, test both vehicles and existing faults, then run a
full ROS1x checker only if runtime changes. Update the source ZIP/manifest and
publish code plus all measured improvements and regressions.
