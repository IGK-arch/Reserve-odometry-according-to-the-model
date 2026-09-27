# Round 4 experimental implementation plan

**Goal:** Improve startup/heading and test larger algorithmic alternatives with reproducible evidence.
**Architecture:** Immutable baseline, independent scratch experiments; root owns production integration.
**Tech stack:** C++17 portable/ROS2Humble, Python/NumPy/SciPy evaluation.

- [x] Freeze5a82712 sources and binaries; save evaluation/results/round4/baseline_freeze.json.
- [ ] Startup owner: evaluation/round4_startup* and results/round4/startup* only. Generate scratch Navigation variants; test independent synthetic geometry first, then42train bags. Freeze recommendation before validation/public. Return exact patch, hashes, coverage and tradeoffs.
- [ ] Kalman owner: evaluation/round4_kalman* and results/round4/kalman* only. Causal speed/acceleration predictor variants, independent known-motion fixtures, train nominal/dropout and fault regressions. Compare immutable executables; freeze recommendation before other splits.
- [ ] Position owner: evaluation/round4_position* and results/round4/position* only. Scratch Navigation offset/drift filter, preserve gates/branch/startup. Test bias drift vs false GNSS; train sparse/delayed/frozen modes and tails. Freeze before validation/public.
- [ ] Root: inspect reports and integrate only supported changes with failing behavioral tests first; no concurrent production writers.
- [ ] Root: validate combined candidate on all97CFR bags,46GNSS bags/four modes as relevant, both synthetic84sets, public full replay and independent code review. Preserve rejected ideas and all regressions.
- [ ] If runtime changed, freeze sources and build Ubuntu packages, run official full1x and contracts, collect latency/resources and common-mask comparison.
- [ ] Update round4 report/metric index, verify extracted source release, regenerate manifest, merge/push main within standing user authorisation.
