# Final integrated speed evidence

This is the final **integrated** estimator comparison against the exact
pre-round-2 baseline, which already had healthy-wheel gain 0.90. The earlier
speed research selection is retained separately in `speed_study_round2.md`
(and `speed_study_round2.json`); its isolated candidate numbers are not the
final implementation numbers below.

Frozen source and binary SHA-256:

- `estimator.cpp`: `c22fa3a0e881c424dfb170fc78ebc7d627c3f5d9abcd3eb4c29f711b08bfbd2c`
- `estimator.hpp`: `d701f247c71df5643eac54b16cc60fd35380723480ccd6937a89607d960007d5`
- Portable final `replay_cli`: `1d537657d2afc1c75d18387d4527719a3107d3ba8c56b31b73d6698451cfdf39`

`speed_study_final.json` contains full per-bag nominal and blackout metrics,
whole-session summaries, exact common-mask hashes, source/binary/data provenance,
and all 72 final synthetic fault cases. `speed_study_raw/` preserves the full per-window JSON results. The local
`evaluation/runs/speed_study_final_raw.tar.gz` additionally preserves final
estimator/header/test snapshots; runtime source hashes identify the same files
in this source release.
The pre-change source is also preserved by the coordinator in
`evaluation/runs/round2_baseline/tram_odometry_source.zip` in the main checkout.

## What is integrated

The accepted short residual activates only when both wheel channels are stale.
It uses past raw wheel acceleration, fades with the existing 0.35 s staleness
parameter, and leaves healthy physical/table prediction, gain and slow-adaptation
residual unchanged. Any detected anomaly clears the residual, both derivative
histories and cached late-wheel projection acceleration. Pending or tentatively
accepted wheel jumps cannot train or use this residual.

The shared recovery work also persists independent common-jump evidence. Review
found and fixed a rate-dependent hole: strong raw discontinuities now use a
separate monotonic raw sensor-header history, independently of the derivative's
45 ms minimum interval. Normal effective-time processing and output timestamps
are preserved. Tests cover 10/20/25/50/100 Hz, both wheel orders and jump signs,
late ordered callbacks sharing one filter frontier, older callbacks, correct
return, normal acceleration, and bounded noise.

## Final nominal GNSS proxy

All before/after pairs use identical common output stamps and reference masks.
Table model is enabled for 30618 and disabled for 30639. Values are m/s RMSE.

| Vehicle/date | Split | Matches | Before | Final |
|---|---|---:|---:|---:|
| 30618 / July 27 | Train | 648,428 | 0.054758101 | 0.054757725 |
| 30618 / September 3 | Train | 107,354 | 0.087993322 | 0.088133195 |
| 30639 / August 26 | Train | 183,696 | 0.045503698 | 0.045503698 |
| 30618 / August 26 | Validation | 286,515 | 0.033145650 | 0.033145619 |
| 30618 / August 10 | Historical holdout | 209,747 | 0.121821625 | 0.122032887 |
| 30639 / May 5 | Historical holdout | 222,021 | 0.114212851 | 0.114210359 |

Combined 30618 train RMSE is 0.060599981 → 0.060628555. The small September and
historical 30618 regressions are retained as recovery tradeoffs. The largest
bag regression is `30618_40ffd323`, +0.001181449 m/s; next is
`30618_27e994fc`, +0.000414607. No tuning followed these results. Historical
holdout influenced prior model-mode choices and is not a fresh independent
holdout. Learned tables/maps were not refit per session, so this is not
session cross-validation.

## Final paired-wheel blackout validation

The 284 identical windows delete both wheel streams for 5.12 seconds. Scoring
uses common causal output endpoints at 1, 3 and 5 seconds. GNSS selects clean
windows only in the evaluator; it never reaches the speed process.

| Horizon | Speed RMSE before → final (m/s) | Distance RMSE before → final (m) |
|---|---:|---:|
| 1 s | 0.169680 → 0.134484 | 0.093979 → 0.076536 |
| 3 s | 0.410095 → 0.379171 | 0.665689 → 0.577486 |
| 5 s | 0.551532 → 0.533883 | 1.597571 → 1.466822 |

All complete train and historical holdout sessions improve speed and distance
RMSE at each of these horizons. Some individual bags/windows regress; full
per-bag results are available in the JSON. Historical 30618 five-second errors
are 0.654402 → 0.640533 m/s and 1.842374 → 1.727527 m.

## Public and synthetic checks

Public bag `30618_88aea4d9` results are exactly unchanged: fused signed-x RMSE
0.04773500545 m/s on 26,188 matches, sparse GNSS 0.03710755357 on 1,045 matches.
The wheel-mean baseline remains better against fused velocity at 0.02683912016.
These are original-timestamp nearest-within-50-ms diagnostics, not official ATS.
The coordinator's final ROS run supplies the official integrated result.

Final synthetic clean speeds are unchanged. Paired-dropout fault-interval RMSE
improves 0.094600 → 0.088982 for steady motion and 0.358740 → 0.321211 for the
trip profile. Common-jump error is lower but not zero: for a 10-second upward
jump, steady fault-interval RMSE improves 3.884318 → 2.923341, trip
3.895431 → 3.149215. Independent model uncertainty eventually permits tentative
acceptance of a sufficiently long common fault; smooth common bias also remains
unobservable. Full 72-case results, coverage and recovery metrics are retained.

The fallback behavioral regression failed recovery-only at 0.432643 m/s after a
one-second paired dropout and passes final at 0.313852. Reset, return, fault
isolation and tentative-history tests pass. Deliberately removing history
invalidation fails the poison regression. A separate cache regression failed
with a 0.005219 m/s discrepancy caused solely by a 10 μs advance between malformed
and late callbacks; clearing the cache makes it pass. All estimator test
executables pass on the frozen source.

## Reproduce the final comparison without rebuilding candidates

Compile the final checkout once and preserve that binary. The study script's
external-executable mode records hashes and never changes estimator source.

```bash
python3 evaluation/speed_candidate_study.py --root /path/to/frozen-baseline \
  --out /tmp/final-speed-validation --split validation --blackouts \
  --baseline-exe /path/to/frozen-baseline/build/replay_cli \
  --candidate-exe /path/to/final/build/replay_cli
```

Use `--split train` or `--split holdout` with separate output directories for
other sessions. Public and synthetic commands are listed in the isolated study;
pass the final fixed binary as their `--after` argument. The old
`speed_study_stale_only.patch` is the exact **isolated scratch candidate** diff,
not an instruction to reapply it over this integrated runtime.
