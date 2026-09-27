# Frozen Kalman candidate: validation and historical holdout

The selected `long_guarded` executable was not changed after train selection.
SHA256: `b37a73245bfc141b2975f4b65482ee6c736cd19e44ebdecf8c5e581e4da2c143`.
Production was not edited. These are GNSS speed-proxy diagnostics, not official
fused-reference scores; historical holdout has been inspected in prior rounds.

| Split / vehicle | Matched nominal outputs | Nominal RMSE before→after, m/s | 5 s dropout speed RMSE, m/s | 5 s dropout distance RMSE, m |
|---|---:|---:|---:|---:|
| Validation 30618 | 286515 | 0.033146→0.030684 | 0.533883→0.388231 | 1.466822→0.984743 |
| Holdout 30618 | 209747 | 0.122033→0.118773 | 0.640533→0.628475 | 1.727527→1.536398 |
| Holdout 30639 | 222021 | 0.114210→0.115283 | 0.791326→0.605479 | 2.106125→1.533353 |

Validation includes all 13 bags and 284 dropout windows per horizon. Holdout
includes 42 bags, of which 23 have a usable proxy; the 19 unscored 30618 bags
are explicitly listed in the report. Holdout uses 442 windows per horizon
(211 for 30618, 231 for 30639). Both versions have identical nominal masks,
output counts and selected blackout endpoint sets. Results include 1/3/5 s.

The validation session (30618, 2026-08-26) improves nominal RMSE by 7.43% and
5 s dropout speed RMSE by 27.28%. Twelve of 13 nominal bags improve; the worst
bag regression is 30618_33bec73f, 0.048968→0.049733 m/s (+1.56%).
The 30618 holdout session (2026-08-10) improves pooled nominal speed by 2.67%,
but 5 s dropout speed improvement is only 1.88%. The 30639 holdout session
(2026-05-05) improves dropout prediction while nominal speed worsens 0.94%;
worst bag 30639_3b3d9eb8 is 0.227560→0.233884 m/s (+2.78%).

The local dropout regressions must accompany these averages. For 30618 holdout,
5 s speed absolute-error p95 worsens 1.000607→1.088767 m/s and maximum worsens
3.912534→4.786080 m/s. Distance absolute-error p95 worsens 2.109787→2.371328 m,
even though maximum improves 12.866068→11.111632 m. The largest local worsening
occurs in 30618_4d487b0d, window start 1786374990 s: 5 s speed error
−0.233230→−2.006331 m/s and distance error −0.868246→−6.213260 m.
Validation also has local regressions: 30618_67b89902, window 1787742720 s,
5 s speed error 0.753738→1.754758 m/s and distance 0.449092→3.439734 m.
All per-window top tails are included in the comparison JSONs.

Recommendation: retain this as a promising experimental candidate; the nominal
30618 benefit and aggregate dropout benefit generalize, but these results do
not justify unconditional production replacement. In particular, the 30618
holdout dropout p95/max speed regressions should be characterized before
adoption, and 30639 has a distinct nominal-versus-dropout tradeoff. No parameter
retuning on these validation/holdout windows was performed. A future structural
revision must return to train and analytic counterexamples and be identified as
a new candidate, rather than silently modifying this frozen result.

Full per-bag/session metrics, per-window errors, coverage and hashes are in
`kalman_validation_{results,summary,comparison}.json` and
`kalman_holdout_{results,summary,comparison}.json`. The report generator is
`evaluation/round4_kalman_generalization.py`.
