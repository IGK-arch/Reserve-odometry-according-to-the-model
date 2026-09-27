# Official XY experiment: reject production change

The official Pathgraph contains 4,709 base_link points over 4,707.960 m on the
return rail. It is not a replacement for both directions or terminal loops.
The organisers' September 27 clarification explicitly allows reconstruction of
loops and depot dead ends. Existing reconstructed loops remain necessary; this
experiment neither deletes them nor claims to have surveyed depot tracks.

On the current return map, applying the physical master→base_link transform
with 6.098 m heading lookahead gives **0.0508 m cross-track RMS**, median
0.0290 m and p95 0.0993 m against the official interior. This covers 2,306 of
2,703 2 m nodes (s=226–4,836 m), with a 50 m official endpoint guard and 8 m
matching corridor. The outbound map is 3.51 m away at the median, RMS 4.28 m;
its local tangent points in the opposite direction (cosine ≈−0.99999). Full
replacement would collapse the opposing rails and miss loops up to 526 m away.

Train-only paired-GNSS geometry diagnostic, substantial direction segments
with ≥100 covered moving samples per bag:

| Vehicle / direction | Bags / samples | Current rail RMS, m | Official line RMS, m |
| --- | ---: | ---: | ---: |
| 30618 out | 15 / 8,373 | 0.6364 | 4.2605 |
| 30618 return | 15 / 8,822 | 0.4132 | 0.4132 |
| 30639 out | 4 / 2,074 | 1.3540 | 3.4760 |
| 30639 return | 4 / 2,543 | 0.4238 | 0.4195 |

These are nearest-rail cross-track diagnostics, not complete odometry errors or
independent truth: the current map itself was constructed from train GNSS.
Heading agreement excludes the opposite direction; pair interpolation and
cleaning are scoring operations. No fused truth or held-out bag was read.

A scratch-map candidate tested the remaining plausible benefit. It displaced
only return master nodes by the official base_link lateral residual (CB→ENU
local Jacobian), with a fixed 0.5 m eligibility bound, 50 m endpoint exclusion,
100 m taper and 10 m Gaussian smoothing. It retained every outbound vertex
exactly, retained uncovered return vertices and endpoint XYZ, and recalculated
return metric s. Maximum node displacement was 0.1512 m; total s grew 0.2540 m.
Analytic finite-segment projection and exact outbound preservation checks passed;
s stayed strictly increasing and the maximum adjacent correction change was
0.0210 m. No production map was edited.

The same immutable 50bd771 Navigation executable replayed all **42 unique train
bags** with identical selected allowed inputs: both GNSS channels for 30 s,
then alternating single-antenna 2 s windows every 120 s. Forty bags had matched
absolute output, yielding **42,567 common proxy samples**; the two unanchored
bags remain counted in coverage and are not zero-error runs. Scoring uses one
in ten cleaned paired-GNSS proxy timestamps, nearest output within 50 ms. There
is no time or position registration. Replay stamps, velocity, wheel distance
and absolute-valid masks matched exactly.

| Common-mask XY statistic | Current | Scratch candidate |
| --- | ---: | ---: |
| Pooled RMS, m | 6.029681 | 6.030769 |
| Median bag RMS, m | 1.793718 | 1.795381 |

Four bags improved, 17 worsened and 19 were unchanged. The candidate provides
no substantial train improvement and complicates geometry for a centimetre-scale
nominal correction. **Retain current XY and the existing official-height use.**
The rejection was frozen without opening the public fused reference. These
results do not claim that GNSS is more accurate than the official survey; they
show that this existing return geometry is already extremely close to it and
that this bounded integration adds no demonstrated benefit.

Reproduction from repository root (NumPy, SciPy and C++ compiler required):

```sh
python3 evaluation/round3_official_xy.py --train --outdir /private/tmp/round3-xy
python3 evaluation/round3_xy_replay.py --outdir /private/tmp/round3-xy \
  --baseline-exe /path/to/frozen-50bd771/navigation_replay
```

The second script intentionally pins
`/private/tmp/odometry-pull-50bd771/navigation_replay`, SHA-256
`1b529e0e02ddecdc1fd0304ed1918ba1fd7f238316e5c7ae1fd5a93b10535b98`.
Raw scratch maps, per-bag replay CSVs, proxy samples and input hashes remain in
`/private/tmp/round3-xy`. JSON reports beside this note retain per-bag metrics,
coverage, masks, map checks, provenance and the decision hash. Summary tables
pool squared errors with sample counts, without averaging bag RMS values.

The saved measurements retain the original script hashes. The current CLI also accepts
`--pathgraph` and `--baseline-exe` to reproduce them outside the original scratch paths.
