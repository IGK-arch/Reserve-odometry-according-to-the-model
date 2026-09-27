# Startup experiment decision

The frozen recommendation is the **pure rover geometry correction**, with the strict RTK heading policy preserved. The three mixed-quality experiments are rejected. Selection was frozen at 2026-09-27 15:01:43 UTC before validation, historical holdout or public-reference evaluation. No public reference was read by this experiment. The already reported public regression was background context only.

## Accepted geometric correction

At the five-second deadline, a sufficient rover RTK window can be selected even when the master has only one or two RTK observations. That sparse master observation can still supply a coherent dual-RTK heading. Previously, map anchoring subtracted the rover lever arm in the map's body direction, while output applied the master-to-base lever in the measured direction. This violates rigid-body geometry.

The pure correction rotates the rover lever around Z by the same observed course difference before storing the master-position residual. It uses the actual map antenna offset vector, preserving map grade. It changes no anchor selection, quality gate, output deadline, paired-heading acceptance, wheel speed or distance logic. Height remains under the existing map/elevation policy.

The regression fixture first failed on immutable baseline: a stationary rover anchor with a −0.35 rad body deviation misplaced base_link by 4.33042 m. The candidate passes eight cases spanning both vehicles, positive and negative course deviations, flat track and a 0.6 vertical tangent component; maximum XY error is below 1e-9 m. Separate curved-map deadline cases pass too. These synthetic positions are independently constructed from physical lever arms, not derived from bag proxies. The grade test explicitly checks XY because production retains its existing mapped height policy.

The complete 120-second replay CSVs are byte-identical on all **42 train bags**. This proves preservation on these recordings, not a measured real-data improvement. Both off-map bags remain included in coverage and produce no scored absolute matches. The frozen source and executable hashes are in `startup_decision.json`; exact per-bag output hashes are in `startup_train_pure_parity.json`.

## Rejected mixed-quality hypotheses

Three predetermined variants separated heading evidence from the RTK position anchor:

1. `mixed`: retain RTK position selection, but allow a non-RTK companion only with three unique temporal pairs, at least 0.15 s of evidence, pair timestamps within 75 ms, horizontal baseline within 0.75 m of 12.436 m, maximum angular deviation within 3°, and map-course disagreement no greater than 35°. Without a map, retain the previous policy.
2. `rigid`: the same heading gate plus consistent rover lever geometry for mixed headings.
3. `joint`: the preceding variant plus a fixed 0.04/1.04 contribution from the companion's inferred anchor. This is an experimental fixed variance ratio, not a covariance calibrated from these receivers.

No parameter was adjusted using the public reference or held-out bags. A later grade-preservation correction replaced an initially horizontal-only rover lever calculation with the map's actual lever vector; it was a geometry fix rather than a parameter fit.

All variants preserved output stamps, absolute coverage, wheel velocity and distance. Across 40 scored bags, every mixed variant improved one bag, worsened two and left 37 unchanged. The changes are too sparse and inconsistent to support adoption.

| Variant | First 30 s XY RMSE, m | First 120 s XY RMSE, m | Worst bag RMSE increase, m |
|---|---:|---:|---:|
| Baseline | 2.140098 | 4.746513 | — |
| Mixed heading | 2.149395 | 4.748694 | 0.299564 |
| Mixed + rigid rover pose | 2.138203 | 4.746651 | 0.025263 |
| Mixed + joint pose | 2.137959 | 4.746609 | 0.019169 |
| Pure RTK geometry | 2.140098 | 4.746513 | 0 |

There are 1,044 first-30-second and 4,244 first-120-second proxy matches. The worst mixed regression is `30618_68d1748a`: baseline 1.362030 m, mixed 1.661594 m, rigid 1.387293 m and joint 1.381199 m. `30639_2b4a6347` also worsens for rigid and joint; `30639_4285f2bc` improves for those variants. Session-level results and every bag/window are saved separately. The two unscored bags are `30618_0d865417` and `30618_bc5e53c2`.

## Synthetic limits and adversarial cases

Each of the five versions was evaluated on 52 cases combining a straight/curved map, RTK master/rover, correct 0°/15°/30° parked course, a coherent 90° bad companion, 9 m/16 m malformed baselines, alternating ±6° angles, shifted timestamps, coherent 4° heading bias, common 2 m position bias, moving startup, no map, and sparse-master RTK deadline selection.

The constrained mixed variants reject the tested 90° companion, malformed lengths and inconsistent angles. Correct parked 15°/30° headings become accurate only with consistent rover geometry. Coherent 4° bias passes all physical and temporal gates: even `rigid` retains approximately 0.689 m master-based or 0.179 m rover-based position error. Common 2 m translation also remains unobservable. These gates cannot establish truth when both readings are coherent but wrong.

A 100 ms shift at 10 Hz is rejected in the early master-anchor case, but rover fallback can re-pair later samples having matching timestamps. On a parked body this is valid; this experiment does **not** claim blanket rejection of delayed companions. Moving mixed-rover startup retains 0.26–0.41 m error because startup aggregation and map shape still affect the inferred pose. No conclusion of universal robustness is supported.

## Data and evaluation limits

Inputs are the existing cached sparse-GNSS replay inputs, dense in their first 30 seconds, truncated to the first 120 seconds by reception time. The evaluator consumes the existing clean paired-GNSS proxy, downsampled one in ten, and compares nearest outputs within 50 ms on the common absolute mask. Proxy timestamps are restricted to the replay interval. It reports XY without fitted translation, rotation or time alignment.

The proxy is constructed separately from paired GNSS, but shares the same physical receivers and early observations with the navigation inputs. It is not independent fused truth. These recordings are previously studied train data, and the 120-second study does not establish full-route performance. Input, proxy, binary and source hashes are saved. There were no lost or gained absolute matches for any variant.

## Reproduction and exact integration

Run from the repository root, with immutable source `/private/tmp/odometry-round4-source`, baseline executable `/private/tmp/odometry-round4-baseline/navigation_replay`, and existing `/private/tmp/round3-xy` input/proxy caches available:

```sh
python3 evaluation/round4_startup_build.py
python3 evaluation/round4_startup_study.py
```

The build script writes scratch sources/executables and the four archived patches. It never modifies production. The accepted source is `/private/tmp/round4-startup/pure/navigation.cpp`; its only difference from immutable baseline is shown in `evaluation/round4_startup_pure.patch`. If no parallel production edits exist, copy that source over `ros2_ws/src/tram_odometry/src/navigation.cpp`. If integrating alongside another Navigation change, transplant only the added rover-offset block immediately after `route_yaw` is computed in `finalizeStartupAnchor`.

Copy `evaluation/round4_startup_geometry_test.cpp` to the production test directory as `navigation_startup_rover_geometry_test.cpp`, replacing its hard-coded scratch fixture directory with `std::filesystem::temp_directory_path()` and a unique fixture filename. The test needs the usual navigation library only. Compile against baseline to see the first assertion fail, and against the accepted source to see all eight cases pass. `startup_geometry_test.json` contains both command outcomes; `startup_ctest.txt` contains the existing portable test suite run against accepted source.

The general synthetic CSVs can be reproduced by compiling `evaluation/round4_startup_synthetic.cpp` with immutable `estimator.cpp`, the desired scratch `navigation.cpp`, C++17, and the immutable include directory, then running the executable. Both synthetic programs use only `/private/tmp/round4-startup` for fixture maps.
