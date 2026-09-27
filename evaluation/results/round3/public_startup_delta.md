# Public startup delta: frozen-code diagnosis

Full-navigation nearest-reference 3D RMSE changes from **1.474923422 m** for frozen 50bd771 to **1.544900912 m** for round3 v1: +0.069977490 m (+4.744%). This is the combined effect of startup anchor/heading selection, not an estimator change or a scoring-mask artifact. No code or parameters were selected from this result.

## Evidence and isolation

The four baseline profiles were freshly replayed with the preserved 50bd771 executable into `evaluation/runs/round3_public_delta_baseline`. All use the exact same permitted-input CSV as `round3_reference`. Speed and wheel distance are identical in every one of 26896 output rows for all four profiles. Historical round2 and fresh50bd positions differ by less than 1e-9 m due to floating-point builds; their scores agree to displayed precision.

Diagnostic copies of the replay append only public yaw, anchor source, heading source and anchor distance fields. Both diagnostic builds use the 50bd771 estimator. One uses 50bd771 navigation, the other the frozen RTK navigation. Their original output fields match their respective baseline and v1 full CSV outputs **exactly**. Sources/binaries and input/output hashes are in `public_startup_delta.json`; the appended-field replay source and analysis script are saved alongside it. Runtime source was not changed.

| Profile | 50bd771 RMSE, m | Round3 v1 RMSE, m | Common position matches |
|---|---:|---:|---:|
| Startup only |5.656809869 |5.676123255 |26884 |
| GNSS corrections |5.623240518 |5.641999085 |26884 |
| Branch selection |1.483447434 |1.553040904 |26884 |
| Full, with elevation |1.474923422 |1.544900912 |26884 |

## Mechanism

The public recording begins with 37 master fixes of status 0 during the first five vehicle-time seconds. Rover RTK first arrives at 1.232810398 s; master RTK first appears at 11.232810398 s, outside the existing startup window. Baseline anchors on the lower-quality master and obtains `dual_antenna` heading from the mixed-quality antenna history. RTK preference instead anchors on the RTK rover after the unchanged rover delay and uses `route_body_heading`, because an RTK/RTK heading pair is unavailable.

Baseline first output: master/dual heading, yaw 1.251341433 rad. RTK first output: rover/route heading, yaw 1.414720446 rad. Their yaw difference is 9.360928 degrees and their initial position difference is 2.136544 m. Position source and heading change together; these measurements do **not** assign all 2.14 m to heading alone.

The RTK output starts 0.099976881 s later and omits one position output (26890→26889). That sample is before the reference coverage begins, so **both versions retain 26884 matched positions**. Coverage loss does not cause the RMSE increase.

The first 60 s contribute +6215.892 m² to the sum of squared errors, exceeding the total net increase of 5681.120 m² because later errors partially improve. At wheel distance below 1 cm, 677 matched rows have 1.332727→2.680867 m RMSE. Startup residual/heading corrections decay with distance, so the stationary beginning prolongs the difference. From 300 s onward the largest output difference is less than 0.000608 m; from 600 s onward it is floating-point noise. This localizes the regression to startup.

## Intentional heading tradeoff

The independent review's mixed-status fixture is valid and its 13.96 m no-map result is retained as a documented risk. The approved policy intentionally prevents a status 0/1 companion from rotating a status 2 anchor, even when the observed baseline happens to pass the existing geometry checks. Those checks cannot prove that a lower-quality antenna is accurate. Conversely, an accurate mixed-quality pair may contain valuable heading that this policy discards.

For a master anchor and a 9.873 m lever arm, heading error θ alone yields 2 × 9.873×sin(|θ|/2) position error at rest: 13.962531 m for 90 degrees and up to 19.746 m for 180 degrees. With no map, configured-heading error also accumulates with motion; it does not receive a map-based decay correction. With a map, learned body heading may differ from the actual parked body axis. The public result above demonstrates an aggregate accuracy cost of the conservative policy on this one known recording. It does not justify tuning the policy against that reference, nor establish that mixed-quality heading is generally safe.

The 46-bag GNSS study (`evaluation/runs/round3_gnss/report.json`) uses a paired-GNSS proxy, not the fused public reference. It reports five lost absolute matches per profile across all sessions and nearly unchanged common-mask session RMSE; that does not refute this startup-local public difference. The proxy's availability and shared GNSS source limit its ability to independently validate mixed-quality startup geometry.

## Limits

This is a post-freeze diagnostic on an already-inspected public recording. Matching is nearest reference within 50 ms, not official ROS ATS. Status2 is a preference signal, not an accuracy guarantee. The comparison isolates Navigation as a whole; it does not separately score counterfactual anchor-only and heading-only variants. No production edit or parameter tuning was performed.
