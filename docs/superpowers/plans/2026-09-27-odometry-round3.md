# Odometry Round 3 Implementation Plan

> For agentic workers: use superpowers:subagent-driven-development. Independent file ownership is required.

**Goal:** Resolve reviewed recovery defects, prefer guaranteed RTK, evaluate official XY geometry and deliver a verified source package.

**Architecture:** Preserve Estimator/Navigation; isolate wheel recovery, startup anchoring and map research. Freeze 50bd771 for every comparison.

**Tech Stack:** C++17, ROS2 Humble, Python replay/evaluation, CMake/CTest.

- [x] Save baseline hashes in evaluation/results/round3/baseline_freeze.json; baseline executables remain in /private/tmp/odometry-pull-50bd771.
- [x] Recovery owner: write failing behavioral tests for single resumed wheel, stale paired endpoint and +4→0→+4; repair estimator.cpp/hpp. Test 10–100Hz, return/reset and healthy plateau counterexamples. Preserve pending correction causality and distinguish unresolved observability.
- [x] RTK owner: write failing startup tests for status0/1 preceding status2, absent RTK, delayed/stale pair, reset; update navigation.cpp/hpp only and dedicated tests. Keep bounded startup deadlines and existing interfaces.
- [x] Map researcher: compare official line with existing base_link geometry using train data and independent geometry fixtures. Work in scratch/evaluation files; production navigation belongs to RTK owner until handback. Return concrete adoption/rejection evidence.
- [x] Root: review both implementations, run portable Release and Python tests, synthetic suite for both vehicles, nominal and dropout comparisons, full public Navigation and GNSS scenario checks where behavior changed.
- [x] Root: build Ubuntu ROS packages and run final official 1x bag with recorder/checker/resource metrics; run no-GNSS and30639 contracts.
- [x] Independent review of final diff; resolve actionable findings and repeat affected checks.
- [x] Update docs/RESULTS and metric comparison with explicit source versions, regenerate source archive, verify extraction/build/tests and manifest.
