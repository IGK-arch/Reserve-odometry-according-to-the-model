# Odometry round 2 implementation plan

> For agentic workers: use systematic-debugging, test-driven-development and dispatching-parallel-agents. User has authorized autonomous execution of the previously discussed plan.

**Goal:** improve 30618 accuracy and failure handling with measured evidence and no hidden-test claims.

**Architecture:** retain the shared Estimator/Navigation core; explore speed changes in isolated copies, own estimator recovery and navigation corrections separately, then integrate accepted candidates. No new sensor dependency.

**Tech Stack:** C++17, Python standard library/NumPy/SciPy, ROS 2 Humble, Docker.

- [x] Preserve current source and build immutable baseline. Record organizer constraints and acceptance criteria in the accompanying spec.
- [x] Speed experiments: create reproducible candidate study under evaluation; own scratch estimator variants, report nominal and blackout metrics before proposing production edits. Parameter selection is train-only. Save accepted/rejected approach evidence.
- [x] Recovery: reproduce persistent shared jump using the public Estimator API in a standalone C++ test; observe failure, change estimator recovery state, verify recovery/acceleration/reset behaviors and all existing core tests.
- [x] GNSS: reproduce biased fixes using synthetic route and known truth; experiment in scratch copies, implement selected guard with tests, verify all validation stress scenarios and branch tests.
- [x] Evaluation: add reproducible synthetic known-truth fault benchmark with common evaluation times, per-scenario and aggregate results, and tests for corruption schedules. Preserve detector-free clean control and held-back deterministic scenarios.
- [x] Integrate only supported improvements; rerun train and validation for 30618 and compatibility checks for 30639. Keep public-reference/holdout evidence separate from parameter selection.
- [x] Review source and claims; run CTest, Python tests, ASan/UBSan, full 1x ROS official checker and resources.
- [x] Update docs, archive experiments, build and independently rebuild release; integrate into main with source hash checks and report limitations.

- [x] Physical body-heading geometry from organizer dimensions; consistent paired-heading lever arm; independent circle/grade tests, train/validation diagnosis, no bag-specific offsets.
