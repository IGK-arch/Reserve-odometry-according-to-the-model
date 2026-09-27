# Third-round design: recovery, RTK and surveyed geometry

User approved continuing the five-step plan after the 50bd771 audit. This
implements that plan without another approval round.

Keep the current physical estimator and shared Navigation interface. Fix the
three demonstrated recovery errors with behavioral regressions first: one fresh
resumed sensor, stale paired endpoints, and notch changes that return to the
original value. Add bounded, timestamp-based evidence so detector behavior is
not an accident of wheel sampling rate. Distinguish unobservable healthy
plateaus from identifiable recovery; report any remaining tradeoff explicitly.

Startup should prefer status=2 fixes during the existing bounded startup window,
while preserving startup without GNSS, fallback to usable fixes when RTK is
absent, paired heading and vehicle-time resets. No fused reference enters the
runtime. Geometry research uses official Pathgraph and train data, compares
covered rail centerlines, and preserves opposite rail and terminal topology.
Adopt an XY correction only if independent geometry tests and train evidence
support it; otherwise retain the current map and document the rejected approach.

Alternatives: replacing the entire estimator by an EKF introduces unvalidated
behavior beyond the audited bugs; blindly snapping all positions to the single
official polyline confuses rails/loops. Targeted fixes and bounded geometric
experiments are the chosen approach.

Freeze 50bd771 source/binary hashes before changes. Measure synthetic faults,
all unique real recordings, full public Navigation, then one final ROS 1x run
with official checker, external DDS latency and resources. Preserve historical
results, publish comparable masks and regressions, build and test the source
archive and refresh the manifest. No parameter fitting to the public reference.
