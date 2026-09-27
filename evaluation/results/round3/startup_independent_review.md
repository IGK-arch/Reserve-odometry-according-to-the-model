# Independent RTK-startup diff review

Reviewed the working-tree `navigation.cpp` / `navigation.hpp` change after the
XY experiment, without production edits.

**P2: a usable mixed-status antenna pair loses heading at an RTK anchor.**
`finalizeStartupAnchor(bool rover, bool rtk)` binds both master and rover heading
collections to the same quality bucket. An RTK master anchor therefore ignores
all lower-status rover observations, even when their timestamps, physical
baseline and headings agree. This regresses the earlier paired-heading behavior
and conflicts with preserving usable paired heading independently of preferring
RTK for the position median.

Reproduced with the current sources: stationary ENU master=(0,0,3),
rover=(0,12.436,3), three pairs at 100.0/100.1/100.2 s; master status=2,
rover status=0. With no map (a supported Navigation mode), expected base_link
is (0,9.873,0), yaw π/2. Actual is (9.873,0,0), yaw 0,
heading_source=configured, despite a successful RTK master anchor. The all-RTK
control yields the expected dual_antenna result. This changes XY by 13.96 m.
The same missing heading can affect the rigid offset and direction selection
with a map wherever its local tangent is not the observed body axis.

Suggested correction: keep RTK position medians isolated; select heading pairs
independently, preferring RTK/RTK and falling back to a valid usable opposite
antenna under the existing time/baseline/coherence gates. Include a mixed-status
heading regression in addition to all-RTK heading tests. This finding is a
review request, not an instruction to use every lower-quality fix unconditionally.

Reproduction source: `/private/tmp/round3-startup-review.cpp`.

```sh
c++ -std=c++17 -O2 -Iros2_ws/src/tram_odometry/include \
  /private/tmp/round3-startup-review.cpp \
  ros2_ws/src/tram_odometry/src/navigation.cpp \
  ros2_ws/src/tram_odometry/src/estimator.cpp \
  -o /private/tmp/round3-startup-review
/private/tmp/round3-startup-review
```

Observed output:

```
rover_status=2 anchored=1 heading=dual_antenna yaw=1.5708 x≈0 y=9.873
rover_status=0 anchored=1 heading=configured yaw=0 x=9.873 y≈0
```

No other actionable regression was established by this focused source review.

## Disposition after owner review

The reproduction is confirmed. Mixed-quality heading exclusion is intentional in the approved RTK policy: an RTK anchor must not be rotated by a lower-quality companion. The user-directed disposition is to retain that policy and document this as an accepted accuracy limitation; no runtime edit is made from the public score. The original suggested fallback is therefore not adopted. `public_startup_delta.md` quantifies the 9.36-degree/2.14 m startup difference and +0.06998 m aggregate public RMSE cost. The no-map 90-degree fixture has 13.962531 m position error at rest and can diverge further with travelled distance. This disposition does not dispute the review's reproduction or claim that excluding mixed-quality pairs always improves accuracy.
