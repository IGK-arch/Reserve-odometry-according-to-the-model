# Tram backup odometry (ROS 2 Humble)

The node consumes only driver notch and the two bogie speed channels in its running velocity/distance estimator. The supplied wheel values are **km/h** despite the custom message comment; `/result/velocity` is **m/s**. GNSS `master/fix` sets the initial route position and can provide bounded later corrections to the route anchor. If master is absent, `rover/fix` provides a lower-confidence startup fallback; both antennas can supply later corrections and branch evidence. Fixes never update estimated wheel speed or travelled distance. There is no IMU, GNSS velocity, camera, lidar or `/localization/kinematic_state` subscription.

Historical WSL/ROS 2 Humble reports for the earlier startup-only implementation are retained in `evaluation/ros_smoke` at repository root. They do not validate the current shared navigation implementation. Current measured results belong in [`docs/RESULTS.md`](../../../docs/RESULTS.md). The organisers specified the judge's MGRS coordinate convention and antenna-to-`base_link` transform; these are the defaults below.

The workspace includes a local copy of the dataset's `tram_vehicle_msgs` package with the standard `ament_cmake` discovery, maintainer field and ROSIDL runtime export required by `colcon`. The original files under `dataset/tram_vehicle_msgs` are unchanged.

## Build and replay

Ubuntu 22.04 with ROS 2 Humble and `colcon`:

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --packages-up-to tram_odometry
source install/setup.bash
ros2 launch tram_odometry tram_odometry.launch.py vehicle_id:=30618 route_direction:=auto
```

In a second sourced shell:

```bash
ros2 bag play /path/to/30618_2050d396 --clock
```

In a third sourced shell:

```bash
ros2 topic info /result/velocity -v
ros2 topic info /result/position -v
ros2 topic hz /result/velocity
ros2 topic echo /result/diagnostics
```

`/result/diagnostics` contains `callback_to_position_publish_ms` measured with a monotonic wall clock from callback entry until the position publication returns, and `output_rate_hz_1s` measured from publications in the latest one-second window. These fields provide runtime checks independent of bag timestamps. For the judge, record several minutes of this topic and inspect the latency distribution alongside `ros2 topic hz /result/velocity`.

The subscriptions use best-effort SensorDataQoS to match rosbag publishers. The first accepted wheel measurement initializes the estimator and starts `/result/velocity`; subsequent accepted events publish when they advance its time. `/result/position` begins after the startup GNSS anchor is established, or after `startup_gnss_window_s` expires without one; this avoids sending temporary relative coordinates into the judge's MGRS comparison. It never backfills old timestamps. Both outputs use the exact triggering input `header.stamp`. The 20 Hz driver channel provides regular publication triggers after initialization; measure the actual output rate in each ROS run. Use a single-threaded executor, as in `main`, for deterministic event order.

The shared `Navigation` class contains map matching, startup, GNSS corrections,
branch selection and coordinate output. The ROS node is an adapter; the portable
replay calls the same class. From the repository root, without ROS or a bag:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j2
ctest --test-dir build --output-on-failure
python3 -m unittest discover -s evaluation/tests -v
```

CTest runs the C++ estimator, navigation, branch, coordinate, elevation and
startup tests, with asset paths supplied by CMake. Assertions remain enabled
in Release. The Python suite covers decoding, matching, invalid-output coverage,
runner cleanup and runtime measurements without requiring ROS.

## Drive model and reproducible evaluation

The estimator combines a nonlinear torque/power/brake model with causal
front/rear wheel fusion. A compact acceleration residual table was learned
offline from the **train split only**. The ROS node enables this table by
default for vehicle `30618` and uses physics only for `30639`. The standalone
C++ `EstimatorConfig` defaults to physics only, so experiments must request
the CSV explicitly. The exact equations, gating, assumptions and split
protocol are in [`docs/CORE_MODEL.md`](../../../docs/CORE_MODEL.md) and
[`docs/DRIVE_CALIBRATION.md`](../../../docs/DRIVE_CALIBRATION.md).

To override the vehicle default, add `enable_drive_table: false` or `true`
under `ros__parameters` in `config/default.yaml`. Set `drive_table_path` to
use another CSV; an empty value resolves to the installed
`assets/drive_accel_table.csv`. At startup the node warns if a requested file did not load. `/result/diagnostics` reports `drive_table_active` and
`drive_table_used`. If a requested table cannot be read, prediction falls
back to physics and emits a warning.

From the repository root, the **exact C++ core** can be replayed offline:

```bash
g++ -std=c++17 -O2 -I ros2_ws/src/tram_odometry/include \
  ros2_ws/src/tram_odometry/src/estimator.cpp evaluation/replay_cli.cpp \
  -o evaluation/replay_cli
python3 evaluation/core_benchmark.py --split validation \
  --exe evaluation/replay_cli --out evaluation/validation_physics.csv
python3 evaluation/core_benchmark.py --split validation \
  --exe evaluation/replay_cli \
  --drive-table ros2_ws/src/tram_odometry/assets/drive_accel_table.csv \
  --out evaluation/validation_table.csv
python3 evaluation/distance_proxy.py --split validation \
  --exe evaluation/replay_cli \
  --drive-table ros2_ws/src/tram_odometry/assets/drive_accel_table.csv \
  --out evaluation/validation_distance_table.csv
python3 evaluation/table_blackout_ablation.py --exe evaluation/replay_cli
```

These commands score the longitudinal core against the historical GNSS proxy;
they do not run the complete position pipeline. The table/physics choice for
30639 was informed by the old holdout, so those old scores are not an independent
assessment of the final configuration. The current healthy-pair gain ablation
and its external-reference tradeoff are documented in
[`HEALTHY_WHEEL_GAIN_2026-09-27.md`](../../../docs/HEALTHY_WHEEL_GAIN_2026-09-27.md).

For a bag containing the organiser's reference, replay the whole navigation
pipeline from the repository root (after the portable build):

```bash
python3 evaluation/reference_benchmark.py --bag /absolute/path/to/bag \
  --export-input /tmp/tram_inputs.csv
build/navigation_replay --input /tmp/tram_inputs.csv --output /tmp/tram_outputs.csv \
  --vehicle 30618 --gnss-mode corrections \
  --map ros2_ws/src/tram_odometry/assets/route_map.csv \
  --alternate-map ros2_ws/src/tram_odometry/assets/route_map_branch_a.csv \
  --elevation ros2_ws/src/tram_odometry/assets/official_elevation.csv \
  --drive-table ros2_ws/src/tram_odometry/assets/drive_accel_table.csv
python3 evaluation/reference_benchmark.py --bag /absolute/path/to/bag \
  --candidate /tmp/tram_outputs.csv --out /tmp/tram_metrics.json
```

The exported input contains permitted C/F/R and GNSS channels only. The replay
ignores GNSS velocity and rejects unknown channels. `--gnss-mode startup` disables
later corrections; `off` ignores all fixes. `--table 0` explicitly selects physics;
default vehicle 30618 requires an explicit `--drive-table` path. Asset paths are
explicit in this CLI; ROS resolves empty paths from the installed package.
The replay CLI is not a YAML-config loader, so custom ROS overrides must not be
assumed to have been applied to its defaults.

The offline evaluator uses nearest reference header stamps within inclusive
50 ms, allows reference reuse, and reports signed `twist.twist.linear.x` error,
XYZ/3D position error, coverage and provenance. It is **not** the official
one-to-one ROS approximate synchronizer. `tools/ros_reference_check.sh` starts
the real node, installed official checker and recorder; see
[`docs/JURY_CHECK.md`](../../../docs/JURY_CHECK.md). Its runtime monitor measures
node CPU/RSS and observer receive-to-receive latency separately from callback
latency. No new ROS performance numbers should be inferred from portable tests.

## Topics

| Topic | Type | Purpose |
| --- | --- | --- |
| `/vehicle/driver_position_cmd` | `tram_vehicle_msgs/msg/DriverControllerCommand` | Input driver notch |
| `/vehicle/front_bogie_velocity` | `tram_vehicle_msgs/msg/VelocitySensor` | Input front wheel speed, numeric km/h |
| `/vehicle/rear_bogie_velocity` | `tram_vehicle_msgs/msg/VelocitySensor` | Input rear wheel speed, numeric km/h |
| `/sensing/gnss/master/fix` | `sensor_msgs/msg/NavSatFix` | Optional startup anchor and bounded later route correction |
| `/sensing/gnss/rover/fix` | `sensor_msgs/msg/NavSatFix` | Optional startup fallback, later correction and branch evidence |
| `/result/velocity` | `tram_vehicle_msgs/msg/VelocitySensor` | Estimated speed, m/s |
| `/result/position` | `nav_msgs/msg/Odometry` | Estimated pose and speed |
| `/result/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | Slip, sensor staleness, model-only mode, route match |

## Route and coordinate settings

`assets/route_map.csv` is generated offline from training bags only. It contains `direction,s,x,y,z`, where `s` is distance in metres along either the `out` or `return` route and `x/y/z` use a fixed WGS84 ENU datum. The node buffers allowed startup GNSS fixes, projects them by WGS84 geodetic → ECEF → ENU, takes a componentwise median, matches it to the closest route segment, then stores the corresponding initial route distance. It attempts a master anchor after `startup_min_fixes`; if fewer fixes arrive, the buffered fixes are used when the startup window ends. For startup, master fixes take priority; rover fixes are used only if master fixes are absent after `rover_fallback_delay_s` and carry a larger covariance. Simultaneous master and rover fixes provide initial body-forward heading; with one antenna the map tangent supplies heading. GNSS is **never** fed to the velocity/distance estimator. If no startup GNSS is available, the default output is straight relative odometry (`relative_frame_id`) from `initial_x_m/y_m/z_m` and `relative_heading_rad`. Set `use_map_without_gnss=true` and provide `route_direction` plus `start_s_m` only when the starting route pose is known through configuration.

The organiser specified the target point as `base_link`, centred on the front bogie at rail level. In that body frame, master antenna is `(-9.873, 0, +3.0)` m and rover is `(+2.563, 0, +3.0)` m. The map follows master, so the default output adds **+9.873 m times the local 3D forward unit tangent** to the mapped master point, then subtracts **3.0 m from altitude**. The tangent is estimated from map samples at `s±1 m`; this is a rigid body transform, not an advance of `s` by 9.873 m on a curved track. Rover-only initialization matches the antenna rigidly displaced **+12.436 m** from the master route along the local 3D tangent, then retains the corresponding master route coordinate; it does not subtract 12.436 m along a curved polyline. Neither offset changes estimated speed or distance.

The default `/result/position` uses a fixed, continuous MGRS `37UCB` plane: `x = UTM zone 37N easting − 300000`, `y = UTM northing − 6100000`, and `z = base_link` height in metres. The transformation is WGS84 ENU → ECEF → geodetic → UTM; it remains continuous if a route crosses a 100-km MGRS letter boundary. The official control point `lat=55.8088325462547°, lon=37.4602768500852°` projects to `x=103501.6309, y=85876.1201` m. Our standalone C++ test differs by approximately 0.6 mm in `y`. `output_projection=enu` is for internal inspection. `output_scale`, `output_rotation_rad`, and `output_offset_*_m` remain available for a revised judge frame. `nav_msgs/Odometry` twist is expressed in `child_frame_id`.

Parameters are documented in [`config/default.yaml`](config/default.yaml). `map_file` can override the installed map. With `route_direction=auto`, starts near the east terminal (`ENU x > -200 m`) use `out` and starts near the west terminal (`ENU x < -4400 m`) use `return`; these thresholds are valid for the supplied map datum. Elsewhere it uses the initial dual-antenna heading when available, otherwise the nearest track; a mid-route direction without heading remains ambiguous. Explicitly set `out` or `return` for a mid-route start if known. A bad startup GNSS fix farther than `map_match_max_distance_m` from the selected mapped route is rejected. The west terminal has mapped branch offsets up to tens of metres; an initial horizontal residual is reduced smoothly over `anchor_residual_decay_m`. The GNSS subscriber itself is optional (`use_startup_gnss=false`).

## Position corrections and height

With `enable_gnss_corrections=true`, fresh valid fixes are matched near the
predicted route coordinate (default ±40 m), with an 8 m residual gate. A coherent
three-fix window applies a median route-anchor correction with gain 0.5, capped
at 3 m per update. Duplicate, stale, invalid and isolated inconsistent fixes are
rejected. Repeated position payloads with fresh timestamps are rejected when
both trusted wheels show motion. Corrections affect later published poses only.
They do not rewrite outputs, update the map, or change the speed/distance core.
These gates do not guarantee rejection of every correlated GNSS error.

The installed alternate map is tested before the active branch's residual gate.
In the `out` direction, three temporally progressing coherent fixes must prefer
it by the configured algorithm's separation margins before switching. Without
such evidence the active branch remains unchanged. Use `alternate_map_file=none`
to disable this; `use_rover_fallback=false` disables rover fixes in the shared
core as well as the ROS subscription.

`assets/official_elevation.csv` is exported from the organiser-supplied Pathgraph,
not from recorded reference odometry. It supplies only final `base_link` height
in MGRS: interpolate the nearest segment at final XY, use full weight within
5 m, fade to the trained map height at 10 m, and keep map height outside coverage.
The profile does not replace the two track geometries or terminal loops, and
its heights already describe base_link: do not subtract the antenna height a
second time. `elevation_file=none` disables the profile; an explicitly configured
unreadable profile is a startup error. It is used only for mapped MGRS output.

## Operational limits

- Scalar wheel speeds and notch cannot determine an absolute world pose or resolve a route branch on their own. A map, initial anchor/direction and later valid GNSS branch evidence or route configuration are needed. The unanchored mode still reports relative distance and explicitly marks `startup_gnss_anchor=false` in diagnostics.
- A bag ending before `startup_gnss_window_s` with no GNSS produces speed and a `pending_anchor` diagnostic, but no position. For such a known no-GNSS run, set `use_startup_gnss=false` to publish relative position immediately, or also configure a known start, explicit direction and `use_map_without_gnss=true` with startup GNSS disabled.
- A map bounds lateral drift and supplies height, but incorrect route/direction choice and unmapped terminal branches still bias position.
- The node follows message header time. Inputs with invalid timestamps are discarded; the estimator handles slightly delayed messages from different channels and rejects excessive backward timestamps. A backward jump exceeding `reset_on_large_time_jump_s` starts a new run.
- The route map is clamped at its endpoints, with a diagnostic warning if integrated distance exceeds the mapped interval.

### Round 2 robustness and geometry

`body_heading_lookahead_m=6.098` derives from the master-to-front-bogie offset
9.873 m minus half the 7.55 m bogie spacing. It samples the fixed map ahead only
for body direction; the master position and integrated distance stay at their
current route coordinate. Startup paired-GNSS heading rotates both yaw and the
rigid antenna-to-base lever arm, with the same distance decay. No future sensor
samples are used.

Strong wheel jumps use ordered raw sensor timestamps, including short intervals
and delayed bursts. Agreement of two rejected wheels does not clear the fault.
`front_tentative`/`rear_tentative` distinguish recovery allowed by broad model
uncertainty from independent healthy-wheel evidence. Tentative wheels cannot
train bias/residual acceleration or veto stationary GNSS as frozen.

When both wheels become stale, a recent healthy acceleration residual briefly
assists prediction and fades with the existing 0.35 s stale time. Normal healthy
fusion is unchanged. Invalid or suspect wheels clear this residual and its
projection cache. A synchronized GNSS pair is checked against the physical
12.436 m antenna separation before late correction; a lone bias or common shift
can remain undetectable. See the measured tradeoffs in `docs/ROUND2_IMPROVEMENTS.md`.
