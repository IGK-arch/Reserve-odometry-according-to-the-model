# Tram backup odometry (ROS 2 Humble)

The node consumes only driver notch and the two bogie speed channels in its running velocity/distance estimator. The supplied wheel values are **km/h** despite the custom message comment; `/result/velocity` is **m/s**. GNSS `master/fix` can set the initial route position during a short startup window and is then ignored. If master is absent, `rover/fix` is an explicitly lower-confidence startup fallback with configurable antenna spacing. There is no IMU, GNSS velocity, camera or lidar subscription.

The source was built successfully with ROS 2 Humble on Ubuntu 22.04 in WSL. Live bag replays passed for both vehicles and a no-GNSS mode, including a complete 263-second moving bag; reports are in `evaluation/ros_smoke` at repository root. The organisers specified the judge's MGRS coordinate convention and antenna-to-`base_link` transform; these are the defaults below.

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

The subscriptions use best-effort SensorDataQoS to match rosbag publishers. `/result/velocity` starts with the first causally accepted input message. `/result/position` begins after the startup GNSS anchor is established, or after `startup_gnss_window_s` expires without one; this avoids sending temporary relative coordinates into the judge's MGRS comparison. It never backfills old timestamps. Both outputs use the exact triggering input `header.stamp`. Under the provided bags the 20 Hz driver channel maintains output above the required 10 Hz after initialization. Use a single-threaded executor, as in `main`, for deterministic event order.

The estimator core can also be smoke-tested without ROS (from the repository root):

```bash
g++ -std=c++17 -O2 -Wall -Wextra -pedantic \
  -I ros2_ws/src/tram_odometry/include \
  ros2_ws/src/tram_odometry/src/estimator.cpp \
  ros2_ws/src/tram_odometry/test/estimator_core_test.cpp \
  -o estimator_core_test
./estimator_core_test
g++ -std=c++17 -O2 -Wall -Wextra -pedantic \
  -I ros2_ws/src/tram_odometry/include \
  ros2_ws/src/tram_odometry/test/geo_projection_test.cpp \
  -o geo_projection_test
./geo_projection_test
g++ -std=c++17 -O2 -Wall -Wextra -pedantic \
  -I ros2_ws/src/tram_odometry/include \
  ros2_ws/src/tram_odometry/test/route_tf_test.cpp \
  -o route_tf_test
./route_tf_test
g++ -std=c++17 -O2 -Wall -Wextra -pedantic \
  -I ros2_ws/src/tram_odometry/include \
  ros2_ws/src/tram_odometry/test/startup_output_gate_test.cpp \
  -o startup_output_gate_test
./startup_output_gate_test
g++ -std=c++17 -O2 -Wall -Wextra -pedantic \
  -I ros2_ws/src/tram_odometry/include \
  ros2_ws/src/tram_odometry/test/uncertainty_test.cpp \
  -o uncertainty_test
./uncertainty_test
```

## Drive model and reproducible evaluation

The estimator combines a nonlinear torque/power/brake model with causal
front/rear wheel fusion. A compact acceleration residual table was learned
offline from the **train split only**. The ROS node enables this table by
default for vehicle `30618` and uses physics only for `30639`. The standalone
C++ `EstimatorConfig` defaults to physics only, so experiments must request
the CSV explicitly. The exact equations, gating, assumptions and split
protocol are in [`docs/CORE_MODEL.md`](../../../docs/CORE_MODEL.md) and
[`docs/DRIVE_CALIBRATION.md`](../../../docs/DRIVE_CALIBRATION.md). The
session-split quality and overfitting audit is
[`docs/VALIDATION_AUDIT.md`](../../../docs/VALIDATION_AUDIT.md).

To override the vehicle default, add `enable_drive_table: false` or `true`
under `ros__parameters` in `config/default.yaml`. Set `drive_table_path` to
use another CSV; an empty value resolves to the installed
`assets/drive_accel_table.csv`. At startup the node logs whether the file
loaded. `/result/diagnostics` reports `drive_table_active` and
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

The Python evaluation needs the unpacked bags plus NumPy/SciPy; GNSS is read
only by the scorer. On 13 held-back validation bags of `30618`, speed RMSE
was `0.0672 m/s` for a causal fresh-wheel mean, `0.0359 m/s` for the physics
core, and `0.0345 m/s` with the table. The median absolute end-distance
drift against integrated GNSS speed was `0.233%`, `0.226%`, and `0.188%`
respectively on 12 long bags. In 284 paired-wheel synthetic five-second
outages, table speed RMSE was `0.555 m/s` versus `0.672 m/s` for physics.
These distance values are a one-dimensional proxy, not the judge's mapped
3D pose score. Holdout was used to choose the vehicle-specific table default,
so a new hidden recording is needed for an independent final estimate.

## Topics

| Topic | Type | Purpose |
| --- | --- | --- |
| `/vehicle/driver_position_cmd` | `tram_vehicle_msgs/msg/DriverControllerCommand` | Input driver notch |
| `/vehicle/front_bogie_velocity` | `tram_vehicle_msgs/msg/VelocitySensor` | Input front wheel speed, numeric km/h |
| `/vehicle/rear_bogie_velocity` | `tram_vehicle_msgs/msg/VelocitySensor` | Input rear wheel speed, numeric km/h |
| `/sensing/gnss/master/fix` | `sensor_msgs/msg/NavSatFix` | Optional startup location only |
| `/sensing/gnss/rover/fix` | `sensor_msgs/msg/NavSatFix` | Optional lower-confidence startup fallback |
| `/result/velocity` | `tram_vehicle_msgs/msg/VelocitySensor` | Estimated speed, m/s |
| `/result/position` | `nav_msgs/msg/Odometry` | Estimated pose and speed |
| `/result/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | Slip, sensor staleness, model-only mode, route match |

## Route and coordinate settings

`assets/route_map.csv` is generated offline from training bags only. It contains `direction,s,x,y,z`, where `s` is distance in metres along either the `out` or `return` route and `x/y/z` use a fixed WGS84 ENU datum. The node projects up to `startup_min_fixes` allowed startup GNSS fixes by WGS84 geodetic → ECEF → ENU, takes a componentwise median, matches it to the closest route segment, then stores the corresponding initial route distance. If fewer fixes arrive, the buffered fixes are used when the startup window ends. Master fixes take priority; rover fixes are used only if master fixes are absent after `rover_fallback_delay_s` and carry a larger covariance. Simultaneous master and rover fixes provide initial body-forward heading; with one antenna the map tangent supplies heading. GNSS is **never** fed to the velocity/distance estimator. If no startup GNSS is available, the default output is straight relative odometry (`relative_frame_id`) from `initial_x_m/y_m/z_m` and `relative_heading_rad`. Set `use_map_without_gnss=true` and provide `route_direction` plus `start_s_m` only when the starting route pose is known through configuration.

The organiser specified the target point as `base_link`, centred on the front bogie at rail level. In that body frame, master antenna is `(-9.873, 0, +3.0)` m and rover is `(+2.563, 0, +3.0)` m. The map follows master, so the default output adds **+9.873 m times the local 3D forward unit tangent** to the mapped master point, then subtracts **3.0 m from altitude**. The tangent is estimated from map samples at `s±1 m`; this is a rigid body transform, not an advance of `s` by 9.873 m on a curved track. Rover-only initialization first shifts **−12.436 m** along the map to the corresponding master point. Neither offset changes estimated speed or distance.

The default `/result/position` uses a fixed, continuous MGRS `37UCB` plane: `x = UTM zone 37N easting − 300000`, `y = UTM northing − 6100000`, and `z = base_link` height in metres. The transformation is WGS84 ENU → ECEF → geodetic → UTM; it remains continuous if a route crosses a 100-km MGRS letter boundary. The official control point `lat=55.8088325462547°, lon=37.4602768500852°` projects to `x=103501.6309, y=85876.1201` m. Our standalone C++ test differs by approximately 0.6 mm in `y`. `output_projection=enu` is for internal inspection. `output_scale`, `output_rotation_rad`, and `output_offset_*_m` remain available for a revised judge frame. `nav_msgs/Odometry` twist is expressed in `child_frame_id`.

Parameters are documented in [`config/default.yaml`](config/default.yaml). `map_file` can override the installed map. With `route_direction=auto`, starts near the east terminal (`ENU x > -200 m`) use `out` and starts near the west terminal (`ENU x < -4400 m`) use `return`; these thresholds are valid for the supplied map datum. Elsewhere the node chooses the nearest track and warns that the direction is ambiguous. Explicitly set `out` or `return` for a mid-route start if known. A bad startup GNSS fix farther than `map_match_max_distance_m` from the selected mapped route is rejected. The west terminal has mapped branch offsets up to tens of metres; an initial horizontal residual is reduced smoothly over `anchor_residual_decay_m`. The GNSS subscriber itself is optional (`use_startup_gnss=false`).

`wheel_common_scale_sigma=0.01` adds a train-derived distance-proportional
variance floor to the published along-track covariance. It changes
diagnostics, not the estimated velocity or position.

## Operational limits

- Scalar wheel speeds and notch cannot determine an absolute world pose or select a route branch without an initial anchor, map and direction. The unanchored mode still reports relative distance and explicitly marks `startup_gnss_anchor=false` in diagnostics.
- A bag ending before `startup_gnss_window_s` with no GNSS produces speed and a `pending_anchor` diagnostic, but no position. For such a known no-GNSS run, set `use_startup_gnss=false` to publish relative position immediately, or configure a known start and use `use_map_without_gnss=true`.
- A map bounds lateral drift and supplies height, but incorrect route/direction choice and unmapped terminal branches still bias position.
- The node follows message header time. Inputs with invalid timestamps are discarded; the estimator handles slightly delayed messages from different channels and rejects excessive backward timestamps. A backward jump exceeding `reset_on_large_time_jump_s` starts a new run.
- The route map is clamped at its endpoints, with a diagnostic warning if integrated distance exceeds the mapped interval.
