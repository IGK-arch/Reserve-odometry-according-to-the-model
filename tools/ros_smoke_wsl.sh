#!/usr/bin/env bash
# Short ROS 2 Humble integration smoke on one provided rosbag.
# Run inside Ubuntu 22.04 WSL after tools/install_ros_humble_wsl.sh completes.
set -Eeuo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "$script_dir/.." && pwd)"
ros_ws="${ROS_WS:-$HOME/tram_hack_ros2_ws}"
bag="${ROS_SMOKE_BAG:-$project_root/dataset/data/30618_2050d396}"
mode="${1:-anchored}"
vehicle_id="${ROS_SMOKE_VEHICLE_ID:-30618}"
timeout_s="${ROS_SMOKE_TIMEOUT_S:-25}"
rate="${ROS_SMOKE_RATE:-1.0}"
case "$mode" in
  anchored|no-gnss) ;;
  *) echo 'Usage: tools/ros_smoke_wsl.sh [anchored|no-gnss]' >&2; exit 2 ;;
esac
[[ "$timeout_s" =~ ^[1-9][0-9]*$ ]] || {
  echo 'ROS_SMOKE_TIMEOUT_S must be a positive integer.' >&2; exit 2;
}
[[ "$rate" =~ ^[0-9]*\.?[0-9]+$ && "$rate" != 0 && "$rate" != 0.0 ]] || {
  echo 'ROS_SMOKE_RATE must be a positive decimal.' >&2; exit 2;
}
[[ -f "$bag/metadata.yaml" ]] || { echo "Bag not found: $bag" >&2; exit 2; }
[[ -f "$ros_ws/install/setup.bash" ]] || {
  echo "Built ROS workspace not found: $ros_ws" >&2; exit 2;
}
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1090
source "$ros_ws/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_SMOKE_DOMAIN_ID:-${ROS_DOMAIN_ID:-77}}"

log_parent="${ROS_SMOKE_LOG_PARENT:-$ros_ws/smoke_logs}"
mkdir -p "$log_parent"
log_dir="$(mktemp -d "$log_parent/${mode}_$(date +%Y%m%d_%H%M%S)_XXXX")"
report="$log_dir/report.json"
monitor_extra=()
if [[ "${ROS_SMOKE_RECORD:-0}" == 1 ]]; then
  monitor_extra=(--samples "$log_dir/samples.csv")
fi
launch_pid=''
monitor_pid=''
cleanup() {
  if [[ -n "$monitor_pid" ]] && kill -0 "$monitor_pid" 2>/dev/null; then
    kill -TERM "$monitor_pid" 2>/dev/null || true
    wait "$monitor_pid" 2>/dev/null || true
  fi
  if [[ -n "$launch_pid" ]] && kill -0 "$launch_pid" 2>/dev/null; then
    kill -INT "$launch_pid" 2>/dev/null || true
    for _ in {1..50}; do
      kill -0 "$launch_pid" 2>/dev/null || break
      sleep 0.1
    done
    if kill -0 "$launch_pid" 2>/dev/null; then
      kill -TERM "$launch_pid" 2>/dev/null || true
    fi
    wait "$launch_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT

echo "ROS smoke mode=$mode vehicle=$vehicle_id domain=$ROS_DOMAIN_ID rate=$rate timeout=${timeout_s}s bag=$bag logs=$log_dir"
ros2 pkg executables tram_odometry | grep -q odometry_node
node_executable="$(ros2 pkg prefix tram_odometry)/lib/tram_odometry/odometry_node"
ros2 launch tram_odometry tram_odometry.launch.py vehicle_id:="$vehicle_id" \
  >"$log_dir/launch.log" 2>&1 &
launch_pid=$!
sleep 2
if ! kill -0 "$launch_pid" 2>/dev/null; then
  echo "ROS launch exited early; see $log_dir/launch.log" >&2
  exit 1
fi

python3 "$script_dir/ros_smoke_monitor.py" --mode "$mode" \
  --vehicle-id "$vehicle_id" --node-executable "$node_executable" --report "$report" \
  "${monitor_extra[@]}" \
  >"$log_dir/monitor.log" 2>&1 &
monitor_pid=$!
sleep 1
if ! kill -0 "$monitor_pid" 2>/dev/null; then
  echo "ROS monitor exited early; see $log_dir/monitor.log" >&2
  exit 1
fi

play_args=("$bag" --clock -r "$rate" --delay 2)
if [[ "$mode" == no-gnss ]]; then
  # Humble argparse uses nargs='+': keep --topics and its values last.
  play_args+=(--topics /vehicle/front_bogie_velocity
             /vehicle/rear_bogie_velocity /vehicle/driver_position_cmd)
fi
set +e
timeout --signal=INT --kill-after=5s "${timeout_s}s" ros2 bag play "${play_args[@]}" \
  >"$log_dir/bag.log" 2>&1
play_rc=$?
set -e
if [[ "$play_rc" != 0 && "$play_rc" != 124 ]]; then
  echo "ros2 bag play failed (exit $play_rc); see $log_dir/bag.log" >&2
  exit 1
fi

kill -TERM "$monitor_pid" 2>/dev/null || true
set +e
wait "$monitor_pid"
monitor_rc=$?
set -e
monitor_pid=''
[[ -f "$report" ]] && cat "$report"
if [[ "$monitor_rc" != 0 ]]; then
  echo "ROS smoke FAILED; logs: $log_dir" >&2
  exit 1
fi
echo "ROS smoke PASSED; logs: $log_dir"
