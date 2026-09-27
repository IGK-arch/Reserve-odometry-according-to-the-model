#!/usr/bin/env bash
# Run inside a built ROS 2 Humble environment; no container daemon is invoked.
# Example: tools/ros_reference_check.sh --bag /data/bag --outdir /results/run \
#   --config ros2_ws/src/tram_odometry/config/default.yaml --ros-ws /ws
# Source an external official-checker workspace first, or set CHECKER_WS.
set -Eeuo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "$script_dir/.." && pwd)"
ros_ws="${ROS_WS:-$project_root/ros2_ws}"
forward=()
while (($#)); do
  case "$1" in
    --ros-ws)
      [[ $# -ge 2 ]] || { echo '--ros-ws requires a path' >&2; exit 2; }
      ros_ws="$2"; shift 2 ;;
    *) forward+=("$1"); shift ;;
  esac
done
# Help works on a development host without a ROS installation.
for arg in "${forward[@]}"; do
  if [[ "$arg" == --help || "$arg" == -h ]]; then
    exec python3 "$project_root/evaluation/record_ros_outputs.py" "${forward[@]}"
  fi
done
set +u
if [[ -f "/opt/ros/${ROS_DISTRO:-humble}/setup.bash" ]]; then
  source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
fi
if [[ -n "${CHECKER_WS:-}" ]]; then
  source "$CHECKER_WS/install/setup.bash"
fi
if [[ -f "$ros_ws/install/setup.bash" ]]; then
  source "$ros_ws/install/setup.bash"
fi
set -u
command -v ros2 >/dev/null || { echo 'Source a built ROS environment or provide --ros-ws.' >&2; exit 2; }
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-87}"
export PYTHONUNBUFFERED=1
exec python3 "$project_root/evaluation/record_ros_outputs.py" "${forward[@]}"
