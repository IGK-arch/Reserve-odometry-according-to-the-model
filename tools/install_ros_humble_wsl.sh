#!/usr/bin/env bash
# Run as the normal Ubuntu user after WSL Ubuntu-22.04 first-run setup.
set -Eeuo pipefail
trap 'printf "ERROR at line %s: %s\n" "$LINENO" "$BASH_COMMAND" >&2' ERR

if [[ "$(id -u)" == 0 ]]; then
  echo 'Run as the Ubuntu user created on first launch, not root.' >&2
  exit 2
fi
if [[ ! -r /etc/os-release ]]; then
  echo '/etc/os-release is missing.' >&2
  exit 2
fi
# shellcheck disable=SC1091
source /etc/os-release
if [[ "${ID:-}" != ubuntu || "${VERSION_ID:-}" != 22.04 ]]; then
  echo "Expected Ubuntu 22.04 Jammy; found ${PRETTY_NAME:-unknown}." >&2
  exit 2
fi
command -v sudo >/dev/null || { echo 'sudo is required.' >&2; exit 2; }
sudo -v

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source_ws="$(cd -- "$script_dir/../ros2_ws" && pwd)"
if [[ ! -f "$source_ws/src/tram_odometry/package.xml" ||
      ! -f "$source_ws/src/tram_vehicle_msgs/package.xml" ]]; then
  echo "ROS workspace packages not found at $source_ws." >&2
  exit 2
fi

echo 'Installing Ubuntu prerequisites and locale...'
sudo apt-get update
sudo env DEBIAN_FRONTEND=noninteractive apt-get install -y \
  locales software-properties-common curl ca-certificates
if ! locale -a | grep -Eiq '^en_US\.utf8$'; then
  sudo locale-gen en_US en_US.UTF-8
fi
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8
sudo add-apt-repository -y universe

if ! dpkg-query -W -f='${Status}' ros2-apt-source 2>/dev/null |
    grep -q 'install ok installed'; then
  echo 'Installing the official ROS 2 apt source package...'
  release_json="$(curl -fsSL --retry 3 https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest)"
  version="$(printf '%s\n' "$release_json" | awk -F '"' '/"tag_name"/ {print $4; exit}')"
  if [[ -z "$version" ]]; then
    echo 'Could not determine ros2-apt-source release tag from GitHub API.' >&2
    exit 1
  fi
  deb="/tmp/ros2-apt-source_${version}.jammy_all.deb"
  curl -fL --retry 3 -o "$deb" \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${version}/ros2-apt-source_${version}.jammy_all.deb"
  sudo dpkg -i "$deb"
fi

echo 'Updating Ubuntu packages before ROS 2, as required by the Humble installation guide...'
sudo apt-get update
sudo env DEBIAN_FRONTEND=noninteractive apt-get upgrade -y
sudo env DEBIAN_FRONTEND=noninteractive apt-get install -y \
  ros-humble-ros-base ros-dev-tools python3-colcon-common-extensions \
  ros-humble-rosbag2 ros-humble-rosbag2-storage-default-plugins \
  ros-humble-ament-cmake ros-humble-rosidl-default-generators \
  ros-humble-rclcpp ros-humble-nav-msgs ros-humble-sensor-msgs \
  ros-humble-diagnostic-msgs ros-humble-std-msgs \
  ros-humble-builtin-interfaces ros-humble-ament-index-cpp \
  ros-humble-launch-ros

# ROS-generated setup files may read optional, unset environment variables.
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u
command -v colcon >/dev/null
ros2 bag --help >/dev/null

# Compile on WSL's native ext4 filesystem; source and bag data stay in the
# original Windows project. Re-running copies newer source files and rebuilds.
build_ws="$HOME/tram_hack_ros2_ws"
mkdir -p "$build_ws/src"
cp -a "$source_ws/src/." "$build_ws/src/"
cd "$build_ws"
echo "Building in $build_ws from $source_ws..."
colcon build --packages-up-to tram_odometry --parallel-workers 2 \
  --event-handlers console_direct+
set +u
# shellcheck disable=SC1091
source "$build_ws/install/setup.bash"
set -u
ros2 pkg executables tram_odometry
ros2 interface show tram_vehicle_msgs/msg/VelocitySensor

bag_dir="$script_dir/../dataset/data/30618_2050d396"
if [[ -f "$bag_dir/metadata.yaml" ]]; then
  ros2 bag info "$bag_dir"
fi
echo "Build and bag-info smoke checks passed. ROS workspace: $build_ws"
