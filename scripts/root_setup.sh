#!/bin/bash
# Root part of the D500 lidar PC setup (run by setup_lidar_pc.sh via sudo or pkexec).
set -euo pipefail
export DEBIAN_FRONTEND=noninteractive
# The user who ran setup_lidar_pc.sh (via sudo or pkexec) gets serial access.
TARGET_USER=${SUDO_USER:-$(id -nu "${PKEXEC_UID:-0}")}
REPO=$(cd "$(dirname "$0")/.." && pwd)
CODENAME=$(. /etc/os-release && echo "$VERSION_CODENAME")

echo "==> [1/5] Base tools"
apt-get update
apt-get install -y software-properties-common curl ca-certificates locales build-essential cmake git python3
add-apt-repository -y universe
locale-gen en_US en_US.UTF-8 >/dev/null
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 >/dev/null

echo "==> [2/5] ROS 2 apt repository"
if [ ! -f /etc/apt/sources.list.d/ros2.sources ]; then
  VER=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F '"tag_name"' | sed -E 's/.*"([^"]+)".*/\1/')
  VER=${VER:-1.3.0}
  mkdir -p /root/ros2-apt-dl
  DEB="/root/ros2-apt-dl/ros2-apt-source_${VER}.${CODENAME}_all.deb"
  curl -fL -o "$DEB" "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${VER}/ros2-apt-source_${VER}.${CODENAME}_all.deb"
  dpkg -i "$DEB"
  apt-get update
fi

echo "==> [3/5] ROS 2 lyrical desktop, slam_toolbox, rosbag2 mcap + build tools"
apt-get install -y ros-lyrical-desktop ros-lyrical-tf2-ros ros-lyrical-slam-toolbox ros-lyrical-rosbag2-storage-mcap ros-dev-tools python3-colcon-common-extensions python3-rosdep

echo "==> [4/5] Serial port access"
usermod -aG dialout "$TARGET_USER"
cp "$REPO/scripts/ldlidar.rules" /etc/udev/rules.d/ldlidar.rules
udevadm control --reload-rules
udevadm trigger

echo "==> [5/5] rosdep init"
[ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || rosdep init
echo "ROOT_SETUP_DONE"
