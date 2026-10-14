#!/bin/bash
# One-time setup of this PC for the LDROBOT D500 (LD19P) lidar with ROS 2.
# Run as your normal user:   bash scripts/setup_lidar_pc.sh
# Exactly ONE privilege prompt: all root work is in root_setup.sh.
# The colcon workspace is ~/ldlidar_ws (override with LDLIDAR_WS); this repo is
# symlinked into its src/ so that edits here are picked up by
# colcon build --symlink-install.
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/.." && pwd)
WS="${LDLIDAR_WS:-$HOME/ldlidar_ws}"

echo "==> Root steps (single sudo/pkexec call)"
if sudo -n true 2>/dev/null || [ -t 0 ]; then
  sudo bash "$HERE/root_setup.sh"
else
  pkexec --disable-internal-agent bash "$HERE/root_setup.sh"
fi

mkdir -p "$WS/src"
[ -e "$WS/src/ldlidar_sl_ros2" ] || ln -s "$REPO" "$WS/src/ldlidar_sl_ros2"

echo "==> User steps: rosdep + build"
set +u; source /opt/ros/lyrical/setup.bash; set -u
rosdep update || true
cd "$WS"
colcon build --symlink-install

echo
ls -l /dev/ldlidar /dev/ttyUSB* 2>/dev/null || echo "Lidar not detected on USB right now"
echo "Done. Log out and back in once (dialout group). Optional aliases for ~/.bashrc:"
echo "  alias lidar='$HERE/run_lidar.sh'"
echo "  alias lidar-headless='$HERE/run_lidar.sh headless'"
echo "  alias lidar-map='$HERE/scan_map.sh'"
