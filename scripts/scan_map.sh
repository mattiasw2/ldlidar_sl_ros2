#!/bin/bash
# Interactive mapping session with the hand-held lidar.
#
#   scan_map.sh [NAME] [--rviz]
#
#   1. press a key      -> lidar driver + slam_toolbox start
#   2. walk around      -> (optionally watch RViz with --rviz)
#   3. press a key      -> map is saved as NAME.pgm/.yaml (+ .posegraph/.data),
#                          everything shuts down, the map image is opened
#
# The raw scans are recorded to NAME.bag/ (mcap), so the same walk can be
# run through slam_toolbox again later with different settings.
#
# NAME defaults to ~/maps/map_<date_time>.
if { [ -n "${SNAP:-}" ] || [[ "${GTK_PATH:-}" == */snap/* ]]; } && [ -z "${LDLIDAR_CLEAN_ENV:-}" ]; then
  exec env -u LD_LIBRARY_PATH -u GTK_PATH -u GIO_MODULE_DIR -u LOCPATH \
           -u GSETTINGS_SCHEMA_DIR -u GTK_EXE_PREFIX -u GTK_IM_MODULE_FILE \
           -u SNAP LDLIDAR_CLEAN_ENV=1 bash "$0" "$@"
fi
source /opt/ros/lyrical/setup.bash
source "${LDLIDAR_WS:-$HOME/ldlidar_ws}/install/setup.bash"
set -u

name=""; rviz=false
for arg in "$@"; do
  case "$arg" in
    --rviz) rviz=true ;;
    -h|--help) sed -n '2,14p' "$0"; exit 0 ;;
    *) name="$arg" ;;
  esac
done
[ -n "$name" ] || name="$HOME/maps/map_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$(dirname "$name")"
log="$(dirname "$name")/$(basename "$name").launch.log"

if [ ! -r /dev/ttyUSB0 ]; then
  echo "No readable /dev/ttyUSB0 - is the lidar plugged in?" >&2; exit 1
fi

launch_pid=""
bag_pid=""
cleanup() {
  # Stop the recorder first so the bag is closed cleanly. SIGTERM, not SIGINT:
  # a background job of a non-interactive shell ignores SIGINT. ros2 bag
  # finalizes the file on SIGTERM too.
  if [ -n "$bag_pid" ] && kill -0 "$bag_pid" 2>/dev/null; then
    kill -TERM "$bag_pid" 2>/dev/null
    for _ in $(seq 50); do
      kill -0 "$bag_pid" 2>/dev/null || break
      sleep 0.1
    done
    wait "$bag_pid" 2>/dev/null || true
  fi
  # ros2 launch only shuts its nodes down on SIGINT (ctrl-c). On SIGTERM it
  # exits at once and leaves the driver and slam_toolbox running, which keeps
  # /dev/ttyUSB0 busy for the next run.
  if [ -n "$launch_pid" ] && kill -0 "$launch_pid" 2>/dev/null; then
    kill -INT "$launch_pid" 2>/dev/null
    for _ in $(seq 100); do
      kill -0 "$launch_pid" 2>/dev/null || break
      sleep 0.1
    done
    if kill -0 "$launch_pid" 2>/dev/null; then
      kill -TERM "$launch_pid" 2>/dev/null
    fi
    wait "$launch_pid" 2>/dev/null || true
  fi
  # Belt and braces: nothing from this launch may outlive the script.
  pkill -INT -f "ldlidar_sl_ros2_node|async_slam_toolbox_node" 2>/dev/null || true
  # The two static_transform_publishers of the launch outlive it as well.
  pkill -TERM -f "static_transform_publisher.*(base_laser|odom_to_base_link)" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "Map will be saved as: $name"
echo
read -rsn1 -p "Press any key to START scanning (q to quit) ... " key; echo
[ "$key" = "q" ] && exit 0

ros2 launch ldlidar_sl_ros2 slam_ld14p.launch.py "rviz:=$rviz" > "$log" 2>&1 &
launch_pid=$!

printf "Starting lidar and slam_toolbox "
for _ in $(seq 40); do
  if ! kill -0 "$launch_pid" 2>/dev/null; then
    echo; echo "Launch died, see $log" >&2; tail -20 "$log" >&2; exit 1
  fi
  state=$(timeout 2 ros2 lifecycle get /slam_toolbox 2>/dev/null || true)
  [[ "$state" == active* ]] && break
  printf "."; sleep 0.5
done
echo
if [[ "$state" != active* ]]; then
  echo "slam_toolbox did not become active, see $log" >&2; exit 1
fi

ros2 bag record -s mcap -o "$name.bag" --topics /scan /tf_static > "$name.bag.log" 2>&1 &
bag_pid=$!

echo "Scanning. Keep the lidar level, move at a slow walking pace, turn slowly,"
echo "and come back to where you started so the loop can be closed."
echo
read -rsn1 -p "Press any key to STOP and save the map ... " _; echo

echo "Saving map ..."
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '$name'}}" > /dev/null
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '$name'}" > /dev/null
sleep 1   # let map_saver_cli finish writing

echo "Stopping lidar and slam_toolbox ..."
cleanup; launch_pid=""; bag_pid=""

echo
if [ -f "$name.pgm" ]; then
  echo "Saved:"
  ls -l "$name".pgm "$name".yaml "$name".posegraph "$name".data 2>/dev/null
  [ -d "$name.bag" ] && echo "Raw scans: $name.bag/ ($(du -sh "$name.bag" | cut -f1))"
  echo
  echo "Map metadata ($name.yaml):"
  sed 's/^/  /' "$name.yaml"; echo
  # Dimensions in cells, from the PGM header (P5 <width> <height> <maxval>)
  dims=$(head -c 64 "$name.pgm" | tr '\n' ' ' | awk '{print $2 " x " $3}')
  res=$(awk '/^resolution:/ {print $2}' "$name.yaml")
  echo "  size: $dims cells at ${res} m/cell"
  if [ -n "${DISPLAY:-}" ] && command -v xdg-open >/dev/null; then
    xdg-open "$name.pgm" >/dev/null 2>&1 &
  fi
else
  echo "No map file was written. slam_toolbox log: $log" >&2
  grep -iE "warn|error|map" "$log" | tail -10 >&2
  exit 1
fi
