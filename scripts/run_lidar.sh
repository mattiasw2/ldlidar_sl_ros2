#!/bin/bash
# Launch the D500 lidar driver together with RViz2.
# Usage: run_lidar.sh                  (driver + RViz)
#        run_lidar.sh headless         (driver only)
#        run_lidar.sh slam             (driver + slam_toolbox + RViz map view)
#        run_lidar.sh slam headless    (driver + slam_toolbox, no RViz)
#        run_lidar.sh save-map NAME    (while slam is running: write NAME.pgm/.yaml
#                                       and NAME.posegraph/.data for slam_toolbox)
set -e
# If started from a snap-packaged terminal (e.g. VS Code snap), snap library
# paths break rviz2 ("undefined symbol ... GLIBC_PRIVATE"). Re-exec clean.
if { [ -n "$SNAP" ] || [[ "$GTK_PATH" == */snap/* ]]; } && [ -z "$LDLIDAR_CLEAN_ENV" ]; then
  exec env -u LD_LIBRARY_PATH -u GTK_PATH -u GIO_MODULE_DIR -u LOCPATH \
           -u GSETTINGS_SCHEMA_DIR -u GTK_EXE_PREFIX -u GTK_IM_MODULE_FILE \
           -u SNAP LDLIDAR_CLEAN_ENV=1 bash "$0" "$@"
fi
source /opt/ros/lyrical/setup.bash
source "${LDLIDAR_WS:-$HOME/ldlidar_ws}/install/setup.bash"

if [ "${1:-}" = "save-map" ]; then
  name="${2:-$HOME/maps/map_$(date +%Y%m%d_%H%M%S)}"
  mkdir -p "$(dirname "$name")"
  # Occupancy grid as PGM + YAML (what nav2's map_server loads)
  ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '$name'}}"
  # Pose graph, so slam_toolbox can later continue mapping or localize in this map
  ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '$name'}"
  echo "Saved: $name.pgm $name.yaml $name.posegraph $name.data"
  exit 0
fi

if [ ! -e /dev/ttyUSB0 ]; then
  echo "No /dev/ttyUSB0 - is the lidar plugged in?" >&2; exit 1
fi
if [ ! -r /dev/ttyUSB0 ]; then
  echo "/dev/ttyUSB0 not readable. Log out and back in (dialout group), or: sudo chmod 666 /dev/ttyUSB0" >&2; exit 1
fi
case "${1:-}" in
  headless)
    exec ros2 launch ldlidar_sl_ros2 ld14p.launch.py ;;
  slam)
    shift
    rviz=true
    if [ "${1:-}" = "headless" ]; then rviz=false; shift; fi
    exec ros2 launch ldlidar_sl_ros2 slam_ld14p.launch.py "rviz:=$rviz" "$@" ;;
  *)
    exec ros2 launch ldlidar_sl_ros2 viewer_ld14p.launch.py "$@" ;;
esac
