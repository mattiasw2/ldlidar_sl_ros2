#!/bin/bash
# replay.sh BAG PARAMS.yaml OUT DOMAIN [RATE]
# Run slam_toolbox on a recorded /scan bag (from scan_map.sh) in an isolated
# ROS domain and save OUT.pgm/.yaml. Several replays with different PARAMS can
# run in parallel, each with its own DOMAIN. OUT must not be the PARAMS file
# without .yaml: saving the map writes OUT.yaml. Replays are not deterministic;
# repeat a variant before trusting a difference.
BAG=$1; PARAMS=$2; OUT=$3; export ROS_DOMAIN_ID=$4; RATE=${5:-1.0}
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
source /opt/ros/lyrical/setup.bash
pids=()
ros2 run tf2_ros static_transform_publisher --frame-id odom --child-frame-id base_link --ros-args -p use_sim_time:=true >/dev/null 2>&1 & pids+=($!)
ros2 run tf2_ros static_transform_publisher --z 0.18 --frame-id base_link --child-frame-id base_laser --ros-args -p use_sim_time:=true >/dev/null 2>&1 & pids+=($!)
ros2 run slam_toolbox async_slam_toolbox_node --ros-args -r __node:=slam_toolbox --params-file "$PARAMS" -p use_sim_time:=true -p use_lifecycle_manager:=false > "$OUT.slam.log" 2>&1 & pids+=($!)
sleep 3
if ! kill -0 ${pids[2]} 2>/dev/null; then echo "slam_toolbox died: $OUT"; kill ${pids[0]} ${pids[1]}; exit 1; fi
ros2 lifecycle set /slam_toolbox configure >/dev/null && ros2 lifecycle set /slam_toolbox activate >/dev/null || echo "lifecycle failed $OUT"
ros2 bag play "$BAG" --clock --topics /scan -r "$RATE" > "$OUT.play.log" 2>&1
sleep 4
timeout 60 ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '$OUT'}}" >/dev/null
sleep 2
for p in "${pids[@]}"; do kill -TERM $p 2>/dev/null; done
sleep 1; pkill -TERM -f "__node:=slam_toolbo[x].*$PARAMS" 2>/dev/null
wait 2>/dev/null
ls "$OUT.pgm" >/dev/null 2>&1 && echo "done $OUT" || echo "NO MAP $OUT"
