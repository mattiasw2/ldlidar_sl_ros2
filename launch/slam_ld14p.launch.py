#!/usr/bin/env python3
"""LD14P/LD19P driver + slam_toolbox (online async) + optional RViz map view.

Intended for a hand-carried lidar without odometry: a static identity
odom -> base_link transform is published so that slam_toolbox's TF
requirements are met and all motion ends up in map -> odom.

    ros2 launch ldlidar_sl_ros2 slam_ld14p.launch.py            # with RViz
    ros2 launch ldlidar_sl_ros2 slam_ld14p.launch.py rviz:=false

The person carrying the lidar fills part of its view and moves with it, which
tells the scan matcher that the lidar is not moving. The sector behind the
carrier is therefore blanked by default (body_crop:=false turns that off;
body_crop_min/max are in lidar degrees, clockwise, see ld14p.launch.py).
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            LogInfo, RegisterEventHandler, SetEnvironmentVariable,
                            UnsetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def snap_env_cleanup():
  """Undo the environment of a snap-packaged terminal (see viewer_ld14p.launch.py)."""
  if not (os.environ.get('SNAP') or os.environ.get('SNAP_COMMON')
          or '/snap/' in os.environ.get('GTK_PATH', '')):
    return []
  actions = []
  # slam_toolbox prefixes save paths with $SNAP_COMMON when it sees $SNAP, so
  # the pose graph would land under /var/snap/code/common/... : drop them.
  for var in ('SNAP', 'SNAP_COMMON', 'SNAP_USER_COMMON', 'SNAP_DATA', 'SNAP_USER_DATA'):
    if var in os.environ:
      actions.append(UnsetEnvironmentVariable(var))
  for var in ('GTK_PATH', 'GTK_EXE_PREFIX', 'GIO_MODULE_DIR', 'GSETTINGS_SCHEMA_DIR',
              'LOCPATH', 'GTK_IM_MODULE_FILE', 'XDG_DATA_HOME', 'XDG_DATA_DIRS',
              'XDG_CONFIG_DIRS', 'GDK_BACKEND', 'LD_PRELOAD'):
    orig = os.environ.get(var + '_VSCODE_SNAP_ORIG')
    if orig:
      actions.append(SetEnvironmentVariable(var, orig))
    elif var in os.environ:
      actions.append(UnsetEnvironmentVariable(var))
  return actions


def generate_launch_description():
  share = get_package_share_directory('ldlidar_sl_ros2')

  rviz_arg = DeclareLaunchArgument('rviz', default_value='true',
                                   description='Start RViz with the map view')
  params_arg = DeclareLaunchArgument(
      'slam_params_file',
      default_value=os.path.join(share, 'config', 'slam_toolbox_ld14p.yaml'),
      description='slam_toolbox parameter file')

  # Default sector: measured on a hand-carried walk, where 89-97% of the beams
  # between 160 and 240 deg in /scan (120-200 deg lidar angle) hit the carrier.
  crop_args = [
      DeclareLaunchArgument('body_crop', default_value='true',
                            description='Blank out the sector where the carrier is'),
      DeclareLaunchArgument('body_crop_min', default_value='120.0',
                            description='Start of the blanked sector, lidar degrees (clockwise)'),
      DeclareLaunchArgument('body_crop_max', default_value='200.0',
                            description='End of the blanked sector, lidar degrees (clockwise)'),
  ]

  # Lidar driver + base_link -> base_laser static transform
  ldlidar_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([share, '/launch/ld14p.launch.py']),
      launch_arguments={'angle_crop': LaunchConfiguration('body_crop'),
                        'angle_crop_min': LaunchConfiguration('body_crop_min'),
                        'angle_crop_max': LaunchConfiguration('body_crop_max')}.items())

  # No odometry source: odom and base_link coincide.
  odom_tf = Node(
      package='tf2_ros',
      executable='static_transform_publisher',
      name='odom_to_base_link',
      arguments=['--frame-id', 'odom', '--child-frame-id', 'base_link'])

  slam_node = LifecycleNode(
      package='slam_toolbox',
      executable='async_slam_toolbox_node',
      name='slam_toolbox',
      namespace='',
      output='screen',
      parameters=[LaunchConfiguration('slam_params_file'),
                  {'use_lifecycle_manager': False, 'use_sim_time': False}])

  # slam_toolbox is a lifecycle node: configure it, then activate once configured.
  configure_slam = EmitEvent(event=ChangeState(
      lifecycle_node_matcher=matches_action(slam_node),
      transition_id=Transition.TRANSITION_CONFIGURE))
  activate_slam = RegisterEventHandler(OnStateTransition(
      target_lifecycle_node=slam_node,
      start_state='configuring', goal_state='inactive',
      entities=[LogInfo(msg='slam_toolbox configured, activating.'),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(slam_node),
                    transition_id=Transition.TRANSITION_ACTIVATE))]))

  rviz_node = Node(
      package='rviz2',
      executable='rviz2',
      name='rviz2_slam',
      arguments=['-d', os.path.join(share, 'rviz2', 'ldlidar_slam.rviz')],
      output='screen',
      condition=IfCondition(LaunchConfiguration('rviz')))

  ld = LaunchDescription()
  for action in snap_env_cleanup():
    ld.add_action(action)
  ld.add_action(rviz_arg)
  ld.add_action(params_arg)
  for action in crop_args:
    ld.add_action(action)
  ld.add_action(ldlidar_launch)
  ld.add_action(odom_tf)
  ld.add_action(slam_node)
  ld.add_action(configure_slam)
  ld.add_action(activate_slam)
  ld.add_action(rviz_node)
  return ld
