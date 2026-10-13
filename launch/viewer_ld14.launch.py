#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, UnsetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource

def snap_env_cleanup():
  """Actions that undo the environment of a snap-packaged terminal.

  When this launch file is started from e.g. the snap-packaged VS Code
  terminal, the snap injects GTK/GIO/locale paths under /snap/code. rviz2
  then loads libc from the snap and dies immediately with exit code 127
  ("undefined symbol __libc_pthread_init, version GLIBC_PRIVATE").
  Restore the original values (the snap keeps them in *_VSCODE_SNAP_ORIG)
  or unset the variables. No-op outside a snap environment.
  """
  if not (os.environ.get('SNAP') or '/snap/' in os.environ.get('GTK_PATH', '')):
    return []
  actions = []
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
  # RViZ2 settings
  rviz2_config = os.path.join(
      get_package_share_directory('ldlidar_sl_ros2'),
      'rviz2',
      'ldlidar.rviz'
  )
  
  rviz2_node = Node(
      package='rviz2',
      executable='rviz2',
      name='rviz2_show_ld14',
      arguments=['-d',rviz2_config],
      output='screen'
  )

  #Include LDLidar launch file
  ldlidar_launch = IncludeLaunchDescription(
      launch_description_source=PythonLaunchDescriptionSource([
          get_package_share_directory('ldlidar_sl_ros2'),
          '/launch/ld14.launch.py'
      ])
  )

  # Define LaunchDescription variable
  ld = LaunchDescription()

  for action in snap_env_cleanup():
    ld.add_action(action)
  ld.add_action(ldlidar_launch)
  ld.add_action(rviz2_node)
  
  return ld