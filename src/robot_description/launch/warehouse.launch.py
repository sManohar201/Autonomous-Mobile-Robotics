"""
warehouse.launch.py — Launch the warehouse world simulation.

World: 5 rows of tall shelving with narrow aisles, loading dock,
       4 dynamic forklift obstacles patrolling aisles and staging area.

Usage:
  ros2 launch robot_description warehouse.launch.py
  ros2 launch robot_description warehouse.launch.py x:=0.0 y:=0.0
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg = get_package_share_directory('robot_description')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={
                'world': os.path.join(pkg, 'worlds', 'warehouse.sdf'),
                'x': '0.0',
                'y': '0.0',
                'z': '0.1',
            }.items(),
        )
    ])
