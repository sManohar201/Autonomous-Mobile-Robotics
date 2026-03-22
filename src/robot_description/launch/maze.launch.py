"""
maze.launch.py — Launch the maze world simulation.

World: complex 22×22 m maze with dead ends, symmetric sections,
       and 4 dynamic box obstacles roaming maze sections.

SLAM challenge: backtracking required, symmetric corridors cause
loop-closure failures in weaker SLAM implementations.

Usage:
  ros2 launch robot_description maze.launch.py
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
                'world': os.path.join(pkg, 'worlds', 'maze.sdf'),
                'x': '0.0',
                'y': '0.0',
                'z': '0.1',
            }.items(),
        )
    ])
