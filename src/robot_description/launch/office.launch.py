"""
office.launch.py — Launch the open-plan office world simulation.

World: 28×28 m open office with cubicle clusters, structural pillars,
       2 glass-walled conference rooms, kitchen area,
       and 5 dynamic person obstacles walking between desks.

SLAM challenge: sparse irregular features, repeated cubicle clusters
cause feature ambiguity; round pillars are poor landmarks.

Usage:
  ros2 launch robot_description office.launch.py
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
                'world': os.path.join(pkg, 'worlds', 'office.sdf'),
                'x': '0.0',
                'y': '0.0',
                'z': '0.1',
            }.items(),
        )
    ])
