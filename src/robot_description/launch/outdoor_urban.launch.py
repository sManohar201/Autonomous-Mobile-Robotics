"""
outdoor_urban.launch.py — Launch the outdoor urban world simulation.

World: 50×50 m city block with 4 large buildings, 8 trees, street
       furniture (benches, planters, bollards, parked car),
       and 6 dynamic pedestrian obstacles navigating paths.

SLAM challenge: mix of feature-rich building corners and feature-sparse
open plazas; round tree trunks are poor landmarks; large open areas
cause long-range lidar ambiguity.

Usage:
  ros2 launch robot_description outdoor_urban.launch.py
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
                'world': os.path.join(pkg, 'worlds', 'outdoor_urban.sdf'),
                'x': '0.0',
                'y': '0.0',
                'z': '0.1',
            }.items(),
        )
    ])
