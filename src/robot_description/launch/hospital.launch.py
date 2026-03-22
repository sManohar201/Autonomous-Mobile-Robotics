"""
hospital.launch.py — Launch the hospital world simulation.

World: long corridors, branching ward rooms, dense medical equipment,
       4 dynamic medical trolley obstacles.

Usage:
  ros2 launch robot_description hospital.launch.py
  ros2 launch robot_description hospital.launch.py x:=5.0 y:=0.0
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
                'world': os.path.join(pkg, 'worlds', 'hospital.sdf'),
                'x': '0.0',
                'y': '0.0',
                'z': '0.1',
            }.items(),
        )
    ])
