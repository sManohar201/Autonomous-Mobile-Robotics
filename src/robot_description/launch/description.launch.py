"""
description.launch.py

Standalone robot visualization launch file (no simulation).
Use this to inspect the URDF model in RViz2 with joint_state_publisher_gui
to manually move joints — useful for verifying the robot model.

Nodes started:
  robot_state_publisher  — publishes TF tree from URDF + joint states
  joint_state_publisher_gui — GUI slider to publish joint states manually
  rviz2                  — visualization
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('robot_description')

    # Expand the xacro file into a robot_description string at launch time.
    # Command() runs the xacro tool and captures its stdout.
    robot_description_content = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ',
            PathJoinSubstitution([pkg_share, 'urdf', 'automaton.urdf.xacro']),
        ]),
        value_type=str,
    )

    rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([pkg_share, 'rviz_config', 'automaton.rviz']),
        description='Path to RViz2 config file',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}],
    )

    # Provides a GUI with sliders to publish JointState messages.
    # Only used in this standalone description launch — in simulation,
    # Gazebo's JointStatePublisher system takes this role.
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
    )

    return LaunchDescription([
        rviz_config,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz2,
    ])
