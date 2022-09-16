from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

  # add clearpath race world
  world_file = PathJoinSubstitution(
    [FindPackageShare('automaton_gazebo'),
    'worlds',
    'clearpath_race_world.world']
  )

  gazebo_launch = PathJoinSubstitution(
    [FindPackageShare('automaton_gazebo'),
    'launch',
    'gazebo.launch.py']
  )

  gazebo_sim = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([gazebo_launch]),
    launch_arguments={'world_path': world_file}.items()
  )

  rviz_launch = PathJoinSubstitution(
    [FindPackageShare('automaton_rviz'),
    'launch',
    'automaton_rviz.launch.py']
  )

  rviz_sim = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([rviz_launch]),
  )
  
  ld = LaunchDescription()
  ld.add_action(gazebo_sim)
  ld.add_action(rviz_sim)

  return ld