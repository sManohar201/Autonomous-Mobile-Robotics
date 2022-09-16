from unicodedata import name
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, FindExecutable, Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

  robot_description_arg = DeclareLaunchArgument(
    'robot_description_command',
    default_value=[
      PathJoinSubstitution([FindExecutable(name='xacro')]),
      ' ',
      PathJoinSubstitution([FindPackageShare('automaton_description'), 'urdf', 
                              'automaton.urdf.xacro'])
    ]
  )

  # robot_description_content = ParameterValue(
  #       Command(LaunchConfiguration('robot_description_command')),
  #       value_type=str
  # )

  pkg_jackal_viz = FindPackageShare('automaton_rviz')
  # TODO: later add dynamic loading of rviz config files
  rviz_config = PathJoinSubstitution(
        [pkg_jackal_viz, 'rviz', 'model.rviz']
  )

  joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
  )

  rviz_node = Node(
    package='rviz2',
    executable='rviz2',
    output='screen',
    arguments=['-d', rviz_config]
  )

  ld = LaunchDescription()
  ld.add_action(robot_description_arg)
  ld.add_action(joint_state_publisher_gui_node)
  ld.add_action(rviz_node)

  return ld

