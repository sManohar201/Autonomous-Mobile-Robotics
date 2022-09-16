from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, FindExecutable, Command, EnvironmentVariable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit

from pathlib import Path
from ament_index_python.packages import get_package_share_directory

ARGUMENTS = [
  DeclareLaunchArgument('world_path', default_value='', description='The world path, with empty world as default world file'),
]


def generate_launch_description():

  gz_resource_path = SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=[
                                                EnvironmentVariable('GAZEBO_MODEL_PATH',
                                                                    default_value=''),
                                                '/usr/share/gazebo-11/models/:',
                                                str(Path(get_package_share_directory('automaton_description')).
                                                    parent.resolve())])

  # launch args
  world_path = LaunchConfiguration('world_path')

  # Get URDF via xacro
  robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("automaton_description"),
                                  "urdf",
                                  "automaton.urdf.xacro"])
        ]
  )

  robot_description = {"robot_description": robot_description_content}

  # controller config is loaded properly
  controllers_file = PathJoinSubstitution(
      [FindPackageShare("automaton_control"),
      "config",
      "control.yaml"]
  )

  spawn_jackal_velocity_controller = Node(
      package='controller_manager',
      executable='spawner.py',
      parameters=[controllers_file],
      arguments=['automaton_base_controller', '-c', '/controller_manager'],
      output='screen',
  )

  node_robot_state_publisher = Node(
      package="robot_state_publisher",
      executable="robot_state_publisher",
      output="screen",
      parameters=[{'use_sim_time': True}, robot_description],
      remappings=[("/automaton_base_controller/cmd_vel", "/cmd_vel"),],
  )

  spawn_joint_state_broadcaster = Node(
      package='controller_manager',
      executable='spawner.py',
      arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
      output='screen',
  )

  # Make sure spawn_husky_velocity_controller starts after spawn_joint_state_broadcaster
  diffdrive_controller_spawn_callback = RegisterEventHandler(
      event_handler=OnProcessExit(
          target_action=spawn_joint_state_broadcaster,
          on_exit=[spawn_jackal_velocity_controller],
      )
  )
  # Gazebo server
  gzserver = ExecuteProcess(
      cmd=['gzserver',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            world_path],
      output='screen',
  )

  # Gazebo client
  gzclient = ExecuteProcess(
      cmd=['gzclient'],
      output='screen',
      # condition=IfCondition(LaunchConfiguration('gui')),
  )

  # Spawn robot
  spawn_robot = Node(
      package='gazebo_ros',
      executable='spawn_entity.py',
      name='spawn_automaton',
      arguments=['-entity',
                  'automaton',
                  '-topic',
                  'robot_description'],
      output='screen',
  )

  ld = LaunchDescription(ARGUMENTS)
  ld.add_action(gz_resource_path)
  ld.add_action(node_robot_state_publisher)
  ld.add_action(spawn_joint_state_broadcaster)
  ld.add_action(diffdrive_controller_spawn_callback)
  ld.add_action(gzserver)
  ld.add_action(gzclient)
  ld.add_action(spawn_robot)

  return ld