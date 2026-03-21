"""
gazebo.launch.py

Full simulation launch: Gazebo Harmonic + robot spawn + ROS2 bridge + RViz2.

Architecture overview:
                                            ┌─────────────────┐
  keyboard / nav stack                      │  Gazebo Harmonic │
       │                                    │  (gz-sim)        │
       │ /cmd_vel (ROS2 Twist)              │                  │
       ▼                                    │  DiffDrive plugin│
  ros_gz_bridge ──────────────────────────► │  → wheel joints  │
                                            │  → /odom (gz)    │
                      /odom (ROS2 Odometry) │  → /tf (gz)      │
  robot_state_pub  ◄──────────────────────  │                  │
  rviz2            ◄─  ros_gz_bridge  ◄───  │  gpu_lidar       │
                      /front_laser/scan      │  → /front_laser/ │
                      /joint_states          │    scan (gz)     │
                      /clock                 │                  │
                                            │  JointStatePub   │
                                            │  → /joint_states │
                                            └─────────────────┘

Topic bridge map (gz ↔ ROS2):
  /cmd_vel             Twist           ros→gz
  /odom                Odometry        gz→ros
  /tf                  TFMessage       gz→ros
  /front_laser/scan    LaserScan       gz→ros
  /joint_states        JointState      gz→ros
  /clock               Clock           gz→ros
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('robot_description')

    # Resolve the installed package share directory at launch-file-load time.
    # Used for GZ_SIM_RESOURCE_PATH — must be a plain string, not a substitution.
    pkg_share_dir = get_package_share_directory('robot_description')

    # --- Fix: mesh URI resolution in Gazebo Harmonic -------------------------
    # When ros_gz_sim converts the URDF to SDF, it rewrites:
    #   package://robot_description/meshes/base.stl
    #     → model://robot_description/meshes/base.stl
    #
    # Gazebo then searches for 'robot_description' as a model directory inside
    # GZ_SIM_RESOURCE_PATH. Adding the parent of our share directory means:
    #   GZ_SIM_RESOURCE_PATH/robot_description/meshes/base.stl  ← found ✓
    gz_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.dirname(pkg_share_dir),  # .../install/robot_description/share
    )

    # --- Arguments -----------------------------------------------------------

    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock',
    )

    world_file = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([pkg_share, 'worlds', 'automaton_world.sdf']),
        description='Path to Gazebo world SDF file',
    )

    x_pos = DeclareLaunchArgument('x', default_value='0.0')
    y_pos = DeclareLaunchArgument('y', default_value='0.0')
    z_pos = DeclareLaunchArgument('z', default_value='0.1')

    # --- Robot description ---------------------------------------------------

    robot_description_content = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ',
            PathJoinSubstitution([pkg_share, 'urdf', 'automaton.urdf.xacro']),
        ]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    # --- Gazebo Harmonic ------------------------------------------------------
    # gz_sim.launch.py accepts gz_args which are passed directly to `gz sim`.
    # -r  = run immediately (don't wait for play button)
    # -v3 = verbosity level 3

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': ['-r -v3 ', LaunchConfiguration('world')],
        }.items(),
    )

    # --- Spawn robot ---------------------------------------------------------
    # ros_gz_sim's `create` executable spawns a model from the /robot_description
    # ROS2 topic (published by robot_state_publisher above).

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'automaton',
            '-topic', 'robot_description',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
        ],
    )

    # --- Joint state publisher -----------------------------------------------
    # Publishes wheel joint positions at 0 so robot_state_publisher can
    # compute TF for all links even before Gazebo's JointStatePublisher
    # plugin is confirmed working.

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    # --- ros_gz_bridge -------------------------------------------------------
    # Bridges gz transport topics ↔ ROS2 topics.
    #
    # Format per argument:
    #   /topic_name@ros_msg_type[gz_msg_type   (gz → ros, unidirectional)
    #   /topic_name@ros_msg_type]gz_msg_type   (ros → gz, unidirectional)
    #   /topic_name@ros_msg_type@gz_msg_type   (bidirectional)

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/front_laser/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            # '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ],
    )

    # --- RViz2 ---------------------------------------------------------------

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=[
            '-d', PathJoinSubstitution([pkg_share, 'rviz_config', 'automaton.rviz'])
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    # --- Startup sequencing --------------------------------------------------
    # Problem: Gazebo sim time starts at t≈0, but ROS nodes using use_sim_time=true
    # start on wall clock time (~1.7e9 seconds, i.e. year 2026). When the /clock
    # bridge starts delivering sim time, every node sees time jump backwards
    # massively, flooding the log with "Moved backwards in time" warnings and
    # causing RViz to reset repeatedly.
    #
    # Fix: delay all ROS nodes by 3 seconds so Gazebo has time to start and
    # publish /clock before any node tries to use sim time. The bridge node
    # starts immediately (it doesn't use sim time itself).

    delayed_ros_nodes = TimerAction(
        period=3.0,
        actions=[
            robot_state_publisher,
            joint_state_publisher,
            spawn_robot,
            rviz2,
        ],
    )

    return LaunchDescription([
        gz_resource_path,        # set GZ_SIM_RESOURCE_PATH before anything starts
        use_sim_time,
        world_file,
        x_pos,
        y_pos,
        z_pos,
        gz_sim,                  # start Gazebo immediately
        bridge,                  # start bridge immediately (no sim time dependency)
        delayed_ros_nodes,       # start ROS nodes after Gazebo clock is established
    ])
