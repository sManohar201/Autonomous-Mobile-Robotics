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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('robot_description')

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
    z_pos = DeclareLaunchArgument('z', default_value='0.1')  # slight offset to avoid ground collision

    # --- Robot description ---------------------------------------------------

    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ',
        PathJoinSubstitution([pkg_share, 'urdf', 'automaton.urdf.xacro']),
    ])

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
    # -v4 = verbosity level 4 (set to 1 in production to reduce noise)

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
            # Simulation clock — must be bridged so ROS2 nodes run on sim time
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',

            # Drive commands: teleop/nav2 publishes ROS2 Twist → gz DiffDrive
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',

            # Odometry: gz DiffDrive → ROS2 (consumed by EKF later)
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',

            # TF from DiffDrive plugin (odom → base_link)
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',

            # Front LiDAR scan
            '/front_laser/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',

            # Wheel joint states → robot_state_publisher for TF tree
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',

            # Uncomment when IMU link is re-enabled in URDF:
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

    return LaunchDescription([
        use_sim_time,
        world_file,
        x_pos,
        y_pos,
        z_pos,
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        bridge,
        rviz2,
    ])
