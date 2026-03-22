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
  /cmd_vel               Twist             ros→gz
  /odom                  Odometry          gz→ros
  /tf                    TFMessage         gz→ros
  /clock                 Clock             gz→ros
  /front_laser/scan      LaserScan         gz→ros   2D lidar
  /top_lidar_3d/points   PointCloud2       gz→ros   3D lidar (VLP-16)
  /imu/data              Imu               gz→ros   100 Hz
  /magnetometer          MagneticField     gz→ros   50 Hz
  /gps/fix               NavSatFix         gz→ros   10 Hz
  /joint_states          JointState        gz→ros

World selection (world:=<name> or world:=<full/path/to/world.sdf>):
  empty         — flat ground plane (default)
  slam_district — large 5-zone world for persistent mapping / loop-closure testing
  office        — indoor office with rooms and corridors
  warehouse     — large open warehouse with shelving
  construction  — outdoor construction site with obstacles
  orchard       — outdoor orchard rows
  pipeline      — industrial pipeline environment
  solar_farm    — open outdoor solar farm
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

# Short-name → filename/path map for built-in worlds.
# Local world filenames (no leading '/') are resolved relative to our
# installed share/worlds/ directory.  Absolute paths are used as-is.
_CLEARPATH_WORLDS = '/opt/ros/jazzy/share/clearpath_gz/worlds'
_WORLDS = {
    'empty':         'automaton_world.sdf',
    'slam_district': 'slam_district.sdf',
    'office':        f'{_CLEARPATH_WORLDS}/office.sdf',
    'warehouse':     f'{_CLEARPATH_WORLDS}/warehouse.sdf',
    'construction':  f'{_CLEARPATH_WORLDS}/construction.sdf',
    'orchard':       f'{_CLEARPATH_WORLDS}/orchard.sdf',
    'pipeline':      f'{_CLEARPATH_WORLDS}/pipeline.sdf',
    'solar_farm':    f'{_CLEARPATH_WORLDS}/solar_farm.sdf',
}


def _resolve_world(context, pkg_share_dir):
    """Resolve world:= argument — short name or full path."""
    raw = LaunchConfiguration('world').perform(context)
    if raw in _WORLDS:
        val = _WORLDS[raw]
        path = val if val.startswith('/') else os.path.join(pkg_share_dir, 'worlds', val)
    else:
        path = raw  # treat as a full file path
    if not os.path.isfile(path):
        raise FileNotFoundError(f'[gazebo.launch.py] World file not found: {path}')
    return path


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

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty',
        description=(
            'World to load. Use a short name (empty, office, warehouse, '
            'construction, orchard, pipeline, solar_farm) or a full path to an SDF file.'
        ),
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
    # OpaqueFunction resolves the world short-name to an absolute path at
    # launch time, then starts Gazebo with that world.
    # -r  = run immediately (don't wait for play button)
    # -v3 = verbosity level 3

    def launch_gazebo(context, *args, **kwargs):
        world_path = _resolve_world(context, pkg_share_dir)
        gz_sim = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
                ])
            ]),
            launch_arguments={
                'gz_args': f'-r -v3 {world_path}',
            }.items(),
        )
        return [gz_sim]

    gz_sim = OpaqueFunction(function=launch_gazebo)

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
    #
    # NOTE: /joint_states is intentionally NOT bridged here.
    #   The Gazebo JointStatePublisher plugin publishes gz msgs with
    #   timestamps that arrive at ROS via DDS with no ordering guarantee.
    #   When robot_state_publisher receives them slightly out of order it
    #   sees time go backwards → continuous "Moved backwards in time" spam
    #   → TF buffer clears → RViz blinks.
    #   Solution: joint_state_publisher (below) is the sole source —
    #   a single ROS timer always publishes in strictly increasing sim time.
    #   Wheel rotation won't be visualised (always 0) until we wire up
    #   ros2_control in a later step.

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            # /tf intentionally NOT bridged — DiffDrive publishes odom→base_link
            # TF with Gazebo sim timestamps over DDS, which arrive out of order
            # and cause RViz TF buffer time-jump resets. odom (Odometry) above
            # is what the EKF consumes; the TF will come from ros2_control later.
            '/front_laser/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/top_lidar_3d/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/magnetometer@sensor_msgs/msg/MagneticField[gz.msgs.Magnetometer',
            '/gps/fix@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
        ],
    )

    # --- Joint state publisher -----------------------------------------------
    # Sole source of /joint_states. Publishes all wheel joints at position 0.
    # Single ROS timer → strictly monotonic sim-time stamps → no backwards
    # time jumps in robot_state_publisher.

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
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
    # Fix: delay all ROS nodes by 5 seconds so Gazebo has time to start and
    # publish /clock before any node tries to use sim time. The bridge node
    # starts immediately (it doesn't use sim time itself).
    #
    # Why NOT joint_state_publisher here:
    #   Having two joint state sources (this ROS node + the Gazebo bridge)
    #   causes robot_state_publisher to receive alternating messages with
    #   slightly different sim time stamps, triggering continuous "moved
    #   backwards in time" warnings and RViz blinking. The Gazebo
    #   JointStatePublisher system plugin is the sole source of joint states.

    delayed_ros_nodes = TimerAction(
        period=5.0,
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
        world_arg,
        x_pos,
        y_pos,
        z_pos,
        gz_sim,                  # 1. start Gazebo (OpaqueFunction resolves world path)
        bridge,                  # 2. start bridge (delivers /clock immediately)
        delayed_ros_nodes,       # 3. start ROS nodes after clock is established
    ])
