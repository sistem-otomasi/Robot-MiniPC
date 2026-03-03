"""
autonomous_robot.launch.py
============================================================================
Launch file untuk Autonomous Mobile Robot dengan:
  - GPS CUAV M3   → /dev/ttyUSB0
  - IMU Yahboom   → /dev/ttyUSB1
  - Motor/encoder → /dev/ttyACM0

Nodes yang dijalankan:
  1. gps_driver_node       – driver GPS CUAV M3
  2. imu_driver_node       – driver IMU Yahboom
  3. robot_controller_node – driver motor + odometri
  4. imu_filter_madgwick   – filter IMU raw → quaternion
  5. robot_localization    – EKF sensor fusion (odom)
  6. robot_localization    – EKF sensor fusion (map, pakai GPS)
  7. navsat_transform_node – konversi GPS NavSat → Odometry
  8. nav2_bringup          – navigation stack

Usage:
  ros2 launch robot_autonomous autonomous_robot.launch.py
  ros2 launch robot_autonomous autonomous_robot.launch.py gps_port:=/dev/ttyUSB0
  ros2 launch robot_autonomous autonomous_robot.launch.py use_rviz:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ── Package share directories ──────────────────────────────────────────
    pkg_share    = FindPackageShare('robot_autonomous')
    nav2_bringup = FindPackageShare('nav2_bringup')

    # ── Config file paths ──────────────────────────────────────────────────
    ekf_config  = PathJoinSubstitution([pkg_share, 'config', 'ekf_config.yaml'])
    nav2_params = PathJoinSubstitution([pkg_share, 'config', 'nav2_params.yaml'])

    # ── Declare launch arguments ───────────────────────────────────────────
    declare_gps_port = DeclareLaunchArgument(
        'gps_port', default_value='/dev/ttyUSB0',
        description='Serial port untuk GPS CUAV M3')

    declare_gps_baud = DeclareLaunchArgument(
        'gps_baudrate', default_value='9600',
        description='Baudrate GPS CUAV M3')

    declare_imu_port = DeclareLaunchArgument(
        'imu_port', default_value='/dev/ttyUSB1',
        description='Serial port untuk IMU Yahboom')

    declare_imu_baud = DeclareLaunchArgument(
        'imu_baudrate', default_value='115200',
        description='Baudrate IMU Yahboom')

    declare_ctrl_port = DeclareLaunchArgument(
        'ctrl_port', default_value='/dev/ttyACM0',
        description='Serial port untuk motor controller')

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='false',
        description='Jalankan RViz2 atau tidak')

    declare_use_nav2 = DeclareLaunchArgument(
        'use_nav2', default_value='true',
        description='Jalankan Nav2 stack atau tidak')

    declare_map_file = DeclareLaunchArgument(
        'map', default_value='',
        description='Path ke map YAML (kosong = SLAM mode)')

    # ── Launch configuration ───────────────────────────────────────────────
    gps_port   = LaunchConfiguration('gps_port')
    gps_baud   = LaunchConfiguration('gps_baudrate')
    imu_port   = LaunchConfiguration('imu_port')
    imu_baud   = LaunchConfiguration('imu_baudrate')
    ctrl_port  = LaunchConfiguration('ctrl_port')
    use_rviz   = LaunchConfiguration('use_rviz')
    use_nav2   = LaunchConfiguration('use_nav2')
    map_file   = LaunchConfiguration('map')

    # =========================================================================
    # NODE DEFINITIONS
    # =========================================================================

    # ── 1. GPS CUAV M3 Driver ─────────────────────────────────────────────
    gps_driver_node = Node(
        package='robot_autonomous',
        executable='gps_driver_node.py',
        name='gps_driver',
        output='screen',
        parameters=[{
            'port':         gps_port,
            'baudrate':     gps_baud,
            'frame_id':     'gps_link',
            'publish_rate': 10.0,
            'min_satellites': 4,
        }],
        remappings=[
            ('/gps/fix', '/gps/fix'),
        ]
    )

    # ── 2. IMU Yahboom Driver ─────────────────────────────────────────────
    imu_driver_node = Node(
        package='robot_autonomous',
        executable='imu_driver_node.py',
        name='imu_driver',
        output='screen',
        parameters=[{
            'port':         imu_port,
            'baudrate':     imu_baud,
            'frame_id':     'imu_link',
            'gyro_range':   250.0,
            'accel_range':  16.0,
            'publish_rate': 100.0,
        }]
    )

    # ── 3. Robot Controller (Motor + Odometri) ────────────────────────────
    robot_controller_node = Node(
        package='robot_autonomous',
        executable='robot_controller_node.py',
        name='robot_controller',
        output='screen',
        parameters=[{
            'serial_port':      ctrl_port,
            'serial_baud':      115200,
            'wheel_base':       0.30,
            'wheel_radius':     0.065,
            'encoder_ticks':    1440,
            'max_linear_vel':   1.0,
            'max_angular_vel':  2.0,
            'odom_frame':       'odom',
            'base_frame':       'base_link',
            'publish_tf':       True,
        }]
    )

    # ── 4. IMU Filter Madgwick ────────────────────────────────────────────
    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick',
        output='screen',
        parameters=[{
            'use_mag':          False,
            'publish_tf':       False,
            'world_frame':      'enu',
            'gain':             0.1,
            'zeta':             0.0,
            'fixed_frame':      'base_link',
            'orientation_stddev': 0.05,
        }],
        remappings=[
            ('/imu/data_raw', '/imu/data_raw'),
            ('/imu/data',     '/imu/data'),
        ]
    )

    # ── 5. EKF Localization – odom frame ─────────────────────────────────
    ekf_odom_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_odom',
        output='screen',
        parameters=[ekf_config],
        remappings=[
            ('/odometry/filtered', '/odometry/filtered_odom'),
        ]
    )

    # ── 6. EKF Localization – map frame (GPS fused) ───────────────────────
    ekf_map_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_map',
        output='screen',
        parameters=[ekf_config],
        remappings=[
            ('/odometry/filtered', '/odometry/filtered'),
        ]
    )

    # ── 7. Navsat Transform (GPS → Odometry) ──────────────────────────────
    navsat_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[ekf_config],
        remappings=[
            ('/gps/fix',                '/gps/fix'),
            ('/imu/data',               '/imu/data'),
            ('/odometry/filtered',      '/odometry/filtered'),
            ('/odometry/gps',           '/odometry/gps'),
        ]
    )

    # ── 8. Static TF Publishers ───────────────────────────────────────────
    # base_link → imu_link
    tf_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_imu',
        output='screen',
        arguments=['0.0', '0.0', '0.05',   # x y z
                   '0',   '0',   '0', '1',  # qx qy qz qw
                   'base_link', 'imu_link']
    )

    # base_link → gps_link
    tf_gps = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_gps',
        output='screen',
        arguments=['0.1', '0.0', '0.15',   # x y z
                   '0',   '0',   '0', '1',  # qx qy qz qw
                   'base_link', 'gps_link']
    )

    # ── 9. Nav2 Bringup (delayed 5s, menunggu sensor siap) ────────────────
    nav2_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        nav2_bringup, 'launch', 'bringup_launch.py'
                    ])
                ]),
                launch_arguments={
                    'use_sim_time':  'false',
                    'params_file':   nav2_params,
                    'map':           map_file,
                    'autostart':     'true',
                }.items(),
            )
        ],
        condition=IfCondition(use_nav2)
    )

    # ── 10. RViz2 (opsional) ──────────────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(use_rviz)
    )

    # ── Log startup info ──────────────────────────────────────────────────
    log_start = LogInfo(msg=[
        '\n================================================================\n'
        '  Autonomous Mobile Robot – ROS Humble\n'
        '  GPS  : CUAV M3   → ', gps_port,  '\n'
        '  IMU  : Yahboom   → ', imu_port,  '\n'
        '  Ctrl : Embedded  → ', ctrl_port, '\n'
        '  Nav2 : ', use_nav2, '\n'
        '================================================================'
    ])

    # =========================================================================
    # LAUNCH DESCRIPTION
    # =========================================================================
    return LaunchDescription([
        # Declare arguments
        declare_gps_port,
        declare_gps_baud,
        declare_imu_port,
        declare_imu_baud,
        declare_ctrl_port,
        declare_use_rviz,
        declare_use_nav2,
        declare_map_file,

        # Log
        log_start,

        # Sensor drivers
        gps_driver_node,
        imu_driver_node,
        robot_controller_node,

        # IMU filter
        imu_filter_node,

        # Static TFs
        tf_imu,
        tf_gps,

        # Localization
        ekf_odom_node,
        ekf_map_node,
        navsat_node,

        # Navigation + RViz
        nav2_launch,
        rviz_node,
    ])
