#!/usr/bin/env python3
"""
bringup for Waveshare UGV:
- ugv_bringup (UART I/O)
- one IMU orientation filter (complementary OR madgwick)
- ugv_base_node (EKF variant) -> /odom_raw
- robot_localization EKF -> /odom + TF odom->base_footprint
- ugv_description display (URDF/TF; RViz optional)

"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # --------------------
    # Launch-time args
    # --------------------
    use_rviz_arg    = DeclareLaunchArgument('use_rviz',    default_value='false')
    ugv_model_arg   = DeclareLaunchArgument('ugv_model',   default_value='ugv_rover') 
    use_madgwick_arg= DeclareLaunchArgument('use_madgwick',default_value='false')      # false -> complementary filter

    use_rviz     = LaunchConfiguration('use_rviz')
    ugv_model    = LaunchConfiguration('ugv_model')
    use_madgwick = LaunchConfiguration('use_madgwick')

    # --------------------
    # Locate shared files
    # --------------------
    ws_bringup_share = get_package_share_directory('ugv_bringup')
    ws_desc_share    = get_package_share_directory('ugv_description')

    # Reuse Waveshare's IMU filter params (for madgwick) and EKF config
    madgwick_param = os.path.join(ws_bringup_share, 'param', 'imu_filter_param.yaml')
    waveshare_ekf  = os.path.join(ws_bringup_share, 'param', 'ekf.yaml')

    # --------------------
    # Nodes
    # --------------------

    # 1) Serial bringup (publishes /imu/data_raw, /odom/odom_raw; subscribes /cmd_vel)
    bringup_node = Node(
        package='ugv_bringup',
        executable='ugv_bringup',
        name='ugv_bringup',
        output='screen'
        # If a serial port param exists in your version, you can add:
        # parameters=[{'port': '/dev/ttyUSB0'}],
    )

    # 2) Driver (forwards /cmd_vel to the MCU)
    driver_node = Node(
        package='ugv_bringup',
        executable='ugv_driver',
        name='ugv_driver',
        output='screen'
    )

    # 3) IMU orientation filter (choose one)
    imu_comp_filter = Node(
        package='imu_complementary_filter',
        executable='complementary_filter_node',
        name='imu_filter',
        # Minimal, sane defaults; you can tune later or swap to YAML-based madgwick
        parameters=[{
            'do_bias_estimation': True,
            'use_mag': False,
            'publish_tf': False
        }],
        output='screen',
        condition=UnlessCondition(use_madgwick)
    )

    imu_madgwick = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        parameters=[madgwick_param],
        output='screen',
        condition=IfCondition(use_madgwick)
    )

    # 4) Base node (EKF variant) -> /odom_raw ; let EKF own TF
    base_node_ekf = Node(
        package='ugv_base_node',
        executable='base_node_ekf',
        name='base_node_ekf',
        parameters=[{'pub_odom_tf': False}],   # IMPORTANT: EKF will publish odom->base_footprint
        output='screen'
    )

    # 5) Robot Localization EKF -> fuse /odom_raw + /imu/data ; remap to /odom
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[waveshare_ekf],            # start by reusing Waveshare's EKF config
        remappings=[('/odometry/filtered', '/odom')],
        output='screen'
    )

    # 6) Description / TF tree (RViz optional)
    display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ws_desc_share, 'launch', 'display.launch.py')),
        launch_arguments={
            'use_rviz': use_rviz,
            'rviz_config': 'bringup'          # matches a config shipped by ugv_description
        }.items()
    )

    # Waveshare display.launch.py selects URDF by env var UGV_MODEL
    set_model_env = SetEnvironmentVariable(name='UGV_MODEL', value=ugv_model)

    # --------------------
    # Assemble launch
    # --------------------
    return LaunchDescription([
        # Args
        use_rviz_arg, ugv_model_arg, use_madgwick_arg,

        # URDF/TF (no sensors here)
        set_model_env,
        display_launch,

        # Core pipeline (no LiDAR)
        bringup_node,
        driver_node,
        imu_comp_filter,
        imu_madgwick,
        base_node_ekf,
        ekf_node,
    ])
