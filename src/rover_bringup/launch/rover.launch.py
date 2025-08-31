#!/usr/bin/env python3
# rover_bringup/launch/rover.launch.py
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # ---- Args ----
    use_rviz_arg  = DeclareLaunchArgument('use_rviz', default_value='false')
    # Pick the URDF file that exists in ugv_description/urdf/
    # If your repo uses a different filename (e.g., ugv_rover.urdf.xacro), pass it at launch.
    urdf_file_arg = DeclareLaunchArgument('urdf_file', default_value='ugv_rover.urdf')

    use_rviz  = LaunchConfiguration('use_rviz')
    urdf_file = LaunchConfiguration('urdf_file')

    # ---- Paths ----
    ugv_desc_share = FindPackageShare('ugv_description')
    model_path = PathJoinSubstitution([ugv_desc_share, 'urdf', urdf_file])

    # Build robot_description using xacro (works for .urdf.xacro; also passes through .urdf)
    robot_description = {'robot_description': ParameterValue(Command(['xacro', model_path]), value_type=str)}

    # ---- Nodes ----

    # 1) Serial bringup (publishes /imu/data_raw, /odom/odom_raw; subscribes /cmd_vel)
    bringup_node = Node(
        package='ugv_bringup', executable='ugv_bringup', name='ugv_bringup', output='screen'
        # parameters=[{'port': '/dev/ttyUSB0'}],  # uncomment/adjust if your version exposes this param
    )

    # 2) Driver (sends /cmd_vel to MCU)
    driver_node = Node(
        package='ugv_bringup', executable='ugv_driver', name='ugv_driver', output='screen'
    )

    # 3) IMU filter (choose one: complementary by default; switch to madgwick with your own arg if desired)
    imu_complementary = Node(
        package='imu_complementary_filter', executable='complementary_filter_node', name='imu_filter',
        parameters=[{'do_bias_estimation': True, 'use_mag': False, 'publish_tf': False}],
        output='screen'
    )
    # If you prefer Madgwick, comment the node above and use:
    # imu_madgwick = Node(
    #     package='imu_filter_madgwick', executable='imu_filter_madgwick_node', name='imu_filter',
    #     parameters=[PathJoinSubstitution([FindPackageShare('ugv_bringup'),'param','imu_filter_param.yaml'])],
    #     output='screen'
    # )

    # 4) Base node (EKF variant) -> /odom_raw; EKF will own TF
    base_node_ekf = Node(
        package='ugv_base_node', executable='base_node_ekf', name='base_node_ekf',
        parameters=[{'pub_odom_tf': False}], output='screen'
    )

    # 5) Robot Localization EKF -> /odom (remapped)
    ekf_node = Node(
        package='robot_localization', executable='ekf_node', name='ekf_filter_node',
        parameters=[PathJoinSubstitution([FindPackageShare('ugv_bringup'),'param','ekf.yaml'])],
        remappings=[('/odometry/filtered','/odom')], output='screen'
    )

    # 6) Robot State Publisher (direct; no Waveshare display.launch.py; no SLAM refs)
    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher', name='robot_state_publisher',
        parameters=[robot_description], output='screen'
    )

    # 7) (Optional) RViz from YOUR package (provide your own RViz config later if you want)
    rviz_cfg = PathJoinSubstitution([FindPackageShare('rover_bringup'), 'rviz', 'minimal.rviz'])
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', rviz_cfg], output='screen',
        condition=IfCondition(use_rviz)
    )

    return LaunchDescription([
        use_rviz_arg, urdf_file_arg,
        bringup_node,
        driver_node,
        imu_complementary,     # or swap for imu_madgwick above
        base_node_ekf,
        ekf_node,
        rsp,
        rviz,
    ])
