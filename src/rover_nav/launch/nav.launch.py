from launch import LaunchDescription
from launch.actions import DeclareLaunchArugument
from launch.substitutions import LaunchConfigurations
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    params_file_arg = DeclareLaunchArugument(
        'params_file',
        default_value=os.path.join(
            get_package_share_directory('rover_nav'),
            'params',
            'nav2_params.yaml'
        )
    )

    params_file = LaunchConfigurations('params_file')

    controller = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[params_file]
    )

    planner = Node(
        package='nav2_planner',
        executable='planner_server',
        output='screen',
        parameters=[params_file]
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        output='screen',
        parameters=[params_file]
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
                    {'autostart': True},
                    {'bond_timeout': 0.0},
                    {'node_names': ['controller_server',
                                    'planner_server',
                                    'bt_navigator',
                                    'global_costmap',
                                    'local_costmap']},
                                    params_file])
    
    global_costmap = Node(
        package='nav2_costmap_2d',
        executable='costmap_2d_server',
        output='screen',
        parameters=[params_file])
    
    local_costmap = Node(
        package='nav2_costmap_2d',
        executable='costmap_2d_server',
        output='screen',
        parameters=[params_file])
    
    return LaunchDescription([
        params_file_arg,
        controller,
        planner,
        bt_navigator,
        lifecycle_manager,
        global_costmap,
        local_costmap
    ])

