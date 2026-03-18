from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to RPLIDAR launch file
    rplidar_launch = os.path.join(
        get_package_share_directory('rplidar_ros'),
        'launch',
        'rplidar.launch.py'
    )

    # Path to slam_toolbox params
    slam_params = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'config',
        'mapper_params_online_sync.yaml'
    )

    return LaunchDescription([
        # Include RPLIDAR launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rplidar_launch)
        ),

        # Start slam_toolbox node
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params,
                        {'use_sim_time': False,
                         'odom_frame': 'odom',
                         'map_frame': 'map',
                         'base_frame': 'base_link',
                         'scan_topic': '/scan'}]
        )
    ])
