# leo_autonomy_launch.py
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get nav2 launch file
    nav2_launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    return LaunchDescription([
        # SLAM Toolbox
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),

        # Navigation2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_launch_dir, 'navigation_launch.py')
            ),
            launch_arguments={'use_sim_time': 'false'}.items()
        ),

        # Frontier Explorer
        Node(
            package='WavefrontFrontierExplorer',
            executable='frontier_explorer.py',
            name='frontier_explorer',
            output='screen',
            parameters=[
                {'map_topic': '/map'},
                {'goal_topic': '/move_base_simple/goal'}
            ]
        ),

        # Object Detector
        Node(
            package='ObjectDetector',
            executable='object_detector.py',
            name='object_detector',
            output='screen',
            parameters=[
                {'camera_topic': '/camera/color/image_raw'},
                {'detection_topic': '/detected_object'}
            ]
        ),

        # Goal Manager
        Node(
            package='GoalManager',
            executable='goal_manager.py',
            name='goal_manager',
            output='screen',
            parameters=[
                {'exploration_goal_topic': '/move_base_simple/goal'},
                {'object_goal_topic': '/detected_object'},
                {'arm_trigger_topic': '/arm_pick_trigger'}
            ]
        ),

        # Arm Controller
        Node(
            package='TheArm',
            executable='arm_controller.py',
            name='arm_controller',
            output='screen',
            parameters=[
                {'arm_trigger_topic': '/arm_pick_trigger'}
            ]
        )
    ])