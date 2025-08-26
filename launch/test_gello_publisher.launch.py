
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    robot_config = LaunchConfiguration('robot_config')
    common_config = LaunchConfiguration('common_config')

    gello_ros_share_dir = get_package_share_directory('gello_ros')

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_config',
            default_value=os.path.join(gello_ros_share_dir, 'config', 'ur5e.yaml'),
            description='Path to the robot config file'),
        DeclareLaunchArgument(
            'common_config',
            default_value=os.path.join(gello_ros_share_dir, 'config', 'common.yaml'),
            description='Path to the common config file'),

        Node(
            package='gello_ros',
            executable='run_gello.py',
            name='gello_publisher',
            output='screen',
            parameters=[
                robot_config,
                common_config
            ]
        )
    ])
