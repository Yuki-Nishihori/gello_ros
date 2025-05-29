from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # === Declare launch arguments ===
    declared_args = [
        DeclareLaunchArgument(
            'robot_config',
            default_value=os.path.join(
                get_package_share_directory('gello_ros'), 'config', 'ur5e.yaml'
            )
        ),
        DeclareLaunchArgument(
            'common_config',
            default_value=os.path.join(
                get_package_share_directory('gello_ros'), 'config', 'common.yaml'
            )
        ),
    ]

    # === Launch configurations ===
    robot_config = LaunchConfiguration('robot_config')
    common_config = LaunchConfiguration('common_config')

    # === Node ===
    gello_node = Node(
        package='gello_ros',
        executable='run_gello.py',  # Pythonスクリプトであればentry_pointsに登録されている必要あり
        name='gello_publisher',
        output='screen',
        parameters=[robot_config, common_config]
    )

    return LaunchDescription(declared_args + [gello_node])
