from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
        DeclareLaunchArgument('save_episode', default_value='false'),
        DeclareLaunchArgument('node_start_delay', default_value='10.0'),
        DeclareLaunchArgument('agent_type', default_value='gello'),
    ]

    # === LaunchConfigurations ===
    robot_config = LaunchConfiguration('robot_config')
    common_config = LaunchConfiguration('common_config')
    node_start_delay = LaunchConfiguration('node_start_delay')

    # === Include realsense.launch.py ===
    realsense_launch_path = os.path.join(
        get_package_share_directory('gello_ros'),
        'launch',
        'realsense.launch.py'  # <- ROS 2 launch file expected to be .py
    )
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch_path)
    )

    # === Gello camera server node with delayed start ===
    camera_server_node = TimerAction(
        period=node_start_delay,
        actions=[
            Node(
                package='gello_ros',
                executable='run_realsense_ros_nodes.py',
                name='gello_camera_server',
                output='screen',
                parameters=[robot_config, common_config],
            )
        ]
    )

    # === Combine everything ===
    return LaunchDescription(declared_args + [
        realsense_launch,
        camera_server_node
    ])
