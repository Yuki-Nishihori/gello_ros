from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    declared_args = [
        DeclareLaunchArgument('robot_config', default_value='/home/ubuntu/ros2_ws/src/gello_ros/config/ur5e.yaml'),
        DeclareLaunchArgument('common_config', default_value='/home/ubuntu/ros2_ws/src/gello_ros/config/common.yaml'),
        DeclareLaunchArgument('gello_config', default_value='/home/ubuntu/ros2_ws/src/gello_ros/config/gello.yaml'),
        DeclareLaunchArgument('touch_config', default_value='/home/ubuntu/ros2_ws/src/gello_ros/config/touch.yaml'),
        DeclareLaunchArgument('save_episode', default_value='false'),
        DeclareLaunchArgument('agent_type', default_value='touch'),
        DeclareLaunchArgument('node_start_delay', default_value='0'),
    ]

    # Launch configurations
    robot_config = LaunchConfiguration('robot_config')
    common_config = LaunchConfiguration('common_config')
    save_episode = LaunchConfiguration('save_episode')
    agent_type = LaunchConfiguration('agent_type')
    node_start_delay = LaunchConfiguration('node_start_delay')

    # Node with delay (equivalent to launch-prefix="sleep ...")
    agent_node = TimerAction(
        period=node_start_delay,
        actions=[
            Node(
                package='gello_ros',
                executable='run_agent_node.py',
                name='run_agent_node',
                output='screen',
                parameters=[
                    robot_config,
                    common_config,
                    {'save_episode': save_episode},
                    {'agent_type': agent_type}
                ]
            )
        ]
    )

    return LaunchDescription(declared_args + [agent_node])
