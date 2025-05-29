from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    robot_config_arg = DeclareLaunchArgument(
        'robot_config',
        default_value=['/home/ubuntu/ros2_ws/src/gello_ros/config/cobotta.yaml']
    )

    common_config_arg = DeclareLaunchArgument(
        'common_config',
        default_value=['/home/ubuntu/ros2_ws/src/gello_ros/config/common.yaml']
    )

    save_episode_arg = DeclareLaunchArgument(
        'save_episode',
        default_value='false'
    )

    node_start_delay_arg = DeclareLaunchArgument(
        'node_start_delay',
        default_value='5.0'
    )

    agent_type_arg = DeclareLaunchArgument(
        'agent_type',
        default_value='gello'
    )

    # Load configurations
    robot_config = LaunchConfiguration('robot_config')
    common_config = LaunchConfiguration('common_config')
    save_episode = LaunchConfiguration('save_episode')
    node_start_delay = LaunchConfiguration('node_start_delay')
    agent_type = LaunchConfiguration('agent_type')

    # Nodes
    camera_node = Node(
        package='gello_ros',
        executable='run_camera_nodes.py',
        name='gello_camera_server',
        output='screen',
        parameters=[robot_config, common_config]
    )

    robot_node = Node(
        package='gello_ros',
        executable='run_robot_node.py',
        name='gello_robot_server',
        output='screen',
        parameters=[robot_config, common_config]
    )

    agent_node = TimerAction(
        period=LaunchConfiguration('node_start_delay'),
        actions=[
            Node(
                package='gello_ros',
                executable='run_agent_node.py',
                name='gello_agent',
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

    return LaunchDescription([
        robot_config_arg,
        common_config_arg,
        save_episode_arg,
        node_start_delay_arg,
        agent_type_arg,
        camera_node,
        robot_node,
        agent_node
    ])
