import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, TimerAction
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    PythonExpression,
    PathJoinSubstitution,
    FindExecutable,
)
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # --- Launch Configurations ---
    gello_config = LaunchConfiguration('gello_config')
    touch_config = LaunchConfiguration('touch_config')
    node_start_delay = LaunchConfiguration('node_start_delay')
    device_type = LaunchConfiguration('device_type')

    # --- Path Definitions (for config files) ---
    gello_ros_share_dir = get_package_share_directory('gello_ros')

    # --- Robot Description Definition (New, Recommended Way) ---
    # Find the 'cat' executable and the URDF file path using ROS 2 substitutions
    robot_description_content = Command([
        FindExecutable(name='cat'), ' ',
        PathJoinSubstitution([
            FindPackageShare('touch_description'),
            'urdf',
            'touch.urdf'
        ])
    ])
    # Wrap the command output in a ParameterValue for clarity and type safety
    robot_description_param = ParameterValue(robot_description_content, value_type=str)


    return LaunchDescription([
        # --- Declare Launch Arguments ---
        DeclareLaunchArgument(
            'gello_config',
            default_value=os.path.join(gello_ros_share_dir, 'config', 'gello.yaml'),
            description='Path to the gello config file'),
        DeclareLaunchArgument(
            'touch_config',
            default_value=os.path.join(gello_ros_share_dir, 'config', 'touch.yaml'),
            description='Path to the touch config file'),
        DeclareLaunchArgument(
            'node_start_delay',
            default_value='2.0',
            description='Delay before starting the node'),
        DeclareLaunchArgument(
            'device_type',
            default_value='touch',
            description='Device type (gello or touch)'),

        # --- Group for 'gello' device ---
        GroupAction(
            condition=IfCondition(PythonExpression(["'", device_type, "' == 'gello'"])),
            actions=[
                Node(
                    package='gello_ros',
                    executable='run_gello.py',
                    name='run_gello',
                    output='screen',
                    parameters=[gello_config]
                )
            ]
        ),

        # --- Group for 'touch' device ---
        GroupAction(
            condition=IfCondition(PythonExpression(["'", device_type, "' == 'touch'"])),
            actions=[
                Node(
                    package='touch_common',
                    executable='touch_state',
                    name='touch_state',
                    output='screen',
                    parameters=[touch_config]
                ),
                # --- Updated robot_state_publisher Node ---
                Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    name='touch_robot_state_publisher',
                    output='screen',
                    # Use the newly defined parameter for robot_description
                    parameters=[{'robot_description': robot_description_param}],
                    remappings=[
                        ('joint_states', 'touch/joint_states'),
                        ('robot_description', 'touch_robot_description')
                    ]
                ),
                TimerAction(
                    period=node_start_delay,
                    actions=[
                        Node(
                            package='tf2_ros',
                            executable='static_transform_publisher',
                            name='static_transform_publisher',
                            arguments=['1', '0.5', '0', '0', '0', '0', 'world', 'touch_base']
                        )
                    ]
                )
            ]
        )
    ])
