import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    PathJoinSubstitution,
    FindExecutable,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # --- Launch Configurations ---
    touch_config = LaunchConfiguration('touch_config')

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
    
    # --- Declare Launch Arguments ---
    touch_config_arg = DeclareLaunchArgument(
        'touch_config',
        default_value=os.path.join(gello_ros_share_dir, 'config', 'touch.yaml'),
        description='Path to the touch config file')

    touch_state_node = Node(
        package='touch_common',
        executable='touch_state',
        name='touch_state',
        namespace='touch', 
        output='screen',
        parameters=[
            touch_config,
            {'robot_description': robot_description_param}
        ]
    )
    # --- Updated robot_state_publisher Node ---
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='touch_robot_state_publisher',
        namespace='touch', 
        output='screen',
        # Use the newly defined parameter for robot_description
        parameters=[{'robot_description': robot_description_param, 'publish_frequency': 30.0}],
    )

    touch_base_tf_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        namespace='touch', 
        arguments=['1', '0.5', '0', '0', '0', '0', 'world', 'touch_base']
    )

    return LaunchDescription([
        touch_config_arg,
        GroupAction([
            touch_base_tf_publisher_node,
            robot_state_publisher_node,
            touch_state_node,
        ])
    ])
