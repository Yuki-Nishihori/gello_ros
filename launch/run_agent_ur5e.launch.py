
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    robot_config = LaunchConfiguration('robot_config')
    common_config = LaunchConfiguration('common_config')
    gello_config = LaunchConfiguration('gello_config')
    touch_config = LaunchConfiguration('touch_config')
    save_episode = LaunchConfiguration('save_episode')
    agent_type = LaunchConfiguration('agent_type')
    skip_initial_move = LaunchConfiguration('skip_initial_move')
    controller_type = LaunchConfiguration('controller_type')

    gello_ros_share_dir = get_package_share_directory('gello_ros')

    # Robot description generation
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("grinding_robot_description"), "urdf", "ur", "ur5e_with_pestle.urdf.xacro"]),
            " ",
            "robot_ip:=192.168.58.42",
            " ",
            "joint_limit_params:=",
            PathJoinSubstitution([FindPackageShare("grinding_robot_description"), "config", "ur5e", "joint_limits.yaml"]),
            " ",
            "kinematics_params:=",
            PathJoinSubstitution([FindPackageShare("grinding_robot_description"), "config", "ur5e", "default_kinematics.yaml"]),
            " ",
            "physical_params:=",
            PathJoinSubstitution([FindPackageShare("grinding_robot_description"), "config", "ur5e", "physical_parameters.yaml"]),
            " ",
            "visual_params:=",
            PathJoinSubstitution([FindPackageShare("grinding_robot_description"), "config", "ur5e", "visual_parameters.yaml"]),
            " ",
            "safety_limits:=true",
            " ",
            "safety_pos_margin:=0.15",
            " ",
            "safety_k_position:=20",
            " ",
            "name:=ur5e",
            " ",
            "tf_prefix:=",
            " ",
            "use_fake_hardware:=false",
            " ",
            "fake_sensor_commands:=false",
            " ",
            "headless_mode:=false",
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(value=robot_description_content, value_type=str)
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_config',
            default_value=os.path.join(gello_ros_share_dir, 'config', 'ur5e.yaml'),
            description='Path to the robot config file'),
        DeclareLaunchArgument(
            'common_config',
            default_value=os.path.join(gello_ros_share_dir, 'config', 'common.yaml'),
            description='Path to the common config file'),
        DeclareLaunchArgument(
            'gello_config',
            default_value=os.path.join(gello_ros_share_dir, 'config', 'gello.yaml'),
            description='Path to the gello config file'),
        DeclareLaunchArgument(
            'touch_config',
            default_value=os.path.join(gello_ros_share_dir, 'config', 'touch.yaml'),
            description='Path to the touch config file'),
        DeclareLaunchArgument(
            'save_episode',
            default_value='false',
            description='Whether to save the episode'),
        DeclareLaunchArgument(
            'agent_type',
            default_value='touch',
            description='Type of agent to use'),
        DeclareLaunchArgument(
            'skip_initial_move',
            default_value='true',
            description='Whether to skip the initial move'),
        DeclareLaunchArgument(
            'controller_type',
            default_value='cartesian_compliance_controller',
            description='Controller type'),

        Node(
            package='gello_ros',
            executable='run_agent_node.py',
            name='run_agent_node',
            output='screen',
            parameters=[
                robot_config,
                common_config,
                robot_description,
                {
                    'save_episode': save_episode,
                    'agent_type': agent_type,
                    'skip_initial_move': skip_initial_move,
                    'controller_type': controller_type,
                }
            ]
        )
    ])
