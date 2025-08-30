#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    """Launch setup function to handle parameters dynamically"""
    
    # Get launch arguments
    agent_type = LaunchConfiguration('agent_type').perform(context)
    camera_names = LaunchConfiguration('camera_names').perform(context)
    control_hz = int(LaunchConfiguration('control_hz').perform(context))
    controller_type = LaunchConfiguration('controller_type').perform(context)
    control_mode = LaunchConfiguration('control_mode').perform(context)
    use_gripper = LaunchConfiguration('use_gripper').perform(context) == 'true'
    use_FT_sensor = LaunchConfiguration('use_FT_sensor').perform(context) == 'true'
    skip_initial_move = LaunchConfiguration('skip_initial_move').perform(context) == 'true'
    save_episode = LaunchConfiguration('save_episode').perform(context) == 'true'
    gello_port = LaunchConfiguration('gello_port').perform(context)
    number_of_episodes = int(LaunchConfiguration('number_of_episodes').perform(context))
    number_of_steps = int(LaunchConfiguration('number_of_steps').perform(context))
    eval_ckpt_dir = LaunchConfiguration('eval_ckpt_dir').perform(context)
    
    # Parse camera names (comma-separated string to list)
    camera_names_list = [name.strip() for name in camera_names.split(',') if name.strip()]
    
    # Parse joint arrays from string (space-separated)
    robot_home_joints_str = LaunchConfiguration('robot_home_joints_with_gello').perform(context)
    robot_home_joints = [float(x) for x in robot_home_joints_str.split()]
    
    robot_home_pose_str = LaunchConfiguration('robot_home_pose_with_touch').perform(context)
    robot_home_pose = [float(x) for x in robot_home_pose_str.split()]
    
    robot_start_pose_str = LaunchConfiguration('robot_start_pose_with_touch').perform(context)
    robot_start_pose = [float(x) for x in robot_start_pose_str.split()]

    # Create component container
    container = ComposableNodeContainer(
        name='agent_component_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='gello_ros',
                plugin='gello_ros::AgentNodeComponent',  # This would be for C++ components
                name='agent_node',
                parameters=[{
                    'agent_type': agent_type,
                    'camera_names': camera_names_list,
                    'control_hz': control_hz,
                    'robot_home_joints_with_gello': robot_home_joints,
                    'robot_home_pose_with_touch': robot_home_pose,
                    'robot_start_pose_with_touch': robot_start_pose,
                    'controller_type': controller_type,
                    'control_mode': control_mode,
                    'use_gripper': use_gripper,
                    'use_FT_sensor': use_FT_sensor,
                    'skip_initial_move': skip_initial_move,
                    'save_episode': save_episode,
                    'gello_port': gello_port,
                    'number_of_episodes': number_of_episodes,
                    'number_of_steps': number_of_steps,
                    'eval_ckpt_dir': eval_ckpt_dir,
                }],
                extra_arguments=[{'use_intra_process_comms': True}],
            )
        ],
        output='screen',
    )
    
    # Alternative: Use regular Node with component_mode parameter
    # Since Python components work differently, we use a regular node with component_mode flag
    agent_node = Node(
        package='gello_ros',
        executable='run_agent_node.py',
        name='agent_node_component',
        parameters=[{
            'agent_type': agent_type,
            'camera_names': camera_names_list,
            'control_hz': control_hz,
            'robot_home_joints_with_gello': robot_home_joints,
            'robot_home_pose_with_touch': robot_home_pose,
            'robot_start_pose_with_touch': robot_start_pose,
            'controller_type': controller_type,
            'control_mode': control_mode,
            'use_gripper': use_gripper,
            'use_FT_sensor': use_FT_sensor,
            'skip_initial_move': skip_initial_move,
            'save_episode': save_episode,
            'gello_port': gello_port,
            'number_of_episodes': number_of_episodes,
            'number_of_steps': number_of_steps,
            'eval_ckpt_dir': eval_ckpt_dir,
        }],
        output='screen',
    )

    return [agent_node]


def generate_launch_description():
    """Generate launch description with all necessary arguments"""
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'agent_type',
            default_value='gello',
            description='Type of agent to use (gello, touch, act, dummy)'
        ),
        DeclareLaunchArgument(
            'camera_names',
            default_value='',
            description='Comma-separated list of camera names'
        ),
        DeclareLaunchArgument(
            'control_hz',
            default_value='100',
            description='Control frequency in Hz'
        ),
        DeclareLaunchArgument(
            'robot_home_joints_with_gello',
            default_value='0.0 0.0 0.0 0.0 0.0 0.0 0.0',
            description='Home joint positions for Gello agent'
        ),
        DeclareLaunchArgument(
            'robot_home_pose_with_touch',
            default_value='0.0 0.0 0.0 0.0 0.0 0.0 1.0',
            description='Home pose for Touch agent (x y z qx qy qz qw)'
        ),
        DeclareLaunchArgument(
            'robot_start_pose_with_touch',
            default_value='0.0 0.0 0.0 0.0 0.0 0.0 1.0',
            description='Start pose for Touch agent (x y z qx qy qz qw)'
        ),
        DeclareLaunchArgument(
            'controller_type',
            default_value='cartesian_impedance_controller',
            description='Type of robot controller'
        ),
        DeclareLaunchArgument(
            'control_mode',
            default_value='cartesian',
            description='Control mode (joint or cartesian)'
        ),
        DeclareLaunchArgument(
            'use_gripper',
            default_value='false',
            description='Whether to use gripper'
        ),
        DeclareLaunchArgument(
            'use_FT_sensor',
            default_value='false',
            description='Whether to use force/torque sensor'
        ),
        DeclareLaunchArgument(
            'skip_initial_move',
            default_value='false',
            description='Whether to skip initial move to home position'
        ),
        DeclareLaunchArgument(
            'save_episode',
            default_value='false',
            description='Whether to enable episode saving interface'
        ),
        DeclareLaunchArgument(
            'gello_port',
            default_value='',
            description='Serial port for Gello device'
        ),
        DeclareLaunchArgument(
            'number_of_episodes',
            default_value='1',
            description='Number of episodes to record'
        ),
        DeclareLaunchArgument(
            'number_of_steps',
            default_value='10000',
            description='Number of steps per episode'
        ),
        DeclareLaunchArgument(
            'eval_ckpt_dir',
            default_value='policy_last.ckpt',
            description='Path to evaluation checkpoint for ACT agent'
        ),
        
        # Launch setup function
        OpaqueFunction(function=launch_setup)
    ])