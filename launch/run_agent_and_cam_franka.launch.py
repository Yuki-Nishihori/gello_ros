#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction, GroupAction
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    """Launch setup function to handle parameters dynamically"""
    
    # Get launch arguments - Agent parameters
    robot_config = LaunchConfiguration('robot_config').perform(context)
    common_config = LaunchConfiguration('common_config').perform(context)
    touch_config = LaunchConfiguration('touch_config').perform(context)
    save_episode = LaunchConfiguration('save_episode').perform(context) == 'true'
    agent_type = LaunchConfiguration('agent_type').perform(context)
    skip_initial_move = LaunchConfiguration('skip_initial_move').perform(context) == 'true'
    controller_type = LaunchConfiguration('controller_type').perform(context)
    
    # Get launch arguments - Camera parameters
    camera_device = LaunchConfiguration('camera_device').perform(context)
    camera_width = int(LaunchConfiguration('camera_width').perform(context))
    camera_height = int(LaunchConfiguration('camera_height').perform(context))
    camera_framerate = float(LaunchConfiguration('camera_framerate').perform(context))
    camera_name = LaunchConfiguration('camera_name').perform(context)
    
    # Get launch arguments - System parameters
    enable_touch_system = LaunchConfiguration('enable_touch_system').perform(context) == 'true'
    num_threads = int(LaunchConfiguration('num_threads').perform(context))
    
    # Leptrino FT sensor component
    leptrino_component = ComposableNode(
        package='leptrino_force_torque',
        plugin='LeptrinoNode',
        name='leptrino_force_torque_node',
        namespace='leptrino',
        parameters=[{
            'com_port': '/dev/ttyACM0',
            'frame_id': 'leptrino_frame',
            'rate': 1200.0
        }],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    # Leptrino wrench filter component
    leptrino_filter_component = ComposableNode(
        package='grinding_force_torque',
        plugin='grinding_force_torque::WrenchFilter',
        name='leptrino_wrench_filter',
        namespace='leptrino',
        parameters=[{
            'input_topic': '/leptrino/wrench',
            'output_topic': '/leptrino/wrench/filtered',
            'sampling_frequency': 1200.0,
            'cutoff_frequency': 2.5,
            'filter_order': 3,
            'data_window': 100,
            'initial_zero': True,
            'disable_filtering': False
        }],
        extra_arguments=[{'use_intra_process_comms': True}],
    )
    
    leptrino_container = ComposableNodeContainer(
        name='leptrino_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[leptrino_component, leptrino_filter_component],
        parameters=[{'use_intra_process_comms': True}],
        output='screen',
    )
    
    # Robot descriptions
    # Franka FR3 robot description
    franka_robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare("onolab_robot_description"), "robots", "fr3", "fr3_with_pestle_and_leptrino.urdf.xacro"]),
    ])
    
    # Touch device robot description (if enabled)
    touch_robot_description_content = None
    if enable_touch_system:
        touch_robot_description_content = Command([
            FindExecutable(name='cat'), ' ',
            PathJoinSubstitution([
                FindPackageShare('touch_description'),
                'urdf',
                'touch.urdf'
            ])
        ])
    
    # Create USB Camera Component (to be started after touch system)
    camera_component = ComposableNode(
        package='usb_cam',
        plugin='usb_cam::UsbCamNode',
        name=camera_name,
        namespace=camera_name,
        parameters=[{
            'video_device': camera_device,
            'image_width': camera_width,
            'image_height': camera_height,
            'framerate': camera_framerate,
            'pixel_format': 'mjpeg2rgb',
            'camera_name': camera_name,
            'camera_info_url': '',
            'io_method': 'mmap',
            'frame_id': f'{camera_name}_optical_frame',
        }],
        remappings=[
            ('image_raw', 'color/image_raw')
        ],
        extra_arguments=[{'use_intra_process_comms': True}],
    )
    
    # Create ComponentManager container (initially empty, components loaded later)
    main_container = ComposableNodeContainer(
        name='franka_integrated_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',  # Multi-threaded container
        composable_node_descriptions=[],  # Start empty, load components in sequence
        parameters=[
            {
                'use_intra_process_comms': True,
                'num_threads': num_threads,
            }
        ],
        output='screen',
    )
    
    # Agent Node (Python with component mode)
    agent_node = Node(
        package='gello_ros',
        executable='run_agent_component.py',
        name='agent_node_franka_integrated',
        output='screen',
        parameters=[
            robot_config,
            common_config,
            {
                'robot_description': ParameterValue(value=franka_robot_description_content, value_type=str),
                'save_episode': save_episode,
                'agent_type': agent_type,
                'skip_initial_move': skip_initial_move,
                'controller_type': controller_type,
                'component_mode': True,  # Enable component mode
                'camera_names': [camera_name],  # Pass camera name to agent
            }
        ]
    )
    
    # GUI Node (only when save_episode is enabled)
    gui_node = None
    if save_episode:
        gui_node = Node(
            package='gello_ros',
            executable='run_GUI_component.py',
            name='gui_button_publisher',
            output='screen',
            parameters=[{
                'component_mode': True,  # Enable component mode
            }]
        )
    
    # Start with ComponentManager container
    nodes_to_launch = [main_container, leptrino_container]
    
    # Touch system nodes (if enabled) - STEP 1
    if enable_touch_system:
        # Touch state node
        touch_state_node = Node(
            package='touch_common',
            executable='touch_state',
            name='touch_state',
            namespace='touch',
            output='screen',
            parameters=[
                touch_config,
                {'robot_description': ParameterValue(value=touch_robot_description_content, value_type=str)}
            ]
        )
        
        # Touch robot state publisher
        touch_robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='touch_robot_state_publisher',
            namespace='touch',
            output='screen',
            parameters=[{
                'robot_description': ParameterValue(value=touch_robot_description_content, value_type=str),
                'publish_frequency': 30.0
            }],
        )
        
        # Static transform publisher for touch base
        touch_base_tf_publisher = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='touch_base_tf_publisher',
            namespace='touch',
            arguments=['1', '0.5', '0', '-1.5708', '0', '0', 'base', 'touch_base']
        )
        
        # STEP 1: Start touch system immediately
        nodes_to_launch.extend([
            touch_base_tf_publisher,  # Start TF publisher immediately
            TimerAction(
                period=0.5,
                actions=[
                    GroupAction([
                        touch_state_node,
                        touch_robot_state_publisher,
                    ])
                ]
            )
        ])
        
        # STEP 2: Start USB camera after touch system is ready
        delayed_camera_container = TimerAction(
            period=1.5,  # After touch system is fully initialized
            actions=[
                ComposableNodeContainer(
                    name='camera_container',
                    namespace='',
                    package='rclcpp_components',
                    executable='component_container_mt',
                    composable_node_descriptions=[camera_component],
                    parameters=[{'use_intra_process_comms': True}],
                    output='screen',
                )
            ]
        )
        nodes_to_launch.append(delayed_camera_container)
        
        # STEP 3: Start agent after camera is ready
        agent_start_delay = 2.5
    else:
        # No touch system - start camera earlier
        delayed_camera_container = TimerAction(
            period=0.5,
            actions=[
                ComposableNodeContainer(
                    name='camera_container',
                    namespace='',
                    package='rclcpp_components',
                    executable='component_container_mt',
                    composable_node_descriptions=[camera_component],
                    parameters=[{'use_intra_process_comms': True}],
                    output='screen',
                )
            ]
        )
        nodes_to_launch.append(delayed_camera_container)
        agent_start_delay = 1.0
    
    # STEP 3: Start agent node last
    delayed_agent = TimerAction(
        period=agent_start_delay,
        actions=[agent_node]
    )
    nodes_to_launch.append(delayed_agent)
    
    # STEP 4: Start GUI if save_episode is enabled (after agent)
    if save_episode and gui_node:
        delayed_gui = TimerAction(
            period=agent_start_delay + 1.0,  # Start GUI 1 second after agent
            actions=[gui_node]
        )
        nodes_to_launch.append(delayed_gui)
    
    return nodes_to_launch


def generate_launch_description():
    """Generate launch description with all necessary arguments"""
    
    gello_ros_share_dir = get_package_share_directory('gello_ros')
    
    return LaunchDescription([
        # Agent system parameters
        DeclareLaunchArgument(
            'robot_config',
            default_value=os.path.join(gello_ros_share_dir, 'config', 'franka_fr3.yaml'),
            description='Path to the robot config file'
        ),
        DeclareLaunchArgument(
            'common_config',
            default_value=os.path.join(gello_ros_share_dir, 'config', 'common.yaml'),
            description='Path to the common config file'
        ),
        DeclareLaunchArgument(
            'touch_config',
            default_value=os.path.join(gello_ros_share_dir, 'config', 'touch.yaml'),
            description='Path to the touch config file'
        ),
        DeclareLaunchArgument(
            'save_episode',
            default_value='false',
            description='Whether to save episodes'
        ),
        DeclareLaunchArgument(
            'agent_type',
            default_value='touch',
            description='Type of agent to use (gello, touch, act, dummy)'
        ),
        DeclareLaunchArgument(
            'skip_initial_move',
            default_value='false',
            description='Whether to skip initial move to home position'
        ),
        DeclareLaunchArgument(
            'controller_type',
            default_value='cartesian_impedance_controller',
            description='Controller type for robot'
        ),
        
        # Camera system parameters
        DeclareLaunchArgument(
            'camera_device',
            default_value='/dev/video0',
            description='USB camera device path'
        ),
        DeclareLaunchArgument(
            'camera_width',
            default_value='640',
            description='Camera image width'
        ),
        DeclareLaunchArgument(
            'camera_height',
            default_value='480',
            description='Camera image height'
        ),
        DeclareLaunchArgument(
            'camera_framerate',
            default_value='30.0',
            description='Camera frame rate'
        ),
        DeclareLaunchArgument(
            'camera_name',
            default_value='bottom_camera',
            description='Camera name for topics and frames'
        ),
        
        # System parameters
        DeclareLaunchArgument(
            'enable_touch_system',
            default_value='true',
            description='Whether to enable touch system nodes'
        ),
        DeclareLaunchArgument(
            'num_threads',
            default_value='4',
            description='Number of threads for ComponentManager'
        ),
        
        # Launch setup function
        OpaqueFunction(function=launch_setup)
    ])