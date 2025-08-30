import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    GroupAction, 
    TimerAction, 
    LogInfo
)
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    PathJoinSubstitution,
    FindExecutable,
)
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """
    カメラ統合マルチスレッド対応Franka FR3 + Touchデバイスlaunchファイル
    - RealSense、USB Cam対応
    - 高速画像処理用マルチスレッド
    """
    
    # --- Launch Configurations ---
    robot_config = LaunchConfiguration('robot_config')
    common_config = LaunchConfiguration('common_config')
    touch_config = LaunchConfiguration('touch_config')
    save_episode = LaunchConfiguration('save_episode')
    agent_type = LaunchConfiguration('agent_type')
    skip_initial_move = LaunchConfiguration('skip_initial_move')
    controller_type = LaunchConfiguration('controller_type')
    robot_ip = LaunchConfiguration('robot_ip')
    
    # Camera configurations
    enable_realsense = LaunchConfiguration('enable_realsense')
    enable_usb_cam = LaunchConfiguration('enable_usb_cam')
    realsense_serial = LaunchConfiguration('realsense_serial')
    usb_cam_device = LaunchConfiguration('usb_cam_device')
    camera_fps = LaunchConfiguration('camera_fps')
    image_width = LaunchConfiguration('image_width')
    image_height = LaunchConfiguration('image_height')

    # --- Package Share Directory ---
    gello_ros_share_dir = get_package_share_directory('gello_ros')

    # --- Franka Robot Description ---
    franka_robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare("franka_description"), "robots", "fr3", "fr3.urdf.xacro"]),
        " ",
        "hand:=true",
        " ",
        "ee_id:=franka_hand",
        " ",
        "robot_ip:=", robot_ip,
    ])
    franka_robot_description = {
        "robot_description": ParameterValue(value=franka_robot_description_content, value_type=str)
    }

    # --- Touch Device Description ---
    touch_robot_description_content = Command([
        FindExecutable(name='cat'), 
        ' ',
        PathJoinSubstitution([
            FindPackageShare('touch_description'),
            'urdf',
            'touch.urdf'
        ])
    ])
    touch_robot_description_param = ParameterValue(touch_robot_description_content, value_type=str)

    # --- Launch Arguments ---
    launch_arguments = [
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
            description='Whether to save the episode'
        ),
        DeclareLaunchArgument(
            'agent_type',
            default_value='touch',
            description='Type of agent to use (touch, gello, act)'
        ),
        DeclareLaunchArgument(
            'skip_initial_move',
            default_value='true',
            description='Whether to skip the initial move'
        ),
        DeclareLaunchArgument(
            'controller_type',
            default_value='cartesian_impedance_controller',
            description='Controller type'
        ),
        DeclareLaunchArgument(
            'robot_ip',
            default_value='192.168.1.1',
            description='IP address of Franka robot'
        ),
        
        # Camera arguments
        DeclareLaunchArgument(
            'enable_realsense',
            default_value='true',
            description='Enable RealSense camera'
        ),
        DeclareLaunchArgument(
            'enable_usb_cam',
            default_value='false',
            description='Enable USB camera'
        ),
        DeclareLaunchArgument(
            'realsense_serial',
            default_value='',
            description='RealSense camera serial number'
        ),
        DeclareLaunchArgument(
            'usb_cam_device',
            default_value='/dev/video0',
            description='USB camera device path'
        ),
        DeclareLaunchArgument(
            'camera_fps',
            default_value='30',
            description='Camera frame rate'
        ),
        DeclareLaunchArgument(
            'image_width',
            default_value='640',
            description='Camera image width'
        ),
        DeclareLaunchArgument(
            'image_height',
            default_value='480',
            description='Camera image height'
        ),
    ]

    # --- Camera Nodes Container (マルチスレッド) ---
    camera_container = ComposableNodeContainer(
        name='camera_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',  # Multi-threaded container
        composable_node_descriptions=[
            # RealSense camera
            ComposableNode(
                package='realsense2_camera',
                plugin='realsense2_camera::RealSenseNodeFactory',
                name='base_camera',
                namespace='base',
                parameters=[{
                    'serial_no': realsense_serial,
                    'enable_color': True,
                    'enable_depth': True,
                    'color_width': image_width,
                    'color_height': image_height,
                    'color_fps': camera_fps,
                    'depth_width': image_width,
                    'depth_height': image_height,
                    'depth_fps': camera_fps,
                    'align_depth': True,
                    'publish_tf': False,  # Avoid TF conflicts
                }],
                condition=LaunchConfiguration('enable_realsense')
            ),
        ],
        output='screen',
    )

    # --- USB Camera Node (separate for USB hardware interaction) ---
    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='side_camera',
        namespace='side',
        output='screen',
        parameters=[{
            'video_device': usb_cam_device,
            'framerate': camera_fps,
            'image_width': image_width,
            'image_height': image_height,
            'pixel_format': 'yuyv',
            'camera_frame_id': 'side_camera_frame',
            'io_method': 'mmap',
        }],
        condition=LaunchConfiguration('enable_usb_cam')
    )

    # --- Touch Device Nodes ---
    touch_state_node = Node(
        package='touch_common',
        executable='touch_state',
        name='touch_state',
        namespace='touch', 
        output='screen',
        parameters=[
            touch_config,
            {'robot_description': touch_robot_description_param}
        ]
    )

    touch_robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='touch_robot_state_publisher',
        namespace='touch', 
        output='screen',
        parameters=[{
            'robot_description': touch_robot_description_param, 
            'publish_frequency': 30.0
        }],
    )

    # --- Static TF Publishers ---
    touch_base_tf_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='touch_base_tf_publisher',
        namespace='touch', 
        arguments=['1', '0.5', '0', '-1.5708', '0', '0', 'base_link', 'touch_base']
    )

    # Camera TF publishers
    base_camera_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_camera_tf_publisher',
        arguments=['0.1', '0.0', '0.8', '0', '0.5236', '0', 'base_link', 'base_camera_frame']  # 30度下向き
    )

    side_camera_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='side_camera_tf_publisher',
        arguments=['0.5', '0.5', '0.5', '0', '0', '0.7854', 'base_link', 'side_camera_frame']  # 45度横向き
    )

    # --- Main Agent Node (高性能マルチスレッド対応) ---
    agent_node = Node(
        package='gello_ros',
        executable='run_multithred_agent_node.py',
        name='run_agent_node',
        output='screen',
        parameters=[
            robot_config,
            common_config,
            touch_config,
            franka_robot_description,
            {
                'save_episode': save_episode,
                'agent_type': agent_type,
                'skip_initial_move': skip_initial_move,
                'controller_type': controller_type,
                'camera_names': ['base', 'side'],  # Enable multi-camera support
            }
        ],
        ros_arguments=[
            '--ros-args', 
            '--log-level', 'INFO'
        ],
    )

    # --- 段階的起動シーケンス ---
    # Phase 1: Essential TF and static transforms (immediate)
    essential_nodes = GroupAction([
        LogInfo(msg="📹 カメラ統合マルチスレッドシステムを起動中..."),
        touch_base_tf_publisher_node,
        base_camera_tf_node,
        side_camera_tf_node,
    ])

    # Phase 2: Camera systems (0.5s delay)
    camera_startup = TimerAction(
        period=0.5,
        actions=[
            GroupAction([
                camera_container,
                usb_cam_node,
            ])
        ]
    )

    # Phase 3: Touch device infrastructure (1.0s delay)
    touch_infrastructure = TimerAction(
        period=1.0,
        actions=[
            GroupAction([
                touch_state_node,
                touch_robot_state_publisher_node,
            ])
        ]
    )

    # Phase 4: Main agent (2.0s delay for full camera initialization)
    agent_startup = TimerAction(
        period=2.0,
        actions=[agent_node]
    )

    # Phase 5: Ready notification (3.0s delay)
    ready_notification = TimerAction(
        period=3.0,
        actions=[
            LogInfo(msg="🚀📹 マルチスレッドカメラ統合Frankaシステムが起動完了しました！")
        ]
    )

    return LaunchDescription(
        launch_arguments + [
            essential_nodes,
            camera_startup,
            touch_infrastructure,
            agent_startup,
            ready_notification,
        ]
    )