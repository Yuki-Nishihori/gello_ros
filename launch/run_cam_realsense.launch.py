from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    launch_args = [
        DeclareLaunchArgument('serial_no_camera1', default_value='134222070753'),
        DeclareLaunchArgument('serial_no_camera2', default_value='132422070313'),
        DeclareLaunchArgument('camera1', default_value='base_camera'),
        DeclareLaunchArgument('camera2', default_value='side_camera'),
        DeclareLaunchArgument('initial_reset', default_value='false'),
        DeclareLaunchArgument('reconnect_timeout', default_value='6.0'),
        DeclareLaunchArgument('color_width', default_value='640'),
        DeclareLaunchArgument('color_height', default_value='480'),
        DeclareLaunchArgument('enable_color', default_value='true'),
        DeclareLaunchArgument('color_fps', default_value='60'),
        DeclareLaunchArgument('enable_depth', default_value='true'),
        DeclareLaunchArgument('enable_confidence', default_value='true'),
        DeclareLaunchArgument('publish_tf', default_value='true'),
        DeclareLaunchArgument('publish_odom_tf', default_value='true'),
    ]

    # Use LaunchConfiguration for substitution
    serial_no_camera1 = LaunchConfiguration('serial_no_camera1')
    serial_no_camera2 = LaunchConfiguration('serial_no_camera2')
    camera1 = LaunchConfiguration('camera1')
    camera2 = LaunchConfiguration('camera2')
    initial_reset = LaunchConfiguration('initial_reset')
    reconnect_timeout = LaunchConfiguration('reconnect_timeout')
    color_width = LaunchConfiguration('color_width')
    color_height = LaunchConfiguration('color_height')
    enable_color = LaunchConfiguration('enable_color')
    color_fps = LaunchConfiguration('color_fps')
    enable_depth = LaunchConfiguration('enable_depth')
    enable_confidence = LaunchConfiguration('enable_confidence')
    publish_tf = LaunchConfiguration('publish_tf')
    publish_odom_tf = LaunchConfiguration('publish_odom_tf')

    # Path to realsense launch file
    realsense_launch = FindPackageShare('realsense2_camera').find('realsense2_camera')
    rs_launch_file = f'{realsense_launch}/launch/rs_launch.py'

    # Group for camera1
    camera1_group = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rs_launch_file),
            launch_arguments={
                'camera_name': camera1,
                'serial_no': serial_no_camera1,
                'initial_reset': initial_reset,
                'reconnect_timeout': reconnect_timeout,
                'color_width': color_width,
                'color_height': color_height,
                'enable_color': enable_color,
                'color_fps': color_fps,
                'enable_depth': enable_depth,
                'enable_confidence': enable_confidence,
                'publish_tf': publish_tf,
                'publish_odom_tf': publish_odom_tf
            }.items()
        )
    ])

    # Group for camera2
    camera2_group = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rs_launch_file),
            launch_arguments={
                'camera_name': camera2,
                'serial_no': serial_no_camera2,
                'initial_reset': initial_reset,
                'reconnect_timeout': reconnect_timeout,
                'color_width': color_width,
                'color_height': color_height,
                'enable_color': enable_color,
                'color_fps': color_fps,
                'enable_depth': enable_depth,
                'enable_confidence': enable_confidence,
                'publish_tf': publish_tf,
                'publish_odom_tf': publish_odom_tf
            }.items()
        )
    ])

    return LaunchDescription(launch_args + [
        camera1_group,
        camera2_group
    ])
