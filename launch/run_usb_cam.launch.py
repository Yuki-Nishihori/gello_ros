#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='/dev/video1',
        description='UVC camera device path'
    )
    
    width_arg = DeclareLaunchArgument(
        'width',
        default_value='640',
        description='Camera image width'
    )
    
    height_arg = DeclareLaunchArgument(
        'height',
        default_value='480', 
        description='Camera image height'
    )
    
    framerate_arg = DeclareLaunchArgument(
        'framerate',
        default_value='30.0',
        description='Camera frame rate'
    )
    
    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='bottom_camera',
        description='Camera name for topics'
    )

    # UVC Camera node (using usb_cam package)
    uvc_camera_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name=LaunchConfiguration('camera_name'),
        namespace=LaunchConfiguration('camera_name'),
        parameters=[{
            'video_device': LaunchConfiguration('device'),
            'image_width': LaunchConfiguration('width'),
            'image_height': LaunchConfiguration('height'),
            'framerate': LaunchConfiguration('framerate'),
            'pixel_format': 'mjpeg2rgb',
            'camera_name': LaunchConfiguration('camera_name'),
            'camera_info_url': '',
            'io_method': 'mmap',
            'frame_id': [LaunchConfiguration('camera_name'), TextSubstitution(text='_optical_frame')],
        }],
        output='screen',
        remappings=[
            ('image_raw', 'color/image_raw')
        ]
    )

    return LaunchDescription([
        device_arg,
        width_arg,
        height_arg,
        framerate_arg,
        camera_name_arg,
        uvc_camera_node
    ])