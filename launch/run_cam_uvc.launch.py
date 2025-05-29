from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import Node

def generate_launch_description():
    # Declare arguments if needed
    declared_args = [
        DeclareLaunchArgument('manager_name', default_value='nodelet_manager'),
    ]

    # Replace nodelet manager with separate node launches (since no nodelet in ROS 2)

    # bottom_camera node (only this one is uncommented in original)
    bottom_camera_node = Node(
        package='libuvc_camera',  # Replace with actual ROS 2 ported package name
        executable='uvc_camera_node',  # Typical name; adjust if different
        name='uvc_bottom_camera_node',
        output='screen',
        remappings=[
            ('image_raw', '/bottom_camera/color/image_raw')
        ],
        parameters=[{
            'vendor': '0x04f2',
            'product': '0x1601',
            'width': 640,
            'height': 480,
            'video_mode': 'mjpeg',
            'frame_rate': 30
        }]
    )

    # (Optional) If you want to restore the other cameras, you can replicate this pattern:
    # - uvc_base_camera_node
    # - uvc_side_camera_node

    return LaunchDescription(declared_args + [
        bottom_camera_node
    ])
