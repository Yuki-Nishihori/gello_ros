
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    realsense_launch_path = os.path.join(
        get_package_share_directory("realsense2_camera"),
        "launch",
        "rs_launch.py",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "serial_no_camera1", default_value="134222070753"
            ),
            DeclareLaunchArgument(
                "serial_no_camera2", default_value="132422070313"
            ),
            DeclareLaunchArgument("camera1", default_value="base_camera"),
            DeclareLaunchArgument("camera2", default_value="side_camera"),
            DeclareLaunchArgument(
                "tf_prefix_camera1",
                default_value=LaunchConfiguration("camera1"),
            ),
            DeclareLaunchArgument(
                "tf_prefix_camera2",
                default_value=LaunchConfiguration("camera2"),
            ),
            DeclareLaunchArgument("initial_reset", default_value="false"),
            DeclareLaunchArgument("reconnect_timeout", default_value="6.0"),
            DeclareLaunchArgument("color_width", default_value="640"),
            DeclareLaunchArgument("color_height", default_value="480"),
            DeclareLaunchArgument("enable_color", default_value="true"),
            DeclareLaunchArgument("color_fps", default_value="60"),
            DeclareLaunchArgument("enable_depth", default_value="true"),
            DeclareLaunchArgument("enable_confidence", default_value="true"),
            DeclareLaunchArgument("publish_tf", default_value="true"),
            DeclareLaunchArgument("publish_odom_tf", default_value="true"),
            GroupAction(
                [
                    PushRosNamespace(LaunchConfiguration("camera1")),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(realsense_launch_path),
                        launch_arguments={
                            "serial_no": LaunchConfiguration("serial_no_camera1"),
                            "camera_name": LaunchConfiguration("camera1"),
                            "tf_prefix": LaunchConfiguration("tf_prefix_camera1"),
                            "initial_reset": LaunchConfiguration("initial_reset"),
                            "reconnect_timeout": LaunchConfiguration(
                                "reconnect_timeout"
                            ),
                            "color_width": LaunchConfiguration("color_width"),
                            "color_height": LaunchConfiguration("color_height"),
                            "enable_color": LaunchConfiguration("enable_color"),
                            "color_fps": LaunchConfiguration("color_fps"),
                            "enable_depth": LaunchConfiguration("enable_depth"),
                            "enable_confidence": LaunchConfiguration(
                                "enable_confidence"
                            ),
                            "publish_tf": LaunchConfiguration("publish_tf"),
                            "publish_odom_tf": LaunchConfiguration(
                                "publish_odom_tf"
                            ),
                        }.items(),
                    ),
                ]
            ),
            GroupAction(
                [
                    PushRosNamespace(LaunchConfiguration("camera2")),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(realsense_launch_path),
                        launch_arguments={
                            "serial_no": LaunchConfiguration("serial_no_camera2"),
                            "camera_name": LaunchConfiguration("camera2"),
                            "tf_prefix": LaunchConfiguration("tf_prefix_camera2"),
                            "initial_reset": LaunchConfiguration("initial_reset"),
                            "reconnect_timeout": LaunchConfiguration(
                                "reconnect_timeout"
                            ),
                            "color_width": LaunchConfiguration("color_width"),
                            "color_height": LaunchConfiguration("color_height"),
                            "enable_color": LaunchConfiguration("enable_color"),
                            "color_fps": LaunchConfiguration("color_fps"),
                            "enable_depth": LaunchConfiguration("enable_depth"),
                            "enable_confidence": LaunchConfiguration(
                                "enable_confidence"
                            ),
                            "publish_tf": LaunchConfiguration("publish_tf"),
                            "publish_odom_tf": LaunchConfiguration(
                                "publish_odom_tf"
                            ),
                        }.items(),
                    ),
                ]
            ),
        ]
    )
