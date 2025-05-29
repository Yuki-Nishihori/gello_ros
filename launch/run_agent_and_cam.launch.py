from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import os

def generate_launch_description():
    # Declare arguments
    declared_args = [
        DeclareLaunchArgument('robot_config', default_value='/home/ubuntu/ros2_ws/src/gello_ros/config/ur5e.yaml'),
        DeclareLaunchArgument('common_config', default_value='/home/ubuntu/ros2_ws/src/gello_ros/config/common.yaml'),
        DeclareLaunchArgument('touch_config', default_value='/home/ubuntu/ros2_ws/src/gello_ros/config/touch.yaml'),
        DeclareLaunchArgument('save_episode', default_value='false'),
        DeclareLaunchArgument('agent_type', default_value='touch'),
        DeclareLaunchArgument('camera_type', default_value='uvc_cam'),
    ]

    # Launch configurations
    robot_config = LaunchConfiguration('robot_config')
    common_config = LaunchConfiguration('common_config')
    touch_config = LaunchConfiguration('touch_config')
    save_episode = LaunchConfiguration('save_episode')
    agent_type = LaunchConfiguration('agent_type')
    camera_type = LaunchConfiguration('camera_type')

    # Launch file paths
    pkg_gello = os.path.join(get_package_share_directory('gello_ros'), 'launch')

    # Realsense group
    realsense_group = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_gello, 'run_cam_realsense.launch.py'))
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_gello, 'run_agent.launch.py')),
                launch_arguments={
                    'robot_config': robot_config,
                    'common_config': common_config,
                    'touch_config': touch_config,
                    'node_start_delay': '10',
                    'save_episode': save_episode,
                    'agent_type': agent_type,
                }.items()
            )
        ],
        condition=IfCondition(PythonExpression(["'", camera_type, "' == 'realsense'"]))
    )

    # UVC camera group
    uvc_cam_group = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_gello, 'run_cam_uvc.launch.py'))
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_gello, 'run_agent.launch.py')),
                launch_arguments={
                    'robot_config': robot_config,
                    'common_config': common_config,
                    'touch_config': touch_config,
                    'node_start_delay': '1',
                    'save_episode': save_episode,
                    'agent_type': agent_type,
                }.items()
            )
        ],
        condition=IfCondition(PythonExpression(["'", camera_type, "' == 'uvc_cam'"]))
    )

    # GUI node if save_episode is true
    gui_node = Node(
        package='gello_ros',
        executable='run_GUI.py',
        name='run_gui',
        output='screen',
        condition=IfCondition(save_episode)
    )

    return LaunchDescription(declared_args + [
        realsense_group,
        uvc_cam_group,
        gui_node
    ])

# Required for get_package_share_directory
from ament_index_python.packages import get_package_share_directory
