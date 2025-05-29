from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, TimerAction
from launch.substitutions import LaunchConfiguration, PythonExpression, Command
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # === Launch Arguments ===
    declared_args = [
        DeclareLaunchArgument('gello_config', default_value=os.path.join(
            get_package_share_directory('gello_ros'), 'config', 'gello.yaml')),
        DeclareLaunchArgument('touch_config', default_value=os.path.join(
            get_package_share_directory('gello_ros'), 'config', 'touch.yaml')),
        DeclareLaunchArgument('node_start_delay', default_value='2.0'),
        DeclareLaunchArgument('device_type', default_value='touch'),
    ]

    # === Launch Configurations ===
    gello_config = LaunchConfiguration('gello_config')
    touch_config = LaunchConfiguration('touch_config')
    node_start_delay = LaunchConfiguration('node_start_delay')
    device_type = LaunchConfiguration('device_type')

    # === robot_description as str ===
    robot_description_content = ParameterValue(
        Command(['cat ',
            os.path.join(get_package_share_directory('omni_description'), 'urdf/omni.urdf')
        ]),
        value_type=str
    )

    # === Gello Group ===
    gello_group = GroupAction(
        condition=IfCondition(PythonExpression(["'", device_type, "' == 'gello'"])),
        actions=[
            Node(
                package='gello_ros',
                executable='run_gello.py',
                name='run_gello',
                output='screen',
                parameters=[gello_config]
            )
        ]
    )

    # === Touch Group ===
    touch_group = GroupAction(
        condition=IfCondition(PythonExpression(["'", device_type, "' == 'touch'"])),
        actions=[
            Node(
                package='omni_common',
                executable='omni_state',
                name='omni_state',
                output='screen',
                parameters=[touch_config]
            ),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='touch_robot_state_publisher',
                remappings=[
                    ('joint_states', 'touch/joint_states'),
                    ('robot_description', 'touch_robot_description')
                ],
                parameters=[{
                    'robot_description': robot_description_content
                }]
            ),
            TimerAction(
                period=node_start_delay,
                actions=[
                    Node(
                        package='tf2_ros',
                        executable='static_transform_publisher',
                        name='static_transform_publisher',
                        arguments=['1', '0.5', '0', '0', '0', '0', 'world', 'touch_base'],
                        output='screen'
                    )
                ]
            )
        ]
    )

    # === Combine and Return ===
    return LaunchDescription(declared_args + [
        gello_group,
        touch_group
    ])
